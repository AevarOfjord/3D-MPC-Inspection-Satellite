
#include "linearizer.hpp"

namespace satellite_control {

Linearizer::Linearizer(const SatelliteParams& params, bool enable_gyro_jacobian)
    : params_(params),
      cw_dynamics_(params.orbital_mean_motion > 0.0 ? params.orbital_mean_motion : 0.0),
      two_body_dynamics_(params.orbital_mu, params.orbital_radius),
      use_two_body_(params.use_two_body),
      enable_gyro_jacobian_(enable_gyro_jacobian) {
    precompute_thrusters();
    affine_ = VectorXd::Zero(16);
}

void Linearizer::precompute_thrusters() {
    body_forces_.resize(params_.num_thrusters);
    body_torques_.resize(params_.num_thrusters);

    for(int i=0; i < params_.num_thrusters; ++i) {
        Vector3d pos = params_.thruster_positions[i];
        Vector3d rel_pos = pos - params_.com_offset;
        Vector3d dir = params_.thruster_directions[i].normalized(); // Ensure normalized
        double force_mag = params_.thruster_forces[i];

        Vector3d force = force_mag * dir;
        body_forces_[i] = force;
        body_torques_[i] = rel_pos.cross(force);
    }
}

Eigen::Matrix3d Linearizer::compute_rotation_matrix(const Eigen::Vector4d& q) {
    // q = [w, x, y, z]
    // Standard conversion
    double w = q[0], x = q[1], y = q[2], z = q[3];
    Eigen::Matrix3d R;
    R << 1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w,
         2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
         2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y;
    return R;
}

std::pair<MatrixXd, MatrixXd> Linearizer::linearize(const VectorXd& x_current) {
    // x = [px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, wrx, wry, wrz] (16)
    // u = [rw_x, rw_y, rw_z, th_1 ... th_N]

    int nx = 16;
    int nu = params_.num_rw + params_.num_thrusters;

    MatrixXd A = MatrixXd::Identity(nx, nx);
    MatrixXd B = MatrixXd::Zero(nx, nu);
    affine_.setZero();

    // Extract quaternion
    Eigen::Vector4d q = x_current.segment<4>(3);
    double q_norm = q.norm();
    if (q_norm > 1e-12) {
        q /= q_norm;
    } else {
        q << 1.0, 0.0, 0.0, 0.0;
    }
    Eigen::Matrix3d R = compute_rotation_matrix(q);

    double dt = params_.dt;

    // Linearization logic from python
    // dPos/dVel = I * dt
    A(0, 7) = dt;
    A(1, 8) = dt;
    A(2, 9) = dt;

    // dQuat/dOmega = 0.5 * G(q) * dt
    double w = q[0], x = q[1], y = q[2], z = q[3];
    Eigen::Matrix<double, 4, 3> G;
    G << -x, -y, -z,
          w, -z,  y,
          z,  w, -x,
         -y,  x,  w;
    G = 0.5 * G * dt;
    A.block<4, 3>(3, 10) = G;

    // Orbital dynamics (gravity) terms
    Eigen::Vector3d r_rel = x_current.segment<3>(0);
    if (use_two_body_) {
        Eigen::Vector3d target_pos = two_body_dynamics_.get_target_position();
        Eigen::Vector3d r_abs = target_pos + r_rel;
        double r_norm = r_abs.norm();
        if (r_norm > 1.0) {
            double r_norm3 = r_norm * r_norm * r_norm;
            double r_norm5 = r_norm3 * r_norm * r_norm;
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d rrT = (r_abs * r_abs.transpose());
            Eigen::Matrix3d J = (-params_.orbital_mu) * (I / r_norm3 - 3.0 * rrT / r_norm5);

            // Relative acceleration (inspector - target)
            Eigen::Vector3d a_inspector = (-params_.orbital_mu / r_norm3) * r_abs;
            Eigen::Vector3d a_target = two_body_dynamics_.compute_acceleration(target_pos);
            Eigen::Vector3d a_rel = a_inspector - a_target;

            // Add linearized gravity to velocity rows (7-9) w.r.t. position (0-2)
            A.block<3, 3>(7, 0) += J * dt;

            // Affine term for velocity update
            affine_.segment<3>(7) = dt * (a_rel - J * r_rel);
        }

        if (!freeze_target_) {
            two_body_dynamics_.propagate_target(dt);
        }
    } else if (params_.orbital_mean_motion > 0.0) {
        auto [A_cw, _] = cw_dynamics_.get_mpc_dynamics_matrices(dt);
        A += A_cw;
    }

    // Angular velocity dynamics Jacobian (gyroscopic coupling):
    // I * w_dot + w x (I w) = tau  =>  w_dot = I^{-1}(tau - w x (I w))
    if (enable_gyro_jacobian_ && x_current.size() >= 13) {
        Eigen::Vector3d omega = x_current.segment<3>(10);
        double Ix = params_.inertia[0];
        double Iy = params_.inertia[1];
        double Iz = params_.inertia[2];
        if (Ix > 1e-9 && Iy > 1e-9 && Iz > 1e-9) {
            Eigen::Matrix3d Jw = Eigen::Matrix3d::Zero();
            Jw(0, 1) = -((Iz - Iy) / Ix) * omega(2);
            Jw(0, 2) = -((Iz - Iy) / Ix) * omega(1);
            Jw(1, 0) = -((Ix - Iz) / Iy) * omega(2);
            Jw(1, 2) = -((Ix - Iz) / Iy) * omega(0);
            Jw(2, 0) = -((Iy - Ix) / Iz) * omega(1);
            Jw(2, 1) = -((Iy - Ix) / Iz) * omega(0);
            A.block<3, 3>(10, 10) += Jw * dt;
        }
    }

    // B matrix
    // Reaction wheels (first num_rw inputs)
    for(int i=0; i < params_.num_rw; ++i) {
        if(params_.rw_torque_limits[i] == 0.0) continue;

        // 1. Angular velocity dynamics: Idot_w = -tau_rw
        // B[10+i, i] = -1/I_sat * tau_max * dt
        // Note: This is simplified diagonal inertia assumption for body axes aligned with principal axes
        B(10 + i, i) = -(1.0 / params_.inertia[i]) * dt * params_.rw_torque_limits[i];

        // 2. Wheel speed dynamics: dot_wr = tau_rw / I_rw
        // B[13+i, i] = 1/I_rw * tau_max * dt
        if (i < params_.rw_inertia.size()) {
             B(13 + i, i) = (1.0 / params_.rw_inertia[i]) * dt * params_.rw_torque_limits[i];
        }
    }

    int th_offset = params_.num_rw;
    for(int i=0; i < params_.num_thrusters; ++i) {
        // Velocities: F_world / mass * dt
        Vector3d F_body = body_forces_[i];
        Vector3d F_world = R * F_body;

        B.block<3, 1>(7, th_offset + i) = F_world / params_.mass * dt;

        // Angular velocities: T_body / inertia * dt
        Vector3d T_body = body_torques_[i];
        // Element-wise division for diagonal inertia
        B.block<3, 1>(10, th_offset + i) = T_body.cwiseQuotient(params_.inertia) * dt;
    }

    return {A, B};
}

} // namespace satellite_control

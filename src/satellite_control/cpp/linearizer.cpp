
#include "linearizer.hpp"
#include <cmath>

namespace satellite_dt {

Linearizer::Linearizer(const SatelliteParams& params) : params_(params) {
    precompute_thrusters();
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
    // x = [px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz] (13)
    // u = [rw_x, rw_y, rw_z, th_1 ... th_N]
    
    int nx = 13;
    int nu = params_.num_rw + params_.num_thrusters;
    
    MatrixXd A = MatrixXd::Identity(nx, nx);
    MatrixXd B = MatrixXd::Zero(nx, nu);
    
    // Extract quaternion
    Eigen::Vector4d q = x_current.segment<4>(3);
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

    // B matrix
    // Reaction wheels (first num_rw inputs)
    for(int i=0; i < params_.num_rw; ++i) {
        if(params_.rw_torque_limits[i] == 0.0) continue;
        // B[10+i, i]
        B(10 + i, i) = -(1.0 / params_.inertia[i]) * dt; 
        // Note: Python didn't multiply by max_torque here in B assignment line 289?
        // Wait, Python code: self.rw_torque_limits[i] / self.moment_of_inertia[i] * self.dt
        // In python, the control input is normalized [-1, 1], so we multiply by max_torque in B.
        // Let's match Python.
        B(10 + i, i) *= params_.rw_torque_limits[i]; 
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

} // namespace satellite_dt

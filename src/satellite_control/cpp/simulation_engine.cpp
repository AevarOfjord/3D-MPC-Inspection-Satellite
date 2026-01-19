#include "simulation_engine.hpp"
#include <iostream>

namespace satellite_control {

SimulationEngine::SimulationEngine(const SatelliteParams& params, double mean_motion,
                                   double mu, double target_radius, bool use_nonlinear)
    : params_(params),
      cw_dynamics_(mean_motion),
      two_body_dynamics_(mu, target_radius),
      use_nonlinear_(use_nonlinear) {
    state_ = Eigen::VectorXd::Zero(16);
    // Initialize quaternion to [1, 0, 0, 0]
    state_(3) = 1.0;
}

void SimulationEngine::reset(const Eigen::VectorXd& state) {
    if (state.size() == 13) {
        // Pad with 3 zeros for wheel speeds
        state_ = Eigen::VectorXd::Zero(16);
        state_.segment(0, 13) = state;
    } else if (state.size() == 16) {
        state_ = state;
    } else {
        throw std::invalid_argument("State must have 13 or 16 elements");
    }
}

Eigen::VectorXd SimulationEngine::get_state() const {
    return state_;
}

Eigen::Vector3d SimulationEngine::get_rw_speeds() const {
    return state_.segment<3>(13);
}

void SimulationEngine::step(double dt, const std::vector<double>& thruster_cmds, const std::vector<double>& rw_torques) {
    if (thruster_cmds.size() != params_.num_thrusters) {
        // Just warning or throw? Throw for safety
        throw std::invalid_argument("Thruster commands size mismatch");
    }

    // 4th Order Runge-Kutta Integration (RK4)
    // k1 = f(y)
    // k2 = f(y + dt/2 * k1)
    // k3 = f(y + dt/2 * k2)
    // k4 = f(y + dt * k3)
    // y_next = y + dt/6 * (k1 + 2k2 + 2k3 + k4)

    // Compute Forces & Torques (constant over step for zero-order hold assumption)
    // In reality, thrusters might pulse, but 'thruster_cmds' is average duty cycle over dt?
    // If dt is small (physics step ~5ms), we can treat as constant force * duty.
    
    // Quaternion extraction from state
    Eigen::Vector4d q = state_.segment<4>(3);
    
    Eigen::Vector3d forces = compute_total_force(thruster_cmds, q);
    Eigen::Vector3d torques = compute_total_torque(thruster_cmds, rw_torques);

    // RK4 for Rigid Body Dynamics
    // Note: forces depend on attitude (if body fixed) but here we precalc in BODY frame usually?
    // compute_total_force needs to return force in INERTIAL/HILL frame for CW dynamics if we apply it there.
    // Actually, `body_forces_` are in body frame. We rotate them to Hill frame using 'q'.
    // `forces` passed to derivative must be handled carefully.
    
    // Let's refine the compute_force logic inside derivative or outside?
    // The force in Hill frame depends on 'q'. Since 'q' changes during step, force direction changes.
    // So 'forces' should be re-evaluated inside derivative function if we want high precision.
    // However, for small steps, holding it constant in BODY frame is correct (thrusters attached to body),
    // but its INERTIAL direction changes.
    
    // So compute_state_derivative needs to take BODY forces and rotate them.

    // (Moved to RK4 block below)
    
    // Re-architecture: compute_state_derivative calculates forces internally from stored cmds?
    // Or we pass body frame forces and it rotates them?
    // Let's pass body frame force/torque vectors and let derivative function rotate them.
    
    Eigen::Vector3d body_force_cmds = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_torque_cmds = Eigen::Vector3d::Zero();
    
    // Sum thrusters in body frame
    for (size_t i = 0; i < params_.num_thrusters; ++i) {
        double f = params_.thruster_forces[i] * thruster_cmds[i];
        body_force_cmds += params_.thruster_directions[i] * f;
        
        // Torque t = r x F
        Eigen::Vector3d r = params_.thruster_positions[i] - params_.com_offset;
        Eigen::Vector3d F = params_.thruster_directions[i] * f;
        body_torque_cmds += r.cross(F);
    }
    
    // Add RW torques to body torque (reaction)
    // RW torque T_rw on wheel => -T_rw on body
    // Using mapping params_.rw_torque_limits to scale if needed? 
    // Assuming rw_torques input is already in Newton-meters [x, y, z].
    for (size_t i = 0; i < std::min((size_t)3, rw_torques.size()); ++i) {
         body_torque_cmds(i) -= rw_torques[i]; // Negative because reaction
    }
    
    // --- RK4 Implementation ---
    // Prepare RW Torque Vector (size 3) for derivative
    Eigen::Vector3d rw_torque_vec = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < std::min((size_t)3, rw_torques.size()); ++i) {
        rw_torque_vec(i) = rw_torques[i];
    }
    
    Eigen::VectorXd k1_state = compute_state_derivative(state_, body_force_cmds, body_torque_cmds, rw_torque_vec);
    
    Eigen::VectorXd s2 = state_ + 0.5 * dt * k1_state;
    // Normalize quaternion in s2
    s2.segment<4>(3).normalize();
    Eigen::VectorXd k2_state = compute_state_derivative(s2, body_force_cmds, body_torque_cmds, rw_torque_vec);
    
    Eigen::VectorXd s3 = state_ + 0.5 * dt * k2_state;
    s3.segment<4>(3).normalize();
    Eigen::VectorXd k3_state = compute_state_derivative(s3, body_force_cmds, body_torque_cmds, rw_torque_vec);
    
    Eigen::VectorXd s4 = state_ + dt * k3_state;
    s4.segment<4>(3).normalize();
    Eigen::VectorXd k4_state = compute_state_derivative(s4, body_force_cmds, body_torque_cmds, rw_torque_vec);
    
    state_ = state_ + (dt / 6.0) * (k1_state + 2.0*k2_state + 2.0*k3_state + k4_state);
    
    // Re-normalize quaternion
    state_.segment<4>(3).normalize();
    
    // Propagate target orbit (for nonlinear dynamics)
    if (use_nonlinear_) {
        two_body_dynamics_.propagate_target(dt);
    }
    
    // Reaction Wheel Speed Accel: update internal state via derivative now
}

Eigen::VectorXd SimulationEngine::compute_state_derivative(const Eigen::VectorXd& s, const Eigen::Vector3d& body_force, const Eigen::Vector3d& body_torque, const Eigen::Vector3d& rw_torque) {
    /*
     State: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, wrx, wry, wrz]
     */
    
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(16);
    
    // 1. Position Derivative: v
    dxdt.segment<3>(0) = s.segment<3>(7);
    
    // 2. Quaternion Derivative: 0.5 * q * omega
    Eigen::Vector4d q = s.segment<4>(3);
    Eigen::Vector3d w = s.segment<3>(10);
    
    // Omega pure quaternion [0, wx, wy, wz]
    // q_dot = 0.5 * q * w (quaternion mult)
    // standard formula: 
    // qw_dot = -0.5 * (qx*wx + qy*wy + qz*wz)
    // qx_dot = 0.5 * (qw*wx + qy*wz - qz*wy)
    // qy_dot = 0.5 * (qw*wy - qx*wz + qz*wx)
    // qz_dot = 0.5 * (qw*wz + qx*wy - qy*wx)
    
    dxdt(3) = -0.5 * (q.x()*w.x() + q.y()*w.y() + q.z()*w.z()); // Note q(1)=x, q(2)=y, q(3)=z in Eigen if using Vector4d but we manually map 0=w, 1=x, 2=y, 3=z
    // Wait, Eigen quaternions are usually x,y,z,w or w,x,y,z?
    // Eigen::Quaterniond stores as (x,y,z,w) internally usually? NO, it's coefficient access.
    // BUT we are using Vector4d. Let's stick to our convention: [w, x, y, z] (scalar first)
    // q(0)=w, q(1)=x, q(2)=y, q(3)=z
    
    dxdt(3) = -0.5 * (q(1)*w(0) + q(2)*w(1) + q(3)*w(2));
    dxdt(4) = 0.5 * (q(0)*w(0) + q(2)*w(2) - q(3)*w(1));
    dxdt(5) = 0.5 * (q(0)*w(1) - q(1)*w(2) + q(3)*w(0));
    dxdt(6) = 0.5 * (q(0)*w(2) + q(1)*w(1) - q(2)*w(0));
    

    // 3. Velocity Derivative: Gravity + Forces/Mass
    Eigen::Vector3d r = s.segment<3>(0);
    Eigen::Vector3d v = s.segment<3>(7);
    
    // Orbital gravity acceleration (Hill Frame)
    Eigen::Vector3d grav_acc;
    if (use_nonlinear_) {
        // Full two-body (nonlinear) gravity
        Eigen::Vector3d target_pos = two_body_dynamics_.get_target_position();
        grav_acc = two_body_dynamics_.compute_relative_acceleration(r, v, target_pos);
    } else {
        // Linear CW approximation
        grav_acc = cw_dynamics_.compute_acceleration(r, v);
    }
    
    // External Forces (Thrusters) - Rotate from Body to Hill/Inertial
    // R * F_body
    // Construct rotation matrix from quaternion
    // q = [w, x, y, z]
    Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
    Eigen::Matrix3d R = quat.toRotationMatrix();
    
    Eigen::Vector3d f_inertial = R * body_force;
    Eigen::Vector3d acc_thrust = f_inertial / params_.mass;
    
    dxdt.segment<3>(7) = grav_acc + acc_thrust;
    
    // 4. Angular Velocity Derivative: Euler's Eqs
    // I * w_dot + w x (I * w) = Torque
    // w_dot = inv(I) * (Torque - w x (I * w))
    
    // Assuming diagonal inertia for simplicity or full matrix
    Eigen::Matrix3d I_mat = Eigen::Matrix3d::Zero();
    I_mat.diagonal() = params_.inertia;
    Eigen::Matrix3d I_inv = I_mat.inverse();
    
    Eigen::Vector3d Iw = I_mat * w;
    Eigen::Vector3d gyroscopic = w.cross(Iw);
    
    dxdt.segment<3>(10) = I_inv * (body_torque - gyroscopic);
    
    // 5. Wheel Speed Derivative: T_rw / I_rw
    // RW speeds are at indices 13-15
    for(int i=0; i<3; ++i) {
        double inertia = 0.001; // Default fallback
        if (i < params_.rw_inertia.size() && params_.rw_inertia[i] > 1e-6) {
            inertia = params_.rw_inertia[i];
        }
        
        // omega_dot = T_rw / I_rw
        dxdt(13 + i) = rw_torque(i) / inertia;
    }
    
    return dxdt;
}

Eigen::Vector3d SimulationEngine::compute_total_force(const std::vector<double>& cmds, const Eigen::Vector4d& q) const {
    // Helper not used in main step logic anymore
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d SimulationEngine::compute_total_torque(const std::vector<double>& cmds, const std::vector<double>& rw_torques) const {
    // Helper not used in main step logic anymore
    return Eigen::Vector3d::Zero();
}

} // namespace satellite_control

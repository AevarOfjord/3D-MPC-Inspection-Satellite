#include "orbital_dynamics.hpp"
#include <cmath>

namespace satellite_control {

// ============================================================================
// Clohessy-Wiltshire (CW) Dynamics
// ============================================================================

CWDynamics::CWDynamics(double mean_motion) : n_(mean_motion), n_sq_(mean_motion * mean_motion) {}

Eigen::Vector3d CWDynamics::compute_acceleration(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) const {
    double x = position.x();
    double z = position.z();
    double vx = velocity.x();
    double vy = velocity.y();
    
    // CW equations of motion (Hill's equations):
    // ax = 3n^2x + 2nvy  (Radial)
    // ay = -2nvx         (Along-track)
    // az = -n^2z         (Cross-track / Normal)
    
    double ax = 3.0 * n_sq_ * x + 2.0 * n_ * vy;
    double ay = -2.0 * n_ * vx;
    double az = -n_sq_ * z;

    return Eigen::Vector3d(ax, ay, az);
}

std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 3>> CWDynamics::get_state_matrices(double dt) const {
    /*
     * Continuous-time A matrix (6x6)
     * State: [x, y, z, vx, vy, vz]
     * 
     * x_dot = vx
     * y_dot = vy
     * z_dot = vz
     * vx_dot = 3n^2x + 2nvy
     * vy_dot = -2nvx
     * vz_dot = -n^2z
     */
    Eigen::Matrix<double, 6, 6> A_cont = Eigen::Matrix<double, 6, 6>::Zero();
    
    // Identity blocks for position derivatives (dr/dt = v)
    A_cont(0, 3) = 1.0;
    A_cont(1, 4) = 1.0;
    A_cont(2, 5) = 1.0;

    // Dynamics blocks (dv/dt = f(r,v))
    A_cont(3, 0) = 3.0 * n_sq_;   // dvx/dx
    A_cont(3, 4) = 2.0 * n_;      // dvx/dvy
    A_cont(4, 3) = -2.0 * n_;     // dvy/dvx
    A_cont(5, 2) = -n_sq_;        // dvz/dz

    // B matrix (6x3) - input is acceleration [ax, ay, az]
    Eigen::Matrix<double, 6, 3> B_cont = Eigen::Matrix<double, 6, 3>::Zero();
    B_cont(3, 0) = 1.0;
    B_cont(4, 1) = 1.0;
    B_cont(5, 2) = 1.0;

    // Discretize using forward Euler: A_d = I + A*dt
    Eigen::Matrix<double, 6, 6> A_disc = Eigen::Matrix<double, 6, 6>::Identity() + A_cont * dt;
    Eigen::Matrix<double, 6, 3> B_disc = B_cont * dt;

    return {A_disc, B_disc};
}

std::pair<Eigen::Matrix<double, 16, 16>, Eigen::Matrix<double, 16, 9>> CWDynamics::get_mpc_dynamics_matrices(double dt) const {
    /*
     * Full 16-state dynamics, adding just the CW terms to an empty matrix.
     * The MPC controller will add this to the standard rigid body dynamics.
     * 
     * State indices in 16-state vector:
     * 0-2: pos (x,y,z)
     * 7-9: vel (vx,vy,vz)
     */
    Eigen::Matrix<double, 16, 16> A_cw = Eigen::Matrix<double, 16, 16>::Zero();

    // Position -> Velocity coupling (Gravity Gradient & Coriolis)
    
    // vx_dot += 3n^2x + 2nvy
    A_cw(7, 0) = 3.0 * n_sq_ * dt;  // dvx/dx
    A_cw(7, 8) = 2.0 * n_ * dt;     // dvx/dvy (Coriolis, note: vy corresponds to state index 8)

    // vy_dot += -2nvx
    A_cw(8, 7) = -2.0 * n_ * dt;    // dvy/dvx

    // vz_dot += -n^2z
    A_cw(9, 2) = -n_sq_ * dt;       // dvz/dz

    // B matrix is zero for CW gravity effects (it's passive dynamics)
    Eigen::Matrix<double, 16, 9> B_cw = Eigen::Matrix<double, 16, 9>::Zero();

    return {A_cw, B_cw};
}

// ============================================================================
// TwoBodyDynamics Implementation
// ============================================================================

TwoBodyDynamics::TwoBodyDynamics(double mu, double target_radius)
    : mu_(mu),
      target_radius_(target_radius),
      mean_motion_(std::sqrt(mu / (target_radius * target_radius * target_radius))),
      orbit_phase_(0.0),
      target_pos_(target_radius, 0.0, 0.0),
      target_vel_(0.0, mean_motion_ * target_radius, 0.0)
{
}

Eigen::Vector3d TwoBodyDynamics::compute_acceleration(const Eigen::Vector3d& abs_position) const {
    // Newton's law of gravitation: a = -μ/|r|³ × r
    double r_mag = abs_position.norm();
    if (r_mag < 1.0) {
        // Prevent singularity at center
        return Eigen::Vector3d::Zero();
    }
    double r_cubed = r_mag * r_mag * r_mag;
    return (-mu_ / r_cubed) * abs_position;
}

Eigen::Vector3d TwoBodyDynamics::compute_relative_acceleration(
    const Eigen::Vector3d& rel_position,
    const Eigen::Vector3d& rel_velocity,
    const Eigen::Vector3d& target_pos
) const {
    // Inspector absolute position = target + relative
    Eigen::Vector3d inspector_pos = target_pos + rel_position;
    
    // Gravitational accelerations
    Eigen::Vector3d a_inspector = compute_acceleration(inspector_pos);
    Eigen::Vector3d a_target = compute_acceleration(target_pos);
    
    // Relative acceleration (differential gravity)
    // Note: rel_velocity is unused here but kept for interface consistency
    (void)rel_velocity;
    
    return a_inspector - a_target;
}

void TwoBodyDynamics::propagate_target(double dt) {
    // Simple circular orbit propagation
    // For circular orbit: phase increases at rate n
    orbit_phase_ += mean_motion_ * dt;
    
    // Wrap phase to [0, 2π)
    while (orbit_phase_ >= 2.0 * M_PI) {
        orbit_phase_ -= 2.0 * M_PI;
    }
    
    // Update target position (circular orbit in x-y plane)
    double r = target_radius_;
    target_pos_.x() = r * std::cos(orbit_phase_);
    target_pos_.y() = r * std::sin(orbit_phase_);
    target_pos_.z() = 0.0;
    
    // Update target velocity (tangent to orbit)
    double v = mean_motion_ * r;
    target_vel_.x() = -v * std::sin(orbit_phase_);
    target_vel_.y() =  v * std::cos(orbit_phase_);
    target_vel_.z() = 0.0;
}

} // namespace satellite_control

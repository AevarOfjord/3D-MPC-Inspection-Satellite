#pragma once

#include <Eigen/Dense>
#include <utility>

namespace satellite_control {

class CWDynamics {
public:
    /**
     * @brief Construct a new CWDynamics object
     * 
     * @param mean_motion Orbital mean motion (n) [rad/s]
     */
    explicit CWDynamics(double mean_motion);

    /**
     * @brief Compute CW gravitational acceleration
     * 
     * @param position Relative position [x, y, z] [m]
     * @param velocity Relative velocity [vx, vy, vz] [m/s]
     * @return Eigen::Vector3d Acceleration [ax, ay, az] [m/s^2]
     */
    Eigen::Vector3d compute_acceleration(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) const;

    /**
     * @brief Get discrete-time state transition matrices for CW dynamics (6x6 state)
     * 
     * State: [x, y, z, vx, vy, vz]
     * 
     * @param dt Time step [s]
     * @return std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 3>> (A, B)
     */
    std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 3>> get_state_matrices(double dt) const;

    /**
     * @brief Get 16-state dynamics matrices including CW terms for MPC
     * 
     * A_cw adds CW gravity gradient terms to the full 16-state dynamics.
     * State: [pos(3), quat(4), vel(3), ang_vel(3), rw_speeds(3)]
     * 
     * @param dt Time step [s]
     * @return std::pair<Eigen::Matrix<double, 16, 16>, Eigen::Matrix<double, 16, 9>> (A_cw, B_dummy)
     */
    std::pair<Eigen::Matrix<double, 16, 16>, Eigen::Matrix<double, 16, 9>> get_mpc_dynamics_matrices(double dt) const;

private:
    double n_;      // Mean motionn
    double n_sq_;   // n^2
};

/**
 * @brief Full two-body (Newtonian) orbital dynamics
 * 
 * Computes gravitational acceleration using Newton's law: a = -μ/|r|³ × r
 * Provides higher fidelity than linearized CW equations for:
 * - Eccentric orbits
 * - Large relative distances
 * - Long propagation times
 */
class TwoBodyDynamics {
public:
    /**
     * @brief Construct TwoBodyDynamics
     * 
     * @param mu Gravitational parameter [m³/s²] (Earth: 3.986004418e14)
     * @param target_radius Initial target orbital radius [m]
     */
    TwoBodyDynamics(double mu, double target_radius);

    /**
     * @brief Compute gravitational acceleration at absolute position
     * 
     * @param abs_position Absolute position from central body [m]
     * @return Eigen::Vector3d Gravitational acceleration [m/s²]
     */
    Eigen::Vector3d compute_acceleration(const Eigen::Vector3d& abs_position) const;

    /**
     * @brief Compute relative acceleration between inspector and target
     * 
     * @param rel_position Relative position (inspector - target) [m]
     * @param rel_velocity Relative velocity [m/s]
     * @param target_pos Target absolute position [m]
     * @return Eigen::Vector3d Relative acceleration [m/s²]
     */
    Eigen::Vector3d compute_relative_acceleration(
        const Eigen::Vector3d& rel_position,
        const Eigen::Vector3d& rel_velocity,
        const Eigen::Vector3d& target_pos
    ) const;

    /**
     * @brief Propagate target position on circular orbit
     * 
     * @param dt Time step [s]
     */
    void propagate_target(double dt);

    /**
     * @brief Get current target position
     */
    Eigen::Vector3d get_target_position() const { return target_pos_; }

    /**
     * @brief Get current target velocity  
     */
    Eigen::Vector3d get_target_velocity() const { return target_vel_; }

private:
    double mu_;           // Gravitational parameter
    double target_radius_;// Target orbital radius
    double mean_motion_;  // n = sqrt(mu/r³)
    double orbit_phase_;  // Current orbital phase angle
    Eigen::Vector3d target_pos_;  // Target absolute position
    Eigen::Vector3d target_vel_;  // Target absolute velocity
};

} // namespace satellite_control

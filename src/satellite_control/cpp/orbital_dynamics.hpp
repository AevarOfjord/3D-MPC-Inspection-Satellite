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

} // namespace satellite_control

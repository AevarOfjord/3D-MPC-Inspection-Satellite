#pragma once

#include <Eigen/Dense>
#include <vector>
#include <random>
#include "satellite_params.hpp"
#include "orbital_dynamics.hpp"

namespace satellite_control {

// Using directives not needed since we are in the same namespace now
// using namespace satellite_dt; 
// using namespace satellite_control; // for CWDynamics

class __attribute__((visibility("default"))) SimulationEngine {
public:
    /**
     * @brief Construct a new Simulation Engine object
     * 
     * @param params Satellite parameters (mass, inertia, thrusters)
     * @param mean_motion Orbital mean motion [rad/s]
     * @param mu Gravitational parameter [m³/s²] (default: Earth)
     * @param target_radius Target orbital radius [m]
     * @param use_nonlinear Use full two-body dynamics instead of CW
     */
    SimulationEngine(const SatelliteParams& params, double mean_motion,
                     double mu = 3.986004418e14, double target_radius = 6.778e6,
                     bool use_nonlinear = true);

    /**
     * @brief reset state to initial conditions
     * 
     * @param state Initial state [pos(3), quat(4), vel(3), ang_vel(3), rw_speeds(3)] length 16 (or 13, padded with 0)
     */
    void reset(const Eigen::VectorXd& state);

    /**
     * @brief Perform one simulation step
     * 
     * @param dt Time step [s]
     * @param thruster_cmds Thruster duty cycles [0-1] (length num_thrusters)
     * @param rw_torques Reaction wheel torques [N.m] (length 3 or num_rw)
     */
    void step(double dt, const std::vector<double>& thruster_cmds, const std::vector<double>& rw_torques);

    /**
     * @brief Get the current state
     * 
     * @return Eigen::VectorXd State vector (16 elements)
     */
    Eigen::VectorXd get_state() const;

    /**
     * @brief Get reaction wheel speeds
     * 
     * @return Eigen::Vector3d Reaction wheel speeds [rad/s]
     */
    Eigen::Vector3d get_rw_speeds() const;

private:
    SatelliteParams params_;
    CWDynamics cw_dynamics_;
    TwoBodyDynamics two_body_dynamics_;
    bool use_nonlinear_;

    // State vector: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, wrx, wry, wrz] (16)
    // Note: Quaternions are stored [w, x, y, z] to match customized layout
    Eigen::VectorXd state_;
    
    // rw_speeds_ integrated into state_ (indices 13-15)
    
    // Helpers
    Eigen::Vector3d compute_total_force(const std::vector<double>& thruster_cmds, const Eigen::Vector4d& quat) const;
    Eigen::Vector3d compute_total_torque(const std::vector<double>& thruster_cmds, const std::vector<double>& rw_torques) const;
    
    // Dynamics
    Eigen::VectorXd compute_state_derivative(const Eigen::VectorXd& x, const Eigen::Vector3d& total_force, const Eigen::Vector3d& total_torque, const Eigen::Vector3d& total_rw_torque);
};

} // namespace satellite_control

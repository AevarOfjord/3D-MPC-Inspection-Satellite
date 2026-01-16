#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <memory>
#include "osqp.h"
#include "linearizer.hpp"
#include "obstacle.hpp"

namespace satellite_control {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;

/**
 * @brief Configuration parameters for the MPC controller.
 */
struct MPCParams {
    // Dimensions
    int prediction_horizon = 50;    ///< Number of steps to predict
    double dt = 0.05;               ///< Time step [s]
    double solver_time_limit = 0.05;///< Max solver time per step [s]
    
    // Weights
    double Q_pos = 10.0;            ///< Position error weight
    double Q_vel = 1.0;             ///< Velocity error weight
    double Q_ang = 10.0;            ///< Angle error weight
    double Q_angvel = 1.0;          ///< Angular velocity error weight
    double R_thrust = 0.1;          ///< Thruster usage weight
    double R_rw_torque = 0.1;       ///< Reaction wheel torque usage weight
    
    // Constraints
    double max_velocity = 1.0;          ///< Max velocity magnitude [m/s]
    double max_angular_velocity = 1.0;  ///< Max angular velocity magnitude [rad/s]
    double position_bounds = 10.0;      ///< Box constraint for position [m]
    
    // Z-tilt
    bool enable_z_tilt = true;          ///< Enable heuristic Z-tilt correction
    double z_tilt_gain = 0.35;          ///< Gain for Z-tilt correction
    double z_tilt_max_rad = 0.35;       ///< Max tilt angle [rad] (~20 deg)
    
    // Collision avoidance (V3.0.0)
    bool enable_collision_avoidance = false; ///< Enable obstacle avoidance
    double obstacle_margin = 0.5;            ///< Safety margin for obstacles [m]
};

/**
 * @brief Result structure returned by the controller.
 */
struct ControlResult {
    VectorXd u;         ///< Optimal control vector (RW + Thrusters)
    int status;         ///< OSQP solver status
    double solve_time;  ///< Time taken to solve [s]
    bool timeout;       ///< Whether the solver timed out
};

/**
 * @brief Model Predictive Controller for Satellite Attitude and Position Control.
 * 
 * Uses a linearized dynamics model and OSQP solver to optimize control inputs
 * over a finite horizon. Supports reaction wheels and thrusters.
 */
class MPCControllerCpp {
public:
    /**
     * @brief Construct a new MPCControllerCpp object.
     * 
     * @param sat_params Satellite physical parameters.
     * @param mpc_params Controller configuration parameters.
     */
    MPCControllerCpp(const SatelliteParams& sat_params, const MPCParams& mpc_params);

    /**
     * @brief Destroy the MPCControllerCpp object and cleanup OSQP workspace.
     */
    ~MPCControllerCpp();

    /**
     * @brief Compute the optimal control action for a single target state.
     * 
     * @param x_current Current state vector (13x1).
     * @param x_target Target state vector (13x1).
     * @return ControlResult containing the optimal inputs and solver stats.
     */
    ControlResult get_control_action(const VectorXd& x_current, const VectorXd& x_target);

    /**
     * @brief Compute optimal control action for a trajectory of target states.
     * 
     * @param x_current Current state vector (13x1).
     * @param x_target_traj Matrix of target states (Horizon x 13).
     * @return ControlResult containing the optimal inputs and solver stats.
     */
    ControlResult get_control_action_trajectory(const VectorXd& x_current, const MatrixXd& x_target_traj);
    
    // -- Collision Avoidance --

    /**
     * @brief Set the set of obstacles for collision avoidance.
     * @param obstacles Set of obstacle objects.
     */
    void set_obstacles(const ObstacleSet& obstacles);

    /**
     * @brief Clear all obstacles.
     */
    void clear_obstacles();
    
    // -- Accessors --
    int num_controls() const { return nu_; }
    int prediction_horizon() const { return N_; }
    double dt() const { return dt_; }

private:
    // Dimensions
    int nx_ = 13;  // State dimension
    int nu_;       // Control dimension (RW + thrusters)
    int N_;        // Prediction horizon
    double dt_;
    
    // Parameters
    MPCParams mpc_params_;
    SatelliteParams sat_params_;
    
    // Linearizer
    std::unique_ptr<Linearizer> linearizer_;
    
    // OSQP workspace
    OSQPWorkspace* work_ = nullptr;
    OSQPSettings* settings_ = nullptr;
    OSQPData* data_ = nullptr;
    
    // Problem matrices (stored for updates)
    SparseMatrix<double> P_;  // Cost matrix
    SparseMatrix<double> A_;  // Constraint matrix
    VectorXd q_;              // Linear cost vector
    VectorXd l_, u_;          // Constraint bounds (lower, upper)
    
    // Precomputed Weight Vectors
    VectorXd Q_diag_;
    VectorXd R_diag_;
    VectorXd control_lower_;
    VectorXd control_upper_;
    
    // State tracking
    Eigen::Vector4d prev_quat_;
    
    // -- Initialization Helpers --
    /**
     * @brief Initialize the OSQP solver, matrices, and settings.
     */
    void init_solver();

    /**
     * @brief Build the cost matrix P.
     * Uses Q_diag_ and R_diag_ to construct diagonal usage costs.
     */
    void build_P_matrix(std::vector<Eigen::Triplet<double>>& triplets, int n_vars);

    /**
     * @brief Build the constraint matrix A.
     * Includes dynamics, initial state, state/control bounds, and obstacle slots.
     */
    void build_A_matrix(std::vector<Eigen::Triplet<double>>& triplets);

    /**
     * @brief Configure and load OSQP settings and data.
     * @param n_vars Number of variables.
     * @param n_constraints Number of constraints.
     */
    void setup_osqp_workspace(int n_vars, int n_constraints);
    
    // CSC matrix helpers (raw data for OSQP)
    std::vector<c_float> P_data_;
    std::vector<c_int> P_indices_;
    std::vector<c_int> P_indptr_;
    std::vector<c_float> A_data_;
    std::vector<c_int> A_indices_;
    std::vector<c_int> A_indptr_;
    
    // -- Index Maps for Fast Updates --
    
    /// Maps [row][col] -> index in A_data_ for B matrix entries (actuator dynamics).
    std::vector<std::vector<int>> B_idx_map_;
    
    /// Maps [row][col] -> index in A_data_ for quaternion dynamics entries.
    std::vector<std::vector<int>> A_idx_map_;
    
    // -- Collision Avoidance Internals --
    ObstacleSet obstacles_;
    int n_obs_constraints_ = 0;
    std::vector<std::vector<int>> obs_A_indices_; // Map[step][col] -> A index
    
    // -- Runtime Methods --
    void update_dynamics(const VectorXd& x_current);
    void update_cost(const VectorXd& x_target);
    void update_cost_trajectory(const MatrixXd& x_target_traj);
    void update_constraints(const VectorXd& x_current);
    void update_obstacle_constraints(const VectorXd& x_current, const VectorXd& x_target);
    VectorXd apply_z_tilt(const VectorXd& x_current, const VectorXd& x_target);
    
    // Deprecated
    void apply_obstacle_constraints(const VectorXd& x_current);
};

} // namespace satellite_control

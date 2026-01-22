#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
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
    
    // Weights (MPCC)
    double Q_contour = 1000.0;          ///< Weight for contouring error (stay on path)
    double Q_progress = 100.0;          ///< Weight for speed tracking (move forward)
    double Q_smooth = 10.0;             ///< Weight for velocity smoothness
    double Q_angvel = 1.0;              ///< Angular velocity error weight (retain for stabilization)

    double R_thrust = 0.1;          ///< Thruster usage weight
    double R_rw_torque = 0.1;       ///< Reaction wheel torque usage weight
    

    
    // Collision avoidance (V3.0.0)
    bool enable_collision_avoidance = false; ///< Enable obstacle avoidance
    double obstacle_margin = 0.5;            ///< Safety margin for obstacles [m]

    // Path Following (V4.0.0) - General Path MPCC
    double path_speed = 0.1;           ///< Path speed along reference [m/s]
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
     * @brief Compute the optimal control action for the current state.
     *
     * Path-following MPCC computes reference values internally from the path.
     *
     * @param x_current Current state vector (17x1 if augmented with s).
     * @return ControlResult containing the optimal inputs and solver stats.
     */
    ControlResult get_control_action(const VectorXd& x_current);
    
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
    int nx_ = 17;  // State dimension (13 base + 3 wheel speeds + 1 path s)
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
    
    // Map [step][0..2] -> Index in P_data_ for (x,s), (y,s), (z,s) cross terms
    std::vector<std::vector<int>> path_P_indices_; 
    // Map [step] -> Index in P_data_ for (s,s) diagonal entry
    std::vector<int> path_s_diag_indices_;
    // Map [step][0..5] -> Index in P_data_ for velocity block upper triangle
    // Order: (0,0),(0,1),(0,2),(1,1),(1,2),(2,2)
    std::vector<std::vector<int>> path_vel_P_indices_;
    
    // -- Runtime Methods --
    void update_dynamics(const VectorXd& x_current);
    void update_cost();
    void update_constraints(const VectorXd& x_current);
    void update_obstacle_constraints(const VectorXd& x_current);
    void update_path_cost(const VectorXd& x_current); // Path following linearization
    
    // Path following internal state
    std::vector<double> s_guess_; // Guess for path parameter s over horizon
    
    // -- General Path Data (V4.0.1) --
    // Path is defined as a list of (s, x, y, z) samples
    // where s is the arc-length parameter
    std::vector<double> path_s_;              // Arc-length samples [0, total_length]
    std::vector<Eigen::Vector3d> path_points_; // Position samples
    double path_total_length_ = 0.0;          // Total path length
    bool path_data_valid_ = false;            // True if path data has been set
    
    // Helper methods for path interpolation
    Eigen::Vector3d get_path_point(double s) const;
    Eigen::Vector3d get_path_tangent(double s) const;
    
public:
    // Set path data for general path following
    void set_path_data(const std::vector<std::array<double, 4>>& path_data);
    
private:
    // Deprecated
    void apply_obstacle_constraints(const VectorXd& x_current);
};

} // namespace satellite_control

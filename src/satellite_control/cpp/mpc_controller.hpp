#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <vector>
#include <memory>
#include <tuple>
#include <string>
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
    int control_horizon = 50;       ///< Number of steps with independent control inputs
    double dt = 0.05;               ///< Time step [s]
    double solver_time_limit = 0.05;///< Max solver time per step [s]
    std::string solver_type = "OSQP"; ///< Solver backend identifier
    bool verbose_mpc = false;       ///< Verbose solver output
    
    // Weights (MPCC)
    double Q_contour = 1000.0;          ///< Weight for contouring error (stay on path)
    double Q_progress = 100.0;          ///< Weight for speed tracking (move forward)
    double progress_reward = 0.0;       ///< Reward for forward progress (auto speed)
    double Q_lag = 0.0;                 ///< Weight for lag error (along tangent, 0 = auto)
    double Q_smooth = 10.0;             ///< Weight for velocity smoothness
    double Q_angvel = 1.0;              ///< Angular velocity error weight (retain for stabilization)
    double Q_attitude = 0.0;            ///< Attitude tracking weight (align body x-axis to path tangent)

    double R_thrust = 0.1;          ///< Thruster usage weight
    double R_rw_torque = 0.1;       ///< Reaction wheel torque usage weight
    double thrust_l1_weight = 0.0;  ///< Linear thruster penalty (fuel bias)
    double thrust_pair_weight = 0.0; ///< Penalty on opposing-pair co-firing
    double coast_pos_tolerance = 0.0; ///< Coasting band position error [m] (0 = off)
    double coast_vel_tolerance = 0.0; ///< Coasting band lateral velocity [m/s] (0 = off)
    double coast_min_speed = 0.0;     ///< Minimum progress speed when coasting [m/s]
    

    
    // Collision avoidance (V3.0.0)
    bool enable_collision_avoidance = false; ///< Enable obstacle avoidance
    double obstacle_margin = 0.5;            ///< Safety margin for obstacles [m]

    // Path Following (V4.0.0) - General Path MPCC
    double path_speed = 0.1;           ///< Path speed along reference [m/s]
    double path_speed_min = 0.01;      ///< Minimum path speed [m/s]
    double path_speed_max = 0.1;       ///< Maximum path speed [m/s]

    // Terminal handling (MPCC)
    // If set to 0, controller will auto-scale from Q_contour/Q_progress.
    double Q_terminal_pos = 0.0;       ///< Terminal position weight (0 = auto)
    double Q_terminal_s = 0.0;         ///< Terminal progress weight (0 = auto)
    double progress_taper_distance = 0.0; ///< Distance to taper v_ref near end (0 = auto)
    double progress_slowdown_distance = 0.0; ///< Slow down v_ref if contour error is high (0 = auto)
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
    double path_length() const { return path_total_length_; }
    bool has_path() const { return path_data_valid_; }

    // Path utilities (for fast projection in Python)
    std::tuple<double, Eigen::Vector3d, double, double> project_onto_path(
        const Eigen::Vector3d& position) const;

    /**
     * @brief Provide a warm-start control guess.
     * @param u_prev Either thruster-only (num_thrusters) or full control (nu).
     */
    void set_warm_start_control(const VectorXd& u_prev);

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

    /// Maps [row][col] -> index in A_data_ for orbital/velocity dynamics updates (rows 7-9, cols 0-9).
    std::vector<std::vector<int>> A_orbital_idx_map_;
    
    // -- Collision Avoidance Internals --
    ObstacleSet obstacles_;
    int n_obs_constraints_ = 0;
    int obs_per_step_ = 0;
    int obs_row_start_ = 0;
    std::vector<std::array<int, 3>> obs_A_indices_; // Map[row] -> A index for x,y,z

    // Map [step][0..2] -> Index in P_data_ for (x,s), (y,s), (z,s) cross terms
    std::vector<std::vector<int>> path_P_indices_; 
    // Map [step] -> Index in P_data_ for (s,s) diagonal entry
    std::vector<int> path_s_diag_indices_;
    // Map [step][0..2] -> Index in P_data_ for (x,x), (y,y), (z,z) diagonal entries
    std::vector<std::vector<int>> path_pos_diag_indices_;
    // Map [step][0..2] -> Index in P_data_ for (x,y), (x,z), (y,z) off-diagonal entries
    std::vector<std::vector<int>> path_pos_offdiag_indices_;
    // Map [step][0..5] -> Index in P_data_ for velocity block upper triangle
    // Order: (0,0),(0,1),(0,2),(1,1),(1,2),(2,2)
    std::vector<std::vector<int>> path_vel_P_indices_;
    std::vector<c_int> path_P_update_indices_;
    std::vector<c_float> path_P_update_values_;

    // Constraint bookkeeping
    int n_dyn_ = 0;
    int n_init_ = 0;
    int n_bounds_x_ = 0;
    int n_bounds_u_ = 0;
    int n_control_horizon_constraints_ = 0;
    int control_horizon_ = 0;

    // Dynamic affine term (for gravity, etc.)
    VectorXd dyn_affine_;

    // A-matrix update tracking
    bool A_dirty_ = false;

    // Warm-start storage
    VectorXd warm_start_control_;
    VectorXd warm_start_x_;
    bool has_warm_start_control_ = false;
    
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
    void cleanup_solver();
    void rebuild_solver();
};

} // namespace satellite_control

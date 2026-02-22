#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <vector>
#include <memory>
#include <tuple>
#include <string>
#include <limits>
#include <chrono>
#include "osqp.h"
#include "linearizer.hpp"

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
    double Q_lag_default = -1.0;        ///< Default lag weight if Q_lag <= 0 (negative = auto use Q_contour)
    double Q_velocity_align = 0.0;      ///< Velocity-alignment weight (0 = reuse Q_progress)
    double Q_s_anchor = -1.0;           ///< Progress-state anchor weight (negative = auto)
    double Q_smooth = 10.0;             ///< Weight for control increment smoothness (Δu)
    double Q_angvel = 1.0;              ///< Angular velocity error weight (retain for stabilization)
    double Q_attitude = 0.0;            ///< Attitude tracking weight (align body x-axis to path tangent)
    double Q_axis_align = 0.0;          ///< Extra axis-alignment weight (adds to Q_attitude)
    double Q_quat_norm = 0.0;           ///< Soft quaternion normalization weight

    double R_thrust = 0.1;          ///< Thruster usage weight
    double R_rw_torque = 0.1;       ///< Reaction wheel torque usage weight
    double thrust_l1_weight = 0.0;  ///< Linear thruster penalty (fuel bias)
    double thrust_pair_weight = 0.0; ///< Penalty on opposing-pair co-firing
    double max_linear_velocity = 0.0; ///< Linear velocity bound [m/s] (0 = auto)
    double max_angular_velocity = 0.0; ///< Angular velocity bound [rad/s] (0 = auto)
    bool enable_delta_u_coupling = false; ///< Enable full Δu temporal coupling in smoothness cost
    bool enable_gyro_jacobian = false; ///< Enable gyroscopic Jacobian updates in angular dynamics
    bool auto_enable_gyro_jacobian = true; ///< Auto-enable gyro Jacobian above angular-rate threshold
    double gyro_enable_threshold_radps = 0.1; ///< Angular-rate threshold for auto gyro Jacobian
    bool enable_auto_state_bounds = false; ///< Auto-derive velocity bounds when explicit bounds are unset
    bool enable_online_dare_terminal = true; ///< Recompute DARE terminal diagonal online from local linearization
    int dare_update_period_steps = 8; ///< Number of control steps between DARE updates
    std::string terminal_cost_profile = "diagonal"; ///< Terminal cost profile: diagonal|dense_terminal
    std::string robustness_mode = "none"; ///< Robustness mode: none|tube
    double constraint_tightening_scale = 0.0; ///< Constraint tightening fraction for robust scaffold [0..0.3]
    double tube_feedback_gain_scale = 0.15; ///< Ancillary tube-feedback gain scale [0..1]
    double tube_feedback_max_correction = 0.25; ///< Max absolute tube correction per control channel
    bool enable_variable_scaling = true; ///< Solve in scaled decision coordinates for better conditioning




    // Path following (general MPCC)
    double path_speed = 0.1;           ///< Path speed along reference [m/s]
    double path_speed_min = 0.01;      ///< Minimum path speed [m/s]
    double path_speed_max = 0.1;       ///< Maximum path speed [m/s]

    // Terminal handling (MPCC)
    // If set to 0, controller will auto-scale from Q_contour/Q_progress.
    double Q_terminal_pos = 0.0;       ///< Terminal position weight (0 = auto)
    double Q_terminal_s = 0.0;         ///< Terminal progress weight (0 = auto)
    double recover_contour_scale = 2.0; ///< RECOVER contour multiplier
    double recover_lag_scale = 2.0; ///< RECOVER lag multiplier
    double recover_progress_scale = 0.6; ///< RECOVER progress multiplier
    double recover_attitude_scale = 0.8; ///< RECOVER attitude multiplier
    double settle_progress_scale = 0.0; ///< SETTLE progress multiplier
    double settle_terminal_pos_scale = 2.0; ///< SETTLE/HOLD terminal position multiplier
    double settle_terminal_attitude_scale = 1.5; ///< SETTLE/HOLD terminal attitude multiplier
    double settle_velocity_align_scale = 1.5; ///< SETTLE/HOLD velocity alignment multiplier
    double settle_angular_velocity_scale = 2.0; ///< SETTLE/HOLD angular velocity multiplier
    double hold_smoothness_scale = 1.5; ///< HOLD smoothness multiplier
    double hold_thruster_pair_scale = 1.2; ///< HOLD opposing-thruster penalty multiplier
    double solver_fallback_hold_s = 0.30; ///< Hold last-feasible command for this duration [s]
    double solver_fallback_decay_s = 0.70; ///< Linearly decay fallback command after hold [s]
    double solver_fallback_zero_after_s = 1.00; ///< Hard-zero fallback command after this age [s]
};

/**
 * @brief Result structure returned by the controller.
 */
struct ControlResult {
    VectorXd u;         ///< Optimal control vector (RW + Thrusters)
    int status;         ///< 1 for success, -1 for non-success
    int solver_status;  ///< Raw OSQP status code
    int iterations;     ///< Solver iteration count
    double objective;   ///< Solver objective value
    double solve_time;  ///< Time taken to solve [s]
    bool timeout;       ///< Whether the solver timed out
    double path_s = 0.0;        ///< Path progress value used by MPC this step
    double path_s_proj = 0.0;   ///< Raw geometric projection on path
    double path_s_pred = 0.0;   ///< Predicted next progress (s + v_s*dt)
    double path_error = std::numeric_limits<double>::infinity(); ///< Distance to path
    double path_endpoint_error = std::numeric_limits<double>::infinity(); ///< Distance to path endpoint
    bool fallback_active = false; ///< Whether fallback command scaling is active this step
    double fallback_age_s = 0.0;  ///< Age of active fallback command [s]
    double fallback_scale = 0.0;  ///< Multiplicative scale applied to fallback command [0..1]
    // Per-step timing breakdown for profiling/contract monitoring.
    double t_linearization_s = 0.0;
    double t_cost_update_s = 0.0;
    double t_constraint_update_s = 0.0;
    double t_matrix_update_s = 0.0;
    double t_warmstart_s = 0.0;
    double t_solve_only_s = 0.0;
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


    // -- Accessors --
    int num_controls() const { return nu_; }
    int prediction_horizon() const { return N_; }
    double dt() const { return dt_; }
    double path_length() const { return path_total_length_; }
    bool has_path() const { return path_data_valid_; }
    double current_path_s() const { return s_runtime_; }

    // Path utilities (for fast projection in Python)
    std::tuple<double, Eigen::Vector3d, double, double> project_onto_path(
        const Eigen::Vector3d& position) const;

    // Reference utilities (single source of truth for path/scan attitude frame).
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector4d> get_reference_at_s(
        double s_query,
        const Eigen::Vector4d& q_current
    ) const;

    /**
     * @brief Provide a warm-start control guess.
     * @param u_prev Either thruster-only (num_thrusters) or full control (nu).
     */
    void set_warm_start_control(const VectorXd& u_prev);

    /**
     * @brief Configure scan-attitude context for stable object-facing attitude.
     */
    void set_scan_attitude_context(
        const Eigen::Vector3d& center,
        const Eigen::Vector3d& axis,
        const std::string& direction
    );

    /**
     * @brief Disable scan-attitude context and fall back to tangent/up behavior.
     */
    void clear_scan_attitude_context();

    /**
     * @brief Set runtime controller mode (TRACK/RECOVER/SETTLE/HOLD/COMPLETE).
     *
     * Unknown values default to TRACK.
     */
    void set_runtime_mode(const std::string& mode);

private:
    enum class RuntimeMode {
        TRACK,
        RECOVER,
        SETTLE,
        HOLD,
        COMPLETE
    };

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
    VectorXd state_var_scale_;
    VectorXd control_var_scale_;

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
     * @brief Compute exactly stabilizing DARE terminal costs for the nominal state.
     */
    VectorXd compute_dare_terminal_cost(
        const VectorXd& Q_diag,
        const VectorXd& R_diag,
        const VectorXd& x_nominal_phys
    );
    MatrixXd compute_dare_terminal_matrix(
        const VectorXd& Q_diag,
        const VectorXd& R_diag,
        const VectorXd& x_nominal_phys
    );

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

    /// Maps [step][row][col] -> index in A_data_ for B matrix entries (actuator dynamics).
    std::vector<std::vector<std::vector<int>>> B_idx_map_;

    /// Maps [step][row][col] -> index in A_data_ for quaternion dynamics entries.
    std::vector<std::vector<std::vector<int>>> A_idx_map_;

    /// Maps [step][row][col] -> index in A_data_ for orbital/velocity dynamics updates (rows 7-9, cols 0-9).
    std::vector<std::vector<std::vector<int>>> A_orbital_idx_map_;
    /// Maps [step][row][col] -> index in A_data_ for angular dynamics updates (rows 10-12, cols 10-12).
    std::vector<std::vector<std::vector<int>>> A_angvel_idx_map_;


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
    // Map [step][0..3] -> Index in P_data_ for quaternion diagonals (qw,qx,qy,qz)
    std::vector<std::vector<int>> path_att_diag_indices_;
    // Map [step][0..2] -> Index in P_data_ for angular velocity diagonals (wx,wy,wz)
    std::vector<std::vector<int>> path_angvel_diag_indices_;
    // Map [step] -> Index in P_data_ for progress control (v_s, v_s) diagonal entry
    std::vector<int> path_vs_diag_indices_;
    std::vector<c_int> path_P_update_indices_;
    std::vector<c_float> path_P_update_values_;
    std::vector<int> terminal_phys_diag_indices_;
    std::vector<c_int> terminal_phys_update_indices_;
    std::vector<c_float> terminal_phys_update_values_;

    // Constraint bookkeeping
    int n_dyn_ = 0;
    int n_init_ = 0;
    int n_bounds_x_ = 0;
    int n_bounds_u_ = 0;
    int n_control_horizon_constraints_ = 0;
    int control_horizon_ = 0;
    int base_control_horizon_ = 0;
    int ctrl_row_start_ = 0;

    // Dynamic affine term (for gravity, etc.)
    VectorXd dyn_affine_;

    // A-matrix update tracking
    bool A_dirty_ = false;

    // Warm-start storage
    VectorXd warm_start_control_;
    VectorXd warm_start_x_;
    bool has_warm_start_control_ = false;
    VectorXd last_feasible_control_;
    bool has_last_feasible_control_ = false;
    bool active_gyro_jacobian_ = false;
    int control_step_counter_ = 0;

    // Auto-derived state bounds (used when params are unset)
    double max_linear_velocity_bound_ = 0.0;
    double max_angular_velocity_bound_ = 0.0;

    // -- Runtime Methods --
    void update_dynamics(const std::vector<VectorXd>& x_traj);
    void update_cost();
    void update_constraints(const VectorXd& x_current);

    void update_path_cost(const VectorXd& x_current); // Path following linearization
    double compute_dynamic_vs_min(double s_curr) const;

    // Path following internal state
    std::vector<double> s_guess_; // Guess for path parameter s over horizon
    RuntimeMode runtime_mode_ = RuntimeMode::TRACK;
    double base_q_smooth_ = 0.0;
    double base_thrust_pair_weight_ = 0.0;
    double active_smoothness_scale_ = 1.0;
    double active_thruster_pair_scale_ = 1.0;
    bool fallback_active_ = false;
    std::chrono::steady_clock::time_point fallback_started_at_{};

    // General path data
    // Path is defined as a list of (s, x, y, z) samples
    // where s is the arc-length parameter
    std::vector<double> path_s_;              // Arc-length samples [0, total_length]
    std::vector<Eigen::Vector3d> path_points_; // Position samples
    double path_total_length_ = 0.0;          // Total path length
    bool path_data_valid_ = false;            // True if path data has been set
    bool scan_attitude_enabled_ = false;      // If true, keep object-facing side stable
    Eigen::Vector3d scan_center_ = Eigen::Vector3d::Zero();
    bool scan_center_valid_ = false;
    Eigen::Vector3d scan_axis_ = Eigen::Vector3d(0.0, 0.0, 1.0);
    bool scan_direction_cw_ = true;
    mutable bool ref_frame_initialized_ = false;
    mutable Eigen::Vector3d ref_prev_x_axis_ = Eigen::Vector3d::Zero();
    mutable Eigen::Vector3d ref_prev_y_axis_ = Eigen::Vector3d::Zero();
    mutable Eigen::Vector3d ref_prev_z_axis_ = Eigen::Vector3d::Zero();
    double s_runtime_ = 0.0;
    bool s_runtime_initialized_ = false;

    // Helper methods for path interpolation
    Eigen::Vector3d get_path_point(double s) const;
    Eigen::Vector3d get_path_tangent(double s) const;
    double clamp_path_s(double s) const;
    Eigen::Vector4d build_reference_quaternion(
        const Eigen::Vector3d& p_ref,
        const Eigen::Vector3d& t_ref,
        const Eigen::Vector4d& q_curr
    ) const;
    RuntimeMode parse_runtime_mode(const std::string& mode) const;
    void update_mode_dependent_regularizers();
    void initialize_variable_scaling();
    double decision_var_scale(int decision_index) const;

public:
    // Set path data for general path following
    void set_path_data(const std::vector<std::array<double, 4>>& path_data);

private:
    void cleanup_solver();
    void rebuild_solver();
};

} // namespace satellite_control

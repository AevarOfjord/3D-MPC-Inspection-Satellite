/**
 * @file sqp_types.hpp
 * @brief Common types and structures for the V2 SQP-MPC controller.
 *
 * Defines the parameter structs, result type, and runtime mode enum
 * shared across all V2 MPC components.
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

namespace satellite_control {
namespace v2 {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

// ---------------------------------------------------------------------------
// Runtime mode FSM
// ---------------------------------------------------------------------------

enum class RuntimeMode {
    TRACK,
    RECOVER,
    SETTLE,
    HOLD,
    COMPLETE,
};

inline RuntimeMode parse_runtime_mode(const std::string& mode) {
    if (mode == "RECOVER") return RuntimeMode::RECOVER;
    if (mode == "SETTLE")  return RuntimeMode::SETTLE;
    if (mode == "HOLD")    return RuntimeMode::HOLD;
    if (mode == "COMPLETE") return RuntimeMode::COMPLETE;
    return RuntimeMode::TRACK;
}

// ---------------------------------------------------------------------------
// MPC Parameters (V2)
// ---------------------------------------------------------------------------

struct MPCV2Params {
    // Horizon
    int prediction_horizon = 50;
    int control_horizon = 40;
    double dt = 0.05;
    double solver_time_limit = 0.035;
    bool verbose = false;

    // Cost weights (MPCC)
    double Q_contour = 2400.0;
    double Q_lag = 4000.0;
    double Q_progress = 70.0;
    double progress_reward = 0.0;
    double Q_velocity_align = 160.0;
    double Q_s_anchor = 500.0;
    double Q_attitude = 3500.0;
    double Q_axis_align = 3000.0;
    double Q_quat_norm = 20.0;
    double Q_angvel = 1200.0;
    double R_thrust = 0.02;
    double R_rw_torque = 0.003;
    double Q_smooth = 20.0;
    double thrust_pair_weight = 0.8;
    double thrust_l1_weight = 0.0;

    // Path following
    double path_speed = 0.1;
    double path_speed_min = 0.05;
    double path_speed_max = 0.2;

    // Terminal
    double Q_terminal_pos = 0.0;   // 0 = auto
    double Q_terminal_s = 0.0;     // 0 = auto
    double Q_terminal_att = 0.0;
    double Q_terminal_angvel = 0.0;
    double Q_terminal_vel = 0.0;

    // DARE
    bool enable_dare_terminal = true;
    int dare_update_period_steps = 8;
    std::string terminal_cost_profile = "diagonal";

    // Velocity bounds
    double max_linear_velocity = 0.0;   // 0 = auto
    double max_angular_velocity = 0.0;  // 0 = auto

    // Progress policy
    std::string progress_policy = "speed_tracking";
    double error_priority_min_vs = 0.01;
    double error_priority_error_speed_gain = 8.0;

    // SQP settings
    int sqp_max_iter = 1;       // 1 = RTI mode (single QP per step)
    double sqp_tol = 1e-4;

    // OSQP settings
    int osqp_max_iter = 800;
    double osqp_eps_abs = 1e-3;
    double osqp_eps_rel = 1e-3;
    bool osqp_warm_start = true;

    // Mode scaling
    double recover_contour_scale = 2.0;
    double recover_lag_scale = 2.0;
    double recover_progress_scale = 0.6;
    double recover_attitude_scale = 0.8;
    double settle_progress_scale = 0.0;
    double settle_terminal_pos_scale = 2.0;
    double settle_terminal_attitude_scale = 1.5;
    double settle_velocity_align_scale = 1.5;
    double settle_angular_velocity_scale = 2.0;
    double hold_smoothness_scale = 1.5;
    double hold_thruster_pair_scale = 1.2;

    // Fallback
    double solver_fallback_hold_s = 0.30;
    double solver_fallback_decay_s = 0.70;
    double solver_fallback_zero_after_s = 1.00;
};

// ---------------------------------------------------------------------------
// Control Result (V2 — identical interface to V1 for compatibility)
// ---------------------------------------------------------------------------

struct ControlResultV2 {
    VectorXd u;                     ///< Optimal control vector (RW + Thrusters + v_s)
    int status = -1;                ///< 1 = success, -1 = failure
    int solver_status = 0;          ///< Raw OSQP status code
    int iterations = 0;             ///< Solver iteration count
    double objective = 0.0;         ///< QP objective value
    double solve_time = 0.0;        ///< Total time taken [s]
    bool timeout = false;           ///< Whether solver timed out

    // Path tracking telemetry
    double path_s = 0.0;
    double path_s_proj = 0.0;
    double path_s_pred = 0.0;
    double path_error = std::numeric_limits<double>::infinity();
    double path_endpoint_error = std::numeric_limits<double>::infinity();

    // Fallback state
    bool fallback_active = false;
    double fallback_age_s = 0.0;
    double fallback_scale = 0.0;

    // Timing breakdown
    double t_linearization_s = 0.0;
    double t_cost_update_s = 0.0;
    double t_constraint_update_s = 0.0;
    double t_matrix_update_s = 0.0;
    double t_warmstart_s = 0.0;
    double t_solve_only_s = 0.0;

    // SQP-specific
    int sqp_iterations = 0;         ///< Number of SQP iterations performed
    double sqp_kkt_residual = 0.0;  ///< Final KKT residual
};

// ---------------------------------------------------------------------------
// Path data
// ---------------------------------------------------------------------------

struct PathData {
    std::vector<double> s;                ///< Arc-length samples
    std::vector<Vector3d> points;         ///< Position samples
    double total_length = 0.0;
    bool valid = false;

    /// Interpolate position at arc-length s_query.
    Vector3d get_point(double s_query) const;

    /// Interpolate unit tangent at arc-length s_query.
    Vector3d get_tangent(double s_query) const;

    /// Clamp s to [0, total_length].
    double clamp_s(double s_query) const;

    /// Project position onto path: returns (s, closest_point, distance, endpoint_error).
    std::tuple<double, Vector3d, double, double> project(const Vector3d& pos) const;
};

// ---------------------------------------------------------------------------
// Scan attitude context
// ---------------------------------------------------------------------------

struct ScanAttitudeContext {
    bool enabled = false;
    Vector3d center = Vector3d::Zero();
    bool center_valid = false;
    Vector3d axis = Vector3d(0.0, 0.0, 1.0);
    bool direction_cw = true;

    // Reference frame continuity state
    mutable bool ref_initialized = false;
    mutable Vector3d ref_prev_x = Vector3d::Zero();
    mutable Vector3d ref_prev_y = Vector3d::Zero();
    mutable Vector3d ref_prev_z = Vector3d::Zero();
};

}  // namespace v2
}  // namespace satellite_control

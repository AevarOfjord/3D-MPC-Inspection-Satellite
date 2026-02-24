/**
 * @file sqp_controller.cpp
 * @brief RTI-SQP MPC Controller implementation (V2).
 *
 * This is the core MPC engine. It uses CasADi-generated functions
 * (called from Python, with results passed to C++) for dynamics
 * linearisation, and OSQP for QP solving.
 *
 * Key design differences from V1:
 *   - No hand-derived Jacobians — CasADi AD provides exact derivatives
 *   - Clean separation: DynamicsEvaluator / CostBuilder / ConstraintBuilder
 *   - Mode changes adjust cost weights via diagonal scaling (no QP realloc)
 *   - Control horizon tying via constraints (fixed sparsity pattern)
 *   - RTI-SQP: single linearisation + single QP per step
 */

#include "sqp_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

namespace satellite_control {
namespace v2 {

// ============================================================================
// Construction / Destruction
// ============================================================================

SQPController::SQPController(
    const SatelliteParams& sat_params,
    const MPCV2Params& mpc_params
)
    : sat_params_(sat_params)
    , params_(mpc_params)
    , num_thrusters_(sat_params.num_thrusters)
    , num_rw_(sat_params.num_rw)
    , nu_(sat_params.num_rw + sat_params.num_thrusters + 1)
    , N_(mpc_params.prediction_horizon)
    , M_(std::min(mpc_params.control_horizon, mpc_params.prediction_horizon))
    , dt_(mpc_params.dt)
{
    // Allocate trajectory storage
    x_traj_.resize(N_ + 1, VectorXd::Zero(nx_));
    u_traj_.resize(N_, VectorXd::Zero(nu_));

    // Allocate linearisation storage
    A_stages_.resize(N_, MatrixXd::Zero(nx_, nx_));
    B_stages_.resize(N_, MatrixXd::Zero(nx_, nu_));
    d_stages_.resize(N_, VectorXd::Zero(nx_));

    // DARE terminal
    dare_diag_ = VectorXd::Zero(16);

    // Pack satellite parameters for CasADi
    pack_casadi_params();

    // Apply initial mode scaling
    apply_mode_scaling();

    // Build QP structure
    init_qp();
}

SQPController::~SQPController() {
    cleanup_osqp();
}

// ============================================================================
// Parameter packing (mirrors Python pack_params)
// ============================================================================

void SQPController::pack_casadi_params() {
    int np = 1 + 3 + num_thrusters_ * 7 + num_rw_ * 5 + 2;
    casadi_params_.resize(np);

    int idx = 0;
    casadi_params_[idx++] = sat_params_.mass;
    casadi_params_.segment(idx, 3) = sat_params_.inertia;
    idx += 3;

    for (int i = 0; i < num_thrusters_; ++i) {
        casadi_params_.segment(idx, 3) = sat_params_.thruster_positions[i];
        idx += 3;
        casadi_params_.segment(idx, 3) = sat_params_.thruster_directions[i];
        idx += 3;
        casadi_params_[idx++] = sat_params_.thruster_forces[i];
    }

    for (int i = 0; i < num_rw_; ++i) {
        casadi_params_.segment(idx, 3) = sat_params_.rw_axes[i];
        idx += 3;
        casadi_params_[idx++] = sat_params_.rw_torque_limits[i];
        casadi_params_[idx++] = sat_params_.rw_inertia[i];
    }

    casadi_params_[idx++] = sat_params_.orbital_mu;
    casadi_params_[idx++] = sat_params_.orbital_radius;
}

// ============================================================================
// QP Initialisation
// ============================================================================

void SQPController::init_qp() {
    // Decision variable layout: z = [x_0, ..., x_N, u_0, ..., u_{N-1}]
    n_vars_ = (N_ + 1) * nx_ + N_ * nu_;

    // Constraint layout:
    //   1. Dynamics equalities:  N * nx_
    //   2. Initial state:        nx_
    //   3. State bounds:         (N+1) * nx_
    //   4. Control bounds:       N * nu_
    //   5. Control horizon tie:  (N - M_) * nu_
    n_dyn_ = N_ * nx_;
    n_init_ = nx_;
    n_bounds_x_ = (N_ + 1) * nx_;
    n_bounds_u_ = N_ * nu_;
    n_ctrl_tie_ = std::max(0, N_ - M_) * nu_;
    n_constraints_ = n_dyn_ + n_init_ + n_bounds_x_ + n_bounds_u_ + n_ctrl_tie_;

    // Build sparsity patterns
    build_P_structure();
    build_A_structure();

    // Configure and create OSQP workspace
    setup_osqp();
}

void SQPController::build_P_structure() {
    // P is (n_vars × n_vars) upper-triangular sparse.
    // For now, build as diagonal (stage costs are separable in the QP).
    // Off-diagonals for cross-terms (smoothness, path linearisation) added later.
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(n_vars_ * 2);

    // State diagonals
    for (int k = 0; k <= N_; ++k) {
        int x_offset = k * nx_;
        for (int i = 0; i < nx_; ++i) {
            triplets.emplace_back(x_offset + i, x_offset + i, 1e-6);
        }
    }

    // Control diagonals
    int u_base = (N_ + 1) * nx_;
    for (int k = 0; k < N_; ++k) {
        int u_offset = u_base + k * nu_;
        for (int i = 0; i < nu_; ++i) {
            triplets.emplace_back(u_offset + i, u_offset + i, 1e-6);
        }
    }

    // Smoothness cross-terms: u_k and u_{k-1} coupling
    for (int k = 1; k < N_; ++k) {
        int u_prev_offset = u_base + (k - 1) * nu_;
        int u_curr_offset = u_base + k * nu_;
        for (int i = 0; i < nu_; ++i) {
            // Upper triangle: (prev, curr) where prev < curr
            int row = u_prev_offset + i;
            int col = u_curr_offset + i;
            if (row < col) {
                triplets.emplace_back(row, col, 0.0);
            }
        }
    }

    // Thrust pair cross-terms: opposing thruster pairs (upper triangle)
    // Pre-allocate so update_cost doesn't insert into compressed matrix.
    if (num_thrusters_ >= 6) {
        for (int k = 0; k < N_; ++k) {
            int thr_base = u_base + k * nu_ + num_rw_;
            for (auto& [a, b] : std::vector<std::pair<int,int>>{{0,1},{2,3},{4,5}}) {
                if (a < num_thrusters_ && b < num_thrusters_) {
                    int r = thr_base + std::min(a, b);
                    int c = thr_base + std::max(a, b);
                    if (r < c) {
                        triplets.emplace_back(r, c, 0.0);
                    }
                }
            }
        }
    }

    P_qp_.resize(n_vars_, n_vars_);
    P_qp_.setFromTriplets(triplets.begin(), triplets.end());
    P_qp_.makeCompressed();

    // Extract CSC data
    P_data_.resize(P_qp_.nonZeros());
    P_indices_.resize(P_qp_.nonZeros());
    P_indptr_.resize(n_vars_ + 1);
    std::copy(P_qp_.valuePtr(), P_qp_.valuePtr() + P_qp_.nonZeros(), P_data_.data());
    std::copy(P_qp_.innerIndexPtr(), P_qp_.innerIndexPtr() + P_qp_.nonZeros(), P_indices_.data());
    std::copy(P_qp_.outerIndexPtr(), P_qp_.outerIndexPtr() + n_vars_ + 1, P_indptr_.data());

    q_qp_ = VectorXd::Zero(n_vars_);
}

void SQPController::build_A_structure() {
    std::vector<Eigen::Triplet<double>> triplets;
    int est_nnz = n_dyn_ * (nx_ + nu_) + n_init_ * 1 + n_bounds_x_ + n_bounds_u_ + n_ctrl_tie_;
    triplets.reserve(est_nnz);

    int row = 0;

    // 1. Dynamics equalities: x_{k+1} = A_k x_k + B_k u_k + d_k
    //    Row: x_{k+1} - A_k x_k - B_k u_k = d_k
    //    In terms of decision vars: I * x_{k+1} + (-A_k) * x_k + (-B_k) * u_k
    int u_base = (N_ + 1) * nx_;
    for (int k = 0; k < N_; ++k) {
        int x_k_offset = k * nx_;
        int x_k1_offset = (k + 1) * nx_;
        int u_k_offset = u_base + k * nu_;

        for (int i = 0; i < nx_; ++i) {
            // x_{k+1} coefficient: +I
            triplets.emplace_back(row + i, x_k1_offset + i, 1.0);

            // x_k coefficients: -A_k (all columns)
            for (int j = 0; j < nx_; ++j) {
                triplets.emplace_back(row + i, x_k_offset + j, 0.0);
            }

            // u_k coefficients: -B_k (all columns)
            for (int j = 0; j < nu_; ++j) {
                triplets.emplace_back(row + i, u_k_offset + j, 0.0);
            }
        }
        row += nx_;
    }

    // 2. Initial state: x_0 = x_measured
    for (int i = 0; i < nx_; ++i) {
        triplets.emplace_back(row + i, i, 1.0);
    }
    row += nx_;

    // 3. State bounds: I * x_k for k=0..N
    for (int k = 0; k <= N_; ++k) {
        int x_offset = k * nx_;
        for (int i = 0; i < nx_; ++i) {
            triplets.emplace_back(row + i, x_offset + i, 1.0);
        }
        row += nx_;
    }

    // 4. Control bounds: I * u_k for k=0..N-1
    for (int k = 0; k < N_; ++k) {
        int u_offset = u_base + k * nu_;
        for (int i = 0; i < nu_; ++i) {
            triplets.emplace_back(row + i, u_offset + i, 1.0);
        }
        row += nu_;
    }

    // 5. Control horizon tying: u_k - u_{M-1} = 0  for k >= M
    for (int k = M_; k < N_; ++k) {
        int u_k_offset = u_base + k * nu_;
        int u_m_offset = u_base + (M_ - 1) * nu_;
        for (int i = 0; i < nu_; ++i) {
            triplets.emplace_back(row + i, u_k_offset + i, 1.0);
            triplets.emplace_back(row + i, u_m_offset + i, -1.0);
        }
        row += nu_;
    }

    A_qp_.resize(n_constraints_, n_vars_);
    A_qp_.setFromTriplets(triplets.begin(), triplets.end());
    A_qp_.makeCompressed();

    // Extract CSC data
    A_data_.resize(A_qp_.nonZeros());
    A_indices_.resize(A_qp_.nonZeros());
    A_indptr_.resize(n_vars_ + 1);
    std::copy(A_qp_.valuePtr(), A_qp_.valuePtr() + A_qp_.nonZeros(), A_data_.data());
    std::copy(A_qp_.innerIndexPtr(), A_qp_.innerIndexPtr() + A_qp_.nonZeros(), A_indices_.data());
    std::copy(A_qp_.outerIndexPtr(), A_qp_.outerIndexPtr() + n_vars_ + 1, A_indptr_.data());

    // Initialize bounds
    l_qp_ = VectorXd::Constant(n_constraints_, -OSQP_INFTY);
    u_qp_ = VectorXd::Constant(n_constraints_, OSQP_INFTY);
}

void SQPController::setup_osqp() {
    cleanup_osqp();

    // Settings
    osqp_settings_ = new OSQPSettings;
    osqp_set_default_settings(osqp_settings_);

    osqp_settings_->max_iter = params_.osqp_max_iter;
    osqp_settings_->eps_abs = static_cast<c_float>(params_.osqp_eps_abs);
    osqp_settings_->eps_rel = static_cast<c_float>(params_.osqp_eps_rel);
    osqp_settings_->warm_start = params_.osqp_warm_start ? 1 : 0;
    osqp_settings_->adaptive_rho = 1;
    osqp_settings_->polish = 1;
    osqp_settings_->verbose = params_.verbose ? 1 : 0;
    osqp_settings_->scaled_termination = 1;
    osqp_settings_->sigma = 1e-4;  // regularisation for PSD enforcement (increased for conditioning)

    double time_limit = std::min(
        params_.solver_time_limit,
        0.85 * dt_
    );
    osqp_settings_->time_limit = static_cast<c_float>(time_limit);

    // Data
    osqp_data_ = new OSQPData;
    osqp_data_->n = n_vars_;
    osqp_data_->m = n_constraints_;

    // P (upper triangular)
    osqp_data_->P = csc_matrix(
        n_vars_, n_vars_,
        static_cast<c_int>(P_data_.size()),
        P_data_.data(),
        P_indices_.data(),
        P_indptr_.data()
    );

    // q
    osqp_data_->q = q_qp_.data();

    // A
    osqp_data_->A = csc_matrix(
        n_constraints_, n_vars_,
        static_cast<c_int>(A_data_.size()),
        A_data_.data(),
        A_indices_.data(),
        A_indptr_.data()
    );

    // l, u
    osqp_data_->l = l_qp_.data();
    osqp_data_->u = u_qp_.data();

    // Setup workspace
    c_int exitflag = osqp_setup(&osqp_work_, osqp_data_, osqp_settings_);
    if (exitflag != 0) {
        std::cerr << "[SQPController] OSQP setup failed with code " << exitflag << std::endl;
        osqp_work_ = nullptr;
    }
}

void SQPController::cleanup_osqp() {
    if (osqp_work_) {
        osqp_cleanup(osqp_work_);
        osqp_work_ = nullptr;
    }
    if (osqp_data_) {
        // Don't free P and A — they point to our vectors
        if (osqp_data_->P) {
            c_free(osqp_data_->P);
            osqp_data_->P = nullptr;
        }
        if (osqp_data_->A) {
            c_free(osqp_data_->A);
            osqp_data_->A = nullptr;
        }
        delete osqp_data_;
        osqp_data_ = nullptr;
    }
    if (osqp_settings_) {
        delete osqp_settings_;
        osqp_settings_ = nullptr;
    }
}

// ============================================================================
// Mode scaling
// ============================================================================

void SQPController::apply_mode_scaling() {
    // Base weights
    active_Q_contour_ = params_.Q_contour;
    active_Q_lag_ = params_.Q_lag;
    active_Q_progress_ = params_.Q_progress;
    active_Q_attitude_ = params_.Q_attitude + params_.Q_axis_align;
    active_Q_smooth_ = params_.Q_smooth;
    active_Q_velocity_ = params_.Q_velocity_align;
    active_Q_angvel_ = params_.Q_angvel;
    active_thrust_pair_ = params_.thrust_pair_weight;
    active_path_speed_ = params_.path_speed;

    switch (mode_) {
        case RuntimeMode::RECOVER:
            active_Q_contour_ *= params_.recover_contour_scale;
            active_Q_lag_ *= params_.recover_lag_scale;
            active_Q_progress_ *= params_.recover_progress_scale;
            active_Q_attitude_ *= params_.recover_attitude_scale;
            break;
        case RuntimeMode::SETTLE:
        case RuntimeMode::COMPLETE:
            active_Q_contour_ *= params_.settle_terminal_pos_scale;
            active_Q_lag_ *= params_.settle_terminal_pos_scale;
            active_Q_progress_ *= params_.settle_progress_scale;
            active_Q_attitude_ *= params_.settle_terminal_attitude_scale;
            active_Q_velocity_ *= params_.settle_velocity_align_scale;
            active_Q_angvel_ *= params_.settle_angular_velocity_scale;
            active_path_speed_ = 0.0;  // no further path progress once settled
            break;
        case RuntimeMode::HOLD:
            active_Q_smooth_ *= params_.hold_smoothness_scale;
            active_thrust_pair_ *= params_.hold_thruster_pair_scale;
            active_path_speed_ = 0.0;  // hold position, no path progress
            break;
        case RuntimeMode::TRACK:
        default:
            break;
    }
}

// ============================================================================
// Main control loop
// ============================================================================

ControlResultV2 SQPController::get_control_action(const VectorXd& x_current) {
    auto t_start = std::chrono::steady_clock::now();
    ControlResultV2 result;
    result.u = VectorXd::Zero(nu_);

    if (!osqp_work_) {
        result.status = -1;
        result.solver_status = -99;
        return result;
    }

    // Update path progress tracking
    if (path_.valid && !s_initialized_) {
        auto [s_proj, _, dist, ep_err] = path_.project(x_current.head<3>());
        s_runtime_ = s_proj;
        s_initialized_ = true;
    }

    // 1. Shift trajectory (warm-start from previous solution)
    auto t0 = std::chrono::steady_clock::now();

    // Set initial state BEFORE shift so the first-call
    // initialisation propagates from x_current, not zeros.
    x_traj_[0] = x_current;
    shift_trajectory();

    // Ensure x_traj_[0] is the measured state after shift
    x_traj_[0] = x_current;

    // 2. Linearise dynamics at each stage
    auto t1 = std::chrono::steady_clock::now();
    linearise_dynamics();
    auto t2 = std::chrono::steady_clock::now();

    // 3. Update QP cost
    update_cost(x_current);
    auto t3 = std::chrono::steady_clock::now();

    // 4. Update constraints
    update_constraints(x_current);
    auto t4 = std::chrono::steady_clock::now();

    // 5. Update OSQP data in-place
    osqp_update_P(osqp_work_, P_data_.data(),
                  OSQP_NULL, static_cast<c_int>(P_data_.size()));
    osqp_update_A(osqp_work_, A_data_.data(),
                  OSQP_NULL, static_cast<c_int>(A_data_.size()));
    osqp_update_lin_cost(osqp_work_, q_qp_.data());
    osqp_update_bounds(osqp_work_, l_qp_.data(), u_qp_.data());

    // Warm-start primal
    if (has_warm_start_) {
        // Build full warm-start vector from trajectory
        VectorXd z_warm(n_vars_);
        for (int k = 0; k <= N_; ++k) {
            z_warm.segment(k * nx_, nx_) = x_traj_[k];
        }
        int u_base = (N_ + 1) * nx_;
        for (int k = 0; k < N_; ++k) {
            z_warm.segment(u_base + k * nu_, nu_) = u_traj_[k];
        }
        osqp_warm_start_x(osqp_work_, z_warm.data());
    }
    auto t5 = std::chrono::steady_clock::now();

    // 6. Solve
    c_int solve_flag = osqp_solve(osqp_work_);
    auto t6 = std::chrono::steady_clock::now();

    // 7. Extract result
    bool solve_success = (osqp_work_->info->status_val == 1 ||  // SOLVED
                          osqp_work_->info->status_val == 2);   // SOLVED_INACCURATE

    if (solve_success) {
        extract_solution(osqp_work_->solution->x);

        // Update result
        result.u = u_traj_[0];
        result.status = 1;
        result.solver_status = static_cast<int>(osqp_work_->info->status_val);
        result.iterations = static_cast<int>(osqp_work_->info->iter);
        result.objective = static_cast<double>(osqp_work_->info->obj_val);
        result.sqp_iterations = 1;

        // Update path tracking
        if (path_.valid) {
            double v_s = u_traj_[0][nu_ - 1];  // virtual speed
            // Advance s by the optimal virtual speed × dt.
            // x_traj_[0][16] is pinned to the input s by the initial-state
            // equality; the ACTUAL progression comes from v_s.
            double s_prev = x_traj_[0][16];
            s_runtime_ = s_prev + v_s * dt_;
            // Clamp to [0, path_length]
            if (path_.total_length > 0) {
                s_runtime_ = std::max(0.0, std::min(s_runtime_, path_.total_length));
            }
            result.path_s = s_runtime_;
            result.path_s_pred = s_runtime_ + v_s * dt_;

            auto [s_p, _, pe, epe] = path_.project(x_current.head<3>());
            result.path_s_proj = s_p;
            result.path_error = pe;
            result.path_endpoint_error = epe;
        }

        // Store for fallback
        last_feasible_control_ = u_traj_[0];
        has_last_feasible_ = true;
        fallback_active_ = false;
        has_warm_start_ = true;
    } else {
        // Solver failed — apply fallback
        result.solver_status = static_cast<int>(osqp_work_->info->status_val);
        result.iterations = static_cast<int>(osqp_work_->info->iter);

        // If OSQP reported non-convex (-6) or primal infeasible (-3),
        // rebuild the workspace so the next call starts fresh.
        int sv = osqp_work_->info->status_val;
        if (sv == -6 || sv == -3 || sv == -4) {
            setup_osqp();
            has_warm_start_ = false;
        }

        if (has_last_feasible_) {
            auto fallback_result = apply_fallback(dt_);
            result.u = fallback_result.u;
            result.fallback_active = true;
            result.fallback_age_s = fallback_result.fallback_age_s;
            result.fallback_scale = fallback_result.fallback_scale;
        }
        result.status = -1;
    }

    result.timeout = (osqp_work_->info->status_val == -2);  // MAX_ITER

    auto t_end = std::chrono::steady_clock::now();
    auto to_sec = [](auto a, auto b) {
        return std::chrono::duration<double>(b - a).count();
    };

    result.solve_time = to_sec(t_start, t_end);
    result.t_warmstart_s = to_sec(t0, t1);
    result.t_linearization_s = to_sec(t1, t2);
    result.t_cost_update_s = to_sec(t2, t3);
    result.t_constraint_update_s = to_sec(t3, t4);
    result.t_matrix_update_s = to_sec(t4, t5);
    result.t_solve_only_s = to_sec(t5, t6);

    control_step_counter_++;
    return result;
}

// ============================================================================
// Trajectory management
// ============================================================================

void SQPController::shift_trajectory() {
    if (!has_warm_start_) {
        // First call — initialise trajectory from current state.
        // Use warm_start_control_ (previous control from Python) if available,
        // otherwise fall back to zero.  v_s is always reset to nominal path
        // speed so the virtual progress integrator starts sensibly.
        for (int k = 0; k < N_; ++k) {
            x_traj_[k + 1] = x_traj_[k];
            if (warm_start_control_.size() == nu_) {
                u_traj_[k] = warm_start_control_;
            } else {
                u_traj_[k] = VectorXd::Zero(nu_);
            }
            // Always reset v_s to nominal path speed regardless of warm-start.
            u_traj_[k][nu_ - 1] = params_.path_speed;
        }
        return;
    }

    // Shift: x_traj_[k] ← x_traj_[k+1], u_traj_[k] ← u_traj_[k+1]
    for (int k = 0; k < N_ - 1; ++k) {
        x_traj_[k + 1] = x_traj_[k + 2];
        u_traj_[k] = u_traj_[k + 1];
    }
    // Terminal: extrapolate last stage
    x_traj_[N_] = x_traj_[N_ - 1];
    u_traj_[N_ - 1] = u_traj_[N_ - 2];
}

void SQPController::extract_solution(const c_float* primal) {
    // Extract state trajectory
    for (int k = 0; k <= N_; ++k) {
        for (int i = 0; i < nx_; ++i) {
            x_traj_[k][i] = static_cast<double>(primal[k * nx_ + i]);
        }
        // Re-normalise quaternion
        Eigen::Vector4d q = x_traj_[k].segment<4>(3);
        double qn = q.norm();
        if (qn > 1e-10) {
            x_traj_[k].segment<4>(3) = q / qn;
        }
    }

    // Extract control trajectory and clip to bounds
    // (OSQP may return slightly or significantly out-of-bounds values
    //  when status=2 / solved_inaccurate, especially with ill-conditioned problems)
    int u_base = (N_ + 1) * nx_;
    for (int k = 0; k < N_; ++k) {
        for (int i = 0; i < nu_; ++i) {
            double val = static_cast<double>(primal[u_base + k * nu_ + i]);
            if (i < num_rw_) {
                // RW torques: clamp to [-1, 1]
                val = std::max(-1.0, std::min(1.0, val));
            } else if (i < num_rw_ + num_thrusters_) {
                // Thrusters: clamp to [0, 1]
                val = std::max(0.0, std::min(1.0, val));
            }
            // v_s left unclamped here — bounds handled by OSQP
            u_traj_[k][i] = val;
        }
    }
}

// ============================================================================
// Linearisation (placeholder — actual CasADi calls come from Python bridge)
// ============================================================================

void SQPController::linearise_dynamics() {
    // In the V2 architecture, linearisation is performed by calling
    // CasADi-generated functions from Python, which fills A_stages_,
    // B_stages_, and d_stages_ via the pybind11 interface.
    //
    // For the C++-only path (unit testing / embedded deployment),
    // a DynamicsEvaluator class wraps the CasADi-generated C functions.
    //
    // This method updates the A_data_ (constraint matrix CSC values)
    // using the per-stage A, B matrices.

    int row_start = 0;  // dynamics rows start at row 0
    int u_base = (N_ + 1) * nx_;

    // For each dynamics stage, update the A constraint values
    // The sparsity pattern was set in build_A_structure with placeholders.
    // We need to map (row, col) -> index in A_data_ for efficient updates.
    //
    // Because the sparse matrix was built with specific triplets, we can
    // iterate over the coefficients using the Eigen sparse matrix API.
    // For production performance, we'd maintain index maps (like V1).
    // For now, rebuild the dynamics block directly.

    for (int k = 0; k < N_; ++k) {
        const MatrixXd& Ak = A_stages_[k];
        const MatrixXd& Bk = B_stages_[k];
        int base_row = k * nx_;

        // Update dynamics coefficients in A_qp_
        int x_k_col = k * nx_;
        int x_k1_col = (k + 1) * nx_;
        int u_k_col = u_base + k * nu_;

        for (int i = 0; i < nx_; ++i) {
            // -A_k entries
            for (int j = 0; j < nx_; ++j) {
                A_qp_.coeffRef(base_row + i, x_k_col + j) = -Ak(i, j);
            }
            // +I entries (already set to 1.0, no update needed)

            // -B_k entries
            for (int j = 0; j < nu_; ++j) {
                A_qp_.coeffRef(base_row + i, u_k_col + j) = -Bk(i, j);
            }
        }
    }

    // Update CSC data from the modified sparse matrix
    std::copy(A_qp_.valuePtr(), A_qp_.valuePtr() + A_qp_.nonZeros(), A_data_.data());
}

// ============================================================================
// Cost update
// ============================================================================

void SQPController::update_cost(const VectorXd& x_current) {
    // Zero out
    q_qp_.setZero();

    // Reset ALL P_qp_ values to zero (not just P_data_) so that
    // off-diagonal entries (thrust pairs, smoothness) don't accumulate.
    for (int i = 0; i < P_qp_.nonZeros(); ++i) {
        P_qp_.valuePtr()[i] = 0.0;
    }

    int u_base = (N_ + 1) * nx_;

    // Build P and q from active weights
    for (int k = 0; k <= N_; ++k) {
        int x_offset = k * nx_;
        bool is_terminal = (k == N_);
        double scale = is_terminal ? 10.0 : 1.0;

        // Position: contouring + lag
        // OSQP cost is ½ z^T P z + q^T z.  For tracking (x-ref)^T Q (x-ref)
        // we need P = 2Q and q = -2Q*ref so min is at x = ref.
        for (int i = 0; i < 3; ++i) {
            double w_pos = scale * (active_Q_contour_ + active_Q_lag_);
            P_qp_.coeffRef(x_offset + i, x_offset + i) = 2.0 * w_pos + 1e-6;
        }

        // Quaternion: attitude tracking (same 2Q pattern)
        for (int i = 3; i < 7; ++i) {
            double w_att = scale * active_Q_attitude_ + params_.Q_quat_norm;
            P_qp_.coeffRef(x_offset + i, x_offset + i) = 2.0 * w_att + 1e-6;
        }

        // Velocity: damping + alignment (P = 2Q for OSQP ½z^TPz)
        for (int i = 7; i < 10; ++i) {
            double w_vel = scale * active_Q_velocity_;
            if (is_terminal) w_vel += params_.Q_terminal_vel;
            P_qp_.coeffRef(x_offset + i, x_offset + i) = 2.0 * w_vel + 1e-6;
        }

        // Angular velocity: damping (P = 2Q for OSQP ½z^TPz)
        for (int i = 10; i < 13; ++i) {
            P_qp_.coeffRef(x_offset + i, x_offset + i) =
                2.0 * scale * active_Q_angvel_ + 1e-6;
        }

        // RW wheel speeds: small regularisation
        for (int i = 13; i < 16; ++i) {
            P_qp_.coeffRef(x_offset + i, x_offset + i) = 1e-6;
        }

        // Path progress s: anchor
        double w_s = scale * params_.Q_s_anchor;
        if (is_terminal && params_.Q_terminal_s > 0) {
            w_s += params_.Q_terminal_s;
        }
        P_qp_.coeffRef(x_offset + 16, x_offset + 16) = 2.0 * w_s + 1e-6;

        // Linear cost (path reference tracking)
        if (path_.valid) {
            double s_k = (k < static_cast<int>(x_traj_.size()))
                         ? x_traj_[k][16]
                         : s_runtime_ + k * dt_ * params_.path_speed;
            s_k = path_.clamp_s(s_k);

            Vector3d p_ref = path_.get_point(s_k);
            Vector3d t_ref = path_.get_tangent(s_k);

            // Linear position term: q_pos = -2 * Q * p_ref
            for (int i = 0; i < 3; ++i) {
                double w_pos = scale * (active_Q_contour_ + active_Q_lag_);
                q_qp_[x_offset + i] = -2.0 * w_pos * p_ref[i];
            }

            // Linear s term: q_s = -2 * Q_s_anchor * s_ref
            q_qp_[x_offset + 16] = -2.0 * w_s * s_k;

            // Quaternion reference (linear term for attitude)
            if (x_traj_[0].size() >= 7) {
                Vector4d q_curr = x_traj_[0].segment<4>(3);
                Vector4d q_ref = build_reference_quaternion(p_ref, t_ref, q_curr);
                for (int i = 0; i < 4; ++i) {
                    q_qp_[x_offset + 3 + i] =
                        -2.0 * (scale * active_Q_attitude_ + params_.Q_quat_norm)
                        * q_ref[i];
                }
            }

            // Velocity alignment: bias velocity toward path tangent direction.
            // v_ref = active_path_speed_ * t_ref; linear term = -2*Q_vel*v_ref.
            // Zero when active_path_speed_==0 (SETTLE/HOLD), so no spurious
            // incentive to depart from rest in terminal modes.
            if (active_Q_velocity_ > 0.0 && active_path_speed_ > 0.0) {
                double w_vel = scale * active_Q_velocity_;
                for (int i = 0; i < 3; ++i) {
                    q_qp_[x_offset + 7 + i] =
                        -2.0 * w_vel * active_path_speed_ * t_ref[i];
                }
            }
        }
    }

    // Control costs
    for (int k = 0; k < N_; ++k) {
        int u_offset = u_base + k * nu_;

        // RW torque (P = 2R for OSQP ½z^TPz)
        for (int i = 0; i < num_rw_; ++i) {
            P_qp_.coeffRef(u_offset + i, u_offset + i) =
                2.0 * (params_.R_rw_torque + active_Q_smooth_) + 1e-6;
        }

        // Thrusters (P = 2R for OSQP ½z^TPz)
        for (int i = num_rw_; i < num_rw_ + num_thrusters_; ++i) {
            P_qp_.coeffRef(u_offset + i, u_offset + i) =
                2.0 * (params_.R_thrust + active_Q_smooth_) + 1e-6;

            // L1 fuel bias (linear term since u_thr >= 0)
            if (params_.thrust_l1_weight > 0) {
                q_qp_[u_offset + i] += params_.thrust_l1_weight;
            }
        }

        // Virtual speed v_s (P = 2Q for OSQP ½z^TPz)
        int vs_idx = u_offset + nu_ - 1;
        P_qp_.coeffRef(vs_idx, vs_idx) =
            2.0 * (active_Q_progress_ + active_Q_smooth_) + 1e-6;
        // Linear: -2*Q_progress*v_ref  (quadratic tracking)
        //         - progress_reward    (pure linear incentive, no 2× factor)
        q_qp_[vs_idx] = -2.0 * active_Q_progress_ * active_path_speed_
                         - params_.progress_reward;

        // Opposing thruster pairs (P = 2w for OSQP ½z^TPz)
        // For pair (i, j): P[i,i] += 2w, P[j,j] += 2w, P[i,j] += 2w (upper tri)
        if (active_thrust_pair_ > 0 && num_thrusters_ >= 6) {
            double w2 = 2.0 * active_thrust_pair_;
            int thr_base = u_offset + num_rw_;
            for (auto& [a, b] : std::vector<std::pair<int,int>>{{0,1},{2,3},{4,5}}) {
                if (a < num_thrusters_ && b < num_thrusters_) {
                    P_qp_.coeffRef(thr_base + a, thr_base + a) += w2;
                    P_qp_.coeffRef(thr_base + b, thr_base + b) += w2;
                    // Cross-term (upper triangle)
                    int r = thr_base + std::min(a, b);
                    int c = thr_base + std::max(a, b);
                    if (r != c) {
                        P_qp_.coeffRef(r, c) += w2;
                    }
                }
            }
        }
    }

    // DARE terminal cost (diagonal)
    if (params_.enable_dare_terminal && dare_diag_.size() == 16) {
        if (control_step_counter_ % params_.dare_update_period_steps == 0) {
            dare_diag_ = compute_dare_terminal_diag(x_traj_.back().head(16));
        }
        int x_N_offset = N_ * nx_;
        for (int i = 0; i < 16; ++i) {
            P_qp_.coeffRef(x_N_offset + i, x_N_offset + i) += dare_diag_[i];
        }
    }

    // Update CSC data
    std::copy(P_qp_.valuePtr(), P_qp_.valuePtr() + P_qp_.nonZeros(), P_data_.data());
}

// ============================================================================
// Constraint update
// ============================================================================

void SQPController::update_constraints(const VectorXd& x_current) {
    int row = 0;

    // 1. Dynamics equalities: A x_{k+1} - A_k x_k - B_k u_k = d_k
    for (int k = 0; k < N_; ++k) {
        for (int i = 0; i < nx_; ++i) {
            l_qp_[row + i] = d_stages_[k][i];
            u_qp_[row + i] = d_stages_[k][i];
        }
        row += nx_;
    }

    // 2. Initial state equality
    for (int i = 0; i < nx_; ++i) {
        l_qp_[row + i] = x_current[i];
        u_qp_[row + i] = x_current[i];
    }
    row += nx_;

    // 3. State bounds
    //
    // CRITICAL: the initial-state equality (section 2) already pins x_0.
    // Adding box bounds at k=0 that are tighter than the actual state
    // makes the QP infeasible (e.g. if ω_actual > max_w).
    // → At k=0 we use OSQP_INFTY bounds (no-op, equality handles it).
    // → At k=1..N we enlarge bounds to at least encompass the current
    //   state values so the solver can feasibly drive them down.
    double nominal_max_v = params_.max_linear_velocity > 0
                           ? params_.max_linear_velocity : 2.0;
    double nominal_max_w = params_.max_angular_velocity > 0
                           ? params_.max_angular_velocity : 2.0;

    // Widen bounds to cover initial state + margin
    double actual_v = x_current.segment<3>(7).norm();
    double actual_w = x_current.segment<3>(10).norm();
    double max_v = std::max(nominal_max_v, actual_v * 1.2 + 0.5);
    double max_w = std::max(nominal_max_w, actual_w * 1.2 + 1.0);
    double inf = OSQP_INFTY;

    for (int k = 0; k <= N_; ++k) {
        if (k == 0) {
            // k=0: fully open — the initial equality constraint handles it
            for (int i = 0; i < nx_; ++i) {
                l_qp_[row + i] = -1e20;
                u_qp_[row + i] = 1e20;
            }
            row += nx_;
            continue;
        }

        // Exponential tightening: early stages allow wider bounds,
        // later stages converge toward nominal limits.
        double alpha = std::min(1.0, static_cast<double>(k) / std::max(N_ / 2, 1));
        double stage_max_v = max_v * (1.0 - alpha) + nominal_max_v * alpha;
        double stage_max_w = max_w * (1.0 - alpha) + nominal_max_w * alpha;

        for (int i = 0; i < nx_; ++i) {
            double lb = -inf, ub = inf;

            if (i < 3) {
                // Position: no bounds (or very loose)
                lb = -1e6; ub = 1e6;
            } else if (i < 7) {
                // Quaternion: [-1, 1]
                lb = -1.1; ub = 1.1;
            } else if (i < 10) {
                // Linear velocity
                lb = -stage_max_v; ub = stage_max_v;
            } else if (i < 13) {
                // Angular velocity
                lb = -stage_max_w; ub = stage_max_w;
            } else if (i < 16) {
                // RW wheel speeds
                double rw_max = (i - 13 < num_rw_ && i - 13 < static_cast<int>(sat_params_.rw_speed_limits.size()))
                                ? sat_params_.rw_speed_limits[i - 13] : 600.0;
                lb = -rw_max; ub = rw_max;
            } else {
                // Path progress s: [0, path_length]
                lb = 0.0;
                ub = path_.valid ? path_.total_length : 1e6;
            }

            l_qp_[row + i] = lb;
            u_qp_[row + i] = ub;
        }
        row += nx_;
    }

    // 4. Control bounds
    double vs_min = params_.path_speed_min;
    double vs_max = params_.path_speed_max;

    // Near endpoint: allow stopping
    if (path_.valid && s_runtime_ > path_.total_length * 0.9) {
        vs_min = 0.0;
    }

    // Error-priority: reduce vs_max adaptively
    if (params_.progress_policy == "error_priority" && path_.valid) {
        auto [s_p, _, pe, __] = path_.project(x_current.head<3>());
        double gain = params_.error_priority_error_speed_gain;
        vs_max = vs_max / (1.0 + gain * pe * pe);
        vs_min = std::min(vs_min, params_.error_priority_min_vs);
    }

    // Ensure feasibility: vs_min must not exceed vs_max
    vs_min = std::min(vs_min, vs_max);

    for (int k = 0; k < N_; ++k) {
        for (int i = 0; i < nu_; ++i) {
            double lb, ub;
            if (i < num_rw_) {
                // RW torques: normalized [-1, 1]
                lb = -1.0; ub = 1.0;
            } else if (i < num_rw_ + num_thrusters_) {
                // Thrusters: [0, 1]
                lb = 0.0; ub = 1.0;
            } else {
                // Virtual speed v_s
                lb = vs_min; ub = vs_max;
            }
            l_qp_[row + i] = lb;
            u_qp_[row + i] = ub;
        }
        row += nu_;
    }

    // 5. Control horizon tying: u_k = u_{M-1} for k >= M  → bounds = [0, 0]
    for (int k = M_; k < N_; ++k) {
        for (int i = 0; i < nu_; ++i) {
            l_qp_[row + i] = 0.0;
            u_qp_[row + i] = 0.0;
        }
        row += nu_;
    }
}

// ============================================================================
// DARE terminal cost
// ============================================================================

VectorXd SQPController::compute_dare_terminal_diag(
    const VectorXd& x_nominal
) const {
    // Simplified DARE: use the linearised physics A, B at the terminal state
    // and solve the discrete Riccati equation iteratively for a diagonal P.
    // For the V2 implementation, we use a heuristic scaling:
    //   P_ii = Q_ii * N * dt / damping_factor
    // which approximates the infinite-horizon LQR cost-to-go.

    VectorXd P_diag = VectorXd::Zero(16);

    // Position
    for (int i = 0; i < 3; ++i)
        P_diag[i] = (active_Q_contour_ + active_Q_lag_) * N_ * dt_ * 0.5;

    // Quaternion
    for (int i = 3; i < 7; ++i)
        P_diag[i] = active_Q_attitude_ * N_ * dt_ * 0.3;

    // Velocity
    for (int i = 7; i < 10; ++i)
        P_diag[i] = params_.Q_velocity_align * N_ * dt_ * 0.2;

    // Angular velocity
    for (int i = 10; i < 13; ++i)
        P_diag[i] = params_.Q_angvel * N_ * dt_ * 0.3;

    // RW speeds
    for (int i = 13; i < 16; ++i)
        P_diag[i] = 0.01;

    return P_diag;
}

// ============================================================================
// Fallback policy
// ============================================================================

ControlResultV2 SQPController::apply_fallback(double dt_since_last) {
    ControlResultV2 result;
    result.u = VectorXd::Zero(nu_);

    if (!has_last_feasible_) {
        result.fallback_active = true;
        result.fallback_scale = 0.0;
        return result;
    }

    auto now = std::chrono::steady_clock::now();
    if (!fallback_active_) {
        fallback_active_ = true;
        fallback_started_at_ = now;
    }

    double age = std::chrono::duration<double>(now - fallback_started_at_).count();
    double scale = 1.0;

    if (age < params_.solver_fallback_hold_s) {
        scale = 1.0;  // Hold
    } else if (age < params_.solver_fallback_hold_s + params_.solver_fallback_decay_s) {
        double decay_phase = age - params_.solver_fallback_hold_s;
        scale = 1.0 - decay_phase / params_.solver_fallback_decay_s;
    } else {
        scale = 0.0;  // Zero
    }

    scale = std::max(0.0, std::min(1.0, scale));
    result.u = scale * last_feasible_control_;
    result.fallback_active = true;
    result.fallback_age_s = age;
    result.fallback_scale = scale;

    return result;
}

// ============================================================================
// Reference generation
// ============================================================================

Vector4d SQPController::build_reference_quaternion(
    const Vector3d& p_ref,
    const Vector3d& t_ref,
    const Vector4d& q_curr
) const {
    // Build body frame: +X along path tangent, +Z along scan axis (or up)
    Vector3d x_body = t_ref.normalized();
    Vector3d z_body;

    if (scan_ctx_.enabled) {
        z_body = scan_ctx_.axis.normalized();
    } else {
        z_body = Vector3d(0, 0, 1);  // Default: inertial Z
    }

    // Gram-Schmidt orthogonalise
    z_body = z_body - z_body.dot(x_body) * x_body;
    double z_norm = z_body.norm();
    if (z_norm < 1e-6) {
        // Degenerate — pick arbitrary orthogonal
        z_body = Vector3d(0, 1, 0);
        z_body = z_body - z_body.dot(x_body) * x_body;
        z_norm = z_body.norm();
    }
    z_body /= z_norm;

    Vector3d y_body = z_body.cross(x_body);
    y_body.normalize();

    // Rotation matrix R = [x_body | y_body | z_body]
    Eigen::Matrix3d R;
    R.col(0) = x_body;
    R.col(1) = y_body;
    R.col(2) = z_body;

    // Convert to quaternion (scalar-first)
    Eigen::Quaterniond q_eigen(R);
    q_eigen.normalize();

    // Ensure shortest path to current quaternion
    Vector4d q_ref(q_eigen.w(), q_eigen.x(), q_eigen.y(), q_eigen.z());
    if (q_ref.dot(q_curr) < 0) {
        q_ref = -q_ref;
    }

    return q_ref;
}

// ============================================================================
// Configuration methods
// ============================================================================

void SQPController::set_warm_start_control(const VectorXd& u_prev) {
    warm_start_control_ = u_prev;
    has_warm_start_ = true;
}

void SQPController::set_path_data(
    const std::vector<std::array<double, 4>>& path_data
) {
    path_.s.clear();
    path_.points.clear();
    path_.valid = false;

    if (path_data.size() < 2) return;

    for (const auto& row : path_data) {
        path_.s.push_back(row[0]);
        path_.points.emplace_back(row[1], row[2], row[3]);
    }
    path_.total_length = path_.s.back();
    path_.valid = true;

    // Reset path tracking
    s_runtime_ = 0.0;
    s_initialized_ = false;
}

void SQPController::set_scan_attitude_context(
    const Vector3d& center,
    const Vector3d& axis,
    const std::string& direction
) {
    scan_ctx_.enabled = true;
    scan_ctx_.center = center;
    scan_ctx_.center_valid = !center.array().isNaN().any();
    scan_ctx_.axis = axis;
    scan_ctx_.direction_cw = (direction != "CCW");
    scan_ctx_.ref_initialized = false;
}

void SQPController::clear_scan_attitude_context() {
    scan_ctx_.enabled = false;
    scan_ctx_.ref_initialized = false;
}

void SQPController::set_runtime_mode(const std::string& mode) {
    RuntimeMode new_mode = parse_runtime_mode(mode);
    if (new_mode != mode_) {
        mode_ = new_mode;
        apply_mode_scaling();
    }
}

std::tuple<double, Vector3d, double, double> SQPController::project_onto_path(
    const Vector3d& position
) const {
    return path_.project(position);
}

std::tuple<Vector3d, Vector3d, Vector4d> SQPController::get_reference_at_s(
    double s_query,
    const Vector4d& q_current
) const {
    if (!path_.valid) {
        return {Vector3d::Zero(), Vector3d::UnitX(),
                Vector4d(1, 0, 0, 0)};
    }

    s_query = path_.clamp_s(s_query);
    Vector3d pos = path_.get_point(s_query);
    Vector3d tan = path_.get_tangent(s_query);
    Vector4d q_ref = build_reference_quaternion(pos, tan, q_current);

    return {pos, tan, q_ref};
}

}  // namespace v2
}  // namespace satellite_control

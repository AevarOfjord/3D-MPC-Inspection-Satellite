
#include "mpc_controller.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

namespace satellite_mpc {

MPCControllerCpp::MPCControllerCpp(const satellite_dt::SatelliteParams& sat_params, const MPCParams& mpc_params)
    : sat_params_(sat_params), mpc_params_(mpc_params) {
    
    N_ = mpc_params_.prediction_horizon;
    dt_ = mpc_params_.dt;
    nu_ = sat_params_.num_rw + sat_params_.num_thrusters;
    
    // Create linearizer
    linearizer_ = std::make_unique<satellite_dt::Linearizer>(sat_params_);
    
    // Precompute weight vectors
    Q_diag_.resize(nx_);
    Q_diag_ << VectorXd::Constant(3, mpc_params_.Q_pos),
               VectorXd::Constant(4, mpc_params_.Q_ang),
               VectorXd::Constant(3, mpc_params_.Q_vel),
               VectorXd::Constant(3, mpc_params_.Q_angvel);
    
    R_diag_.resize(nu_);
    R_diag_ << VectorXd::Constant(sat_params_.num_rw, mpc_params_.R_rw_torque),
               VectorXd::Constant(sat_params_.num_thrusters, mpc_params_.R_thrust);
    
    // Control bounds
    control_lower_.resize(nu_);
    control_upper_.resize(nu_);
    control_lower_ << VectorXd::Constant(sat_params_.num_rw, -1.0),
                      VectorXd::Zero(sat_params_.num_thrusters);
    control_upper_ << VectorXd::Constant(sat_params_.num_rw, 1.0),
                      VectorXd::Ones(sat_params_.num_thrusters);
    
    // Initialize quaternion tracking
    prev_quat_ << -999, -999, -999, -999;
    
    // Initialize solver
    init_solver();
}

MPCControllerCpp::~MPCControllerCpp() {
    if (work_) {
        osqp_cleanup(work_);
    }
    if (data_) {
        if (data_->P) c_free(data_->P);
        if (data_->A) c_free(data_->A);
        c_free(data_);
    }
    if (settings_) {
        c_free(settings_);
    }
}

void MPCControllerCpp::init_solver() {
    int n_vars = (N_ + 1) * nx_ + N_ * nu_;
    int n_dyn = N_ * nx_;
    int n_init = nx_;
    int n_bounds_x = (N_ + 1) * nx_;
    int n_bounds_u = N_ * nu_;
    int n_constraints = n_dyn + n_init + n_bounds_x + n_bounds_u;
    
    // Build P matrix (diagonal cost)
    std::vector<Eigen::Triplet<double>> P_triplets;
    VectorXd P_diag(n_vars);
    
    // State costs
    for (int k = 0; k < N_; ++k) {
        P_diag.segment(k * nx_, nx_) = Q_diag_;
    }
    // Terminal cost (10x)
    P_diag.segment(N_ * nx_, nx_) = Q_diag_ * 10.0;
    // Control costs
    for (int k = 0; k < N_; ++k) {
        P_diag.segment((N_ + 1) * nx_ + k * nu_, nu_) = R_diag_;
    }
    
    for (int i = 0; i < n_vars; ++i) {
        P_triplets.emplace_back(i, i, P_diag(i));
    }
    P_.resize(n_vars, n_vars);
    P_.setFromTriplets(P_triplets.begin(), P_triplets.end());
    P_.makeCompressed();
    
    // Build A matrix (dynamics, init, bounds)
    std::vector<Eigen::Triplet<double>> A_triplets;
    
    // Get template A, B matrices
    VectorXd dummy_state = VectorXd::Zero(nx_);
    dummy_state(3) = 1.0;  // Valid quaternion
    auto [A_dyn, B_dyn] = linearizer_->linearize(dummy_state);
    
    int row_idx = 0;
    
    // Dynamics: -A*x_k + x_{k+1} - B*u_k = 0
    for (int k = 0; k < N_; ++k) {
        int x_k_idx = k * nx_;
        int x_kp1_idx = (k + 1) * nx_;
        int u_k_idx = (N_ + 1) * nx_ + k * nu_;
        
        // -A (dynamics matrix)
        for (int r = 0; r < nx_; ++r) {
            for (int c = 0; c < nx_; ++c) {
                // Always include G-block (rows 3-6, cols 10-12) for quaternion dynamics
                // These entries change with orientation and all may be non-zero
                bool is_g_block = (r >= 3 && r < 7 && c >= 10 && c < 13);
                if (is_g_block || std::abs(A_dyn(r, c)) > 1e-12) {
                    A_triplets.emplace_back(row_idx + r, x_k_idx + c, -A_dyn(r, c));
                }
            }
        }
        // +I
        for (int r = 0; r < nx_; ++r) {
            A_triplets.emplace_back(row_idx + r, x_kp1_idx + r, 1.0);
        }
        // -B (always include all entries as they depend on orientation)
        for (int r = 0; r < nx_; ++r) {
            for (int c = 0; c < nu_; ++c) {
                // Always include velocity and angular velocity rows (7-12)
                // as these change with body orientation
                bool is_velocity_row = (r >= 7);
                if (is_velocity_row || std::abs(B_dyn(r, c)) > 1e-12) {
                    A_triplets.emplace_back(row_idx + r, u_k_idx + c, -B_dyn(r, c));
                }
            }
        }
        row_idx += nx_;
    }
    
    // Initial state constraint: x_0 = x_current
    for (int r = 0; r < nx_; ++r) {
        A_triplets.emplace_back(row_idx + r, r, 1.0);
    }
    row_idx += nx_;
    
    // State bounds (identity)
    for (int k = 0; k < N_ + 1; ++k) {
        for (int r = 0; r < nx_; ++r) {
            A_triplets.emplace_back(row_idx + r, k * nx_ + r, 1.0);
        }
        row_idx += nx_;
    }
    
    // Control bounds (identity)
    for (int k = 0; k < N_; ++k) {
        int u_k_idx = (N_ + 1) * nx_ + k * nu_;
        for (int r = 0; r < nu_; ++r) {
            A_triplets.emplace_back(row_idx + r, u_k_idx + r, 1.0);
        }
        row_idx += nu_;
    }
    
    A_.resize(n_constraints, n_vars);
    A_.setFromTriplets(A_triplets.begin(), A_triplets.end());
    A_.makeCompressed();
    
    // Build B index map for fast updates
    B_idx_map_.resize(nx_ * nu_);
    int u_start_idx = (N_ + 1) * nx_;
    for (int k = 0; k < N_; ++k) {
        int current_row_base = k * nx_;
        for (int r = 0; r < nx_; ++r) {
            for (int c = 0; c < nu_; ++c) {
                int col = u_start_idx + k * nu_ + c;
                // Find index in CSC format
                for (int idx = A_.outerIndexPtr()[col]; idx < A_.outerIndexPtr()[col + 1]; ++idx) {
                    if (A_.innerIndexPtr()[idx] == current_row_base + r) {
                        B_idx_map_[r * nu_ + c].push_back(idx);
                    }
                }
            }
        }
    }
    
    // Build A_idx_map for the quaternion-dependent G-block (rows 3-6, cols 10-12)
    // These are the dQuat/dOmega entries in the dynamics matrix
    A_idx_map_.resize(4 * 3);  // 4 quat components x 3 omega components
    for (int k = 0; k < N_; ++k) {
        int row_base = k * nx_;
        int col_base = k * nx_;  // x_k columns
        
        for (int qr = 0; qr < 4; ++qr) {  // Quaternion rows (3-6 relative to state)
            for (int oc = 0; oc < 3; ++oc) {  // Omega cols (10-12 relative to state)
                int row = row_base + 3 + qr;
                int col = col_base + 10 + oc;
                
                for (int idx = A_.outerIndexPtr()[col]; idx < A_.outerIndexPtr()[col + 1]; ++idx) {
                    if (A_.innerIndexPtr()[idx] == row) {
                        A_idx_map_[qr * 3 + oc].push_back(idx);
                    }
                }
            }
        }
    }
    
    // Initialize bounds
    q_ = VectorXd::Zero(n_vars);
    l_ = VectorXd::Zero(n_constraints);
    u_ = VectorXd::Zero(n_constraints);
    
    // Set state bounds to inf
    int state_idx_start = n_dyn + n_init;
    l_.segment(state_idx_start, n_bounds_x).setConstant(-1e20);
    u_.segment(state_idx_start, n_bounds_x).setConstant(1e20);
    
    // Set control bounds
    int ctrl_idx_start = n_dyn + n_init + n_bounds_x;
    for (int k = 0; k < N_; ++k) {
        l_.segment(ctrl_idx_start + k * nu_, nu_) = control_lower_;
        u_.segment(ctrl_idx_start + k * nu_, nu_) = control_upper_;
    }
    
    // Convert to CSC arrays for OSQP
    P_data_.assign(P_.valuePtr(), P_.valuePtr() + P_.nonZeros());
    P_indices_.assign(P_.innerIndexPtr(), P_.innerIndexPtr() + P_.nonZeros());
    P_indptr_.assign(P_.outerIndexPtr(), P_.outerIndexPtr() + P_.cols() + 1);
    
    A_data_.assign(A_.valuePtr(), A_.valuePtr() + A_.nonZeros());
    A_indices_.assign(A_.innerIndexPtr(), A_.innerIndexPtr() + A_.nonZeros());
    A_indptr_.assign(A_.outerIndexPtr(), A_.outerIndexPtr() + A_.cols() + 1);
    
    // Setup OSQP
    data_ = (OSQPData*)c_malloc(sizeof(OSQPData));
    data_->n = n_vars;
    data_->m = n_constraints;
    
    data_->P = csc_matrix(n_vars, n_vars, P_.nonZeros(),
                          P_data_.data(), P_indices_.data(), P_indptr_.data());
    data_->q = q_.data();
    data_->A = csc_matrix(n_constraints, n_vars, A_.nonZeros(),
                          A_data_.data(), A_indices_.data(), A_indptr_.data());
    data_->l = l_.data();
    data_->u = u_.data();
    
    settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->time_limit = mpc_params_.solver_time_limit;
    
    osqp_setup(&work_, data_, settings_);
}

void MPCControllerCpp::update_dynamics(const VectorXd& x_current) {
    auto [A_dyn, B_dyn] = linearizer_->linearize(x_current);
    
    // Update A-block entries (G matrix: dQuat/dOmega)
    // A_dyn rows 3-6, cols 10-12 contain the quaternion-dependent G matrix
    for (int qr = 0; qr < 4; ++qr) {
        for (int oc = 0; oc < 3; ++oc) {
            double val = -A_dyn(3 + qr, 10 + oc);  // Note: stored as -A in the constraint matrix
            for (int idx : A_idx_map_[qr * 3 + oc]) {
                A_data_[idx] = val;
            }
        }
    }
    
    // Update B matrix entries in A_data_
    for (int r = 0; r < nx_; ++r) {
        for (int c = 0; c < nu_; ++c) {
            double val = -B_dyn(r, c);
            for (int idx : B_idx_map_[r * nu_ + c]) {
                A_data_[idx] = val;
            }
        }
    }
    
    osqp_update_A(work_, A_data_.data(), OSQP_NULL, A_.nonZeros());
}

void MPCControllerCpp::update_cost(const VectorXd& x_target) {
    VectorXd Q_term = Q_diag_ * 10.0;
    
    // Update q vector: q = -Q * x_ref
    for (int k = 0; k < N_; ++k) {
        q_.segment(k * nx_, nx_) = -Q_diag_.cwiseProduct(x_target);
    }
    q_.segment(N_ * nx_, nx_) = -Q_term.cwiseProduct(x_target);
    
    osqp_update_lin_cost(work_, q_.data());
}

void MPCControllerCpp::update_constraints(const VectorXd& x_current) {
    int init_start = N_ * nx_;
    l_.segment(init_start, nx_) = x_current;
    u_.segment(init_start, nx_) = x_current;
    
    osqp_update_bounds(work_, l_.data(), u_.data());
}

VectorXd MPCControllerCpp::apply_z_tilt(const VectorXd& x_current, const VectorXd& x_target) {
    if (!mpc_params_.enable_z_tilt) {
        return x_target;
    }
    
    double z_err = x_target(2) - x_current(2);
    if (std::abs(z_err) < 1e-4) {
        return x_target;
    }
    
    // Simplified Z-tilt: just return target for now
    // Full implementation would modify the quaternion
    return x_target;
}

ControlResult MPCControllerCpp::get_control_action(const VectorXd& x_current, const VectorXd& x_target) {
    auto start = std::chrono::high_resolution_clock::now();
    
    ControlResult result;
    result.timeout = false;
    
    // Apply Z-tilt to target
    VectorXd x_targ = apply_z_tilt(x_current, x_target);
    
    // Update dynamics if quaternion changed significantly
    Eigen::Vector4d quat = x_current.segment<4>(3);
    if ((quat - prev_quat_).norm() > 0.05) {
        update_dynamics(x_current);
        prev_quat_ = quat;
    }
    
    // Update cost
    update_cost(x_targ);
    
    // Update constraints
    update_constraints(x_current);
    
    // Solve
    osqp_solve(work_);
    
    auto end = std::chrono::high_resolution_clock::now();
    result.solve_time = std::chrono::duration<double>(end - start).count();
    
    // Check status
    if (work_->info->status_val != OSQP_SOLVED && 
        work_->info->status_val != OSQP_SOLVED_INACCURATE) {
        result.status = -1;
        result.u = VectorXd::Zero(nu_);
        return result;
    }
    
    // Extract control
    int u_idx = (N_ + 1) * nx_;
    result.u = Eigen::Map<VectorXd>(work_->solution->x + u_idx, nu_);
    
    // Clip to bounds
    result.u = result.u.cwiseMax(control_lower_).cwiseMin(control_upper_);
    result.status = 1;
    
    return result;
}

} // namespace satellite_mpc

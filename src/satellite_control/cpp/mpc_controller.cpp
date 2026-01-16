
#include "mpc_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace satellite_control {

MPCControllerCpp::MPCControllerCpp(const SatelliteParams& sat_params, const MPCParams& mpc_params)
    : sat_params_(sat_params), mpc_params_(mpc_params) {
    
    N_ = mpc_params_.prediction_horizon;
    dt_ = mpc_params_.dt;
    nu_ = sat_params_.num_rw + sat_params_.num_thrusters;
    
    // Create linearizer
    linearizer_ = std::make_unique<Linearizer>(sat_params_);
    
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

// ----------------------------------------------------------------------------
// Initialization & Helpers
// ----------------------------------------------------------------------------

void MPCControllerCpp::init_solver() {
    int n_vars = (N_ + 1) * nx_ + N_ * nu_;
    
    // 1. Build P matrix (Cost)
    std::vector<Eigen::Triplet<double>> P_triplets;
    build_P_matrix(P_triplets, n_vars);
    
    P_.resize(n_vars, n_vars);
    P_.setFromTriplets(P_triplets.begin(), P_triplets.end());
    P_.makeCompressed();
    
    // 2. Build A matrix (Constraints)
    std::vector<Eigen::Triplet<double>> A_triplets;
    build_A_matrix(A_triplets);
    
    // Count constraints based on structure:
    // Dynamics (N*nx) + Initial (nx) + State Bounds ((N+1)*nx) + Control Bounds (N*nu) + Obstacles (N)
    int n_dyn = N_ * nx_;
    int n_init = nx_;
    int n_bounds_x = (N_ + 1) * nx_;
    int n_bounds_u = N_ * nu_;
    n_obs_constraints_ = N_;
    int n_constraints = n_dyn + n_init + n_bounds_x + n_bounds_u + n_obs_constraints_;
    
    A_.resize(n_constraints, n_vars);
    A_.setFromTriplets(A_triplets.begin(), A_triplets.end());
    A_.makeCompressed();

    // 3. Build index maps for fast updates
    
    // A. Obstacle update indices
    // Row layout: [dyn, init, bounds_x, bounds_u, obs]
    int obs_row_start = n_dyn + n_init + n_bounds_x + n_bounds_u;
    obs_A_indices_.resize(N_);
    
    for (int k = 0; k < N_; ++k) {
        int row = obs_row_start + k;
        obs_A_indices_[k].resize(3);
        int x_k_idx = k * nx_;
        
        for (int i = 0; i < 3; ++i) {
            int col = x_k_idx + i;
            bool found = false;
            // Search in column 'col' for row 'row' in CSC matrix
            for (int idx = A_.outerIndexPtr()[col]; idx < A_.outerIndexPtr()[col + 1]; ++idx) {
                if (A_.innerIndexPtr()[idx] == row) {
                    obs_A_indices_[k][i] = idx;
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::cerr << "[MPC] Error: Could not find obstacle constraint entry at k=" << k << ", i=" << i << std::endl;
            }
        }
    }
    
    // B. Actuator dynamics index map (B matrix updates)
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
    
    // C. Quaternion dynamics index map (A matrix updates for G-block)
    // A_dyn rows 3-6, cols 10-12
    A_idx_map_.resize(4 * 3);
    for (int k = 0; k < N_; ++k) {
        int row_base = k * nx_;
        int col_base = k * nx_;
        
        for (int qr = 0; qr < 4; ++qr) {      // Relative rows 3-6
            for (int oc = 0; oc < 3; ++oc) {  // Relative cols 10-12
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

    // 4. Setup Bounds and OSQP workspace
    setup_osqp_workspace(n_vars, n_constraints);
}

void MPCControllerCpp::build_P_matrix(std::vector<Eigen::Triplet<double>>& triplets, int n_vars) {
    VectorXd P_diag(n_vars);
    
    // Stage costs (0 to N-1)
    for (int k = 0; k < N_; ++k) {
        P_diag.segment(k * nx_, nx_) = Q_diag_;
    }
    // Terminal cost (N) - scaled by 10
    P_diag.segment(N_ * nx_, nx_) = Q_diag_ * 10.0;
    
    // Control costs (0 to N-1)
    for (int k = 0; k < N_; ++k) {
        P_diag.segment((N_ + 1) * nx_ + k * nu_, nu_) = R_diag_;
    }
    
    for (int i = 0; i < n_vars; ++i) {
        triplets.emplace_back(i, i, P_diag(i));
    }
}

void MPCControllerCpp::build_A_matrix(std::vector<Eigen::Triplet<double>>& triplets) {
    // Get template A, B matrices around a valid quaternion
    VectorXd dummy_state = VectorXd::Zero(nx_);
    dummy_state(3) = 1.0; 
    auto [A_dyn, B_dyn] = linearizer_->linearize(dummy_state);
    
    int row_idx = 0;
    
    // 1. Dynamics constraints: -A*x_k + x_{k+1} - B*u_k = 0
    for (int k = 0; k < N_; ++k) {
        int x_k_idx = k * nx_;
        int x_kp1_idx = (k + 1) * nx_;
        int u_k_idx = (N_ + 1) * nx_ + k * nu_;
        
        // -A block
        for (int r = 0; r < nx_; ++r) {
            for (int c = 0; c < nx_; ++c) {
                // Force inclusion of G-block (rows 3-6, cols 10-12) for quaternion dynamics updates
                bool is_g_block = (r >= 3 && r < 7 && c >= 10 && c < 13);
                if (is_g_block || std::abs(A_dyn(r, c)) > 1e-12) {
                    triplets.emplace_back(row_idx + r, x_k_idx + c, -A_dyn(r, c));
                }
            }
        }
        // +I block (x_{k+1})
        for (int r = 0; r < nx_; ++r) {
            triplets.emplace_back(row_idx + r, x_kp1_idx + r, 1.0);
        }
        // -B block
        for (int r = 0; r < nx_; ++r) {
            for (int c = 0; c < nu_; ++c) {
                // Force inclusion of velocity rows (7-12) for updates
                bool is_velocity_row = (r >= 7);
                if (is_velocity_row || std::abs(B_dyn(r, c)) > 1e-12) {
                    triplets.emplace_back(row_idx + r, u_k_idx + c, -B_dyn(r, c));
                }
            }
        }
        row_idx += nx_;
    }
    
    // 2. Initial state constraint: I*x_0 = x_current
    for (int r = 0; r < nx_; ++r) {
        triplets.emplace_back(row_idx + r, r, 1.0);
    }
    row_idx += nx_;
    
    // 3. State bounds: I*x_k
    for (int k = 0; k < N_ + 1; ++k) {
        for (int r = 0; r < nx_; ++r) {
            triplets.emplace_back(row_idx + r, k * nx_ + r, 1.0);
        }
        row_idx += nx_;
    }
    
    // 4. Control bounds: I*u_k
    for (int k = 0; k < N_; ++k) {
        int u_k_idx = (N_ + 1) * nx_ + k * nu_;
        for (int r = 0; r < nu_; ++r) {
            triplets.emplace_back(row_idx + r, u_k_idx + r, 1.0);
        }
        row_idx += nu_;
    }

    // 5. Obstacle constraints: obs_k^T * p_k >= dist_k
    // We reserve these rows now and update coefficients dynamically.
    // Initial coeff is epsilon to prevent pruning.
    for (int k = 0; k < N_; ++k) {
        int x_k_idx = k * nx_; 
        for (int i = 0; i < 3; ++i) {
             triplets.emplace_back(row_idx, x_k_idx + i, 1e-10);
        }
        row_idx++;
    }
}

void MPCControllerCpp::setup_osqp_workspace(int n_vars, int n_constraints) {
    // Initialize bound vectors
    q_ = VectorXd::Zero(n_vars);
    l_ = VectorXd::Zero(n_constraints);
    u_ = VectorXd::Zero(n_constraints);
    
    int n_dyn = N_ * nx_;
    int n_init = nx_;
    int n_bounds_x = (N_ + 1) * nx_;
    int n_bounds_u = N_ * nu_;
    
    // 1. Dynamics equality (l=0, u=0) - default is 0
    
    // 2. Initial state (will be updated at valid step)
    
    // 3. State bounds (initialized to infinity)
    int state_idx_start = n_dyn + n_init;
    l_.segment(state_idx_start, n_bounds_x).setConstant(-1e20);
    u_.segment(state_idx_start, n_bounds_x).setConstant(1e20);
    
    // 4. Control bounds
    int ctrl_idx_start = n_dyn + n_init + n_bounds_x;
    for (int k = 0; k < N_; ++k) {
        l_.segment(ctrl_idx_start + k * nu_, nu_) = control_lower_;
        u_.segment(ctrl_idx_start + k * nu_, nu_) = control_upper_;
    }

    // 5. Obstacle bounds (initialized to inactive)
    int obs_idx_start = n_dyn + n_init + n_bounds_x + n_bounds_u;
    l_.segment(obs_idx_start, n_obs_constraints_).setConstant(-1e20);
    u_.segment(obs_idx_start, n_obs_constraints_).setConstant(1e20);
    
    // Convert to CSC arrays for OSQP
    P_data_.assign(P_.valuePtr(), P_.valuePtr() + P_.nonZeros());
    P_indices_.assign(P_.innerIndexPtr(), P_.innerIndexPtr() + P_.nonZeros());
    P_indptr_.assign(P_.outerIndexPtr(), P_.outerIndexPtr() + P_.cols() + 1);
    
    A_data_.assign(A_.valuePtr(), A_.valuePtr() + A_.nonZeros());
    A_indices_.assign(A_.innerIndexPtr(), A_.innerIndexPtr() + A_.nonZeros());
    A_indptr_.assign(A_.outerIndexPtr(), A_.outerIndexPtr() + A_.cols() + 1);
    
    // Setup OSQP Structures
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

// ----------------------------------------------------------------------------
// Runtime Updates
// ----------------------------------------------------------------------------

void MPCControllerCpp::update_dynamics(const VectorXd& x_current) {
    auto [A_dyn, B_dyn] = linearizer_->linearize(x_current);
    
    // Update A-block entries (G matrix: dQuat/dOmega)
    // A_dyn rows 3-6, cols 10-12
    for (int qr = 0; qr < 4; ++qr) {
        for (int oc = 0; oc < 3; ++oc) {
            double val = -A_dyn(3 + qr, 10 + oc); // Stored as -A
            for (int idx : A_idx_map_[qr * 3 + oc]) {
                A_data_[idx] = val;
            }
        }
    }
    
    // Update B matrix entries
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
    
    // Update linear cost vector q = -Q * x_ref
    for (int k = 0; k < N_; ++k) {
        q_.segment(k * nx_, nx_) = -Q_diag_.cwiseProduct(x_target);
    }
    q_.segment(N_ * nx_, nx_) = -Q_term.cwiseProduct(x_target);
    
    osqp_update_lin_cost(work_, q_.data());
}

void MPCControllerCpp::update_cost_trajectory(const MatrixXd& x_target_traj) {
    VectorXd Q_term = Q_diag_ * 10.0;
    if (x_target_traj.rows() <= 0) {
        update_cost(VectorXd::Zero(nx_));
        return;
    }

    int last_idx = x_target_traj.rows() - 1;
    for (int k = 0; k < N_; ++k) {
        int idx = std::min(k, last_idx);
        q_.segment(k * nx_, nx_) =
            -Q_diag_.cwiseProduct(x_target_traj.row(idx).transpose());
    }
    int term_idx = std::min(N_, last_idx);
    q_.segment(N_ * nx_, nx_) =
        -Q_term.cwiseProduct(x_target_traj.row(term_idx).transpose());

    osqp_update_lin_cost(work_, q_.data());
}

void MPCControllerCpp::update_constraints(const VectorXd& x_current) {
    // Update initial state constraint bounds (equality constraint)
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
    
    // Simplified Z-tilt: pass-through for now, placeholder for full tilt logic
    return x_target;
}

// ----------------------------------------------------------------------------
// Main Control Interface
// ----------------------------------------------------------------------------

ControlResult MPCControllerCpp::get_control_action(const VectorXd& x_current, const VectorXd& x_target) {
    auto start = std::chrono::high_resolution_clock::now();
    
    ControlResult result;
    result.timeout = false;
    
    VectorXd x_targ = apply_z_tilt(x_current, x_target);
    
    // Successive Linearization: Update dynamics if quaternion changed significantly
    Eigen::Vector4d quat = x_current.segment<4>(3);
    if ((quat - prev_quat_).norm() > 0.001) {
        update_dynamics(x_current);
        prev_quat_ = quat;
    }
    
    update_cost(x_targ);
    update_obstacle_constraints(x_current, x_target);
    update_constraints(x_current);
    
    osqp_solve(work_);
    
    auto end = std::chrono::high_resolution_clock::now();
    result.solve_time = std::chrono::duration<double>(end - start).count();
    
    if (work_->info->status_val != OSQP_SOLVED && 
        work_->info->status_val != OSQP_SOLVED_INACCURATE) {
        result.status = -1; // Error
        result.u = VectorXd::Zero(nu_);
        return result;
    }
    
    // Extract control from solution
    int u_idx = (N_ + 1) * nx_;
    result.u = Eigen::Map<VectorXd>(work_->solution->x + u_idx, nu_);
    
    // Clip to bounds (safety)
    result.u = result.u.cwiseMax(control_lower_).cwiseMin(control_upper_);
    result.status = 1; // Success
    
    return result;
}

ControlResult MPCControllerCpp::get_control_action_trajectory(
    const VectorXd& x_current, const MatrixXd& x_target_traj) {
    auto start = std::chrono::high_resolution_clock::now();

    ControlResult result;
    result.timeout = false;

    Eigen::Vector4d quat = x_current.segment<4>(3);
    if ((quat - prev_quat_).norm() > 0.001) {
        update_dynamics(x_current);
        prev_quat_ = quat;
    }

    update_cost_trajectory(x_target_traj);
    if (x_target_traj.rows() > 0) {
        update_obstacle_constraints(x_current, x_target_traj.row(0).transpose());
    } else {
        update_obstacle_constraints(x_current, x_current);
    }
    update_constraints(x_current);

    osqp_solve(work_);

    if (work_->info->status_val != OSQP_SOLVED) {
        result.status = work_->info->status_val;
        result.u = VectorXd::Zero(nu_);
    } else {
        result.status = work_->info->status_val;
        result.u = VectorXd::Map(work_->solution->x + (N_ + 1) * nx_, nu_);
    }
    
    result.u = result.u.cwiseMax(control_lower_).cwiseMin(control_upper_);

    auto end = std::chrono::high_resolution_clock::now();
    result.solve_time = std::chrono::duration<double>(end - start).count();

    return result;
}

// ----------------------------------------------------------------------------
// Collision Avoidance
// ----------------------------------------------------------------------------

void MPCControllerCpp::set_obstacles(const ObstacleSet& obstacles) {
    obstacles_ = obstacles;
}

void MPCControllerCpp::clear_obstacles() {
    obstacles_.clear();
}

void MPCControllerCpp::apply_obstacle_constraints(const VectorXd& x_current) {
    // Deprecated / Placeholder
}

void MPCControllerCpp::update_obstacle_constraints(const VectorXd& x_current, const VectorXd& x_target) {
    if (!mpc_params_.enable_collision_avoidance || obstacles_.size() == 0) {
        // Disable all constraints
        int obs_idx_start = A_.rows() - n_obs_constraints_;
        l_.segment(obs_idx_start, n_obs_constraints_).setConstant(-1e20);
        osqp_update_bounds(work_, l_.data(), u_.data());
        return;
    }

    int obs_idx_start = A_.rows() - n_obs_constraints_;
    VectorXd p_curr = x_current.segment<3>(0);
    VectorXd p_targ = x_target.segment<3>(0);

    for (int k = 0; k < N_; ++k) {
        // Linear interpolation guess
        double alpha = double(k + 1) / double(N_);
        Eigen::Vector3d p_guess = (p_curr + alpha * (p_targ - p_curr)); 

        // Get constraints
        auto constraints = obstacles_.get_linear_constraints(p_guess, mpc_params_.obstacle_margin);
        
        if (constraints.empty()) {
            l_(obs_idx_start + k) = -1e20;
            // Optionally zero out A coeffs for cleanliness (helper map needed)
            for(int i=0; i<3; ++i) A_data_[obs_A_indices_[k][i]] = 0.0;
        } else {
            // Apply first constraint: n^T * p >= d
            Eigen::Vector3d normal = constraints[0].first;
            double bound_val = constraints[0].second;
            
            for(int i=0; i<3; ++i) {
                A_data_[obs_A_indices_[k][i]] = normal(i);
            }
            l_(obs_idx_start + k) = bound_val;
        }
    }
    
    osqp_update_A(work_, A_data_.data(), OSQP_NULL, A_.nonZeros());
    osqp_update_bounds(work_, l_.data(), u_.data());
}

} // namespace satellite_control

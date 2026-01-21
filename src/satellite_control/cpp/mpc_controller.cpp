
#include "mpc_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace satellite_control {

MPCControllerCpp::MPCControllerCpp(const SatelliteParams& sat_params, const MPCParams& mpc_params)
    : sat_params_(sat_params), mpc_params_(mpc_params) {
    
    // Path Following State Augmentation
    if (mpc_params_.mode_path_following) {
        nx_ = 17; // 13 base + 3 wheel speeds + 1 path param (s)
    } else {
        nx_ = 16; // 13 base + 3 wheel speeds
    }

    N_ = mpc_params_.prediction_horizon;
    dt_ = mpc_params_.dt;
    nu_ = sat_params_.num_rw + sat_params_.num_thrusters;
    if (mpc_params_.mode_path_following) {
        nu_ += 1; // +1 for virtual path velocity v_s
    }
    
    // Create linearizer
    linearizer_ = std::make_unique<Linearizer>(sat_params_);
    
    // Precompute weight vectors
    Q_diag_.resize(nx_);
    if (mpc_params_.mode_path_following) {
        // Path-following: contouring handles position, velocity alignment handled in update_path_cost.
        // Disable quaternion tracking to avoid penalizing unit quaternion magnitude.
        Q_diag_ << VectorXd::Constant(3, 2.0 * mpc_params_.Q_contour),
                   VectorXd::Constant(4, 0.0),
                   VectorXd::Constant(3, 0.0),
                   VectorXd::Constant(3, mpc_params_.Q_angvel),
                   VectorXd::Constant(3, 0.0), // Wheel speeds
                   VectorXd::Constant(1, 0.0); // s diagonal updated per-step
    } else {
        Q_diag_ << VectorXd::Constant(3, mpc_params_.Q_pos),
                   VectorXd::Constant(4, mpc_params_.Q_ang),
                   VectorXd::Constant(3, mpc_params_.Q_vel),
                   VectorXd::Constant(3, mpc_params_.Q_angvel),
                   VectorXd::Constant(3, 0.0); // Wheel speeds
    }
    
    R_diag_.resize(nu_);
    if (mpc_params_.mode_path_following) {
        R_diag_ << VectorXd::Constant(sat_params_.num_rw, mpc_params_.R_rw_torque),
                   VectorXd::Constant(sat_params_.num_thrusters, mpc_params_.R_thrust),
                   VectorXd::Constant(1, mpc_params_.Q_smooth + mpc_params_.Q_progress);
    } else {
        R_diag_ << VectorXd::Constant(sat_params_.num_rw, mpc_params_.R_rw_torque),
                   VectorXd::Constant(sat_params_.num_thrusters, mpc_params_.R_thrust);
    }
    
    // Control bounds
    control_lower_.resize(nu_);
    control_upper_.resize(nu_);
    if (mpc_params_.mode_path_following) {
        control_lower_ << VectorXd::Constant(sat_params_.num_rw, -1.0),
                          VectorXd::Zero(sat_params_.num_thrusters),
                          VectorXd::Constant(1, 0.0); // v_s >= 0 (no backtracking)
                          
        control_upper_ << VectorXd::Constant(sat_params_.num_rw, 1.0),
                          VectorXd::Ones(sat_params_.num_thrusters),
                          VectorXd::Constant(1, 2.0); // v_s max (arbitrary high limit, governed by cost)
    } else {
        control_lower_ << VectorXd::Constant(sat_params_.num_rw, -1.0),
                          VectorXd::Zero(sat_params_.num_thrusters);
        control_upper_ << VectorXd::Constant(sat_params_.num_rw, 1.0),
                          VectorXd::Ones(sat_params_.num_thrusters);
    }
    
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

    // D. Path Following P-matrix index map
    // We need to find the indices in P_data_ for the cross terms (0,16), (1,16), (2,16)
    if (mpc_params_.mode_path_following) {
        path_P_indices_.resize(N_ + 1);
        path_s_diag_indices_.resize(N_ + 1);
        path_vel_P_indices_.resize(N_ + 1);
        int s_offset = 16;
        int v_offset = 7;
        
        for (int k = 0; k <= N_; ++k) { // Include terminal
            path_P_indices_[k].resize(3);
            int base_idx = k * nx_;
            
            // P is symmetric, we added (row, col) with row < col. 
            // In CSC, we look for column 'col' and row 'row'.
            int col = base_idx + s_offset;
            
            for (int i = 0; i < 3; ++i) {
                int row = base_idx + i;
                bool found = false;
                
                // Search column 'col'
                for (int idx = P_.outerIndexPtr()[col]; idx < P_.outerIndexPtr()[col + 1]; ++idx) {
                    if (P_.innerIndexPtr()[idx] == row) {
                        path_P_indices_[k][i] = idx;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                     std::cerr << "[MPC] Error: P-matrix cross term not found at k=" << k << ", i=" << i << std::endl;
                }
            }

            // Find s diagonal index
            int s_row = base_idx + s_offset;
            int s_col = base_idx + s_offset;
            bool s_found = false;
            for (int idx = P_.outerIndexPtr()[s_col]; idx < P_.outerIndexPtr()[s_col + 1]; ++idx) {
                if (P_.innerIndexPtr()[idx] == s_row) {
                    path_s_diag_indices_[k] = idx;
                    s_found = true;
                    break;
                }
            }
            if (!s_found) {
                std::cerr << "[MPC] Error: P-matrix s diagonal not found at k=" << k << std::endl;
            }

            // Find velocity block indices (upper triangular)
            path_vel_P_indices_[k].resize(6);
            int vel_idx = 0;
            for (int i = 0; i < 3; ++i) {
                for (int j = i; j < 3; ++j) {
                    int row = base_idx + v_offset + i;
                    int col = base_idx + v_offset + j;
                    bool v_found = false;
                    for (int idx = P_.outerIndexPtr()[col]; idx < P_.outerIndexPtr()[col + 1]; ++idx) {
                        if (P_.innerIndexPtr()[idx] == row) {
                            path_vel_P_indices_[k][vel_idx++] = idx;
                            v_found = true;
                            break;
                        }
                    }
                    if (!v_found) {
                        std::cerr << "[MPC] Error: P-matrix velocity entry not found at k=" << k
                                  << ", i=" << i << ", j=" << j << std::endl;
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
        P_diag.segment((N_ + 1) * nx_ + k * nu_, nu_) = R_diag_; // Augmented nu handles vs cost
    }
    
    for (int i = 0; i < n_vars; ++i) {
        triplets.emplace_back(i, i, P_diag(i));
    }

    // Path Following: Pre-allocate cross-terms for (x, s), (y, s), (z, s)
    // These correspond to the -2 * weight * r^T * t * s term in the cost expansion.
    // We need sparsity at (k*nx + 0, k*nx + 16), (k*nx + 1, k*nx + 16), etc.
    if (mpc_params_.mode_path_following) {
        int s_offset = 16; // Index of s in state vector
        int v_offset = 7; // Index of velocity block in state vector
        for (int k = 0; k <= N_; ++k) { // Include terminal cost
            int base_idx = k * nx_;
            // Add entries for x, y, z cross s
            // We add symmetric entries (r,c) and (c,r) for OSQP (upper triangular is sufficient? OSQP wants triu)
            // OSQP requires upper triangular part of P.
            // So we add (row, col) where row <= col.
            // pos indices are 0,1,2. s index is 16. So pos < s.
            for (int i = 0; i < 3; ++i) {
                triplets.emplace_back(base_idx + i, base_idx + s_offset, 0.0); // Placeholder
            }
            // Add entries for velocity block (upper triangular) for tangent alignment cost
            for (int i = 0; i < 3; ++i) {
                for (int j = i; j < 3; ++j) {
                    triplets.emplace_back(base_idx + v_offset + i,
                                          base_idx + v_offset + j, 0.0); // Placeholder
                }
            }
        }
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
                // Special handling for Path State (index 16)
                // s_{k+1} = s_k + v_s * dt  =>  -s_k + s_{k+1} - dt*v_s = 0
                // So A term for s is just 1.0 at (16, 16).
                if (mpc_params_.mode_path_following && r == 16) {
                    if (c == 16) triplets.emplace_back(row_idx + r, x_k_idx + c, -1.0);
                    continue;
                }

                // Normal Physics Dynamics (0-15)
                // Linearizer returns 16x16, so checking bounds
                if (r < 16 && c < 16) {
                    // Force inclusion of G-block (rows 3-6, cols 10-12) for quaternion dynamics updates
                    bool is_g_block = (r >= 3 && r < 7 && c >= 10 && c < 13);
                    if (is_g_block || std::abs(A_dyn(r, c)) > 1e-12) {
                        triplets.emplace_back(row_idx + r, x_k_idx + c, -A_dyn(r, c));
                    }
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
                // Special handling for Path Control (v_s)
                // v_s is the last control input.
                // Dynamics: -dt * v_s
                if (mpc_params_.mode_path_following && r == 16) {
                    if (c == nu_ - 1) { // Last control is v_s
                        triplets.emplace_back(row_idx + r, u_k_idx + c, -mpc_params_.dt);
                    }
                    continue;
                }

                // Normal Physics Limits
                // B_dyn is 16 rows, (nu-1) cols (if we ignore v_s)
                int nu_phys = mpc_params_.mode_path_following ? nu_ - 1 : nu_;
                
                if (r < 16 && c < nu_phys) {
                    // Force inclusion of velocity rows (7-12) for updates
                    bool is_velocity_row = (r >= 7);
                    if (is_velocity_row || std::abs(B_dyn(r, c)) > 1e-12) {
                        triplets.emplace_back(row_idx + r, u_k_idx + c, -B_dyn(r, c));
                    }
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

    // 3a. Velocity bounds (indices 7, 8, 9) - DISABLED in MPCC mode
    if (mpc_params_.max_velocity > 0 && !mpc_params_.mode_path_following) {
        for (int k = 0; k < N_ + 1; ++k) {
            int vel_idx = state_idx_start + k * nx_ + 7;
            l_.segment(vel_idx, 3).setConstant(-mpc_params_.max_velocity);
            u_.segment(vel_idx, 3).setConstant(mpc_params_.max_velocity);
        }
    }

    // 3b. Wheel speed limits (indices 13-15)
    // Assume +/- 500 rad/s (~4700 RPM) limit if not specified, or use a param?
    // Using loose bounds for now or derived from somewhere?
    // Let's set loose bounds for now, can be parameterized later.
    // Index offset in state vector is 13.
    for (int k = 0; k < N_ + 1; ++k) {
        int ws_idx = state_idx_start + k * nx_ + 13;
        // Apply limit to 3 wheels
        l_.segment(ws_idx, 3).setConstant(-600.0); // Rad/s
        u_.segment(ws_idx, 3).setConstant(600.0);
    }
    
    // 3c. Path parameter bounds (index 16) - MPCC mode
    if (mpc_params_.mode_path_following) {
        for (int k = 0; k < N_ + 1; ++k) {
            int s_idx = state_idx_start + k * nx_ + 16;
            // s must be within [0, L]
            // We use a slightly relaxed lower bound to handle start-up noise
            l_(s_idx) = -0.5; 
            // Upper bound is path length + margin
            // If path_total_length_ is 0 (not set yet), use large number
            double L = (path_total_length_ > 0) ? path_total_length_ : 100000.0;
            u_(s_idx) = L + 2.0; // Allow slight overshoot for stability
        }
    }
    
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
    
    // Update B matrix entries (physics-only block)
    int nx_phys = mpc_params_.mode_path_following ? 16 : nx_;
    int nu_phys = mpc_params_.mode_path_following ? nu_ - 1 : nu_;
    for (int r = 0; r < nx_phys; ++r) {
        for (int c = 0; c < nu_phys; ++c) {
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

    if (mpc_params_.mode_path_following) {
        q_.setZero();
        osqp_update_lin_cost(work_, q_.data());
        return;
    }
    
    // Update linear cost vector q = -Q * x_ref
    for (int k = 0; k < N_; ++k) {
        q_.segment(k * nx_, nx_) = -Q_diag_.cwiseProduct(x_target);
    }
    q_.segment(N_ * nx_, nx_) = -Q_term.cwiseProduct(x_target);
    
    osqp_update_lin_cost(work_, q_.data());
}

void MPCControllerCpp::update_cost_trajectory(const MatrixXd& x_target_traj) {
    VectorXd Q_term = Q_diag_ * 10.0;
    if (mpc_params_.mode_path_following) {
        q_.setZero();
        osqp_update_lin_cost(work_, q_.data());
        return;
    }
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
    
    VectorXd x_curr_aug = x_current;
    
    // Path Following State Augmentation
    if (mpc_params_.mode_path_following) {
        // If x_current is 16-dimensional (external), we need to append s.
        // Where do we get s from? 
        // The external python code should manage s and pass it in?
        // OR we maintain s internally?
        // Current implementation plan says Python wrapper handles s and passes it.
        // So x_current SHOULD be 17-dim if mode is on.
        // However, standard interface might pass 16-dim physics state.
        // Let's assume x_current is already correct size (17) OR we handle appending if not.
        if (x_current.size() == 16) {
             // Append internal guess or 0? 
             // Ideally Python sends 17. Safe to assume Python sends correct size.
             // If not, we error out or handle it.
             // Let's check size.
        }
        
        // Update s_guess mechanism (Warm Start)
        // Shift s_guess_: s[0] = s[1], s[N] = s[N-1] + v*dt
        if (s_guess_.size() == N_ + 1) {
            std::rotate(s_guess_.begin(), s_guess_.begin() + 1, s_guess_.end());
            s_guess_.back() += mpc_params_.v_target * dt_;
            // Correct the start to match measured s?
            // "Feedback": s_guess_[0] should roughly match x_current(16).
            // But usually we trust the prediction shift.
            // Let's re-anchor to current state to prevent drift?
            double s_curr = x_current(16);
            double delta = s_curr - s_guess_[0]; // diff
            // Shift entire guess by delta?
            // for(auto& val : s_guess_) val += delta;
            // Actually, simply re-predicting from s_curr is robustest for simple integrators.
            for(int k=0; k<=N_; ++k) s_guess_[k] = s_curr + k * mpc_params_.v_target * dt_; 
        }
    }

    VectorXd x_targ = apply_z_tilt(x_curr_aug, x_target);
    
    // Successive Linearization: Update dynamics if quaternion changed significantly
    Eigen::Vector4d quat = x_curr_aug.segment<4>(3);
    if ((quat - prev_quat_).norm() > 0.001) {
        update_dynamics(x_curr_aug);
        prev_quat_ = quat;
    }
    
    update_cost(x_targ); // Sets Q cost (will be overwritten for path terms)
    update_path_cost(x_curr_aug); // Overwrites P/q for path terms
    update_obstacle_constraints(x_curr_aug, x_target);
    update_constraints(x_curr_aug);
    
    osqp_solve(work_);
    
    auto end = std::chrono::high_resolution_clock::now();
    result.solve_time = std::chrono::duration<double>(end - start).count();
    
    if (work_->info->status_val != OSQP_SOLVED && 
        work_->info->status_val != OSQP_SOLVED_INACCURATE) {
        result.status = -1; // Error
        result.u = VectorXd::Zero(nu_); // Note: this is augmented nu size
        return result;
    }
    
    // Extract control from solution
    int u_idx = (N_ + 1) * nx_;
    result.u = Eigen::Map<VectorXd>(work_->solution->x + u_idx, nu_);
    
    // Extract predicted s trajectory for next step (optional, for better linearization)
    if (mpc_params_.mode_path_following) {
        // Update s_guess_ from solution
        for(int k=0; k<=N_; ++k) {
            s_guess_[k] = work_->solution->x[k * nx_ + 16];
        }
    }
    
    // Extract control from solution
    // Extract control from solution
    u_idx = (N_ + 1) * nx_;
    result.u = Eigen::Map<VectorXd>(work_->solution->x + u_idx, nu_);
    
    // Clip to bounds (safety)
    result.u = result.u.cwiseMax(control_lower_).cwiseMin(control_upper_);
    result.status = 1; // Success

    // --- Velocity Governor Safety Check (Post-Solve) ---
    // If strict velocity limit is enabled and we are overspeeding,
    // prevent any thrust that increases velocity in the direction of motion.
    // DISABLED in MPCC mode - path following manages speed via progress cost
    if (mpc_params_.max_velocity > 0 && !mpc_params_.mode_path_following) {
        Eigen::Vector3d v = x_current.segment<3>(7);
        double speed = v.norm();
        
        if (speed > mpc_params_.max_velocity) {
            // Check if control action adds to velocity
            // We need to map thruster forces to world frame to check this precisely,
            // or just conservatively cut thrust if overspeed.
            // Matching the python implementation:
            // "Rotate body-frame force into world frame to compare with world velocity"
            
            // Reconstruct net force in body frame
            Eigen::Vector3d net_force_body = Eigen::Vector3d::Zero();
            // Thruster part of u is at the end? nu = num_rw + num_thrusters
            int num_thr = sat_params_.num_thrusters;
            int num_rw = sat_params_.num_rw;
            
            if (result.u.size() >= num_rw + num_thr) {
                VectorXd thrust_cmds = result.u.tail(num_thr);
                
                for (int i = 0; i < num_thr; ++i) {
                     if (thrust_cmds(i) > 1e-4) {
                         // Direction in params
                         if (i < (int)sat_params_.thruster_directions.size()) {
                            // Assuming max_thrust is stored per thruster or we use 
                            // a simplified check. The Python code used physics config.
                            // Here we have sat_params_.thruster_forces
                            double f_mag = sat_params_.thruster_forces[i];
                            Eigen::Vector3d f_dir = sat_params_.thruster_directions[i];
                            net_force_body += f_dir * (f_mag * thrust_cmds(i));
                         }
                     }
                }
                
                // Rotate to world frame
                Eigen::Vector4d q = x_current.segment<4>(3);
                // Rotate vector by quaternion: v' = q * v * q_conj
                // Helper:
                Eigen::Vector3d t = 2.0 * q.tail<3>().cross(net_force_body);
                Eigen::Vector3d net_force_world = net_force_body + q(0) * t + q.tail<3>().cross(t);
                
                // Check dot product
                if (net_force_world.dot(v) > 0) {
                     // Force contributes to velocity -> Kill thrusters
                     result.u.tail(num_thr).setZero();
                     // std::cout << "[CPP GOVERNOR] Overspeed protection activated. Thrust cut." << std::endl;
                }
            }
        }
    }
    
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
    if (mpc_params_.mode_path_following) {
        update_path_cost(x_current);
    }
    if (x_target_traj.rows() > 0) {
        update_obstacle_constraints(x_current, x_target_traj.row(0).transpose());
    } else {
        update_obstacle_constraints(x_current, x_current);
    }
    update_constraints(x_current);

    // Update path constraints if valid (Runtime Update)
    if (mpc_params_.mode_path_following && path_data_valid_) {
         int state_idx_start = N_ * nx_ + nx_; // after dyn and init
         for (int k = 0; k < N_ + 1; ++k) {
             int s_idx = state_idx_start + k * nx_ + 16;
             // Update upper bound strictly
             double L = (path_total_length_ > 0) ? path_total_length_ : 100000.0;
             u_(s_idx) = L + 0.5;
         }
         osqp_update_bounds(work_, l_.data(), u_.data());
    }

    osqp_solve(work_);

    if (work_->info->status_val != OSQP_SOLVED) {
        result.status = work_->info->status_val;
        result.u = VectorXd::Zero(nu_);
    } else {
        result.status = work_->info->status_val;
        result.u = VectorXd::Map(work_->solution->x + (N_ + 1) * nx_, nu_);
    }
    
    result.u = result.u.cwiseMax(control_lower_).cwiseMin(control_upper_);

    // --- Velocity Governor Safety Check (Post-Solve) ---
    // DISABLED in MPCC mode - path following manages speed via progress cost
    if (mpc_params_.max_velocity > 0 && !mpc_params_.mode_path_following) {
        Eigen::Vector3d v = x_current.segment<3>(7);
        double speed = v.norm();
        
        if (speed > mpc_params_.max_velocity) {
            Eigen::Vector3d net_force_body = Eigen::Vector3d::Zero();
            int num_thr = sat_params_.num_thrusters;
            int num_rw = sat_params_.num_rw;
            
            if (result.u.size() >= num_rw + num_thr) {
                VectorXd thrust_cmds = result.u.tail(num_thr);
                for (int i = 0; i < num_thr; ++i) {
                     if (thrust_cmds(i) > 1e-4 && i < (int)sat_params_.thruster_directions.size()) {
                        double f_mag = sat_params_.thruster_forces[i];
                        Eigen::Vector3d f_dir = sat_params_.thruster_directions[i];
                        net_force_body += f_dir * (f_mag * thrust_cmds(i));
                     }
                }
                Eigen::Vector4d q = x_current.segment<4>(3);
                Eigen::Vector3d t = 2.0 * q.tail<3>().cross(net_force_body);
                Eigen::Vector3d net_force_world = net_force_body + q(0) * t + q.tail<3>().cross(t);
                
                if (net_force_world.dot(v) > 0) {
                     result.u.tail(num_thr).setZero();
                }
            }
        }
    }

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

void MPCControllerCpp::update_path_cost(const VectorXd& x_current) {
    if (!mpc_params_.mode_path_following) return;
    q_.setZero();
    if (!path_data_valid_) {
        // No path data - cannot compute path-following cost
        osqp_update_lin_cost(work_, q_.data());
        return;
    }

    // ==========================================================================
    // General Path-Following MPC (MPCC) Cost Update - V4.0.1
    // ==========================================================================
    //
    // Cost function:
    //   J = Σ [ Q_contour * ||p - p(s)||²      (Contouring: stay on path)
    //         + Q_progress * (v_s - v_target)² (Progress: track target speed)
    //         + R * ||u||² ]                   (Control effort)
    //
    // State augmentation:
    //   x = [p(3), q(4), v(3), w(3), s(1)]  (17 states)
    //   u = [τ_rw(3), f(6), v_s(1)]         (10 controls: 3 RW + 6 thrusters + 1 path vel)
    //
    // Linearization around s_bar:
    //   p(s) ≈ p(s_bar) + t(s_bar) * (s - s_bar)
    //   where t = dp/ds is the unit tangent
    // ==========================================================================

    double Q_c = mpc_params_.Q_contour;   // Contouring weight
    double Q_p = mpc_params_.Q_progress;  // Progress weight
    double Q_v = mpc_params_.Q_vel;       // Velocity alignment weight
    double v_ref = mpc_params_.v_target;  // Target path speed

    // Initialize s_guess if needed
    if (s_guess_.size() != static_cast<size_t>(N_ + 1)) {
        s_guess_.resize(N_ + 1);
        double s0 = (nx_ == 17 && x_current.size() >= 17) ? x_current(16) : 0.0;
        for (int k = 0; k <= N_; ++k) {
            s_guess_[k] = std::min(s0 + k * v_ref * dt_, path_total_length_);
        }
    }

    for (int k = 0; k <= N_; ++k) {
        double s_bar = s_guess_[k];
        double stage_scale = (k == N_) ? 10.0 : 1.0;
        double Q_c_k = Q_c * stage_scale;
        double Q_v_k = Q_v * stage_scale;
        
        // Clamp s_bar to valid range
        s_bar = std::max(0.0, std::min(s_bar, path_total_length_));
        
        // Get path reference point and tangent at s_bar
        Eigen::Vector3d p_ref = get_path_point(s_bar);
        Eigen::Vector3d t_ref = get_path_tangent(s_bar); // Unit tangent
        
        // Linearized contouring error: e = p - p(s) ≈ p - (p_ref + t*(s - s_bar))
        //                              e = p - t*s - (p_ref - t*s_bar)
        // Let C = p_ref - t*s_bar  (constant for this step)
        // Then e = p - t*s - C
        //
        // Cost: ||e||² = (p - t*s - C)ᵀ(p - t*s - C)
        //
        // Expand: p'p - 2p't*s - 2p'C + t't*s² + 2t'C*s + C'C
        //
        // In QP form (OSQP: 0.5 x'Px + q'x):
        //   P terms: 2*Q_c*I (for p), 2*Q_c*|t|² (for s), -2*Q_c*t (cross p,s)
        //   q terms: -2*Q_c*C (for p), 2*Q_c*(t·C) (for s)
        
        Eigen::Vector3d C = p_ref - t_ref * s_bar;
        double t_norm_sq = t_ref.squaredNorm(); // Should be ~1 for unit tangent
        double t_dot_C = t_ref.dot(C);
        
        int x_idx = k * nx_;  // State index for this step
        
        // 1. Update P matrix cross-terms (p, s)
        // These are stored at path_P_indices_[k][0..2]
        // P_ps = -2 * Q_c * t (off-diagonal coupling position with s)
        for (int i = 0; i < 3; ++i) {
            double val = -2.0 * Q_c_k * t_ref(i);
            P_data_[path_P_indices_[k][i]] = val;
        }

        // 1b. Update s diagonal entry
        if (k < static_cast<int>(path_s_diag_indices_.size())) {
            P_data_[path_s_diag_indices_[k]] = 2.0 * Q_c_k * t_norm_sq;
        }
        
        // 2. Update q vector (linear costs)
        // q_p = -2 * Q_c * C  (attracts position toward path)
        for (int i = 0; i < 3; ++i) {
            q_(x_idx + i) = -2.0 * Q_c_k * C(i);
        }
        
        // q_s = 2 * Q_c * (t·C) (pushes s forward)
        q_(x_idx + 16) = 2.0 * Q_c_k * t_dot_C;

        // 2b. Velocity alignment cost: penalize velocity orthogonal to tangent
        // ||v - t(t^T v)||^2 = v^T (I - t t^T) v
        Eigen::Matrix3d M = Eigen::Matrix3d::Identity() - t_ref * t_ref.transpose();
        double vel_scale = 2.0 * Q_v_k;
        if (k < static_cast<int>(path_vel_P_indices_.size())) {
            const auto& vel_idx = path_vel_P_indices_[k];
            if (vel_idx.size() == 6) {
                P_data_[vel_idx[0]] = vel_scale * M(0, 0);
                P_data_[vel_idx[1]] = vel_scale * M(0, 1);
                P_data_[vel_idx[2]] = vel_scale * M(0, 2);
                P_data_[vel_idx[3]] = vel_scale * M(1, 1);
                P_data_[vel_idx[4]] = vel_scale * M(1, 2);
                P_data_[vel_idx[5]] = vel_scale * M(2, 2);
            }
        }
        
        // 3. Progress tracking: (v_s - v_ref)² on control side
        // v_s is the last control in u (index nu_-1)
        // Cost: Q_p * (v_s - v_ref)² = Q_p * v_s² - 2*Q_p*v_ref*v_s + const
        // Linear term: -2 * Q_p * v_ref
        if (k < N_) {
            int u_idx = (N_ + 1) * nx_ + k * nu_ + (nu_ - 1);
            q_(u_idx) = -2.0 * Q_p * v_ref;
        }
    }
    
    // Push updates to OSQP
    osqp_update_P(work_, P_data_.data(), OSQP_NULL, P_.nonZeros());
    osqp_update_lin_cost(work_, q_.data());
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

// ============================================================================
// Path Following: General Path Support (V4.0.1)
// ============================================================================

void MPCControllerCpp::set_path_data(const std::vector<std::array<double, 4>>& path_data) {
    path_s_.clear();
    path_points_.clear();
    
    if (path_data.empty()) {
        path_data_valid_ = false;
        path_total_length_ = 0.0;
        return;
    }
    
    path_s_.reserve(path_data.size());
    path_points_.reserve(path_data.size());
    
    for (const auto& pt : path_data) {
        path_s_.push_back(pt[0]);  // s (arc-length parameter)
        path_points_.push_back(Eigen::Vector3d(pt[1], pt[2], pt[3]));  // x, y, z
    }
    
    path_total_length_ = path_s_.back();
    path_data_valid_ = true;
    
    // Reset s guess when path changes
    s_guess_.clear();
}

Eigen::Vector3d MPCControllerCpp::get_path_point(double s) const {
    if (!path_data_valid_ || path_s_.empty()) {
        // Fallback: return origin
        return Eigen::Vector3d::Zero();
    }
    
    // Clamp s to valid range
    if (s <= path_s_.front()) {
        return path_points_.front();
    }
    if (s >= path_s_.back()) {
        return path_points_.back();
    }
    
    // Binary search for segment
    auto it = std::lower_bound(path_s_.begin(), path_s_.end(), s);
    int idx = std::distance(path_s_.begin(), it);
    if (idx == 0) idx = 1;
    
    // Linear interpolation within segment
    double s0 = path_s_[idx - 1];
    double s1 = path_s_[idx];
    double t = (s - s0) / (s1 - s0 + 1e-12);
    
    return path_points_[idx - 1] + t * (path_points_[idx] - path_points_[idx - 1]);
}

Eigen::Vector3d MPCControllerCpp::get_path_tangent(double s) const {
    if (!path_data_valid_ || path_s_.size() < 2) {
        // Fallback: unit X direction
        return Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    
    // Clamp s to valid range
    double s_clamped = std::max(path_s_.front(), std::min(s, path_s_.back()));
    
    // Binary search for segment
    auto it = std::lower_bound(path_s_.begin(), path_s_.end(), s_clamped);
    int idx = std::distance(path_s_.begin(), it);
    if (idx == 0) idx = 1;
    if (idx >= static_cast<int>(path_s_.size())) idx = path_s_.size() - 1;
    
    // Tangent is direction between adjacent points
    Eigen::Vector3d diff = path_points_[idx] - path_points_[idx - 1];
    double len = diff.norm();
    if (len < 1e-12) {
        return Eigen::Vector3d(1.0, 0.0, 0.0);  // Degenerate case
    }
    
    return diff / len;  // Normalized tangent
}

} // namespace satellite_control

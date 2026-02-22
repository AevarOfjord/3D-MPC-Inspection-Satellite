
#include "mpc_controller.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <unordered_set>

namespace satellite_control {

MPCControllerCpp::MPCControllerCpp(const SatelliteParams& sat_params, const MPCParams& mpc_params)
    : sat_params_(sat_params), mpc_params_(mpc_params) {

    // Path Following State Augmentation
    // Path Following Configuration (Default)
    nx_ = 17; // 13 base + 3 wheel speeds + 1 path param (s)

    N_ = mpc_params_.prediction_horizon;
    dt_ = mpc_params_.dt;
    control_horizon_ = std::max(1, std::min(mpc_params_.control_horizon, N_));
    base_control_horizon_ = control_horizon_;
    base_q_smooth_ = std::max(0.0, mpc_params_.Q_smooth);
    base_thrust_pair_weight_ = std::max(0.0, mpc_params_.thrust_pair_weight);
    active_smoothness_scale_ = 1.0;
    active_thruster_pair_scale_ = 1.0;
    active_gyro_jacobian_ = mpc_params_.enable_gyro_jacobian;

    // Control: RW + Thrusters + 1 virtual path velocity v_s
    nu_ = sat_params_.num_rw + sat_params_.num_thrusters + 1;

    // Create linearizer
    linearizer_ = std::make_unique<Linearizer>(
        sat_params_,
        mpc_params_.enable_gyro_jacobian
    );

    // Precompute weight vectors
    const double Q_att_total = mpc_params_.Q_attitude + mpc_params_.Q_axis_align;
    Q_diag_.resize(nx_);
    // Path-following: contouring handles position, velocity alignment handled in update_path_cost.
    // Disable quaternion tracking to avoid penalizing unit quaternion magnitude.
    Q_diag_ << VectorXd::Constant(3, 2.0 * mpc_params_.Q_contour),
               VectorXd::Constant(4, 2.0 * Q_att_total),
               VectorXd::Constant(3, 0.0),
               VectorXd::Constant(3, mpc_params_.Q_angvel),
               VectorXd::Constant(3, 0.0), // Wheel speeds
               VectorXd::Constant(1, 0.0); // s diagonal updated per-step

    R_diag_.resize(nu_);
    R_diag_ << VectorXd::Constant(sat_params_.num_rw, mpc_params_.R_rw_torque),
               VectorXd::Constant(sat_params_.num_thrusters, mpc_params_.R_thrust),
               VectorXd::Constant(1, mpc_params_.Q_progress);
    R_diag_ = R_diag_.cwiseMax(1e-6);

    // Control bounds
    control_lower_.resize(nu_);
    control_upper_.resize(nu_);

    double v_s_min = std::max(0.0, mpc_params_.path_speed_min);
    double v_s_max = mpc_params_.path_speed_max;
    if (v_s_max <= 0.0) {
        v_s_max = mpc_params_.path_speed;
    }
    if (v_s_min > v_s_max) {
        std::swap(v_s_min, v_s_max);
    }
    control_lower_ << VectorXd::Constant(sat_params_.num_rw, -1.0),
                      VectorXd::Zero(sat_params_.num_thrusters),
                      VectorXd::Constant(1, v_s_min); // v_s >= min speed
    control_upper_ << VectorXd::Constant(sat_params_.num_rw, 1.0),
                      VectorXd::Ones(sat_params_.num_thrusters),
                      VectorXd::Constant(1, v_s_max); // v_s max bound

    // Derive practical state bounds only when requested.
    max_linear_velocity_bound_ =
        (mpc_params_.max_linear_velocity > 0.0) ? mpc_params_.max_linear_velocity : 0.0;
    max_angular_velocity_bound_ =
        (mpc_params_.max_angular_velocity > 0.0) ? mpc_params_.max_angular_velocity : 0.0;

    if (mpc_params_.enable_auto_state_bounds) {
        if (max_linear_velocity_bound_ <= 0.0) {
            double total_thruster_force = 0.0;
            for (double f : sat_params_.thruster_forces) {
                total_thruster_force += std::max(0.0, f);
            }
            double accel_linear = 0.0;
            if (sat_params_.mass > 1e-9) {
                accel_linear = total_thruster_force / sat_params_.mass;
            }
            double auto_v = std::max(
                0.25,
                std::max(
                    mpc_params_.path_speed_max * 4.0,
                    accel_linear * dt_ * static_cast<double>(N_)
                )
            );
            max_linear_velocity_bound_ = auto_v;
        }

        if (max_angular_velocity_bound_ <= 0.0) {
            double torque_total = 0.0;
            for (double t : sat_params_.rw_torque_limits) {
                torque_total += std::max(0.0, t);
            }
            for (size_t i = 0; i < sat_params_.thruster_positions.size() &&
                              i < sat_params_.thruster_directions.size() &&
                              i < sat_params_.thruster_forces.size(); ++i) {
                Eigen::Vector3d r = sat_params_.thruster_positions[i] - sat_params_.com_offset;
                Eigen::Vector3d f = sat_params_.thruster_forces[i] * sat_params_.thruster_directions[i];
                torque_total += r.cross(f).norm();
            }
            double inertia_min = std::min(
                sat_params_.inertia.x(),
                std::min(sat_params_.inertia.y(), sat_params_.inertia.z())
            );
            double accel_angular = (inertia_min > 1e-9) ? (torque_total / inertia_min) : 0.0;
            double auto_w = std::max(0.5, accel_angular * dt_ * static_cast<double>(N_));
            max_angular_velocity_bound_ = auto_w;
        }
    }

    // Initialize path following state
    s_guess_.resize(N_ + 1, 0.0);
    last_feasible_control_ = VectorXd::Zero(nu_);
    initialize_variable_scaling();

    // Initialize solver
    init_solver();

    dyn_affine_ = VectorXd::Zero(nx_);
}

MPCControllerCpp::~MPCControllerCpp() {
    cleanup_solver();
}

// ----------------------------------------------------------------------------
// Initialization & Helpers
// ----------------------------------------------------------------------------

void MPCControllerCpp::cleanup_solver() {
    if (work_) {
        osqp_cleanup(work_);
        work_ = nullptr;
    }
    if (data_) {
        if (data_->P) {
            c_free(data_->P);
        }
        if (data_->A) {
            c_free(data_->A);
        }
        c_free(data_);
        data_ = nullptr;
    }
    if (settings_) {
        c_free(settings_);
        settings_ = nullptr;
    }
}

void MPCControllerCpp::rebuild_solver() {
    cleanup_solver();
    initialize_variable_scaling();
    init_solver();
}

void MPCControllerCpp::initialize_variable_scaling() {
    state_var_scale_ = VectorXd::Ones(nx_);
    control_var_scale_ = VectorXd::Ones(nu_);
    if (!mpc_params_.enable_variable_scaling) {
        return;
    }

    const double pos_scale = std::max(1.0, 2.0 * mpc_params_.path_speed * dt_ * N_);
    state_var_scale_.segment(0, 3).setConstant(pos_scale);
    state_var_scale_.segment(3, 4).setConstant(1.0);

    double vel_scale = std::max(
        0.2,
        std::max(
            max_linear_velocity_bound_,
            std::max(mpc_params_.path_speed, mpc_params_.path_speed_max)
        )
    );
    state_var_scale_.segment(7, 3).setConstant(vel_scale);

    double ang_scale = std::max(0.2, max_angular_velocity_bound_ > 0.0 ? max_angular_velocity_bound_ : 1.0);
    state_var_scale_.segment(10, 3).setConstant(ang_scale);

    for (int i = 0; i < 3; ++i) {
        double ws = 600.0;
        if (i < static_cast<int>(sat_params_.rw_speed_limits.size()) &&
            sat_params_.rw_speed_limits[i] > 0.0) {
            ws = sat_params_.rw_speed_limits[i];
        }
        state_var_scale_(13 + i) = std::max(1.0, ws);
    }
    state_var_scale_(16) = std::max(1.0, mpc_params_.path_speed * dt_ * N_);

    control_var_scale_.segment(0, sat_params_.num_rw).setConstant(1.0);
    control_var_scale_.segment(sat_params_.num_rw, sat_params_.num_thrusters).setConstant(1.0);
    control_var_scale_(nu_ - 1) = std::max(0.05, control_upper_(nu_ - 1));
}

double MPCControllerCpp::decision_var_scale(int decision_index) const {
    if (!mpc_params_.enable_variable_scaling) {
        return 1.0;
    }
    int state_span = (N_ + 1) * nx_;
    if (decision_index < state_span) {
        int local = decision_index % nx_;
        return state_var_scale_(local);
    }
    int u_local = (decision_index - state_span) % nu_;
    return control_var_scale_(u_local);
}

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
    if (linearizer_) {
        linearizer_->set_freeze_target(true);
    }
    build_A_matrix(A_triplets);
    if (linearizer_) {
        linearizer_->set_freeze_target(false);
    }

    // Count constraints based on structure:
    // Dynamics (N*nx) + Initial (nx) + State Bounds ((N+1)*nx) + Control Bounds (N*nu)
    // + Control Horizon ((N-M)*nu)
    n_dyn_ = N_ * nx_;
    n_init_ = nx_;
    n_bounds_x_ = (N_ + 1) * nx_;
    n_bounds_u_ = N_ * nu_;
    n_control_horizon_constraints_ = (control_horizon_ < N_) ? (N_ - control_horizon_) * nu_ : 0;
    int n_constraints = n_dyn_ + n_init_ + n_bounds_x_ + n_bounds_u_ + n_control_horizon_constraints_;

    A_.resize(n_constraints, n_vars);
    A_.setFromTriplets(A_triplets.begin(), A_triplets.end());
    A_.makeCompressed();

    // Copy to OSQP flat arrays
    P_data_.assign(P_.valuePtr(), P_.valuePtr() + P_.nonZeros());
    P_indices_.assign(P_.innerIndexPtr(), P_.innerIndexPtr() + P_.nonZeros());
    P_indptr_.assign(P_.outerIndexPtr(), P_.outerIndexPtr() + P_.outerSize() + 1);

    A_data_.assign(A_.valuePtr(), A_.valuePtr() + A_.nonZeros());
    A_indices_.assign(A_.innerIndexPtr(), A_.innerIndexPtr() + A_.nonZeros());
    A_indptr_.assign(A_.outerIndexPtr(), A_.outerIndexPtr() + A_.outerSize() + 1);

    // B. Actuator dynamics index map (B matrix updates)
    B_idx_map_.resize(N_);
    for (int k = 0; k < N_; ++k) {
        B_idx_map_[k].resize(nx_ * nu_);
        for (auto &v : B_idx_map_[k]) {
            v.clear();
        }
    }
    int u_start_idx = (N_ + 1) * nx_;
    for (int k = 0; k < N_; ++k) {
        int current_row_base = k * nx_;
        for (int r = 0; r < nx_; ++r) {
            for (int c = 0; c < nu_; ++c) {
                int col = u_start_idx + k * nu_ + c;
                // Find index in CSC format
                for (int idx = A_.outerIndexPtr()[col]; idx < A_.outerIndexPtr()[col + 1]; ++idx) {
                    if (A_.innerIndexPtr()[idx] == current_row_base + r) {
                        B_idx_map_[k][r * nu_ + c].push_back(idx);
                    }
                }
            }
        }
    }

    // C. Quaternion dynamics index map (A matrix updates for G-block)
    // A_dyn rows 3-6, cols 10-12
    A_idx_map_.resize(N_);
    for (int k = 0; k < N_; ++k) {
        A_idx_map_[k].resize(4 * 3);
        for (auto &v : A_idx_map_[k]) {
            v.clear();
        }
    }
    for (int k = 0; k < N_; ++k) {
        int row_base = k * nx_;
        int col_base = k * nx_;

        for (int qr = 0; qr < 4; ++qr) {      // Relative rows 3-6
            for (int oc = 0; oc < 3; ++oc) {  // Relative cols 10-12
                int row = row_base + 3 + qr;
                int col = col_base + 10 + oc;

                for (int idx = A_.outerIndexPtr()[col]; idx < A_.outerIndexPtr()[col + 1]; ++idx) {
                    if (A_.innerIndexPtr()[idx] == row) {
                        A_idx_map_[k][qr * 3 + oc].push_back(idx);
                    }
                }
            }
        }
    }

    // D. Orbital/velocity dynamics index map (rows 7-9, cols 0-9)
    A_orbital_idx_map_.resize(N_);
    for (int k = 0; k < N_; ++k) {
        A_orbital_idx_map_[k].resize(3 * 10);
        for (auto &v : A_orbital_idx_map_[k]) {
            v.clear();
        }
    }
    for (int k = 0; k < N_; ++k) {
        int row_base = k * nx_;
        int col_base = k * nx_;

        for (int vr = 0; vr < 3; ++vr) {      // Relative rows 7-9
            for (int vc = 0; vc < 10; ++vc) { // Relative cols 0-9
                int row = row_base + 7 + vr;
                int col = col_base + vc;

                for (int idx = A_.outerIndexPtr()[col]; idx < A_.outerIndexPtr()[col + 1]; ++idx) {
                    if (A_.innerIndexPtr()[idx] == row) {
                        A_orbital_idx_map_[k][vr * 10 + vc].push_back(idx);
                    }
                }
            }
        }
    }

    // E. Angular Velocity dynamics index map (rows 10-12, cols 10-12)
    A_angvel_idx_map_.resize(N_);
    for (int k = 0; k < N_; ++k) {
        A_angvel_idx_map_[k].resize(3 * 3);
        for (auto &v : A_angvel_idx_map_[k]) {
            v.clear();
        }
    }
    for (int k = 0; k < N_; ++k) {
        int row_base = k * nx_;
        int col_base = k * nx_;

        for (int wr = 0; wr < 3; ++wr) {      // Relative rows 10-12
            for (int wc = 0; wc < 3; ++wc) { // Relative cols 10-12
                int row = row_base + 10 + wr;
                int col = col_base + 10 + wc;

                for (int idx = A_.outerIndexPtr()[col]; idx < A_.outerIndexPtr()[col + 1]; ++idx) {
                    if (A_.innerIndexPtr()[idx] == row) {
                        A_angvel_idx_map_[k][wr * 3 + wc].push_back(idx);
                    }
                }
            }
        }
    }



    // F. Path Following P-matrix index map
    // We need to find the indices in P_data_ for the cross terms (0,16), (1,16), (2,16)
    path_P_indices_.resize(N_ + 1);
    path_s_diag_indices_.resize(N_ + 1);
    path_vel_P_indices_.resize(N_ + 1);
    path_pos_diag_indices_.resize(N_ + 1);
    path_pos_offdiag_indices_.resize(N_ + 1);
    path_att_diag_indices_.resize(N_ + 1);
    path_angvel_diag_indices_.resize(N_ + 1);
    path_vs_diag_indices_.assign(static_cast<size_t>(N_), -1);
    int s_offset = 16;
    int v_offset = 7;

    for (int k = 0; k <= N_; ++k) { // Include terminal
        path_P_indices_[k].resize(3);
        path_pos_diag_indices_[k].resize(3);
        path_pos_offdiag_indices_[k].resize(3);
        path_att_diag_indices_[k].assign(4, -1);
        path_angvel_diag_indices_[k].assign(3, -1);
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

        // Find position diagonal indices for x,y,z
        for (int i = 0; i < 3; ++i) {
            int diag_col = base_idx + i;
            int diag_row = base_idx + i;
            bool pos_found = false;
            for (int idx = P_.outerIndexPtr()[diag_col]; idx < P_.outerIndexPtr()[diag_col + 1]; ++idx) {
                if (P_.innerIndexPtr()[idx] == diag_row) {
                    path_pos_diag_indices_[k][i] = idx;
                    pos_found = true;
                    break;
                }
            }
            if (!pos_found) {
                std::cerr << "[MPC] Error: P-matrix position diagonal not found at k=" << k
                          << ", i=" << i << std::endl;
            }
        }

        // Find position off-diagonal indices for (x,y), (x,z), (y,z)
        struct PosPair { int row; int col; int slot; };
        std::array<PosPair, 3> pairs = {
            PosPair{base_idx + 0, base_idx + 1, 0},
            PosPair{base_idx + 0, base_idx + 2, 1},
            PosPair{base_idx + 1, base_idx + 2, 2},
        };
        for (const auto &pair : pairs) {
            bool off_found = false;
            for (int idx = P_.outerIndexPtr()[pair.col]; idx < P_.outerIndexPtr()[pair.col + 1]; ++idx) {
                if (P_.innerIndexPtr()[idx] == pair.row) {
                    path_pos_offdiag_indices_[k][pair.slot] = idx;
                    off_found = true;
                    break;
                }
            }
            if (!off_found) {
                std::cerr << "[MPC] Error: P-matrix position off-diagonal not found at k=" << k
                          << ", slot=" << pair.slot << std::endl;
            }
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

        // Find quaternion diagonal indices for attitude tracking updates.
        for (int qi = 0; qi < 4; ++qi) {
            int diag = base_idx + 3 + qi;
            bool q_found = false;
            for (int idx = P_.outerIndexPtr()[diag]; idx < P_.outerIndexPtr()[diag + 1]; ++idx) {
                if (P_.innerIndexPtr()[idx] == diag) {
                    path_att_diag_indices_[k][qi] = idx;
                    q_found = true;
                    break;
                }
            }
            if (!q_found) {
                std::cerr << "[MPC] Error: P-matrix attitude diagonal not found at k="
                          << k << ", qi=" << qi << std::endl;
            }
        }

        // Find angular-velocity diagonal indices for mode-dependent damping updates.
        for (int wi = 0; wi < 3; ++wi) {
            int diag = base_idx + 10 + wi;
            bool w_found = false;
            for (int idx = P_.outerIndexPtr()[diag]; idx < P_.outerIndexPtr()[diag + 1]; ++idx) {
                if (P_.innerIndexPtr()[idx] == diag) {
                    path_angvel_diag_indices_[k][wi] = idx;
                    w_found = true;
                    break;
                }
            }
            if (!w_found) {
                std::cerr << "[MPC] Error: P-matrix angular velocity diagonal not found at k="
                          << k << ", wi=" << wi << std::endl;
            }
        }

        // Find progress control diagonal index for v_s at stage k.
        if (k < N_) {
            int vs_idx = (N_ + 1) * nx_ + k * nu_ + (nu_ - 1);
            bool vs_found = false;
            for (int idx = P_.outerIndexPtr()[vs_idx]; idx < P_.outerIndexPtr()[vs_idx + 1]; ++idx) {
                if (P_.innerIndexPtr()[idx] == vs_idx) {
                    path_vs_diag_indices_[static_cast<size_t>(k)] = idx;
                    vs_found = true;
                    break;
                }
            }
            if (!vs_found) {
                std::cerr << "[MPC] Error: P-matrix progress control diagonal not found at k="
                          << k << std::endl;
            }
        }
    }

    // Cache indices for partial P updates (path-related entries only)
    {
        std::unordered_set<c_int> unique;
        auto add_index = [&](int idx) {
            if (idx >= 0) {
                unique.insert(static_cast<c_int>(idx));
            }
        };
        for (int k = 0; k <= N_; ++k) {
            for (int i = 0; i < 3; ++i) {
                add_index(path_P_indices_[k][i]);
                add_index(path_pos_diag_indices_[k][i]);
            }
            for (int idx : path_pos_offdiag_indices_[k]) {
                add_index(idx);
            }
            for (int idx : path_vel_P_indices_[k]) {
                add_index(idx);
            }
            for (int idx : path_att_diag_indices_[k]) {
                add_index(idx);
            }
            for (int idx : path_angvel_diag_indices_[k]) {
                add_index(idx);
            }
            add_index(path_s_diag_indices_[k]);
            if (k < N_) {
                add_index(path_vs_diag_indices_[static_cast<size_t>(k)]);
            }
        }

        path_P_update_indices_.assign(unique.begin(), unique.end());
        std::sort(path_P_update_indices_.begin(), path_P_update_indices_.end());
        path_P_update_values_.resize(path_P_update_indices_.size(), 0.0);
    }

    // G. Terminal-state diagonal indices (physics states only 0..15 at k=N)
    terminal_phys_diag_indices_.assign(16, -1);
    int terminal_base = N_ * nx_;
    for (int i = 0; i < 16; ++i) {
        int col = terminal_base + i;
        int row = terminal_base + i;
        for (int idx = P_.outerIndexPtr()[col]; idx < P_.outerIndexPtr()[col + 1]; ++idx) {
            if (P_.innerIndexPtr()[idx] == row) {
                terminal_phys_diag_indices_[i] = idx;
                break;
            }
        }
    }
    terminal_phys_update_indices_.clear();
    terminal_phys_update_values_.clear();
    for (int idx : terminal_phys_diag_indices_) {
        if (idx >= 0) {
            terminal_phys_update_indices_.push_back(static_cast<c_int>(idx));
            terminal_phys_update_values_.push_back(0.0);
        }
    }

    // 4. Setup Bounds and OSQP workspace
    setup_osqp_workspace(n_vars, n_constraints);
}

MatrixXd MPCControllerCpp::compute_dare_terminal_matrix(
    const VectorXd& Q_diag,
    const VectorXd& R_diag,
    const VectorXd& x_nominal_phys
) {
    int nx_phys = 16;
    int nu_phys = nu_ - 1; // exclude v_s virtual control

    VectorXd x_nom = x_nominal_phys;
    if (x_nom.size() != nx_phys) {
        x_nom = VectorXd::Zero(nx_phys);
    }
    auto [A_phys, B_phys] = linearizer_->linearize(x_nom);

    if (A_phys.rows() != nx_phys || A_phys.cols() != nx_phys ||
        B_phys.rows() != nx_phys || B_phys.cols() != nu_phys) {
        return Q_diag.head(nx_phys).asDiagonal();
    }

    // Extract physics portion of the weight vectors
    VectorXd Q_phys_vec = Q_diag.head(nx_phys);
    VectorXd R_phys_vec = R_diag.head(nu_phys);

    MatrixXd Q_phys = Q_phys_vec.asDiagonal();
    MatrixXd R_phys = R_phys_vec.asDiagonal();

    // Regularize R to ensure PD (v_s cost may be very small)
    R_phys.diagonal() = R_phys.diagonal().cwiseMax(1e-6);

    MatrixXd P = Q_phys;
    int max_iter = 500;
    double tol = 1e-4;

    for (int i = 0; i < max_iter; ++i) {
        MatrixXd S = R_phys + B_phys.transpose() * P * B_phys;
        // Riccati iteration: P_new = Q + A'PA - A'PB inv(S) B'PA
        MatrixXd K = S.llt().solve(B_phys.transpose() * P * A_phys);
        // DARE: P_next = Q + (A - B*K)'*P*(A - B*K) + K'*R*K
        MatrixXd AminBK = A_phys - B_phys * K;
        MatrixXd P_next = Q_phys + AminBK.transpose() * P * AminBK + K.transpose() * R_phys * K;

        if ((P_next - P).norm() < tol) {
            P = P_next;
            break;
        }
        P = P_next;
    }

    return P;
}

VectorXd MPCControllerCpp::compute_dare_terminal_cost(
    const VectorXd& Q_diag,
    const VectorXd& R_diag,
    const VectorXd& x_nominal_phys
) {
    int nx_phys = 16;
    MatrixXd P = compute_dare_terminal_matrix(Q_diag, R_diag, x_nominal_phys);
    VectorXd result = Q_diag * 10.0; // default fallback for augmented entries
    result.head(nx_phys) = P.diagonal();
    return result;
}

void MPCControllerCpp::build_P_matrix(std::vector<Eigen::Triplet<double>>& triplets, int n_vars) {
    VectorXd P_diag(n_vars);

    // Stage costs (0 to N-1)
    for (int k = 0; k < N_; ++k) {
        P_diag.segment(k * nx_, nx_) = Q_diag_;
    }
    // Terminal cost (N) - exact DARE solution diagonal
    MatrixXd P_dare_full = compute_dare_terminal_matrix(
        Q_diag_,
        R_diag_,
        VectorXd::Zero(16)
    );
    VectorXd P_dare_diag = compute_dare_terminal_cost(
        Q_diag_,
        R_diag_,
        VectorXd::Zero(16)
    );
    P_diag.segment(N_ * nx_, nx_) = P_dare_diag;

    // Control costs (0 to N-1)
    for (int k = 0; k < N_; ++k) {
        P_diag.segment((N_ + 1) * nx_ + k * nu_, nu_) = R_diag_; // Augmented nu handles vs cost
    }

    // Smoothness penalty: Σ ||u_k - u_{k-1}||^2 for k=1..N-1
    if (mpc_params_.Q_smooth > 0.0 && N_ > 1) {
        double w = mpc_params_.Q_smooth;
        int u0 = (N_ + 1) * nx_;
        for (int k = 1; k < N_; ++k) {
            int uk = u0 + k * nu_;
            int ukm1 = u0 + (k - 1) * nu_;
            for (int c = 0; c < nu_; ++c) {
                P_diag(uk + c) += 2.0 * w;
                P_diag(ukm1 + c) += 2.0 * w;
                if (mpc_params_.enable_delta_u_coupling) {
                    double s_i = decision_var_scale(ukm1 + c);
                    double s_j = decision_var_scale(uk + c);
                    triplets.emplace_back(ukm1 + c, uk + c, -2.0 * w * s_i * s_j);
                }
            }
        }
    }

    // Penalize opposing thruster co-firing: w * (u_i + u_j)^2 per axis.
    if (mpc_params_.thrust_pair_weight > 0.0 && sat_params_.num_thrusters >= 2) {
        double w = mpc_params_.thrust_pair_weight;
        int thruster_offset = sat_params_.num_rw;
        int pair_count = sat_params_.num_thrusters / 2;
        for (int k = 0; k < N_; ++k) {
            int base = (N_ + 1) * nx_ + k * nu_;
            for (int p = 0; p < pair_count; ++p) {
                int i = base + thruster_offset + 2 * p;
                int j = i + 1;
                if (j >= base + thruster_offset + sat_params_.num_thrusters) {
                    break;
                }
                P_diag(i) += 2.0 * w;
                P_diag(j) += 2.0 * w;
                double s_i = decision_var_scale(i);
                double s_j = decision_var_scale(j);
                triplets.emplace_back(i, j, 2.0 * w * s_i * s_j);
            }
        }
    }

    for (int i = 0; i < n_vars; ++i) {
        double s = decision_var_scale(i);
        triplets.emplace_back(i, i, P_diag(i) * s * s);
    }

    if (mpc_params_.terminal_cost_profile == "dense_terminal") {
        int terminal_base = N_ * nx_;
        for (int i = 0; i < 16; ++i) {
            for (int j = i + 1; j < 16; ++j) {
                double value = P_dare_full(i, j);
                if (std::abs(value) > 1e-12) {
                    int gi = terminal_base + i;
                    int gj = terminal_base + j;
                    double si = decision_var_scale(gi);
                    double sj = decision_var_scale(gj);
                    triplets.emplace_back(gi, gj, value * si * sj);
                }
            }
        }
    }

    // Path Following: Pre-allocate cross-terms for (x, s), (y, s), (z, s)
    // These correspond to the -2 * weight * r^T * t * s term in the cost expansion.
    // We keep explicit sparsity slots at (k*nx + {0,1,2}, k*nx + 16) for fast updates.
    {
        int s_offset = 16; // Index of s in state vector
        int v_offset = 7; // Index of velocity block in state vector
        for (int k = 0; k <= N_; ++k) { // Include terminal cost
            int base_idx = k * nx_;
            // Add entries for x, y, z cross s
            for (int i = 0; i < 3; ++i) {
                triplets.emplace_back(base_idx + i, base_idx + s_offset, 0.0);
            }
            // Add entries for position block off-diagonals (x,y), (x,z), (y,z)
            triplets.emplace_back(base_idx + 0, base_idx + 1, 0.0);
            triplets.emplace_back(base_idx + 0, base_idx + 2, 0.0);
            triplets.emplace_back(base_idx + 1, base_idx + 2, 0.0);
            // Add entries for velocity block (upper triangular) for tangent alignment cost
            for (int i = 0; i < 3; ++i) {
                for (int j = i; j < 3; ++j) {
                    triplets.emplace_back(base_idx + v_offset + i,
                                          base_idx + v_offset + j, 0.0);
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
    auto add_A = [&](int row, int col, double val) {
        triplets.emplace_back(row, col, val * decision_var_scale(col));
    };

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
                if (r == 16) {
                    if (c == 16) add_A(row_idx + r, x_k_idx + c, -1.0);
                    continue;
                }

                // Normal Physics Dynamics (0-15)
                // Linearizer returns 16x16, so checking bounds
                if (r < 16 && c < 16) {
                    // Force inclusion of G-block (rows 3-6, cols 10-12) for quaternion dynamics updates
                    bool is_g_block = (r >= 3 && r < 7 && c >= 10 && c < 13);
                    bool is_angvel_block = (r >= 10 && r < 13 && c >= 10 && c < 13);
                    if (is_g_block || is_angvel_block || std::abs(A_dyn(r, c)) > 1e-12) {
                        add_A(row_idx + r, x_k_idx + c, -A_dyn(r, c));
                    }
                }
            }
        }
        // +I block (x_{k+1})
        for (int r = 0; r < nx_; ++r) {
            add_A(row_idx + r, x_kp1_idx + r, 1.0);
        }
        // -B block
        for (int r = 0; r < nx_; ++r) {
            for (int c = 0; c < nu_; ++c) {
                // Special handling for Path Control (v_s)
                // v_s is the last control input.
                // Dynamics: -dt * v_s
                if (r == 16) {
                    if (c == nu_ - 1) { // Last control is v_s
                        add_A(row_idx + r, u_k_idx + c, -mpc_params_.dt);
                    }
                    continue;
                }

                // Normal Physics Limits
                // B_dyn is 16 rows, (nu-1) cols (if we ignore v_s)
                int nu_phys = nu_ - 1;

                if (r < 16 && c < nu_phys) {
                    // Force inclusion of velocity rows (7-12) for updates
                    bool is_velocity_row = (r >= 7);
                    if (is_velocity_row || std::abs(B_dyn(r, c)) > 1e-12) {
                        add_A(row_idx + r, u_k_idx + c, -B_dyn(r, c));
                    }
                }
            }
        }
        row_idx += nx_;
    }

    // 2. Initial state constraint: I*x_0 = x_current
    for (int r = 0; r < nx_; ++r) {
        add_A(row_idx + r, r, 1.0);
    }
    row_idx += nx_;

    // 3. State bounds: I*x_k
    for (int k = 0; k < N_ + 1; ++k) {
        for (int r = 0; r < nx_; ++r) {
            add_A(row_idx + r, k * nx_ + r, 1.0);
        }
        row_idx += nx_;
    }

    // 4. Control bounds: I*u_k
    for (int k = 0; k < N_; ++k) {
        int u_k_idx = (N_ + 1) * nx_ + k * nu_;
        for (int r = 0; r < nu_; ++r) {
            add_A(row_idx + r, u_k_idx + r, 1.0);
        }
        row_idx += nu_;
    }

    // 5. Control horizon constraints: u_k == u_{M-1} for k >= M
    if (control_horizon_ < N_) {
        int u_anchor_idx = (N_ + 1) * nx_ + (control_horizon_ - 1) * nu_;
        for (int k = control_horizon_; k < N_; ++k) {
            int u_k_idx = (N_ + 1) * nx_ + k * nu_;
            for (int r = 0; r < nu_; ++r) {
                add_A(row_idx + r, u_k_idx + r, 1.0);
                add_A(row_idx + r, u_anchor_idx + r, -1.0);
            }
            row_idx += nu_;
        }
    }


}

void MPCControllerCpp::setup_osqp_workspace(int n_vars, int n_constraints) {
    // Initialize bound vectors
    q_ = VectorXd::Zero(n_vars);
    l_ = VectorXd::Zero(n_constraints);
    u_ = VectorXd::Zero(n_constraints);

    int n_dyn = n_dyn_;
    int n_init = n_init_;
    int n_bounds_x = n_bounds_x_;
    int n_bounds_u = n_bounds_u_;
    const bool tube_mode = (mpc_params_.robustness_mode == "tube");
    const double tighten = tube_mode
        ? std::clamp(mpc_params_.constraint_tightening_scale, 0.0, 0.3)
        : 0.0;

    // 1. Dynamics equality (l=0, u=0) - default is 0

    // 2. Initial state (will be updated at valid step)

    // 3. State bounds (initialized to infinity)
    int state_idx_start = n_dyn + n_init;
    l_.segment(state_idx_start, n_bounds_x).setConstant(-1e20);
    u_.segment(state_idx_start, n_bounds_x).setConstant(1e20);

    // 3a. Velocity bounds (indices 7-9)
    if (max_linear_velocity_bound_ > 0.0) {
        double linear_limit = max_linear_velocity_bound_ * (1.0 - tighten);
        for (int k = 0; k < N_ + 1; ++k) {
            int vel_idx = state_idx_start + k * nx_ + 7;
            l_.segment(vel_idx, 3).setConstant(-linear_limit);
            u_.segment(vel_idx, 3).setConstant(linear_limit);
        }
    }

    // 3b. Angular velocity bounds (indices 10-12)
    if (max_angular_velocity_bound_ > 0.0) {
        double ang_limit = max_angular_velocity_bound_ * (1.0 - tighten);
        for (int k = 0; k < N_ + 1; ++k) {
            int w_idx = state_idx_start + k * nx_ + 10;
            l_.segment(w_idx, 3).setConstant(-ang_limit);
            u_.segment(w_idx, 3).setConstant(ang_limit);
        }
    }

    // 3c. Wheel speed limits (indices 13-15) - from config if available
    for (int k = 0; k < N_ + 1; ++k) {
        int ws_idx = state_idx_start + k * nx_ + 13;
        for (int i = 0; i < 3; ++i) {
            double limit = 600.0;
            if (i < static_cast<int>(sat_params_.rw_speed_limits.size())) {
                double cfg_limit = sat_params_.rw_speed_limits[i];
                if (cfg_limit > 0.0) {
                    limit = cfg_limit;
                }
            }
            l_(ws_idx + i) = -limit;
            u_(ws_idx + i) = limit;
        }
    }

    // 3d. Path parameter bounds (index 16) - MPCC mode
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

    // 4. Control bounds
    int ctrl_idx_start = n_dyn + n_init + n_bounds_x;
    ctrl_row_start_ = ctrl_idx_start;
    VectorXd control_lower_runtime = control_lower_;
    VectorXd control_upper_runtime = control_upper_;
    if (tighten > 0.0) {
        for (int i = 0; i < nu_; ++i) {
            double center = 0.5 * (control_lower_(i) + control_upper_(i));
            double half = 0.5 * (control_upper_(i) - control_lower_(i)) * (1.0 - tighten);
            control_lower_runtime(i) = center - half;
            control_upper_runtime(i) = center + half;
        }
    }
    for (int k = 0; k < N_; ++k) {
        l_.segment(ctrl_idx_start + k * nu_, nu_) = control_lower_runtime;
        u_.segment(ctrl_idx_start + k * nu_, nu_) = control_upper_runtime;
    }

    // 5. Control horizon constraints (initialized to equality 0)
    int ctrl_horizon_start = n_dyn + n_init + n_bounds_x + n_bounds_u;
    if (n_control_horizon_constraints_ > 0) {
        l_.segment(ctrl_horizon_start, n_control_horizon_constraints_).setZero();
        u_.segment(ctrl_horizon_start, n_control_horizon_constraints_).setZero();
    }

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
    settings_->verbose = mpc_params_.verbose_mpc ? 1 : 0;
    // Keep solver budget comfortably below control period to absorb update overhead.
    // This reduces tail-latency breaches in end-to-end MPC step timing.
    const double effective_time_limit = std::min(
        mpc_params_.solver_time_limit,
        std::max(1e-3, 0.70 * dt_)
    );
    settings_->time_limit = effective_time_limit;
    settings_->warm_start = 1;
    // Favor deterministic, low-jitter behavior for real-time MPC.
    // Adaptive rho can trigger occasional expensive refactorizations.
    settings_->adaptive_rho = 0;
    settings_->rho = 0.1;
    settings_->max_iter = 800;
    settings_->check_termination = 1;
    settings_->eps_abs = 1e-3;
    settings_->eps_rel = 1e-3;
    settings_->polish = 0;

    if (!mpc_params_.solver_type.empty() && mpc_params_.solver_type != "OSQP") {
        std::cerr << "[MPC] Warning: solver_type '" << mpc_params_.solver_type
                  << "' is not supported. Falling back to OSQP." << std::endl;
    }

    osqp_setup(&work_, data_, settings_);
}

// ----------------------------------------------------------------------------
// Runtime Updates
// ----------------------------------------------------------------------------

void MPCControllerCpp::update_dynamics(const std::vector<VectorXd>& x_traj) {
    bool linearize_per_step = (x_traj.size() > 0);
    dyn_affine_.setZero(nx_);

    for (int k = 0; k < N_; ++k) {
        VectorXd x_k = linearize_per_step ?
            (k < x_traj.size() ? x_traj[k] : x_traj.back()) :
            (x_traj.empty() ? VectorXd::Zero(nx_) : x_traj[0]);

        auto [A_dyn, B_dyn] = linearizer_->linearize(x_k);

        if (k == 0) {
            const VectorXd& affine = linearizer_->affine();
            if (affine.size() == 16) {
                dyn_affine_.segment(0, 16) = affine;
            }
        }

        // Update A-block entries (G matrix: dQuat/dOmega)
        // A_dyn rows 3-6, cols 10-12
        for (int qr = 0; qr < 4; ++qr) {
            for (int oc = 0; oc < 3; ++oc) {
                double val = -A_dyn(3 + qr, 10 + oc) * state_var_scale_(10 + oc); // Stored as -A
                for (int idx : A_idx_map_[k][qr * 3 + oc]) {
                    A_data_[idx] = val;
                }
            }
        }

        // Update orbital/velocity dynamics block (rows 7-9, cols 0-9)
        for (int vr = 0; vr < 3; ++vr) {
            for (int vc = 0; vc < 10; ++vc) {
                double val = -A_dyn(7 + vr, vc) * state_var_scale_(vc);
                for (int idx : A_orbital_idx_map_[k][vr * 10 + vc]) {
                    A_data_[idx] = val;
                }
            }
        }

        // Update angular velocity dynamics block (rows 10-12, cols 10-12)
        if (active_gyro_jacobian_) {
            for (int wr = 0; wr < 3; ++wr) {
                for (int wc = 0; wc < 3; ++wc) {
                    double val = -A_dyn(10 + wr, 10 + wc) * state_var_scale_(10 + wc);
                    for (int idx : A_angvel_idx_map_[k][wr * 3 + wc]) {
                        A_data_[idx] = val;
                    }
                }
            }
        }

        // Update B matrix entries (physics-only block)
        int nx_phys = 16;
        int nu_phys = nu_ - 1;
        for (int r = 0; r < nx_phys; ++r) {
            for (int c = 0; c < nu_phys; ++c) {
                double val = -B_dyn(r, c) * control_var_scale_(c);
                for (int idx : B_idx_map_[k][r * nu_ + c]) {
                    A_data_[idx] = val;
                }
            }
        }
    }

    A_dirty_ = true;
}

void MPCControllerCpp::update_cost() {
    // In MPCC mode, q vector is managed by update_path_cost.
    // Standard update_cost is a no-op or reset.
    q_.setZero();
    osqp_update_lin_cost(work_, q_.data());
}

void MPCControllerCpp::update_constraints(const VectorXd& x_current) {
    // Update dynamics affine term (if any) for all horizon steps
    if (dyn_affine_.size() == nx_) {
        for (int k = 0; k < N_; ++k) {
            l_.segment(k * nx_, nx_) = dyn_affine_;
            u_.segment(k * nx_, nx_) = dyn_affine_;
        }
    }

    // Update initial state constraint bounds (equality constraint)
    int init_start = n_dyn_;
    l_.segment(init_start, nx_) = x_current;
    u_.segment(init_start, nx_) = x_current;
    int state_idx_start = n_dyn_ + n_init_;

    // Dynamically relax progress lower bound near the path endpoint so MPC can stop.
    if (ctrl_row_start_ > 0 && nu_ > 0) {
        double s_curr = 0.0;
        if (x_current.size() >= 17) {
            s_curr = x_current(16);
        }
        const bool tube_mode = (mpc_params_.robustness_mode == "tube");
        const double tighten = tube_mode
            ? std::clamp(mpc_params_.constraint_tightening_scale, 0.0, 0.3)
            : 0.0;
        double base_lower_vs = control_lower_(nu_ - 1);
        double base_upper_vs = control_upper_(nu_ - 1);
        if (tighten > 0.0) {
            double center = 0.5 * (base_lower_vs + base_upper_vs);
            double half = 0.5 * (base_upper_vs - base_lower_vs) * (1.0 - tighten);
            base_lower_vs = center - half;
            base_upper_vs = center + half;
        }
        double v_s_min_dynamic = compute_dynamic_vs_min(s_curr);
        for (int k = 0; k < N_; ++k) {
            int row = ctrl_row_start_ + k * nu_ + (nu_ - 1);
            l_(row) = std::max(base_lower_vs, v_s_min_dynamic);
            u_(row) = base_upper_vs;
        }
    }

    osqp_update_bounds(work_, l_.data(), u_.data());
}



// ----------------------------------------------------------------------------
// Main Control Interface
// ----------------------------------------------------------------------------

ControlResult MPCControllerCpp::get_control_action(const VectorXd& x_current) {
    auto start = std::chrono::steady_clock::now();

    ControlResult result;
    result.status = -1;
    result.solver_status = 0;
    result.iterations = 0;
    result.objective = 0.0;
    result.u = VectorXd::Zero(nu_);
    result.timeout = false;

    VectorXd x_curr_aug = x_current;
    A_dirty_ = false;

    double s_proj_raw = s_runtime_;
    double path_error = std::numeric_limits<double>::infinity();
    double endpoint_error = std::numeric_limits<double>::infinity();

    if (path_data_valid_ && x_current.size() >= 3) {
        Eigen::Vector3d p_curr = x_current.segment<3>(0);
        Eigen::Vector3d proj_point = Eigen::Vector3d::Zero();
        std::tie(s_proj_raw, proj_point, path_error, endpoint_error) = project_onto_path(p_curr);
        (void)proj_point;

        if (!s_runtime_initialized_) {
            if (x_current.size() >= 17) {
                s_runtime_ = clamp_path_s(x_current(16));
            } else {
                s_runtime_ = clamp_path_s(s_proj_raw);
            }
            s_runtime_initialized_ = true;
        } else {
            double lead_max = 0.5 * mpc_params_.path_speed * dt_ * static_cast<double>(N_);
            lead_max = std::max(0.2, std::min(1.0, lead_max));
            if (path_total_length_ > 0.0) {
                lead_max = std::min(lead_max, path_total_length_);
            }
            double backtrack_tol = std::max(0.1, 0.5 * mpc_params_.path_speed * dt_);
            if (path_total_length_ > 0.0) {
                backtrack_tol = std::min(backtrack_tol, path_total_length_);
            }

            double s_filtered = clamp_path_s(s_proj_raw);
            if (s_filtered < s_runtime_ - backtrack_tol) {
                s_filtered = s_runtime_;
            }
            if (s_filtered > s_runtime_ + lead_max) {
                s_filtered = s_runtime_ + lead_max;
            }
            if (s_filtered > s_runtime_) {
                s_runtime_ = clamp_path_s(s_filtered);
            }
        }
    } else if (!s_runtime_initialized_) {
        if (x_current.size() >= 17) {
            s_runtime_ = clamp_path_s(x_current(16));
        } else {
            s_runtime_ = 0.0;
        }
        s_runtime_initialized_ = true;
    }

    if (x_current.size() == 16) {
        x_curr_aug.resize(17);
        x_curr_aug.head(16) = x_current;
        x_curr_aug(16) = s_runtime_;
    } else if (x_current.size() >= 17) {
        x_curr_aug = x_current;
        x_curr_aug(16) = s_runtime_;
    }
    result.path_s = s_runtime_;
    result.path_s_proj = s_proj_raw;
    result.path_error = path_error;
    result.path_endpoint_error = endpoint_error;
    result.path_s_pred = s_runtime_;

    // Update s_guess mechanism (Warm Start)
    // Shift s_guess_: s[0] = s[1], s[N] = s[N-1] + v*dt
    if (s_guess_.size() == N_ + 1) {
        std::rotate(s_guess_.begin(), s_guess_.begin() + 1, s_guess_.end());
        s_guess_.back() += mpc_params_.path_speed * dt_;

        double s_curr = s_guess_.front();
        if (x_curr_aug.size() >= 17) {
            s_curr = x_curr_aug(16);
        }
        double delta = s_curr - s_guess_.front();
        for (double &s_k : s_guess_) {
            s_k += delta;
        }
        if (path_total_length_ > 0.0) {
            double prev = 0.0;
            for (double &s_k : s_guess_) {
                s_k = std::max(0.0, std::min(s_k, path_total_length_));
                s_k = std::max(s_k, prev);
                prev = s_k;
            }
        }
    }

    const double gyro_threshold = std::max(0.0, mpc_params_.gyro_enable_threshold_radps);
    double omega_norm = 0.0;
    if (x_curr_aug.size() >= 13) {
        omega_norm = x_curr_aug.segment<3>(10).norm();
    }
    active_gyro_jacobian_ =
        mpc_params_.enable_gyro_jacobian ||
        (mpc_params_.auto_enable_gyro_jacobian && omega_norm >= gyro_threshold);
    if (linearizer_) {
        linearizer_->set_enable_gyro_jacobian(active_gyro_jacobian_);
    }

    // Successive linearization trajectory for per-stage A/B updates.
    std::vector<VectorXd> x_traj;
    x_traj.push_back(x_curr_aug);
    if (has_warm_start_control_ && warm_start_x_.size() > 0) {
        for (int k = 1; k <= N_; ++k) {
            VectorXd x_k_tilde = warm_start_x_.segment(k * nx_, nx_);
            VectorXd x_k = x_k_tilde.array() * state_var_scale_.array();
            x_traj.push_back(x_k);
        }
    } else {
        VectorXd x_pred = x_curr_aug;
        VectorXd u_nom = VectorXd::Zero(nu_);
        if (has_last_feasible_control_ && last_feasible_control_.size() == nu_) {
            u_nom = last_feasible_control_;
        } else {
            u_nom(nu_ - 1) = mpc_params_.path_speed;
        }
        int nu_phys = nu_ - 1;
        for (int k = 1; k <= N_; ++k) {
            auto [A_pred, B_pred] = linearizer_->linearize(x_pred.head(16));
            VectorXd x_next = x_pred;
            x_next.head(16) =
                A_pred * x_pred.head(16) + B_pred * u_nom.head(nu_phys) + linearizer_->affine();
            Eigen::Vector4d q_next = x_next.segment<4>(3);
            double q_norm = q_next.norm();
            if (q_norm > 1e-12) {
                x_next.segment<4>(3) = q_next / q_norm;
            } else {
                x_next.segment<4>(3) = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
            }
            x_next(16) = clamp_path_s(x_pred(16) + dt_ * u_nom(nu_ - 1));
            x_traj.push_back(x_next);
            x_pred = x_next;
        }
    }

    ++control_step_counter_;
    if (mpc_params_.enable_online_dare_terminal &&
        !terminal_phys_update_indices_.empty()) {
        int period = std::max(1, mpc_params_.dare_update_period_steps);
        if ((control_step_counter_ % period) == 0 && !x_traj.empty()) {
            VectorXd dare_diag = compute_dare_terminal_cost(
                Q_diag_,
                R_diag_,
                x_traj.back().head(16)
            );
            for (size_t i = 0; i < terminal_phys_update_indices_.size(); ++i) {
                int p_idx = static_cast<int>(terminal_phys_update_indices_[i]);
                if (static_cast<int>(i) < dare_diag.size() &&
                    p_idx >= 0 &&
                    p_idx < static_cast<int>(P_data_.size())) {
                    double s = state_var_scale_(static_cast<int>(i));
                    P_data_[p_idx] = dare_diag(static_cast<int>(i)) * s * s;
                    terminal_phys_update_values_[i] = P_data_[p_idx];
                }
            }
            osqp_update_P(
                work_,
                terminal_phys_update_values_.data(),
                terminal_phys_update_indices_.data(),
                static_cast<c_int>(terminal_phys_update_indices_.size())
            );
        }
    }
    auto t0 = std::chrono::steady_clock::now();
    update_dynamics(x_traj);
    auto t1 = std::chrono::steady_clock::now();

    // update_path_cost updates both P and q; no need for a redundant q reset/update.
    update_path_cost(x_curr_aug);
    auto t2 = std::chrono::steady_clock::now();
    update_constraints(x_curr_aug);
    auto t3 = std::chrono::steady_clock::now();

    if (A_dirty_) {
        osqp_update_A(work_, A_data_.data(), OSQP_NULL, A_.nonZeros());
    }
    auto t4 = std::chrono::steady_clock::now();

    if (has_warm_start_control_) {
        int n_vars = data_ ? data_->n : 0;
        if (n_vars > 0) {
            if (warm_start_x_.size() != n_vars) {
                warm_start_x_ = VectorXd::Zero(n_vars);
            }
            if (work_ && work_->solution && work_->solution->x) {
                std::memcpy(warm_start_x_.data(), work_->solution->x, sizeof(c_float) * n_vars);
            } else {
                warm_start_x_.setZero();
            }

            VectorXd u_guess = VectorXd::Zero(nu_);
            if (warm_start_control_.size() == nu_) {
                u_guess = warm_start_control_;
            } else if (warm_start_control_.size() == sat_params_.num_thrusters) {
                int thruster_offset = sat_params_.num_rw;
                for (int i = 0; i < sat_params_.num_thrusters; ++i) {
                    u_guess(thruster_offset + i) = warm_start_control_(i);
                }
                u_guess(nu_ - 1) = mpc_params_.path_speed;
            } else if (warm_start_control_.size() == nu_ - 1) {
                u_guess.head(nu_ - 1) = warm_start_control_;
                u_guess(nu_ - 1) = mpc_params_.path_speed;
            }

            int u_idx = (N_ + 1) * nx_;
            for (int k = 0; k < N_; ++k) {
                for (int i = 0; i < nu_; ++i) {
                    warm_start_x_(u_idx + k * nu_ + i) =
                        u_guess(i) / std::max(1e-12, control_var_scale_(i));
                }
            }

            osqp_warm_start_x(work_, warm_start_x_.data());
        }
        has_warm_start_control_ = false;
    }
    auto t5 = std::chrono::steady_clock::now();

    osqp_solve(work_);
    auto t6 = std::chrono::steady_clock::now();

    auto end = std::chrono::steady_clock::now();
    result.solve_time = std::chrono::duration<double>(end - start).count();
    result.t_linearization_s = std::chrono::duration<double>(t1 - t0).count();
    result.t_cost_update_s = std::chrono::duration<double>(t2 - t1).count();
    result.t_constraint_update_s = std::chrono::duration<double>(t3 - t2).count();
    result.t_matrix_update_s = std::chrono::duration<double>(t4 - t3).count();
    result.t_warmstart_s = std::chrono::duration<double>(t5 - t4).count();
    result.t_solve_only_s = std::chrono::duration<double>(t6 - t5).count();

    int solver_status = 0;
    if (work_ && work_->info) {
        solver_status = work_->info->status_val;
        result.iterations = static_cast<int>(work_->info->iter);
        result.objective = static_cast<double>(work_->info->obj_val);
    }
    result.solver_status = solver_status;
#ifdef OSQP_TIME_LIMIT_REACHED
    result.timeout = (solver_status == OSQP_TIME_LIMIT_REACHED);
#endif

    const bool solved = (solver_status == OSQP_SOLVED ||
                         solver_status == OSQP_SOLVED_INACCURATE);
    if (!solved) {
        const double hold_s = std::max(0.0, mpc_params_.solver_fallback_hold_s);
        const double decay_s = std::max(0.0, mpc_params_.solver_fallback_decay_s);
        const double zero_after_s = std::max(
            hold_s,
            mpc_params_.solver_fallback_zero_after_s
        );

        if (has_last_feasible_control_ && last_feasible_control_.size() == nu_) {
            if (!fallback_active_) {
                fallback_active_ = true;
                fallback_started_at_ = end;
            }
            const double fallback_age_s = std::max(
                0.0,
                std::chrono::duration<double>(end - fallback_started_at_).count()
            );
            double fallback_scale = 1.0;
            if (fallback_age_s > hold_s) {
                if (decay_s <= 1e-9) {
                    fallback_scale = 0.0;
                } else {
                    fallback_scale = std::max(
                        0.0,
                        1.0 - ((fallback_age_s - hold_s) / decay_s)
                    );
                }
            }
            if (fallback_age_s >= zero_after_s) {
                fallback_scale = 0.0;
            }
            fallback_scale = std::clamp(fallback_scale, 0.0, 1.0);

            result.u = last_feasible_control_ * fallback_scale;
            result.fallback_active = fallback_scale > 0.0;
            result.fallback_age_s = fallback_age_s;
            result.fallback_scale = fallback_scale;
        } else {
            fallback_active_ = false;
            result.fallback_active = false;
            result.fallback_age_s = 0.0;
            result.fallback_scale = 0.0;
        }
        if (result.u.size() == nu_) {
            double s_pred = s_runtime_ + result.u(nu_ - 1) * dt_;
            result.path_s_pred = clamp_path_s(s_pred);
        }
        return result;
    }

    fallback_active_ = false;
    result.fallback_active = false;
    result.fallback_age_s = 0.0;
    result.fallback_scale = 0.0;

    // Extract predicted s trajectory for next step (optional, for better linearization)
    for (int k = 0; k <= N_; ++k) {
        double s_k = work_->solution->x[k * nx_ + 16] * state_var_scale_(16);
        if (path_total_length_ > 0.0) {
            s_k = std::max(0.0, std::min(s_k, path_total_length_));
        }
        s_guess_[k] = s_k;
    }

    // Extract control from solution
    int u_idx = (N_ + 1) * nx_;
    VectorXd u_tilde = Eigen::Map<VectorXd>(work_->solution->x + u_idx, nu_);
    result.u = u_tilde.array() * control_var_scale_.array();

    // Tube-MPC ancillary feedback: correct nominal control with local linear feedback
    // around last nominal trajectory/state estimate.
    if (mpc_params_.robustness_mode == "tube" && nu_ > 1) {
        VectorXd x_nom = x_curr_aug;
        if (warm_start_x_.size() >= nx_) {
            VectorXd x_nom_tilde = warm_start_x_.segment(0, nx_);
            x_nom = x_nom_tilde.array() * state_var_scale_.array();
        } else if (!x_traj.empty()) {
            x_nom = x_traj.front();
        }
        VectorXd dx = x_curr_aug.head(16) - x_nom.head(16);
        MatrixXd P_tube = compute_dare_terminal_matrix(Q_diag_, R_diag_, x_nom.head(16));
        auto [A_tube, B_tube] = linearizer_->linearize(x_nom.head(16));
        MatrixXd R_tube = R_diag_.head(nu_ - 1).asDiagonal();
        R_tube.diagonal() = R_tube.diagonal().cwiseMax(1e-6);
        MatrixXd S = R_tube + B_tube.transpose() * P_tube * B_tube;
        MatrixXd K = S.llt().solve(B_tube.transpose() * P_tube * A_tube);

        double gain_scale = std::clamp(mpc_params_.tube_feedback_gain_scale, 0.0, 1.0);
        double max_corr = std::max(0.0, mpc_params_.tube_feedback_max_correction);
        VectorXd delta_u = -gain_scale * (K * dx);
        for (int i = 0; i < delta_u.size(); ++i) {
            delta_u(i) = std::clamp(delta_u(i), -max_corr, max_corr);
        }
        result.u.head(nu_ - 1) += delta_u;
    }

    // Clip to bounds (safety)
    VectorXd lower = control_lower_;
    if (x_curr_aug.size() >= 17) {
        lower(nu_ - 1) = compute_dynamic_vs_min(x_curr_aug(16));
    }
    result.u = result.u.cwiseMax(lower).cwiseMin(control_upper_);
    result.status = 1;
    last_feasible_control_ = result.u;
    has_last_feasible_control_ = true;
    if (result.u.size() == nu_) {
        double s_next = s_runtime_ + result.u(nu_ - 1) * dt_;
        s_runtime_ = clamp_path_s(s_next);
    }
    result.path_s_pred = s_runtime_;

    // --- Velocity Governor Safety Check (Post-Solve) ---
    // If strict velocity limit is enabled and we are overspeeding,
    // prevent any thrust that increases velocity in the direction of motion.
    // DISABLED in MPCC mode - path following manages speed via progress cost
    // --- Velocity Governor Safety Check (Post-Solve) ---
    // DISABLED in MPCC mode - path following manages speed via progress cost

    return result;
}

// ----------------------------------------------------------------------------
// Collision Avoidance
// ----------------------------------------------------------------------------


void MPCControllerCpp::set_warm_start_control(const VectorXd& u_prev) {
    if (u_prev.size() == 0) {
        has_warm_start_control_ = false;
        return;
    }
    warm_start_control_ = u_prev;
    has_warm_start_control_ = true;
}

void MPCControllerCpp::set_scan_attitude_context(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& axis,
    const std::string& direction
) {
    scan_center_ = center;
    scan_center_valid_ = center.allFinite();
    double axis_norm = axis.norm();
    if (axis_norm > 1e-9) {
        scan_axis_ = axis / axis_norm;
    } else {
        scan_axis_ = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    std::string dir = direction;
    std::transform(
        dir.begin(),
        dir.end(),
        dir.begin(),
        [](unsigned char c) { return static_cast<char>(std::toupper(c)); }
    );
    scan_direction_cw_ = (dir != "CCW");
    scan_attitude_enabled_ = true;
}

void MPCControllerCpp::clear_scan_attitude_context() {
    scan_attitude_enabled_ = false;
    scan_center_ = Eigen::Vector3d::Zero();
    scan_center_valid_ = false;
    scan_axis_ = Eigen::Vector3d(0.0, 0.0, 1.0);
    scan_direction_cw_ = true;
}

MPCControllerCpp::RuntimeMode MPCControllerCpp::parse_runtime_mode(
    const std::string& mode
) const {
    std::string normalized = mode;
    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::toupper(c)); }
    );

    if (normalized == "RECOVER") {
        return RuntimeMode::RECOVER;
    }
    if (normalized == "SETTLE") {
        return RuntimeMode::SETTLE;
    }
    if (normalized == "HOLD") {
        return RuntimeMode::HOLD;
    }
    if (normalized == "COMPLETE") {
        return RuntimeMode::COMPLETE;
    }
    return RuntimeMode::TRACK;
}

void MPCControllerCpp::update_mode_dependent_regularizers() {
    double smooth_scale = 1.0;
    double pair_scale = 1.0;
    int desired_control_horizon = base_control_horizon_;
    if (runtime_mode_ == RuntimeMode::HOLD) {
        smooth_scale = std::max(0.0, mpc_params_.hold_smoothness_scale);
        pair_scale = std::max(0.0, mpc_params_.hold_thruster_pair_scale);
        desired_control_horizon = std::max(1, std::min(base_control_horizon_, N_ / 2));
    } else if (runtime_mode_ == RuntimeMode::RECOVER) {
        desired_control_horizon = std::max(1, std::min(base_control_horizon_, N_ / 3));
    } else if (runtime_mode_ == RuntimeMode::SETTLE ||
               runtime_mode_ == RuntimeMode::COMPLETE) {
        desired_control_horizon = std::max(1, std::min(base_control_horizon_, N_ / 2));
    }

    const bool smooth_changed =
        std::abs(smooth_scale - active_smoothness_scale_) > 1e-9;
    const bool pair_changed =
        std::abs(pair_scale - active_thruster_pair_scale_) > 1e-9;
    const bool horizon_changed =
        std::abs(desired_control_horizon - control_horizon_) > 0;
    if (!smooth_changed && !pair_changed && !horizon_changed) {
        return;
    }

    active_smoothness_scale_ = smooth_scale;
    active_thruster_pair_scale_ = pair_scale;
    control_horizon_ = desired_control_horizon;
    mpc_params_.Q_smooth = base_q_smooth_ * active_smoothness_scale_;
    mpc_params_.thrust_pair_weight =
        base_thrust_pair_weight_ * active_thruster_pair_scale_;
    rebuild_solver();
}

void MPCControllerCpp::set_runtime_mode(const std::string& mode) {
    runtime_mode_ = parse_runtime_mode(mode);
    update_mode_dependent_regularizers();
}

double MPCControllerCpp::compute_dynamic_vs_min(double s_curr) const {
    double base_min = control_lower_(nu_ - 1);
    if (!path_data_valid_ || path_total_length_ <= 0.0) {
        return base_min;
    }

    double s_clamped = std::max(0.0, std::min(s_curr, path_total_length_));
    double remaining = std::max(0.0, path_total_length_ - s_clamped);

    // Distance covered at minimum speed over one horizon.
    double horizon_min_distance = std::max(
        1e-4,
        std::max(1.0, 0.25 * static_cast<double>(N_)) * mpc_params_.path_speed_min * dt_
    );

    if (remaining <= horizon_min_distance) {
        return 0.0;
    }
    return base_min;
}

void MPCControllerCpp::update_path_cost(const VectorXd& x_current) {
    q_.setZero();
    if (!path_data_valid_) {
        // No path data - cannot compute path-following cost
        osqp_update_lin_cost(work_, q_.data());
        return;
    }

    // ==========================================================================
    // General path-following MPC (MPCC) cost update
    // ==========================================================================
    //
    // Cost function:
    //   J = Σ [ Q_contour * ||p - p(s)||²      (Contouring: stay on path)
    //         + Q_progress * (v_s - v_ref)² (Progress: track path speed)
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
    double Q_p = mpc_params_.Q_progress;  // Progress weight (quadratic)
    double progress_reward = mpc_params_.progress_reward; // Linear reward for v_s
    double Q_v = mpc_params_.Q_velocity_align;  // Velocity alignment weight
    if (Q_v <= 0.0) {
        Q_v = mpc_params_.Q_progress;
    }
    double Q_l = mpc_params_.Q_lag;       // Lag weight (along tangent)
    if (Q_l <= 0.0) {
        if (mpc_params_.Q_lag_default >= 0.0) {
            Q_l = mpc_params_.Q_lag_default;
        } else {
            Q_l = Q_c;
        }
    }
    double Q_s_anchor_cfg = mpc_params_.Q_s_anchor;  // Anchor s to progress reference
    double Q_att = mpc_params_.Q_attitude + mpc_params_.Q_axis_align; // Attitude + explicit axis-alignment
    double Q_quat_norm = std::max(0.0, mpc_params_.Q_quat_norm); // Soft quaternion norm regularizer
    double v_ref = mpc_params_.path_speed;  // Path speed reference (max speed)
    if (mpc_params_.path_speed_max > 0.0) {
        v_ref = std::min(v_ref, mpc_params_.path_speed_max);
    }
    if (mpc_params_.path_speed_min > 0.0) {
        v_ref = std::max(v_ref, mpc_params_.path_speed_min);
    }
    double Q_term_pos = mpc_params_.Q_terminal_pos;
    double Q_term_s = mpc_params_.Q_terminal_s;
    if (Q_term_pos <= 0.0) {
        Q_term_pos = std::max(1.0, Q_c);
    }
    if (Q_term_s <= 0.0) {
        Q_term_s = std::max(Q_c, 10.0 * Q_p);
    }
    Eigen::Vector3d p_end = path_points_.back();
    Eigen::Vector4d q_curr = Eigen::Vector4d::Zero();
    if (x_current.size() >= 7) {
        q_curr = x_current.segment<4>(3);
    }
    double q_curr_norm = q_curr.norm();
    if (q_curr_norm > 1e-12) {
        q_curr /= q_curr_norm;
    } else {
        q_curr = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    }

    double s_curr = 0.0;
    if (x_current.size() >= 17) {
        s_curr = x_current(16);
    } else if (!s_guess_.empty()) {
        s_curr = s_guess_.front();
    }
    s_curr = std::max(0.0, std::min(s_curr, path_total_length_));

    double mode_contour_scale = 1.0;
    double mode_lag_scale = 1.0;
    double mode_progress_scale = 1.0;
    double mode_attitude_scale = 1.0;
    double mode_terminal_pos_scale = 1.0;
    double mode_terminal_attitude_scale = 1.0;
    double mode_velocity_align_scale = 1.0;
    double mode_angular_velocity_scale = 1.0;
    if (runtime_mode_ == RuntimeMode::RECOVER) {
        mode_contour_scale = std::max(0.0, mpc_params_.recover_contour_scale);
        mode_lag_scale = std::max(0.0, mpc_params_.recover_lag_scale);
        mode_progress_scale = std::max(0.0, mpc_params_.recover_progress_scale);
        mode_attitude_scale = std::max(0.0, mpc_params_.recover_attitude_scale);
    } else if (runtime_mode_ == RuntimeMode::SETTLE ||
               runtime_mode_ == RuntimeMode::HOLD ||
               runtime_mode_ == RuntimeMode::COMPLETE) {
        mode_progress_scale = std::max(0.0, mpc_params_.settle_progress_scale);
        mode_terminal_pos_scale =
            std::max(0.0, mpc_params_.settle_terminal_pos_scale);
        mode_terminal_attitude_scale =
            std::max(0.0, mpc_params_.settle_terminal_attitude_scale);
        mode_velocity_align_scale =
            std::max(0.0, mpc_params_.settle_velocity_align_scale);
        mode_angular_velocity_scale =
            std::max(0.0, mpc_params_.settle_angular_velocity_scale);
    }

    const double contour_weight_scale = mode_contour_scale;
    const double progress_weight_scale = mode_progress_scale;
    const double attitude_weight_scale = mode_attitude_scale;

    const double Q_c_eff = Q_c * contour_weight_scale;
    const double Q_l_eff = Q_l * mode_lag_scale;
    const double Q_p_eff = Q_p * progress_weight_scale;
    const double progress_reward_eff = progress_reward * progress_weight_scale;
    const double Q_att_eff = Q_att * attitude_weight_scale;
    const double Q_v_eff = Q_v * mode_velocity_align_scale;
    const double Q_w_eff = mpc_params_.Q_angvel * mode_angular_velocity_scale;
    const double Q_term_pos_eff =
        Q_term_pos * contour_weight_scale * mode_terminal_pos_scale;
    const double Q_term_s_eff = Q_term_s * progress_weight_scale;
    double Q_s_anchor_eff = Q_s_anchor_cfg;
    if (Q_s_anchor_eff < 0.0) {
        Q_s_anchor_eff = std::max(Q_p_eff, 0.5 * Q_c_eff);
    } else {
        Q_s_anchor_eff *= progress_weight_scale;
    }

    double v_ref_base = v_ref;
    bool auto_progress = progress_reward_eff > 0.0;

    // Initialize s_guess if needed
    if (s_guess_.size() != static_cast<size_t>(N_ + 1)) {
        s_guess_.resize(N_ + 1);
        double s0 = (nx_ == 17 && x_current.size() >= 17) ? x_current(16) : s_curr;
        for (int k = 0; k <= N_; ++k) {
            s_guess_[k] = std::min(s0 + k * v_ref_base * dt_, path_total_length_);
        }
    }
    Eigen::Vector4d q_prev_ref = Eigen::Vector4d::Zero();
    bool has_q_prev_ref = false;

    for (int k = 0; k <= N_; ++k) {
        double s_bar = s_guess_[k];
        double stage_scale = (k == N_) ? 10.0 : 1.0;
        double att_stage_scale = stage_scale;
        if (k == N_) {
            att_stage_scale *= mode_terminal_attitude_scale;
        }
        double Q_c_k = Q_c_eff * stage_scale;
        double Q_l_k = Q_l_eff * stage_scale;

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
        double t_dot_pref = t_ref.dot(p_ref);

        int x_idx = k * nx_;  // State index for this step
        auto sx = [this](int i) { return mpc_params_.enable_variable_scaling ? state_var_scale_(i) : 1.0; };
        auto su = [this](int i) { return mpc_params_.enable_variable_scaling ? control_var_scale_(i) : 1.0; };

        // Update quaternion diagonal entries for dynamic attitude weighting.
        if (k < static_cast<int>(path_att_diag_indices_.size())) {
            double q_att_diag = 2.0 * (Q_att_eff + Q_quat_norm) * att_stage_scale;
            auto &att_diag_indices = path_att_diag_indices_[k];
            for (int i = 0; i < 4 && i < static_cast<int>(att_diag_indices.size()); ++i) {
                int idx = att_diag_indices[i];
                if (idx >= 0) {
                    double s = sx(3 + i);
                    P_data_[idx] = q_att_diag * s * s;
                }
            }
        }

        // Update angular-velocity diagonal entries for mode-dependent damping.
        if (k < static_cast<int>(path_angvel_diag_indices_.size())) {
            double q_w_diag = Q_w_eff * stage_scale;
            auto &w_diag_indices = path_angvel_diag_indices_[k];
            for (int i = 0; i < 3 && i < static_cast<int>(w_diag_indices.size()); ++i) {
                int idx = w_diag_indices[i];
                if (idx >= 0) {
                    double s = sx(10 + i);
                    P_data_[idx] = q_w_diag * s * s;
                }
            }
        }

        // 0. Update position diagonal entries (base + optional terminal boost)
        double pos_diag_base = 2.0 * Q_c_eff * stage_scale;
        for (int i = 0; i < 3; ++i) {
            double pos_diag = pos_diag_base;
            pos_diag += 2.0 * Q_l_k * t_ref(i) * t_ref(i);
            if (k == N_) {
                pos_diag += 2.0 * Q_term_pos_eff;
            }
            double s = sx(i);
            P_data_[path_pos_diag_indices_[k][i]] = pos_diag * s * s;
        }

        // 0b. Update position off-diagonals for lag cost (t*t^T)
        if (path_pos_offdiag_indices_.size() > static_cast<size_t>(k)) {
            double xy = 2.0 * Q_l_k * t_ref(0) * t_ref(1);
            double xz = 2.0 * Q_l_k * t_ref(0) * t_ref(2);
            double yz = 2.0 * Q_l_k * t_ref(1) * t_ref(2);
            auto &offdiag = path_pos_offdiag_indices_[k];
            if (offdiag.size() == 3) {
                P_data_[offdiag[0]] = xy;
                P_data_[offdiag[0]] *= sx(0) * sx(1);
                P_data_[offdiag[1]] = xz * sx(0) * sx(2);
                P_data_[offdiag[2]] = yz * sx(1) * sx(2);
            }
        }

        // 1. Update P matrix cross-terms (p, s)
        // These are stored at path_P_indices_[k][0..2]
        // P_ps = -2 * Q_c * t (off-diagonal coupling position with s)
        for (int i = 0; i < 3; ++i) {
            double val = -2.0 * Q_c_k * t_ref(i);
            P_data_[path_P_indices_[k][i]] = val * sx(i) * sx(16);
        }

        // 1b. Update s diagonal entry
        if (k < static_cast<int>(path_s_diag_indices_.size())) {
            double s_diag = 2.0 * Q_c_k * t_norm_sq;
            if (k == N_) {
                s_diag += 2.0 * Q_term_s_eff;
            }
            if (Q_s_anchor_eff > 0.0) {
                s_diag += 2.0 * Q_s_anchor_eff * stage_scale;
            }
            P_data_[path_s_diag_indices_[k]] = s_diag * sx(16) * sx(16);
        }

        // 2. Update q vector (linear costs)
        // q_p = -2 * Q_c * C  (attracts position toward path)
        for (int i = 0; i < 3; ++i) {
            q_(x_idx + i) = (-2.0 * Q_c_k * C(i) + -2.0 * Q_l_k * t_dot_pref * t_ref(i)) * sx(i);
        }

        // q_s = 2 * Q_c * (t·C) (pushes s forward)
        q_(x_idx + 16) = 2.0 * Q_c_k * t_dot_C * sx(16);
        if (k == N_) {
            // Terminal penalties: position and s to endpoint
            for (int i = 0; i < 3; ++i) {
                q_(x_idx + i) += -2.0 * Q_term_pos_eff * p_end(i) * sx(i);
            }
            q_(x_idx + 16) += -2.0 * Q_term_s_eff * path_total_length_ * sx(16);
        }
        if (Q_s_anchor_eff > 0.0) {
            q_(x_idx + 16) += -2.0 * Q_s_anchor_eff * stage_scale * s_bar * sx(16);
        }

        // 2c. Attitude tracking:
        // Default mode: +X follows path tangent.
        // Scan mode: keep +X forward, +Y object-facing, and +Z aligned to scan axis.
        bool build_attitude_ref = (Q_att_eff > 0.0) || scan_attitude_enabled_;
        if (build_attitude_ref) {
            Eigen::Vector4d q_ref = build_reference_quaternion(p_ref, t_ref, q_curr);
            if (has_q_prev_ref && q_prev_ref.dot(q_ref) < 0.0) {
                q_ref = -q_ref;
            } else if (!has_q_prev_ref && q_curr.dot(q_ref) < 0.0) {
                q_ref = -q_ref;
            }
            q_prev_ref = q_ref;
            has_q_prev_ref = true;

            if (Q_att_eff > 0.0) {
                for (int i = 0; i < 4; ++i) {
                    q_(x_idx + 3 + i) =
                        -2.0 * Q_att_eff * att_stage_scale * q_ref(i) * sx(3 + i);
                }
            }
        }
        if (Q_quat_norm > 0.0) {
            for (int i = 0; i < 4; ++i) {
                q_(x_idx + 3 + i) +=
                    -2.0 * Q_quat_norm * att_stage_scale * q_curr(i) * sx(3 + i);
            }
        }

        // 2b. Velocity alignment cost: track v along path tangent
        if (k < static_cast<int>(path_vel_P_indices_.size())) {
            double v_ref_k = v_ref_base;
            if (auto_progress) {
                v_ref_k = 0.0;
            }

            // Update velocity quadratic terms (diagonal only)
            // Order: (0,0),(0,1),(0,2),(1,1),(1,2),(2,2)
            double vel_weight = 2.0 * Q_v_eff * stage_scale;
            auto &vel_indices = path_vel_P_indices_[k];
            if (vel_indices.size() >= 6) {
                if (vel_indices[0] >= 0) P_data_[vel_indices[0]] = vel_weight * sx(7) * sx(7);
                if (vel_indices[1] >= 0) P_data_[vel_indices[1]] = 0.0;
                if (vel_indices[2] >= 0) P_data_[vel_indices[2]] = 0.0;
                if (vel_indices[3] >= 0) P_data_[vel_indices[3]] = vel_weight * sx(8) * sx(8);
                if (vel_indices[4] >= 0) P_data_[vel_indices[4]] = 0.0;
                if (vel_indices[5] >= 0) P_data_[vel_indices[5]] = vel_weight * sx(9) * sx(9);
            }

            // Linear term encourages velocity along tangent
            for (int i = 0; i < 3; ++i) {
                q_(x_idx + 7 + i) +=
                    -2.0 * Q_v_eff * v_ref_k * t_ref(i) * stage_scale * sx(7 + i);
            }
        }

        // 3. Progress tracking: (v_s - v_ref)² on control side
        // v_s is the last control in u (index nu_-1)
        // Cost: Q_p * (v_s - v_ref)² = Q_p * v_s² - 2*Q_p*v_ref*v_s + const
        // Linear term: -2 * Q_p * v_ref
        if (k < N_) {
            double v_ref_k = v_ref_base;
            if (static_cast<size_t>(k) < path_vs_diag_indices_.size()) {
                int vs_diag_idx = path_vs_diag_indices_[static_cast<size_t>(k)];
                if (vs_diag_idx >= 0) {
                    double s = su(nu_ - 1);
                    P_data_[vs_diag_idx] = 2.0 * Q_p_eff * s * s;
                }
            }
            int u_idx = (N_ + 1) * nx_ + k * nu_ + (nu_ - 1);
            if (auto_progress) {
                q_(u_idx) = -2.0 * progress_reward_eff * su(nu_ - 1);
            } else {
                q_(u_idx) = -2.0 * Q_p_eff * v_ref_k * su(nu_ - 1);
            }

            // Fuel bias: linear penalty on thruster usage to promote coasting.
            if (mpc_params_.thrust_l1_weight > 0.0) {
                int thruster_base = (N_ + 1) * nx_ + k * nu_ + sat_params_.num_rw;
                for (int i = 0; i < sat_params_.num_thrusters; ++i) {
                    q_(thruster_base + i) += mpc_params_.thrust_l1_weight * su(sat_params_.num_rw + i);
                }
            }
        }
    }

    // Push updates to OSQP
    if (!path_P_update_indices_.empty()) {
        for (size_t i = 0; i < path_P_update_indices_.size(); ++i) {
            path_P_update_values_[i] = P_data_[path_P_update_indices_[i]];
        }
        osqp_update_P(
            work_,
            path_P_update_values_.data(),
            path_P_update_indices_.data(),
            static_cast<c_int>(path_P_update_indices_.size())
        );
    } else {
        osqp_update_P(work_, P_data_.data(), OSQP_NULL, P_.nonZeros());
    }
    osqp_update_lin_cost(work_, q_.data());
}


// ============================================================================
// Path following: general path support
// ============================================================================

void MPCControllerCpp::set_path_data(const std::vector<std::array<double, 4>>& path_data) {
    path_s_.clear();
    path_points_.clear();

    if (path_data.empty()) {
        path_data_valid_ = false;
        path_total_length_ = 0.0;
        s_runtime_ = 0.0;
        s_runtime_initialized_ = false;
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
    s_runtime_ = 0.0;
    s_runtime_initialized_ = false;
    ref_frame_initialized_ = false;
    ref_prev_x_axis_ = Eigen::Vector3d::Zero();
    ref_prev_y_axis_ = Eigen::Vector3d::Zero();
    ref_prev_z_axis_ = Eigen::Vector3d::Zero();

    // Update s bounds with actual path length (if solver initialized)
    if (work_ != nullptr) {
        int n_dyn = n_dyn_;
        int n_init = n_init_;
        int n_bounds_x = n_bounds_x_;
        int state_idx_start = n_dyn + n_init;

        double L = path_total_length_;
        for (int k = 0; k < N_ + 1; ++k) {
            int s_idx = state_idx_start + k * nx_ + 16;
            l_(s_idx) = -0.5;
            u_(s_idx) = L + 2.0;
        }
        osqp_update_bounds(work_, l_.data(), u_.data());
    }
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

    // At terminal s, prefer the final non-degenerate segment direction
    // (second-last waypoint heading) because no forward segment exists.
    if (s_clamped >= (path_s_.back() - 1e-9)) {
        for (size_t i = path_points_.size(); i-- > 1;) {
            Eigen::Vector3d last_diff = path_points_[i] - path_points_[i - 1];
            double last_len = last_diff.norm();
            if (last_len > 1e-12) {
                return last_diff / last_len;
            }
        }
    }

    // Select the forward segment so +X faces the next waypoint.
    // For exact waypoint hits, this chooses [i -> i+1] (except at terminal s,
    // handled above), not [i-1 -> i].
    auto it = std::upper_bound(path_s_.begin(), path_s_.end(), s_clamped);
    int idx_next = std::distance(path_s_.begin(), it);
    if (idx_next <= 0) {
        idx_next = 1;
    }
    if (idx_next >= static_cast<int>(path_s_.size())) {
        idx_next = static_cast<int>(path_s_.size()) - 1;
    }
    int idx_prev = idx_next - 1;

    // Tangent is direction from current sample to next sample.
    Eigen::Vector3d diff = path_points_[idx_next] - path_points_[idx_prev];
    double len = diff.norm();
    if (len < 1e-12) {
        return Eigen::Vector3d(1.0, 0.0, 0.0);  // Degenerate case
    }

    return diff / len;  // Normalized tangent
}

double MPCControllerCpp::clamp_path_s(double s) const {
    if (!path_data_valid_ || path_total_length_ <= 0.0) {
        return std::max(0.0, s);
    }
    return std::max(0.0, std::min(s, path_total_length_));
}

Eigen::Vector4d MPCControllerCpp::build_reference_quaternion(
    const Eigen::Vector3d& p_ref,
    const Eigen::Vector3d& t_ref,
    const Eigen::Vector4d& q_curr
) const {
    Eigen::Vector3d x_axis = t_ref;
    double x_norm = x_axis.norm();
    if (x_norm > 1e-9) {
        x_axis /= x_norm;
    } else {
        x_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    Eigen::Vector3d y_axis(0.0, 1.0, 0.0);
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);

    if (scan_attitude_enabled_) {
        // 1) Use the scan axis line as +Z/-Z reference.
        Eigen::Vector3d z_line = scan_axis_;
        double z_line_norm = z_line.norm();
        if (z_line_norm > 1e-9) {
            z_line /= z_line_norm;
        } else {
            z_line = Eigen::Vector3d::UnitZ();
        }

        // 2) Build object-facing radial direction in the scan plane (+Y target)
        // when scan center is available; otherwise preserve continuity from current attitude.
        Eigen::Vector3d radial_in = Eigen::Vector3d::Zero();
        if (scan_center_valid_) {
            radial_in = scan_center_ - p_ref;
            radial_in -= radial_in.dot(z_line) * z_line;
        }
        double radial_norm = radial_in.norm();
        Eigen::Vector3d radial_dir = Eigen::Vector3d::UnitY();
        bool has_radial = false;
        if (radial_norm > 1e-9) {
            radial_dir = radial_in / radial_norm;
            has_radial = true;
        } else if (q_curr.norm() > 1e-9) {
            Eigen::Quaterniond q_curr_eig(q_curr(0), q_curr(1), q_curr(2), q_curr(3));
            q_curr_eig.normalize();
            Eigen::Vector3d curr_y = q_curr_eig * Eigen::Vector3d::UnitY();
            curr_y -= curr_y.dot(z_line) * z_line;
            double curr_y_norm = curr_y.norm();
            if (curr_y_norm > 1e-9) {
                radial_dir = curr_y / curr_y_norm;
                has_radial = true;
            }
        }
        if (!has_radial) {
            Eigen::Vector3d ref =
                (std::abs(z_line.z()) < 0.9) ? Eigen::Vector3d::UnitZ()
                                             : Eigen::Vector3d::UnitX();
            radial_dir = z_line.cross(ref);
            double fallback_norm = radial_dir.norm();
            if (fallback_norm > 1e-9) {
                radial_dir /= fallback_norm;
            } else {
                radial_dir = Eigen::Vector3d::UnitY();
            }
            has_radial = true;
        }

        // 3) Keep +X aligned with path travel in the scan plane.
        // Keep +Z fixed to the configured scan-axis direction (no sign flips).
        Eigen::Vector3d t_plane = t_ref - t_ref.dot(z_line) * z_line;
        double t_plane_norm = t_plane.norm();
        if (t_plane_norm > 1e-9) {
            x_axis = t_plane / t_plane_norm;
            z_axis = z_line;
            y_axis = z_axis.cross(x_axis);
        } else {
            z_axis = z_line;
            // Degenerate tangent (parallel to scan axis): preserve previous projected
            // +X when possible for deterministic continuity, then fallback.
            bool have_continuity_x = false;
            if (ref_frame_initialized_) {
                Eigen::Vector3d x_prev = ref_prev_x_axis_;
                x_prev -= x_prev.dot(z_axis) * z_axis;
                double x_prev_norm = x_prev.norm();
                if (x_prev_norm > 1e-9) {
                    x_axis = x_prev / x_prev_norm;
                    have_continuity_x = true;
                }
            }
            if (!have_continuity_x) {
                y_axis = radial_dir;
                x_axis = scan_direction_cw_ ? z_axis.cross(y_axis)
                                            : y_axis.cross(z_axis);
            }
        }

        // Re-orthonormalize and keep +Y object-facing without sacrificing +X.
        x_axis -= x_axis.dot(z_axis) * z_axis;
        x_norm = x_axis.norm();
        if (x_norm > 1e-9) {
            x_axis /= x_norm;
        } else {
            x_axis = scan_direction_cw_ ? z_axis.cross(radial_dir)
                                        : radial_dir.cross(z_axis);
            x_norm = x_axis.norm();
            if (x_norm > 1e-9) {
                x_axis /= x_norm;
            } else {
                x_axis = Eigen::Vector3d::UnitX();
            }
        }
        y_axis = z_axis.cross(x_axis);
        double y_norm = y_axis.norm();
        if (y_norm > 1e-9) {
            y_axis /= y_norm;
        } else {
            y_axis = radial_dir;
        }
        x_axis = y_axis.cross(z_axis);
        x_norm = x_axis.norm();
        if (x_norm > 1e-9) {
            x_axis /= x_norm;
        } else {
            x_axis = Eigen::Vector3d::UnitX();
        }

        // Keep +X forward along path travel (use forward branch).
        if (x_axis.dot(t_ref) < 0.0) {
            x_axis = -x_axis;
            y_axis = -y_axis;
        }
    } else {
        auto choose_seed_axis = [&x_axis]() -> Eigen::Vector3d {
            std::array<Eigen::Vector3d, 3> candidates = {
                Eigen::Vector3d::UnitX(),
                Eigen::Vector3d::UnitY(),
                Eigen::Vector3d::UnitZ(),
            };
            Eigen::Vector3d best = candidates[0];
            double best_abs_dot = std::abs(x_axis.dot(best));
            for (size_t i = 1; i < candidates.size(); ++i) {
                double cand_dot = std::abs(x_axis.dot(candidates[i]));
                if (cand_dot < best_abs_dot) {
                    best = candidates[i];
                    best_abs_dot = cand_dot;
                }
            }
            return best;
        };

        Eigen::Vector3d z_seed = ref_frame_initialized_ ? ref_prev_z_axis_ : choose_seed_axis();
        z_axis = z_seed - z_seed.dot(x_axis) * x_axis;
        double z_norm = z_axis.norm();
        if (z_norm <= 1e-9) {
            z_seed = choose_seed_axis();
            z_axis = z_seed - z_seed.dot(x_axis) * x_axis;
            z_norm = z_axis.norm();
        }
        if (z_norm > 1e-9) {
            z_axis /= z_norm;
        } else {
            z_axis = Eigen::Vector3d::UnitZ();
        }

        y_axis = z_axis.cross(x_axis);
        double y_norm = y_axis.norm();
        if (y_norm > 1e-9) {
            y_axis /= y_norm;
        } else {
            y_axis = Eigen::Vector3d::UnitY();
        }

        z_axis = x_axis.cross(y_axis);
        z_norm = z_axis.norm();
        if (z_norm > 1e-9) {
            z_axis /= z_norm;
        } else {
            z_axis = Eigen::Vector3d::UnitZ();
        }

        if (ref_frame_initialized_ && ref_prev_y_axis_.dot(y_axis) < 0.0) {
            y_axis = -y_axis;
            z_axis = -z_axis;
        }
    }

    ref_prev_x_axis_ = x_axis;
    ref_prev_y_axis_ = y_axis;
    ref_prev_z_axis_ = z_axis;
    ref_frame_initialized_ = true;

    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
    Eigen::Quaterniond q_ref_eig(R);
    Eigen::Vector4d q_ref(q_ref_eig.w(), q_ref_eig.x(), q_ref_eig.y(), q_ref_eig.z());

    if (q_curr.norm() > 1e-9 && q_curr.dot(q_ref) < 0.0) {
        q_ref = -q_ref;
    }
    return q_ref;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector4d> MPCControllerCpp::get_reference_at_s(
    double s_query,
    const Eigen::Vector4d& q_current
) const {
    double s_ref = clamp_path_s(s_query);
    Eigen::Vector3d p_ref = get_path_point(s_ref);
    Eigen::Vector3d t_ref = get_path_tangent(s_ref);
    Eigen::Vector4d q_ref = build_reference_quaternion(p_ref, t_ref, q_current);
    if (q_current.norm() > 1e-9 && q_current.dot(q_ref) < 0.0) {
        q_ref = -q_ref;
    }
    return {p_ref, t_ref, q_ref};
}

std::tuple<double, Eigen::Vector3d, double, double> MPCControllerCpp::project_onto_path(
    const Eigen::Vector3d& position) const {
    if (!path_data_valid_ || path_points_.size() < 2 || path_s_.size() < 2) {
        return {0.0, Eigen::Vector3d::Zero(),
                std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()};
    }

    double best_s = path_s_.front();
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_point = path_points_.front();

    for (size_t i = 0; i + 1 < path_points_.size(); ++i) {
        const Eigen::Vector3d& p0 = path_points_[i];
        const Eigen::Vector3d& p1 = path_points_[i + 1];
        Eigen::Vector3d seg = p1 - p0;
        double seg_len2 = seg.squaredNorm();
        double t = 0.0;
        if (seg_len2 > 1e-12) {
            t = (position - p0).dot(seg) / seg_len2;
            if (t < 0.0) {
                t = 0.0;
            } else if (t > 1.0) {
                t = 1.0;
            }
        }
        Eigen::Vector3d proj = p0 + t * seg;
        double dist = (position - proj).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_point = proj;
            double s0 = path_s_[i];
            double s1 = path_s_[i + 1];
            best_s = s0 + t * (s1 - s0);
        }
    }

    if (path_total_length_ > 0.0) {
        if (best_s < 0.0) {
            best_s = 0.0;
        } else if (best_s > path_total_length_) {
            best_s = path_total_length_;
        }
    }

    double endpoint_error = (position - path_points_.back()).norm();
    return {best_s, best_point, min_dist, endpoint_error};
}

} // namespace satellite_control

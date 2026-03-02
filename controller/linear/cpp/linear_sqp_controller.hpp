/**
 * @file sqp_controller.hpp
 * @brief RTI-SQP Model Predictive Controller (V2) for satellite GNC.
 *
 * Uses CasADi-generated functions for exact dynamics linearisation and
 * cost Hessian/gradient computation, with OSQP as the QP backend.
 *
 * Architecture:
 *   CasADi codegen → exact A_k, B_k, d_k at each horizon stage
 *   MPCC cost      → exact Hessian P and gradient q via CasADi AD
 *   OSQP           → solves the resulting sparse QP
 *   RTI            → one QP iteration per control step (real-time feasible)
 *
 * The class exposes the same public interface as the V1 MPCControllerCpp so
 * that the Python wrapper (MPCController) is a drop-in replacement.
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <tuple>
#include <vector>

#include "osqp.h"
#include "linear_sqp_types.hpp"
#include "../../shared/cpp/satellite_params.hpp"

namespace satellite_control {
namespace v2 {

// Forward declarations
class DynamicsEvaluator;
class CostBuilder;
class ConstraintBuilder;

/**
 * @brief RTI-SQP MPC Controller.
 *
 * Each call to get_control_action():
 *   1. Shift previous trajectory (warm-start)
 *   2. Evaluate linearised dynamics at each horizon stage (CasADi)
 *   3. Build QP cost from MPCC Hessian/gradient (CasADi)
 *   4. Update constraints (dynamics, bounds, initial state)
 *   5. Solve QP with OSQP
 *   6. Extract controls, update trajectory, apply fallback if needed
 */
class SQPController {
public:
    /**
     * @brief Construct SQP controller.
     *
     * @param sat_params  Satellite physical parameters.
     * @param mpc_params  V2 MPC configuration.
     */
    SQPController(const SatelliteParams& sat_params, const MPCV2Params& mpc_params);

    ~SQPController();

    // Non-copyable
    SQPController(const SQPController&) = delete;
    SQPController& operator=(const SQPController&) = delete;

    // ----- Main control interface ----------------------------------------

    /**
     * @brief Compute optimal control for the current state.
     *
     * @param x_current  State vector (17×1 MPCC-augmented).
     * @return ControlResultV2 with optimal controls and solver telemetry.
     */
    ControlResultV2 get_control_action(const VectorXd& x_current);

    // ----- Accessors -----------------------------------------------------

    int num_controls() const { return nu_; }
    int prediction_horizon() const { return N_; }
    double dt() const { return dt_; }
    double path_length() const { return path_.total_length; }
    bool has_path() const { return path_.valid; }
    double current_path_s() const { return s_runtime_; }
    void set_current_path_s(double s_value);

    // ----- Configuration -------------------------------------------------

    void set_warm_start_control(const VectorXd& u_prev);
    void set_path_data(const std::vector<std::array<double, 4>>& path_data);
    void set_scan_attitude_context(
        const Vector3d& center,
        const Vector3d& axis,
        const std::string& direction
    );
    void clear_scan_attitude_context();
    void set_runtime_mode(const std::string& mode);

    // ----- Path utilities ------------------------------------------------

    std::tuple<double, Vector3d, double, double> project_onto_path(
        const Vector3d& position) const;

    std::tuple<Vector3d, Vector3d, Vector4d> get_reference_at_s(
        double s_query,
        const Vector4d& q_current
    ) const;

    // ----- Python-side linearisation data injection ----------------------

    /**
     * @brief Set linearised dynamics for horizon stage k.
     *
     * Called from Python after evaluating CasADi-generated functions
     * on the current trajectory iterate.
     */
    void set_stage_linearisation(
        int k,
        const MatrixXd& A,
        const MatrixXd& B,
        const VectorXd& d
    ) {
        if (k >= 0 && k < N_) {
            A_stages_[k] = A;
            B_stages_[k] = B;
            d_stages_[k] = d;
        }
    }

    /**
     * @brief Set all stages' linearisation data at once.
     *
     * More efficient than per-stage calls when Python evaluates
     * the full horizon in a vectorised CasADi map.
     */
    void set_all_linearisations(
        const std::vector<MatrixXd>& As,
        const std::vector<MatrixXd>& Bs,
        const std::vector<VectorXd>& ds
    ) {
        int n = std::min({static_cast<int>(As.size()),
                          static_cast<int>(Bs.size()),
                          static_cast<int>(ds.size()), N_});
        for (int k = 0; k < n; ++k) {
            A_stages_[k] = As[k];
            B_stages_[k] = Bs[k];
            d_stages_[k] = ds[k];
        }
    }

    /** Get state at horizon stage k (for Python-side CasADi evaluation). */
    VectorXd get_stage_state(int k) const {
        if (k >= 0 && k <= N_) return x_traj_[k];
        return VectorXd::Zero(nx_);
    }

    /** Get control at horizon stage k. */
    VectorXd get_stage_control(int k) const {
        if (k >= 0 && k < N_) return u_traj_[k];
        return VectorXd::Zero(nu_);
    }

    /** Get CasADi parameter vector. */
    const VectorXd& casadi_params() const { return casadi_params_; }

private:
    // ----- Dimensions ----------------------------------------------------
    int nx_ = 17;
    int nu_;
    int N_;
    int M_;      // control horizon
    double dt_;
    int num_thrusters_;
    int num_rw_;

    // ----- Parameters ----------------------------------------------------
    MPCV2Params params_;
    SatelliteParams sat_params_;

    // ----- Trajectory storage --------------------------------------------
    std::vector<VectorXd> x_traj_;  // (N+1) state trajectory
    std::vector<VectorXd> u_traj_;  // (N) control trajectory

    // ----- Linearisation data (per-stage) --------------------------------
    std::vector<MatrixXd> A_stages_;  // A_k for k=0..N-1
    std::vector<MatrixXd> B_stages_;  // B_k for k=0..N-1
    std::vector<VectorXd> d_stages_;  // affine d_k for k=0..N-1

    // ----- QP data -------------------------------------------------------
    Eigen::SparseMatrix<double> P_qp_;   // cost Hessian (upper triangular)
    VectorXd q_qp_;                       // cost gradient
    Eigen::SparseMatrix<double> A_qp_;   // constraint matrix
    VectorXd l_qp_, u_qp_;               // constraint bounds

    // OSQP workspace
    OSQPWorkspace* osqp_work_ = nullptr;
    OSQPSettings* osqp_settings_ = nullptr;
    OSQPData* osqp_data_ = nullptr;

    // CSC storage for OSQP
    std::vector<c_float> P_data_;
    std::vector<c_int> P_indices_;
    std::vector<c_int> P_indptr_;
    std::vector<c_float> A_data_;
    std::vector<c_int> A_indices_;
    std::vector<c_int> A_indptr_;

    // ----- QP dimensions -------------------------------------------------
    int n_vars_ = 0;          // total decision variables
    int n_constraints_ = 0;   // total constraints
    int n_dyn_ = 0;           // dynamics equality rows
    int n_init_ = 0;          // initial state rows
    int n_bounds_x_ = 0;      // state bound rows
    int n_bounds_u_ = 0;      // control bound rows
    int n_ctrl_tie_ = 0;      // control horizon tying rows

    // ----- Runtime state -------------------------------------------------
    RuntimeMode mode_ = RuntimeMode::TRACK;
    PathData path_;
    ScanAttitudeContext scan_ctx_;
    double s_runtime_ = 0.0;
    bool s_initialized_ = false;

    // Warm-start
    VectorXd warm_start_control_;
    bool has_warm_start_ = false;

    // Fallback
    VectorXd last_feasible_control_;
    bool has_last_feasible_ = false;
    bool fallback_active_ = false;
    std::chrono::steady_clock::time_point fallback_started_at_{};

    // DARE terminal cost
    VectorXd dare_diag_;
    int control_step_counter_ = 0;

    // Packed CasADi parameter vector
    VectorXd casadi_params_;

    // Mode-dependent weight cache
    double active_Q_contour_ = 0.0;
    double active_Q_lag_ = 0.0;
    double active_Q_progress_ = 0.0;
    double active_Q_attitude_ = 0.0;
    double active_Q_smooth_ = 0.0;
    double active_Q_velocity_ = 0.0;
    double active_Q_angvel_ = 0.0;
    double active_thrust_pair_ = 0.0;
    // Mode-aware path speed target for the virtual-speed cost term.
    // Set to 0 in SETTLE/HOLD/COMPLETE so the MPC does not incentivise
    // further path progress once the endpoint has been reached.
    double active_path_speed_ = 0.0;
    double last_ref_heading_step_deg_ = 0.0;
    double last_ref_quat_step_deg_max_horizon_ = 0.0;
    double last_ref_slew_limited_fraction_ = 0.0;
    bool last_terminal_progress_reward_active_ = false;
    int last_degenerate_tangent_fallback_count_ = 0;

    // ----- Initialisation helpers ----------------------------------------
    void init_qp();
    void build_P_structure();
    void build_A_structure();
    void setup_osqp();
    void pack_casadi_params();

    // ----- Per-step update methods ---------------------------------------
    void shift_trajectory();
    void linearise_dynamics();
    void update_cost(const VectorXd& x_current);
    void update_constraints(const VectorXd& x_current);
    void extract_solution(const c_float* primal);
    void apply_mode_scaling();
    double compute_dynamic_vs_min(double s_curr) const;
    Vector3d get_smooth_tangent(double s_query) const;
    double mode_ref_quat_max_step_rad() const;
    Vector4d apply_quaternion_slew_limit(
        const Vector4d& q_prev,
        const Vector4d& q_desired,
        double max_step_rad,
        bool* limited,
        double* step_rad
    ) const;
    static double quaternion_step_angle_rad(
        const Vector4d& q_from,
        const Vector4d& q_to
    );

    // ----- Reference generation ------------------------------------------
    Vector4d build_reference_quaternion(
        const Vector3d& p_ref,
        const Vector3d& t_ref,
        const Vector4d& q_curr
    ) const;

    // ----- DARE terminal cost --------------------------------------------
    VectorXd compute_dare_terminal_diag(
        const VectorXd& x_nominal
    ) const;

    // ----- Fallback policy -----------------------------------------------
    ControlResultV2 apply_fallback(double dt_since_last);

    // ----- Cleanup -------------------------------------------------------
    void cleanup_osqp();
};

}  // namespace v2
}  // namespace satellite_control

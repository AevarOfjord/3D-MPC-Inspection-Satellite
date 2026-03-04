#include "unified_runtime.hpp"

#include <algorithm>
#include <stdexcept>

namespace satellite_control {
namespace runtime {

namespace {

RuntimeResult unavailable_result(
    int num_controls,
    const std::string& reason,
    double current_s = 0.0
) {
    RuntimeResult result;
    result.u = VectorXd::Zero(num_controls);
    result.status = -1;
    result.solver_status = "unavailable_backend";
    result.unavailable_reason = reason;
    result.fallback_active = true;
    result.path_s = current_s;
    result.path_s_proj = current_s;
    result.path_s_pred = current_s;
    result.path_error = 0.0;
    result.path_endpoint_error = 0.0;
    return result;
}

RuntimeProfile normalize_profile(RuntimeProfile profile) {
    switch (profile) {
        case RuntimeProfile::CPP_HYBRID_RTI_OSQP:
        case RuntimeProfile::CPP_NONLINEAR_RTI_OSQP:
        case RuntimeProfile::CPP_LINEARIZED_RTI_OSQP:
        case RuntimeProfile::CPP_NONLINEAR_RTI_HPIPM:
        case RuntimeProfile::CPP_NONLINEAR_SQP_HPIPM:
        case RuntimeProfile::CPP_NONLINEAR_FULLNLP_IPOPT:
            return profile;
    }
    return RuntimeProfile::CPP_HYBRID_RTI_OSQP;
}

}  // namespace

UnifiedMpcRuntime::UnifiedMpcRuntime(
    const SatelliteParams& sat_params,
    const v2::MPCV2Params& mpc_params,
    const RuntimeConfig& runtime_config
) : profile_(normalize_profile(runtime_config.profile)),
    num_controls_(sat_params.num_rw + sat_params.num_thrusters + 1) {
    prediction_horizon_ = mpc_params.prediction_horizon;
    dt_ = mpc_params.dt;
    current_path_s_ = 0.0;
    switch (profile_) {
        case RuntimeProfile::CPP_HYBRID_RTI_OSQP:
        case RuntimeProfile::CPP_NONLINEAR_RTI_OSQP:
        case RuntimeProfile::CPP_LINEARIZED_RTI_OSQP:
            osqp_controller_ = std::make_unique<v2::SQPController>(sat_params, mpc_params);
            casadi_params_ = osqp_controller_->casadi_params();
            break;
        case RuntimeProfile::CPP_NONLINEAR_RTI_HPIPM:
            if (has_acados_dependencies()) {
                unavailable_reason_ = (
                    "acados dependencies detected but native acados adapter is not "
                    "enabled in _cpp_mpc_runtime for this build."
                );
            } else {
                unavailable_reason_ = (
                    "acados RTI backend unavailable: missing acados dependencies "
                    "(expected ACADOS_SOURCE_DIR with acados/hpipm/blasfeo libs)."
                );
            }
            break;
        case RuntimeProfile::CPP_NONLINEAR_SQP_HPIPM:
            if (has_acados_dependencies()) {
                unavailable_reason_ = (
                    "acados dependencies detected but native acados adapter is not "
                    "enabled in _cpp_mpc_runtime for this build."
                );
            } else {
                unavailable_reason_ = (
                    "acados SQP backend unavailable: missing acados dependencies "
                    "(expected ACADOS_SOURCE_DIR with acados/hpipm/blasfeo libs)."
                );
            }
            break;
        case RuntimeProfile::CPP_NONLINEAR_FULLNLP_IPOPT:
            if (has_ipopt_dependencies() && has_casadi_cpp_dependencies()) {
                unavailable_reason_ = (
                    "IPOPT/CasADi C++ dependencies detected but native full-NLP adapter "
                    "is not enabled in _cpp_mpc_runtime for this build."
                );
            } else {
                unavailable_reason_ = (
                    "IPOPT full-NLP backend unavailable: missing IPOPT and/or CasADi C++ "
                    "dependencies required for native full-NLP adapter."
                );
            }
            break;
    }
}

RuntimeResult UnifiedMpcRuntime::solve_step(const VectorXd& x_current) {
    if (!osqp_controller_) {
        return unavailable_result(
            num_controls_,
            unavailable_reason_,
            current_path_s_);
    }

    const auto raw = osqp_controller_->get_control_action(x_current);
    RuntimeResult result;
    result.u = raw.u;
    result.status = raw.status;
    result.solver_status = raw.solver_status;
    result.iterations = raw.iterations;
    result.objective = raw.objective;
    result.solve_time = raw.solve_time;
    result.timeout = raw.timeout;
    result.fallback_active = raw.fallback_active;
    result.fallback_age_s = raw.fallback_age_s;
    result.fallback_scale = raw.fallback_scale;
    result.path_s = raw.path_s;
    result.path_s_proj = raw.path_s_proj;
    result.path_s_pred = raw.path_s_pred;
    result.path_error = raw.path_error;
    result.path_endpoint_error = raw.path_endpoint_error;
    result.t_linearization_s = raw.t_linearization_s;
    result.t_cost_update_s = raw.t_cost_update_s;
    result.t_constraint_update_s = raw.t_constraint_update_s;
    result.t_matrix_update_s = raw.t_matrix_update_s;
    result.t_warmstart_s = raw.t_warmstart_s;
    result.t_solve_only_s = raw.t_solve_only_s;
    result.sqp_iterations = raw.sqp_iterations;
    result.sqp_kkt_residual = raw.sqp_kkt_residual;
    current_path_s_ = result.path_s;
    return result;
}

RuntimeResult UnifiedMpcRuntime::get_control_action(const VectorXd& x_current) {
    return solve_step(x_current);
}

void UnifiedMpcRuntime::set_path_data(const std::vector<std::array<double, 4>>& path_data) {
    if (osqp_controller_) {
        osqp_controller_->set_path_data(path_data);
    }
}

void UnifiedMpcRuntime::set_runtime_mode(const std::string& mode) {
    if (osqp_controller_) {
        osqp_controller_->set_runtime_mode(mode);
    }
}

void UnifiedMpcRuntime::set_current_path_s(double s_value) {
    current_path_s_ = std::max(0.0, s_value);
    if (osqp_controller_) {
        osqp_controller_->set_current_path_s(current_path_s_);
    }
}

void UnifiedMpcRuntime::set_scan_attitude_context(
    const Vector3d& center,
    const Vector3d& axis,
    const std::string& direction
) {
    if (osqp_controller_) {
        osqp_controller_->set_scan_attitude_context(center, axis, direction);
    }
}

void UnifiedMpcRuntime::clear_scan_attitude_context() {
    if (osqp_controller_) {
        osqp_controller_->clear_scan_attitude_context();
    }
}

void UnifiedMpcRuntime::set_warm_start_control(const VectorXd& u_prev) {
    if (osqp_controller_) {
        osqp_controller_->set_warm_start_control(u_prev);
    }
}

void UnifiedMpcRuntime::set_stage_linearisation(
    int k,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const VectorXd& d
) {
    if (osqp_controller_) {
        osqp_controller_->set_stage_linearisation(k, A, B, d);
    }
}

void UnifiedMpcRuntime::set_all_linearisations(
    const std::vector<Eigen::MatrixXd>& As,
    const std::vector<Eigen::MatrixXd>& Bs,
    const std::vector<VectorXd>& ds
) {
    if (osqp_controller_) {
        osqp_controller_->set_all_linearisations(As, Bs, ds);
    }
}

std::tuple<double, Vector3d, double, double> UnifiedMpcRuntime::project_onto_path(
    const Vector3d& position
) const {
    if (osqp_controller_) {
        return osqp_controller_->project_onto_path(position);
    }
    return std::make_tuple(current_path_s_, Vector3d::Zero(), 0.0, 0.0);
}

std::tuple<Vector3d, Vector3d, Vector4d> UnifiedMpcRuntime::get_reference_at_s(
    double s_query,
    const Vector4d& q_current
) const {
    if (osqp_controller_) {
        return osqp_controller_->get_reference_at_s(s_query, q_current);
    }
    return std::make_tuple(Vector3d::Zero(), Vector3d::Zero(), q_current);
}

VectorXd UnifiedMpcRuntime::get_stage_state(int k) const {
    if (osqp_controller_) {
        return osqp_controller_->get_stage_state(k);
    }
    return VectorXd::Zero(17);
}

VectorXd UnifiedMpcRuntime::get_stage_control(int k) const {
    if (osqp_controller_) {
        return osqp_controller_->get_stage_control(k);
    }
    return VectorXd::Zero(num_controls_);
}

const VectorXd& UnifiedMpcRuntime::casadi_params() const {
    return casadi_params_;
}

int UnifiedMpcRuntime::num_controls() const {
    if (osqp_controller_) {
        return osqp_controller_->num_controls();
    }
    return num_controls_;
}

int UnifiedMpcRuntime::prediction_horizon() const {
    if (osqp_controller_) {
        return osqp_controller_->prediction_horizon();
    }
    return prediction_horizon_;
}

double UnifiedMpcRuntime::dt() const {
    if (osqp_controller_) {
        return osqp_controller_->dt();
    }
    return dt_;
}

double UnifiedMpcRuntime::path_length() const {
    if (osqp_controller_) {
        return osqp_controller_->path_length();
    }
    return 0.0;
}

bool UnifiedMpcRuntime::has_path() const {
    if (osqp_controller_) {
        return osqp_controller_->has_path();
    }
    return false;
}

double UnifiedMpcRuntime::current_path_s() const {
    if (osqp_controller_) {
        return osqp_controller_->current_path_s();
    }
    return current_path_s_;
}

bool UnifiedMpcRuntime::backend_available() const {
    return osqp_controller_ != nullptr;
}

std::string UnifiedMpcRuntime::unavailable_reason() const {
    return unavailable_reason_;
}

bool UnifiedMpcRuntime::has_acados_backend() {
    return false;
}

bool UnifiedMpcRuntime::has_ipopt_backend() {
    return false;
}

bool UnifiedMpcRuntime::has_acados_dependencies() {
#ifdef SAT_HAS_ACADOS
    return true;
#else
    return false;
#endif
}

bool UnifiedMpcRuntime::has_ipopt_dependencies() {
#ifdef SAT_HAS_IPOPT
    return true;
#else
    return false;
#endif
}

bool UnifiedMpcRuntime::has_casadi_cpp_dependencies() {
#ifdef SAT_HAS_CASADI_CPP
    return true;
#else
    return false;
#endif
}

}  // namespace runtime
}  // namespace satellite_control

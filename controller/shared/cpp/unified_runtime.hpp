#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "../../hybrid/cpp/sqp_controller.hpp"

namespace satellite_control {
namespace runtime {

using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

enum class RuntimeProfile {
    CPP_HYBRID_RTI_OSQP = 0,
    CPP_NONLINEAR_RTI_OSQP = 1,
    CPP_LINEARIZED_RTI_OSQP = 2,
    CPP_NONLINEAR_RTI_HPIPM = 3,
    CPP_NONLINEAR_SQP_HPIPM = 4,
    CPP_NONLINEAR_FULLNLP_IPOPT = 5,
};

struct RuntimeConfig {
    RuntimeProfile profile = RuntimeProfile::CPP_HYBRID_RTI_OSQP;
};

struct RuntimeState {
    VectorXd x_current;
};

struct RuntimeResult {
    VectorXd u;
    int status = -1;
    std::string solver_status = "unavailable_backend";
    int iterations = 0;
    double objective = 0.0;
    double solve_time = 0.0;
    bool timeout = false;
    bool fallback_active = true;
    double fallback_age_s = 0.0;
    double fallback_scale = 0.0;
    double path_s = 0.0;
    double path_s_proj = 0.0;
    double path_s_pred = 0.0;
    double path_error = 0.0;
    double path_endpoint_error = 0.0;
    double t_linearization_s = 0.0;
    double t_cost_update_s = 0.0;
    double t_constraint_update_s = 0.0;
    double t_matrix_update_s = 0.0;
    double t_warmstart_s = 0.0;
    double t_solve_only_s = 0.0;
    int sqp_iterations = 0;
    double sqp_kkt_residual = 0.0;
    std::string unavailable_reason;
};

class UnifiedMpcRuntime {
public:
    UnifiedMpcRuntime(
        const SatelliteParams& sat_params,
        const v2::MPCV2Params& mpc_params,
        const RuntimeConfig& runtime_config);

    RuntimeResult solve_step(const VectorXd& x_current);
    RuntimeResult get_control_action(const VectorXd& x_current);

    void set_path_data(const std::vector<std::array<double, 4>>& path_data);
    void set_runtime_mode(const std::string& mode);
    void set_current_path_s(double s_value);
    void set_scan_attitude_context(
        const Vector3d& center,
        const Vector3d& axis,
        const std::string& direction);
    void clear_scan_attitude_context();
    void set_warm_start_control(const VectorXd& u_prev);
    void set_stage_linearisation(
        int k,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B,
        const VectorXd& d);
    void set_all_linearisations(
        const std::vector<Eigen::MatrixXd>& As,
        const std::vector<Eigen::MatrixXd>& Bs,
        const std::vector<VectorXd>& ds);
    std::tuple<double, Vector3d, double, double> project_onto_path(
        const Vector3d& position) const;
    std::tuple<Vector3d, Vector3d, Vector4d> get_reference_at_s(
        double s_query,
        const Vector4d& q_current) const;
    VectorXd get_stage_state(int k) const;
    VectorXd get_stage_control(int k) const;
    const VectorXd& casadi_params() const;

    int num_controls() const;
    int prediction_horizon() const;
    double dt() const;
    double path_length() const;
    bool has_path() const;
    double current_path_s() const;

    bool backend_available() const;
    std::string unavailable_reason() const;

    static bool has_acados_backend();
    static bool has_ipopt_backend();
    static bool has_acados_dependencies();
    static bool has_ipopt_dependencies();
    static bool has_casadi_cpp_dependencies();

private:
    RuntimeProfile profile_;
    std::unique_ptr<v2::SQPController> osqp_controller_;
    int num_controls_;
    int prediction_horizon_;
    double dt_;
    double current_path_s_;
    VectorXd casadi_params_;
    std::string unavailable_reason_;
};

}  // namespace runtime
}  // namespace satellite_control

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "unified_runtime.hpp"

namespace py = pybind11;
using namespace satellite_control;
using namespace satellite_control::runtime;

PYBIND11_MODULE(_cpp_mpc_runtime, m) {
    m.doc() = "Unified MPC runtime facade (OSQP implemented, acados/IPOPT optional)";

    py::enum_<RuntimeProfile>(m, "RuntimeProfile")
        .value("CPP_HYBRID_RTI_OSQP", RuntimeProfile::CPP_HYBRID_RTI_OSQP)
        .value("CPP_NONLINEAR_RTI_OSQP", RuntimeProfile::CPP_NONLINEAR_RTI_OSQP)
        .value("CPP_LINEARIZED_RTI_OSQP", RuntimeProfile::CPP_LINEARIZED_RTI_OSQP)
        .value("CPP_NONLINEAR_RTI_HPIPM", RuntimeProfile::CPP_NONLINEAR_RTI_HPIPM)
        .value("CPP_NONLINEAR_SQP_HPIPM", RuntimeProfile::CPP_NONLINEAR_SQP_HPIPM)
        .value(
            "CPP_NONLINEAR_FULLNLP_IPOPT",
            RuntimeProfile::CPP_NONLINEAR_FULLNLP_IPOPT
        );

    py::class_<RuntimeConfig>(m, "RuntimeConfig")
        .def(py::init<>())
        .def_readwrite("profile", &RuntimeConfig::profile);

    py::class_<RuntimeState>(m, "RuntimeState")
        .def(py::init<>())
        .def_readwrite("x_current", &RuntimeState::x_current);

    py::class_<RuntimeResult>(m, "RuntimeResult")
        .def(py::init<>())
        .def_readwrite("u", &RuntimeResult::u)
        .def_readwrite("status", &RuntimeResult::status)
        .def_readwrite("solver_status", &RuntimeResult::solver_status)
        .def_readwrite("iterations", &RuntimeResult::iterations)
        .def_readwrite("objective", &RuntimeResult::objective)
        .def_readwrite("solve_time", &RuntimeResult::solve_time)
        .def_readwrite("timeout", &RuntimeResult::timeout)
        .def_readwrite("fallback_active", &RuntimeResult::fallback_active)
        .def_readwrite("fallback_age_s", &RuntimeResult::fallback_age_s)
        .def_readwrite("fallback_scale", &RuntimeResult::fallback_scale)
        .def_readwrite("path_s", &RuntimeResult::path_s)
        .def_readwrite("path_s_proj", &RuntimeResult::path_s_proj)
        .def_readwrite("path_s_pred", &RuntimeResult::path_s_pred)
        .def_readwrite("path_error", &RuntimeResult::path_error)
        .def_readwrite("path_endpoint_error", &RuntimeResult::path_endpoint_error)
        .def_readwrite("t_linearization_s", &RuntimeResult::t_linearization_s)
        .def_readwrite("t_cost_update_s", &RuntimeResult::t_cost_update_s)
        .def_readwrite("t_constraint_update_s", &RuntimeResult::t_constraint_update_s)
        .def_readwrite("t_matrix_update_s", &RuntimeResult::t_matrix_update_s)
        .def_readwrite("t_warmstart_s", &RuntimeResult::t_warmstart_s)
        .def_readwrite("t_solve_only_s", &RuntimeResult::t_solve_only_s)
        .def_readwrite("sqp_iterations", &RuntimeResult::sqp_iterations)
        .def_readwrite("sqp_kkt_residual", &RuntimeResult::sqp_kkt_residual)
        .def_readwrite("unavailable_reason", &RuntimeResult::unavailable_reason);

    py::class_<SatelliteParams>(m, "SatelliteParams", py::module_local())
        .def(py::init<>())
        .def_readwrite("dt", &SatelliteParams::dt)
        .def_readwrite("mass", &SatelliteParams::mass)
        .def_readwrite("inertia", &SatelliteParams::inertia)
        .def_readwrite("num_thrusters", &SatelliteParams::num_thrusters)
        .def_readwrite("num_rw", &SatelliteParams::num_rw)
        .def_readwrite("thruster_positions", &SatelliteParams::thruster_positions)
        .def_readwrite("thruster_directions", &SatelliteParams::thruster_directions)
        .def_readwrite("thruster_forces", &SatelliteParams::thruster_forces)
        .def_readwrite("rw_torque_limits", &SatelliteParams::rw_torque_limits)
        .def_readwrite("rw_inertia", &SatelliteParams::rw_inertia)
        .def_readwrite("rw_speed_limits", &SatelliteParams::rw_speed_limits)
        .def_readwrite("rw_axes", &SatelliteParams::rw_axes)
        .def_readwrite("com_offset", &SatelliteParams::com_offset)
        .def_readwrite("orbital_mean_motion", &SatelliteParams::orbital_mean_motion)
        .def_readwrite("orbital_mu", &SatelliteParams::orbital_mu)
        .def_readwrite("orbital_radius", &SatelliteParams::orbital_radius)
        .def_readwrite("use_two_body", &SatelliteParams::use_two_body);

    py::class_<v2::MPCV2Params>(m, "MPCV2Params", py::module_local())
        .def(py::init<>())
        .def_readwrite("prediction_horizon", &v2::MPCV2Params::prediction_horizon)
        .def_readwrite("control_horizon", &v2::MPCV2Params::control_horizon)
        .def_readwrite("dt", &v2::MPCV2Params::dt)
        .def_readwrite("solver_time_limit", &v2::MPCV2Params::solver_time_limit)
        .def_readwrite("verbose", &v2::MPCV2Params::verbose)
        .def_readwrite("Q_contour", &v2::MPCV2Params::Q_contour)
        .def_readwrite("Q_lag", &v2::MPCV2Params::Q_lag)
        .def_readwrite("Q_progress", &v2::MPCV2Params::Q_progress)
        .def_readwrite("progress_reward", &v2::MPCV2Params::progress_reward)
        .def_readwrite("Q_velocity_align", &v2::MPCV2Params::Q_velocity_align)
        .def_readwrite("Q_s_anchor", &v2::MPCV2Params::Q_s_anchor)
        .def_readwrite("Q_attitude", &v2::MPCV2Params::Q_attitude)
        .def_readwrite("Q_axis_align", &v2::MPCV2Params::Q_axis_align)
        .def_readwrite("Q_quat_norm", &v2::MPCV2Params::Q_quat_norm)
        .def_readwrite("Q_angvel", &v2::MPCV2Params::Q_angvel)
        .def_readwrite("R_thrust", &v2::MPCV2Params::R_thrust)
        .def_readwrite("R_rw_torque", &v2::MPCV2Params::R_rw_torque)
        .def_readwrite("Q_smooth", &v2::MPCV2Params::Q_smooth)
        .def_readwrite("thrust_pair_weight", &v2::MPCV2Params::thrust_pair_weight)
        .def_readwrite("thrust_l1_weight", &v2::MPCV2Params::thrust_l1_weight)
        .def_readwrite("path_speed", &v2::MPCV2Params::path_speed)
        .def_readwrite("path_speed_min", &v2::MPCV2Params::path_speed_min)
        .def_readwrite("path_speed_max", &v2::MPCV2Params::path_speed_max)
        .def_readwrite(
            "ref_tangent_lookahead_m",
            &v2::MPCV2Params::ref_tangent_lookahead_m)
        .def_readwrite(
            "ref_tangent_lookback_m",
            &v2::MPCV2Params::ref_tangent_lookback_m)
        .def_readwrite(
            "ref_quat_max_rate_rad_s",
            &v2::MPCV2Params::ref_quat_max_rate_rad_s)
        .def_readwrite(
            "ref_quat_terminal_rate_scale",
            &v2::MPCV2Params::ref_quat_terminal_rate_scale)
        .def_readwrite("Q_terminal_pos", &v2::MPCV2Params::Q_terminal_pos)
        .def_readwrite("Q_terminal_s", &v2::MPCV2Params::Q_terminal_s)
        .def_readwrite("Q_terminal_att", &v2::MPCV2Params::Q_terminal_att)
        .def_readwrite("Q_terminal_angvel", &v2::MPCV2Params::Q_terminal_angvel)
        .def_readwrite("Q_terminal_vel", &v2::MPCV2Params::Q_terminal_vel)
        .def_readwrite(
            "enable_dare_terminal",
            &v2::MPCV2Params::enable_dare_terminal)
        .def_readwrite(
            "dare_update_period_steps",
            &v2::MPCV2Params::dare_update_period_steps)
        .def_readwrite("terminal_cost_profile", &v2::MPCV2Params::terminal_cost_profile)
        .def_readwrite("max_linear_velocity", &v2::MPCV2Params::max_linear_velocity)
        .def_readwrite("max_angular_velocity", &v2::MPCV2Params::max_angular_velocity)
        .def_readwrite("progress_policy", &v2::MPCV2Params::progress_policy)
        .def_readwrite("error_priority_min_vs", &v2::MPCV2Params::error_priority_min_vs)
        .def_readwrite(
            "error_priority_error_speed_gain",
            &v2::MPCV2Params::error_priority_error_speed_gain)
        .def_readwrite("sqp_max_iter", &v2::MPCV2Params::sqp_max_iter)
        .def_readwrite("sqp_tol", &v2::MPCV2Params::sqp_tol)
        .def_readwrite("osqp_max_iter", &v2::MPCV2Params::osqp_max_iter)
        .def_readwrite("osqp_eps_abs", &v2::MPCV2Params::osqp_eps_abs)
        .def_readwrite("osqp_eps_rel", &v2::MPCV2Params::osqp_eps_rel)
        .def_readwrite("osqp_warm_start", &v2::MPCV2Params::osqp_warm_start)
        .def_readwrite("recover_contour_scale", &v2::MPCV2Params::recover_contour_scale)
        .def_readwrite("recover_lag_scale", &v2::MPCV2Params::recover_lag_scale)
        .def_readwrite("recover_progress_scale", &v2::MPCV2Params::recover_progress_scale)
        .def_readwrite("recover_attitude_scale", &v2::MPCV2Params::recover_attitude_scale)
        .def_readwrite("settle_progress_scale", &v2::MPCV2Params::settle_progress_scale)
        .def_readwrite(
            "settle_terminal_pos_scale",
            &v2::MPCV2Params::settle_terminal_pos_scale)
        .def_readwrite(
            "settle_terminal_attitude_scale",
            &v2::MPCV2Params::settle_terminal_attitude_scale)
        .def_readwrite(
            "settle_velocity_align_scale",
            &v2::MPCV2Params::settle_velocity_align_scale)
        .def_readwrite(
            "settle_angular_velocity_scale",
            &v2::MPCV2Params::settle_angular_velocity_scale)
        .def_readwrite("hold_smoothness_scale", &v2::MPCV2Params::hold_smoothness_scale)
        .def_readwrite(
            "hold_thruster_pair_scale",
            &v2::MPCV2Params::hold_thruster_pair_scale)
        .def_readwrite("solver_fallback_hold_s", &v2::MPCV2Params::solver_fallback_hold_s)
        .def_readwrite(
            "solver_fallback_decay_s",
            &v2::MPCV2Params::solver_fallback_decay_s)
        .def_readwrite(
            "solver_fallback_zero_after_s",
            &v2::MPCV2Params::solver_fallback_zero_after_s);

    py::class_<UnifiedMpcRuntime>(m, "UnifiedMpcRuntime")
        .def(
            py::init<const SatelliteParams&, const v2::MPCV2Params&, const RuntimeConfig&>(),
            py::arg("sat_params"),
            py::arg("mpc_params"),
            py::arg("runtime_config"))
        .def("solve_step", &UnifiedMpcRuntime::solve_step, py::arg("x_current"))
        .def(
            "get_control_action",
            &UnifiedMpcRuntime::get_control_action,
            py::arg("x_current"))
        .def("set_path_data", &UnifiedMpcRuntime::set_path_data, py::arg("path_data"))
        .def("set_runtime_mode", &UnifiedMpcRuntime::set_runtime_mode, py::arg("mode"))
        .def(
            "set_current_path_s",
            &UnifiedMpcRuntime::set_current_path_s,
            py::arg("s_value"))
        .def(
            "set_scan_attitude_context",
            &UnifiedMpcRuntime::set_scan_attitude_context,
            py::arg("center"),
            py::arg("axis"),
            py::arg("direction"))
        .def(
            "clear_scan_attitude_context",
            &UnifiedMpcRuntime::clear_scan_attitude_context)
        .def(
            "set_warm_start_control",
            &UnifiedMpcRuntime::set_warm_start_control,
            py::arg("u_prev"))
        .def(
            "set_stage_linearisation",
            &UnifiedMpcRuntime::set_stage_linearisation,
            py::arg("k"),
            py::arg("A"),
            py::arg("B"),
            py::arg("d"))
        .def(
            "set_all_linearisations",
            &UnifiedMpcRuntime::set_all_linearisations,
            py::arg("As"),
            py::arg("Bs"),
            py::arg("ds"))
        .def(
            "project_onto_path",
            &UnifiedMpcRuntime::project_onto_path,
            py::arg("position"))
        .def(
            "get_reference_at_s",
            &UnifiedMpcRuntime::get_reference_at_s,
            py::arg("s_query"),
            py::arg("q_current"))
        .def("get_stage_state", &UnifiedMpcRuntime::get_stage_state, py::arg("k"))
        .def(
            "get_stage_control",
            &UnifiedMpcRuntime::get_stage_control,
            py::arg("k"))
        .def_property_readonly("casadi_params", &UnifiedMpcRuntime::casadi_params)
        .def_property_readonly("num_controls", &UnifiedMpcRuntime::num_controls)
        .def_property_readonly(
            "prediction_horizon",
            &UnifiedMpcRuntime::prediction_horizon)
        .def_property_readonly("dt", &UnifiedMpcRuntime::dt)
        .def_property_readonly("path_length", &UnifiedMpcRuntime::path_length)
        .def_property_readonly("has_path", &UnifiedMpcRuntime::has_path)
        .def_property_readonly("current_path_s", &UnifiedMpcRuntime::current_path_s)
        .def_property_readonly(
            "backend_available",
            &UnifiedMpcRuntime::backend_available)
        .def_property_readonly(
            "unavailable_reason",
            &UnifiedMpcRuntime::unavailable_reason)
        .def_static("has_acados_backend", &UnifiedMpcRuntime::has_acados_backend)
        .def_static("has_ipopt_backend", &UnifiedMpcRuntime::has_ipopt_backend)
        .def_static(
            "has_acados_dependencies",
            &UnifiedMpcRuntime::has_acados_dependencies)
        .def_static(
            "has_ipopt_dependencies",
            &UnifiedMpcRuntime::has_ipopt_dependencies)
        .def_static(
            "has_casadi_cpp_dependencies",
            &UnifiedMpcRuntime::has_casadi_cpp_dependencies);

    m.attr("HAS_ACADOS_BACKEND") = py::bool_(UnifiedMpcRuntime::has_acados_backend());
    m.attr("HAS_IPOPT_BACKEND") = py::bool_(UnifiedMpcRuntime::has_ipopt_backend());
    m.attr("HAS_ACADOS_DEPENDENCIES") = py::bool_(
        UnifiedMpcRuntime::has_acados_dependencies());
    m.attr("HAS_IPOPT_DEPENDENCIES") = py::bool_(
        UnifiedMpcRuntime::has_ipopt_dependencies());
    m.attr("HAS_CASADI_CPP_DEPENDENCIES") = py::bool_(
        UnifiedMpcRuntime::has_casadi_cpp_dependencies());
}

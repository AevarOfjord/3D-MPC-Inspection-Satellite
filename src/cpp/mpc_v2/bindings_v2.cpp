/**
 * @file bindings_v2.cpp
 * @brief pybind11 bindings for the V2 RTI-SQP MPC controller.
 *
 * Exposes the _cpp_mpc module with:
 *   - MPCV2Params
 *   - ControlResultV2
 *   - SQPController (main class)
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "sqp_controller.hpp"

namespace py = pybind11;
using namespace satellite_control;
using namespace satellite_control::v2;

PYBIND11_MODULE(_cpp_mpc, m) {
    m.doc() = "RTI-SQP MPC backend (CasADi + OSQP)";

    // -----------------------------------------------------------------------
    // MPCV2Params
    // -----------------------------------------------------------------------
    py::class_<MPCV2Params>(m, "MPCV2Params")
        .def(py::init<>())
        // Horizon
        .def_readwrite("prediction_horizon",   &MPCV2Params::prediction_horizon)
        .def_readwrite("control_horizon",      &MPCV2Params::control_horizon)
        .def_readwrite("dt",                   &MPCV2Params::dt)
        .def_readwrite("solver_time_limit",    &MPCV2Params::solver_time_limit)
        .def_readwrite("verbose",              &MPCV2Params::verbose)
        // Cost weights
        .def_readwrite("Q_contour",            &MPCV2Params::Q_contour)
        .def_readwrite("Q_lag",                &MPCV2Params::Q_lag)
        .def_readwrite("Q_progress",           &MPCV2Params::Q_progress)
        .def_readwrite("progress_reward",      &MPCV2Params::progress_reward)
        .def_readwrite("Q_velocity_align",     &MPCV2Params::Q_velocity_align)
        .def_readwrite("Q_s_anchor",           &MPCV2Params::Q_s_anchor)
        .def_readwrite("Q_attitude",           &MPCV2Params::Q_attitude)
        .def_readwrite("Q_axis_align",         &MPCV2Params::Q_axis_align)
        .def_readwrite("Q_quat_norm",          &MPCV2Params::Q_quat_norm)
        .def_readwrite("Q_angvel",             &MPCV2Params::Q_angvel)
        .def_readwrite("R_thrust",             &MPCV2Params::R_thrust)
        .def_readwrite("R_rw_torque",          &MPCV2Params::R_rw_torque)
        .def_readwrite("Q_smooth",             &MPCV2Params::Q_smooth)
        .def_readwrite("thrust_pair_weight",   &MPCV2Params::thrust_pair_weight)
        .def_readwrite("thrust_l1_weight",     &MPCV2Params::thrust_l1_weight)
        // Path following
        .def_readwrite("path_speed",           &MPCV2Params::path_speed)
        .def_readwrite("path_speed_min",       &MPCV2Params::path_speed_min)
        .def_readwrite("path_speed_max",       &MPCV2Params::path_speed_max)
        // Terminal
        .def_readwrite("Q_terminal_pos",       &MPCV2Params::Q_terminal_pos)
        .def_readwrite("Q_terminal_s",         &MPCV2Params::Q_terminal_s)
        .def_readwrite("Q_terminal_att",       &MPCV2Params::Q_terminal_att)
        .def_readwrite("Q_terminal_angvel",    &MPCV2Params::Q_terminal_angvel)
        .def_readwrite("Q_terminal_vel",       &MPCV2Params::Q_terminal_vel)
        // DARE
        .def_readwrite("enable_dare_terminal", &MPCV2Params::enable_dare_terminal)
        .def_readwrite("dare_update_period_steps", &MPCV2Params::dare_update_period_steps)
        .def_readwrite("terminal_cost_profile", &MPCV2Params::terminal_cost_profile)
        // Robustness
        .def_readwrite("robustness_mode",      &MPCV2Params::robustness_mode)
        .def_readwrite("constraint_tightening_scale", &MPCV2Params::constraint_tightening_scale)
        .def_readwrite("tube_feedback_gain_scale",    &MPCV2Params::tube_feedback_gain_scale)
        .def_readwrite("tube_feedback_max_correction", &MPCV2Params::tube_feedback_max_correction)
        // Velocity bounds
        .def_readwrite("max_linear_velocity",  &MPCV2Params::max_linear_velocity)
        .def_readwrite("max_angular_velocity", &MPCV2Params::max_angular_velocity)
        // Progress policy
        .def_readwrite("progress_policy",      &MPCV2Params::progress_policy)
        .def_readwrite("error_priority_min_vs", &MPCV2Params::error_priority_min_vs)
        .def_readwrite("error_priority_error_speed_gain", &MPCV2Params::error_priority_error_speed_gain)
        // SQP settings
        .def_readwrite("sqp_max_iter",         &MPCV2Params::sqp_max_iter)
        .def_readwrite("sqp_tol",              &MPCV2Params::sqp_tol)
        // OSQP settings
        .def_readwrite("osqp_max_iter",        &MPCV2Params::osqp_max_iter)
        .def_readwrite("osqp_eps_abs",         &MPCV2Params::osqp_eps_abs)
        .def_readwrite("osqp_eps_rel",         &MPCV2Params::osqp_eps_rel)
        .def_readwrite("osqp_warm_start",      &MPCV2Params::osqp_warm_start)
        // Mode scaling
        .def_readwrite("recover_contour_scale",  &MPCV2Params::recover_contour_scale)
        .def_readwrite("recover_lag_scale",      &MPCV2Params::recover_lag_scale)
        .def_readwrite("recover_progress_scale", &MPCV2Params::recover_progress_scale)
        .def_readwrite("recover_attitude_scale", &MPCV2Params::recover_attitude_scale)
        .def_readwrite("settle_progress_scale",  &MPCV2Params::settle_progress_scale)
        .def_readwrite("settle_terminal_pos_scale",       &MPCV2Params::settle_terminal_pos_scale)
        .def_readwrite("settle_terminal_attitude_scale",  &MPCV2Params::settle_terminal_attitude_scale)
        .def_readwrite("settle_velocity_align_scale",     &MPCV2Params::settle_velocity_align_scale)
        .def_readwrite("settle_angular_velocity_scale",   &MPCV2Params::settle_angular_velocity_scale)
        .def_readwrite("hold_smoothness_scale",    &MPCV2Params::hold_smoothness_scale)
        .def_readwrite("hold_thruster_pair_scale", &MPCV2Params::hold_thruster_pair_scale)
        // Fallback
        .def_readwrite("solver_fallback_hold_s",  &MPCV2Params::solver_fallback_hold_s)
        .def_readwrite("solver_fallback_decay_s", &MPCV2Params::solver_fallback_decay_s)
        .def_readwrite("solver_fallback_zero_after_s", &MPCV2Params::solver_fallback_zero_after_s);

    // -----------------------------------------------------------------------
    // ControlResultV2
    // -----------------------------------------------------------------------
    py::class_<ControlResultV2>(m, "ControlResultV2")
        .def(py::init<>())
        .def_readwrite("u",                    &ControlResultV2::u)
        .def_readwrite("status",               &ControlResultV2::status)
        .def_readwrite("solver_status",        &ControlResultV2::solver_status)
        .def_readwrite("iterations",           &ControlResultV2::iterations)
        .def_readwrite("objective",            &ControlResultV2::objective)
        .def_readwrite("solve_time",           &ControlResultV2::solve_time)
        .def_readwrite("timeout",              &ControlResultV2::timeout)
        .def_readwrite("path_s",               &ControlResultV2::path_s)
        .def_readwrite("path_s_proj",          &ControlResultV2::path_s_proj)
        .def_readwrite("path_s_pred",          &ControlResultV2::path_s_pred)
        .def_readwrite("path_error",           &ControlResultV2::path_error)
        .def_readwrite("path_endpoint_error",  &ControlResultV2::path_endpoint_error)
        .def_readwrite("fallback_active",      &ControlResultV2::fallback_active)
        .def_readwrite("fallback_age_s",       &ControlResultV2::fallback_age_s)
        .def_readwrite("fallback_scale",       &ControlResultV2::fallback_scale)
        .def_readwrite("t_linearization_s",    &ControlResultV2::t_linearization_s)
        .def_readwrite("t_cost_update_s",      &ControlResultV2::t_cost_update_s)
        .def_readwrite("t_constraint_update_s", &ControlResultV2::t_constraint_update_s)
        .def_readwrite("t_matrix_update_s",    &ControlResultV2::t_matrix_update_s)
        .def_readwrite("t_warmstart_s",        &ControlResultV2::t_warmstart_s)
        .def_readwrite("t_solve_only_s",       &ControlResultV2::t_solve_only_s)
        .def_readwrite("sqp_iterations",       &ControlResultV2::sqp_iterations)
        .def_readwrite("sqp_kkt_residual",     &ControlResultV2::sqp_kkt_residual);

    // -----------------------------------------------------------------------
    // SQPController
    // -----------------------------------------------------------------------
    // SatelliteParams binding (module_local to avoid type conflict with other modules)
    py::class_<SatelliteParams>(m, "SatelliteParams", py::module_local())
        .def(py::init<>())
        .def_readwrite("dt",                   &SatelliteParams::dt)
        .def_readwrite("mass",                 &SatelliteParams::mass)
        .def_readwrite("inertia",              &SatelliteParams::inertia)
        .def_readwrite("num_thrusters",        &SatelliteParams::num_thrusters)
        .def_readwrite("num_rw",               &SatelliteParams::num_rw)
        .def_readwrite("thruster_positions",   &SatelliteParams::thruster_positions)
        .def_readwrite("thruster_directions",  &SatelliteParams::thruster_directions)
        .def_readwrite("thruster_forces",      &SatelliteParams::thruster_forces)
        .def_readwrite("rw_torque_limits",     &SatelliteParams::rw_torque_limits)
        .def_readwrite("rw_inertia",           &SatelliteParams::rw_inertia)
        .def_readwrite("rw_speed_limits",      &SatelliteParams::rw_speed_limits)
        .def_readwrite("rw_axes",              &SatelliteParams::rw_axes)
        .def_readwrite("com_offset",           &SatelliteParams::com_offset)
        .def_readwrite("orbital_mean_motion",  &SatelliteParams::orbital_mean_motion)
        .def_readwrite("orbital_mu",           &SatelliteParams::orbital_mu)
        .def_readwrite("orbital_radius",       &SatelliteParams::orbital_radius)
        .def_readwrite("use_two_body",         &SatelliteParams::use_two_body);

    // SQPController
    py::class_<SQPController>(m, "SQPController")
        .def(py::init<const SatelliteParams&, const MPCV2Params&>(),
             py::arg("sat_params"), py::arg("mpc_params"))
        .def("get_control_action", &SQPController::get_control_action,
             py::arg("x_current"),
             "Compute optimal control via RTI-SQP.")
        .def("set_warm_start_control", &SQPController::set_warm_start_control,
             py::arg("u_prev"))
        .def("set_path_data", &SQPController::set_path_data,
             py::arg("path_data"),
             "Set path: list of [s, x, y, z] arrays.")
        .def("set_scan_attitude_context", &SQPController::set_scan_attitude_context,
             py::arg("center"), py::arg("axis"), py::arg("direction"))
        .def("clear_scan_attitude_context", &SQPController::clear_scan_attitude_context)
        .def("set_runtime_mode", &SQPController::set_runtime_mode,
             py::arg("mode"))
        .def("project_onto_path", &SQPController::project_onto_path,
             py::arg("position"))
        .def("get_reference_at_s", &SQPController::get_reference_at_s,
             py::arg("s_query"), py::arg("q_current"))
        // Python-side CasADi linearisation injection
        .def("set_stage_linearisation", &SQPController::set_stage_linearisation,
             py::arg("k"), py::arg("A"), py::arg("B"), py::arg("d"),
             "Set linearisation data for horizon stage k.")
        .def("set_all_linearisations", &SQPController::set_all_linearisations,
             py::arg("As"), py::arg("Bs"), py::arg("ds"),
             "Set all stages' linearisation data at once.")
        .def("get_stage_state", &SQPController::get_stage_state,
             py::arg("k"), "Get state at horizon stage k.")
        .def("get_stage_control", &SQPController::get_stage_control,
             py::arg("k"), "Get control at horizon stage k.")
        .def_property_readonly("casadi_params", &SQPController::casadi_params)
        // Read-only properties
        .def_property_readonly("num_controls", &SQPController::num_controls)
        .def_property_readonly("prediction_horizon", &SQPController::prediction_horizon)
        .def_property_readonly("dt", &SQPController::dt)
        .def_property_readonly("path_length", &SQPController::path_length)
        .def_property_readonly("has_path", &SQPController::has_path)
        .def_property_readonly("current_path_s", &SQPController::current_path_s);
}

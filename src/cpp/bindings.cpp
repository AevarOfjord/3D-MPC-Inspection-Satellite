
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "linearizer.hpp"
#include "mpc_controller.hpp"
#include "obstacle.hpp"

namespace py = pybind11;
using satellite_control::ControlResult;
using satellite_control::Linearizer;
using satellite_control::MPCControllerCpp;
using satellite_control::MPCParams;
using satellite_control::Obstacle;
using satellite_control::ObstacleSet;
using satellite_control::ObstacleType;
using satellite_control::SatelliteParams;

PYBIND11_MODULE(_cpp_mpc, m) {
    m.doc() = "C++ backend for Satellite MPC controller";

    // Satellite Parameters
    py::class_<satellite_control::SatelliteParams>(m, "SatelliteParams")
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

    // Linearizer
    py::class_<satellite_control::Linearizer>(m, "Linearizer")
        .def(py::init<const satellite_control::SatelliteParams&>())
        .def("linearize", &satellite_control::Linearizer::linearize, "Compute Linearized Dynamics (A, B)");

    // MPC Parameters
    py::class_<satellite_control::MPCParams>(m, "MPCParams")
        .def(py::init<>())
        .def_readwrite("prediction_horizon", &MPCParams::prediction_horizon)
        .def_readwrite("control_horizon", &MPCParams::control_horizon)
        .def_readwrite("dt", &MPCParams::dt)
        .def_readwrite("solver_time_limit", &MPCParams::solver_time_limit)
        .def_readwrite("solver_type", &MPCParams::solver_type)
        .def_readwrite("verbose_mpc", &MPCParams::verbose_mpc)
        .def_readwrite("Q_angvel", &MPCParams::Q_angvel)
        .def_readwrite("Q_attitude", &MPCParams::Q_attitude)
        .def_readwrite("Q_axis_align", &MPCParams::Q_axis_align)
        .def_readwrite("Q_quat_norm", &MPCParams::Q_quat_norm)
        .def_readwrite("R_thrust", &MPCParams::R_thrust)
        .def_readwrite("R_rw_torque", &MPCParams::R_rw_torque)
        .def_readwrite("thrust_l1_weight", &MPCParams::thrust_l1_weight)
        .def_readwrite("thrust_pair_weight", &MPCParams::thrust_pair_weight)
        .def_readwrite("max_linear_velocity", &MPCParams::max_linear_velocity)
        .def_readwrite("max_angular_velocity", &MPCParams::max_angular_velocity)
        .def_readwrite("enable_delta_u_coupling", &MPCParams::enable_delta_u_coupling)
        .def_readwrite("enable_gyro_jacobian", &MPCParams::enable_gyro_jacobian)
        .def_readwrite(
            "auto_enable_gyro_jacobian",
            &MPCParams::auto_enable_gyro_jacobian
        )
        .def_readwrite(
            "gyro_enable_threshold_radps",
            &MPCParams::gyro_enable_threshold_radps
        )
        .def_readwrite("enable_auto_state_bounds", &MPCParams::enable_auto_state_bounds)
        .def_readwrite(
            "enable_online_dare_terminal",
            &MPCParams::enable_online_dare_terminal
        )
        .def_readwrite(
            "dare_update_period_steps",
            &MPCParams::dare_update_period_steps
        )
        .def_readwrite("terminal_cost_profile", &MPCParams::terminal_cost_profile)
        .def_readwrite("robustness_mode", &MPCParams::robustness_mode)
        .def_readwrite(
            "constraint_tightening_scale",
            &MPCParams::constraint_tightening_scale
        )
        .def_readwrite(
            "tube_feedback_gain_scale",
            &MPCParams::tube_feedback_gain_scale
        )
        .def_readwrite(
            "tube_feedback_max_correction",
            &MPCParams::tube_feedback_max_correction
        )
        .def_readwrite("enable_variable_scaling", &MPCParams::enable_variable_scaling)
        .def_readwrite("progress_policy", &MPCParams::progress_policy)
        .def_readwrite("error_priority_min_vs", &MPCParams::error_priority_min_vs)
        .def_readwrite(
            "error_priority_error_speed_gain",
            &MPCParams::error_priority_error_speed_gain
        )
        // Path following (general MPCC)
        .def_readwrite("Q_contour", &MPCParams::Q_contour)
        .def_readwrite("Q_progress", &MPCParams::Q_progress)
        .def_readwrite("progress_reward", &MPCParams::progress_reward)
        .def_readwrite("Q_lag", &MPCParams::Q_lag)
        .def_readwrite("Q_lag_default", &MPCParams::Q_lag_default)
        .def_readwrite("Q_velocity_align", &MPCParams::Q_velocity_align)
        .def_readwrite("Q_s_anchor", &MPCParams::Q_s_anchor)
        .def_readwrite("Q_smooth", &MPCParams::Q_smooth)
        .def_readwrite("path_speed", &MPCParams::path_speed)
        .def_readwrite("path_speed_min", &MPCParams::path_speed_min)
        .def_readwrite("path_speed_max", &MPCParams::path_speed_max)
        .def_readwrite("Q_terminal_pos", &MPCParams::Q_terminal_pos)
        .def_readwrite("Q_terminal_s", &MPCParams::Q_terminal_s)
        .def_readwrite("recover_contour_scale", &MPCParams::recover_contour_scale)
        .def_readwrite("recover_lag_scale", &MPCParams::recover_lag_scale)
        .def_readwrite("recover_progress_scale", &MPCParams::recover_progress_scale)
        .def_readwrite("recover_attitude_scale", &MPCParams::recover_attitude_scale)
        .def_readwrite("settle_progress_scale", &MPCParams::settle_progress_scale)
        .def_readwrite("settle_terminal_pos_scale", &MPCParams::settle_terminal_pos_scale)
        .def_readwrite(
            "settle_terminal_attitude_scale",
            &MPCParams::settle_terminal_attitude_scale
        )
        .def_readwrite(
            "settle_velocity_align_scale",
            &MPCParams::settle_velocity_align_scale
        )
        .def_readwrite(
            "settle_angular_velocity_scale",
            &MPCParams::settle_angular_velocity_scale
        )
        .def_readwrite("hold_smoothness_scale", &MPCParams::hold_smoothness_scale)
        .def_readwrite(
            "hold_thruster_pair_scale",
            &MPCParams::hold_thruster_pair_scale
        )
        .def_readwrite("solver_fallback_hold_s", &MPCParams::solver_fallback_hold_s)
        .def_readwrite("solver_fallback_decay_s", &MPCParams::solver_fallback_decay_s)
        .def_readwrite(
            "solver_fallback_zero_after_s",
            &MPCParams::solver_fallback_zero_after_s
        );

    // Control Result
    py::class_<satellite_control::ControlResult>(m, "ControlResult")
        .def(py::init<>())
        .def_readwrite("u", &ControlResult::u)
        .def_readwrite("status", &ControlResult::status)
        .def_readwrite("solver_status", &ControlResult::solver_status)
        .def_readwrite("iterations", &ControlResult::iterations)
        .def_readwrite("objective", &ControlResult::objective)
        .def_readwrite("solve_time", &ControlResult::solve_time)
        .def_readwrite("timeout", &ControlResult::timeout)
        .def_readwrite("path_s", &ControlResult::path_s)
        .def_readwrite("path_s_proj", &ControlResult::path_s_proj)
        .def_readwrite("path_s_pred", &ControlResult::path_s_pred)
        .def_readwrite("path_error", &ControlResult::path_error)
        .def_readwrite("path_endpoint_error", &ControlResult::path_endpoint_error)
        .def_readwrite("fallback_active", &ControlResult::fallback_active)
        .def_readwrite("fallback_age_s", &ControlResult::fallback_age_s)
        .def_readwrite("fallback_scale", &ControlResult::fallback_scale)
        .def_readwrite("t_linearization_s", &ControlResult::t_linearization_s)
        .def_readwrite("t_cost_update_s", &ControlResult::t_cost_update_s)
        .def_readwrite("t_constraint_update_s", &ControlResult::t_constraint_update_s)
        .def_readwrite("t_matrix_update_s", &ControlResult::t_matrix_update_s)
        .def_readwrite("t_warmstart_s", &ControlResult::t_warmstart_s)
        .def_readwrite("t_solve_only_s", &ControlResult::t_solve_only_s);

    // Obstacle Types
    py::enum_<satellite_control::ObstacleType>(m, "ObstacleType")
        .value("SPHERE", satellite_control::ObstacleType::SPHERE)
        .value("CYLINDER", satellite_control::ObstacleType::CYLINDER)
        .value("BOX", satellite_control::ObstacleType::BOX)
        .export_values();

    py::class_<satellite_control::Obstacle>(m, "Obstacle")
        .def(py::init<>())
        .def_readwrite("type", &Obstacle::type)
        .def_readwrite("position", &Obstacle::position)
        .def_readwrite("radius", &Obstacle::radius)
        .def_readwrite("size", &Obstacle::size)
        .def_readwrite("axis", &Obstacle::axis)
        .def_readwrite("name", &Obstacle::name);

    py::class_<satellite_control::ObstacleSet>(m, "ObstacleSet")
        .def(py::init<>())
        .def("add", &ObstacleSet::add)
        .def("clear", &ObstacleSet::clear)
        .def("size", &ObstacleSet::size);

    // MPC Controller
    py::class_<satellite_control::MPCControllerCpp>(m, "MPCControllerCpp")
        .def(py::init<const satellite_control::SatelliteParams&, const satellite_control::MPCParams&>())
        .def("get_control_action", &satellite_control::MPCControllerCpp::get_control_action,
             py::arg("x_current"),
             "Compute optimal control action")
        .def("set_warm_start_control", &satellite_control::MPCControllerCpp::set_warm_start_control,
             py::arg("u_prev"),
             "Provide a warm-start control guess")
        .def("set_path_data", &satellite_control::MPCControllerCpp::set_path_data,
             py::arg("path_data"),
             "Set path data for general path following. path_data is list of [s, x, y, z] arrays.")
        .def("set_scan_attitude_context", &satellite_control::MPCControllerCpp::set_scan_attitude_context,
             py::arg("center"), py::arg("axis"), py::arg("direction"),
             "Set scan attitude context (center, axis, optional direction hint).")
        .def("clear_scan_attitude_context", &satellite_control::MPCControllerCpp::clear_scan_attitude_context,
             "Disable scan attitude context.")
        .def("set_runtime_mode", &satellite_control::MPCControllerCpp::set_runtime_mode,
             py::arg("mode"),
             "Set runtime controller mode (TRACK/RECOVER/SETTLE/HOLD/COMPLETE).")
        .def("project_onto_path", &satellite_control::MPCControllerCpp::project_onto_path,
             py::arg("position"),
             "Project a position onto the current path. Returns (s, point, distance, endpoint_error).")
        .def("get_reference_at_s", &satellite_control::MPCControllerCpp::get_reference_at_s,
             py::arg("s_query"), py::arg("q_current"),
             "Get path reference at arc-length s: returns (position, tangent, quaternion_ref).")
        .def_property_readonly("num_controls", &satellite_control::MPCControllerCpp::num_controls)
        .def_property_readonly("prediction_horizon", &satellite_control::MPCControllerCpp::prediction_horizon)
        .def_property_readonly("dt", &satellite_control::MPCControllerCpp::dt)
        .def_property_readonly("path_length", &satellite_control::MPCControllerCpp::path_length)
        .def_property_readonly("has_path", &satellite_control::MPCControllerCpp::has_path)
        .def_property_readonly("current_path_s", &satellite_control::MPCControllerCpp::current_path_s);
}

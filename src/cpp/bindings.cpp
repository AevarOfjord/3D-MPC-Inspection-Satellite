
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "linearizer.hpp"
#include "mpc_controller.hpp"
#include "obstacle.hpp"

namespace py = pybind11;
using namespace satellite_control;

PYBIND11_MODULE(_cpp_mpc, m) {
    m.doc() = "C++ backend for Satellite MPC controller";

    // Satellite Parameters
    py::class_<SatelliteParams>(m, "SatelliteParams")
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
        .def_readwrite("com_offset", &SatelliteParams::com_offset)
        .def_readwrite("orbital_mean_motion", &SatelliteParams::orbital_mean_motion)
        .def_readwrite("orbital_mu", &SatelliteParams::orbital_mu)
        .def_readwrite("orbital_radius", &SatelliteParams::orbital_radius)
        .def_readwrite("use_two_body", &SatelliteParams::use_two_body);

    // Linearizer
    py::class_<Linearizer>(m, "Linearizer")
        .def(py::init<const SatelliteParams&>())
        .def("linearize", &Linearizer::linearize, "Compute Linearized Dynamics (A, B)");

    // MPC Parameters
    py::class_<MPCParams>(m, "MPCParams")
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
        .def_readwrite("R_thrust", &MPCParams::R_thrust)
        .def_readwrite("R_rw_torque", &MPCParams::R_rw_torque)
        .def_readwrite("thrust_l1_weight", &MPCParams::thrust_l1_weight)
        .def_readwrite("thrust_pair_weight", &MPCParams::thrust_pair_weight)
        .def_readwrite("coast_pos_tolerance", &MPCParams::coast_pos_tolerance)
        .def_readwrite("coast_vel_tolerance", &MPCParams::coast_vel_tolerance)
        .def_readwrite("coast_min_speed", &MPCParams::coast_min_speed)
        .def_readwrite("max_linear_velocity", &MPCParams::max_linear_velocity)
        .def_readwrite("max_angular_velocity", &MPCParams::max_angular_velocity)
        .def_readwrite("enable_delta_u_coupling", &MPCParams::enable_delta_u_coupling)
        .def_readwrite("enable_gyro_jacobian", &MPCParams::enable_gyro_jacobian)
        .def_readwrite("enable_auto_state_bounds", &MPCParams::enable_auto_state_bounds)

        // Collision avoidance (V3.0.0)
        .def_readwrite("enable_collision_avoidance", &MPCParams::enable_collision_avoidance)
        .def_readwrite("obstacle_margin", &MPCParams::obstacle_margin)
        // Path Following (V4.0.1) - General Path MPCC
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
        .def_readwrite("progress_taper_distance", &MPCParams::progress_taper_distance)
        .def_readwrite("progress_slowdown_distance", &MPCParams::progress_slowdown_distance)
        .def_readwrite("tracking_recovery_error_m", &MPCParams::tracking_recovery_error_m)
        .def_readwrite(
            "tracking_recovery_contour_boost",
            &MPCParams::tracking_recovery_contour_boost
        )
        .def_readwrite(
            "tracking_recovery_progress_scale",
            &MPCParams::tracking_recovery_progress_scale
        )
        .def_readwrite(
            "tracking_recovery_attitude_scale",
            &MPCParams::tracking_recovery_attitude_scale
        );

    // Control Result
    py::class_<ControlResult>(m, "ControlResult")
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
        .def_readwrite("path_endpoint_error", &ControlResult::path_endpoint_error);

    // Obstacle Types
    py::enum_<ObstacleType>(m, "ObstacleType")
        .value("SPHERE", ObstacleType::SPHERE)
        .value("CYLINDER", ObstacleType::CYLINDER)
        .value("BOX", ObstacleType::BOX)
        .export_values();

    py::class_<Obstacle>(m, "Obstacle")
        .def(py::init<>())
        .def_readwrite("type", &Obstacle::type)
        .def_readwrite("position", &Obstacle::position)
        .def_readwrite("radius", &Obstacle::radius)
        .def_readwrite("size", &Obstacle::size)
        .def_readwrite("axis", &Obstacle::axis)
        .def_readwrite("name", &Obstacle::name);

    py::class_<ObstacleSet>(m, "ObstacleSet")
        .def(py::init<>())
        .def("add", &ObstacleSet::add)
        .def("clear", &ObstacleSet::clear)
        .def("size", &ObstacleSet::size);

    // MPC Controller
    py::class_<MPCControllerCpp>(m, "MPCControllerCpp")
        .def(py::init<const SatelliteParams&, const MPCParams&>())
        .def("get_control_action", &MPCControllerCpp::get_control_action,
             py::arg("x_current"),
             "Compute optimal control action")
        .def("set_obstacles", &MPCControllerCpp::set_obstacles, "Set obstacles for collision avoidance")
        .def("clear_obstacles", &MPCControllerCpp::clear_obstacles, "Clear all obstacles")
        .def("set_warm_start_control", &MPCControllerCpp::set_warm_start_control,
             py::arg("u_prev"),
             "Provide a warm-start control guess")
        .def("set_path_data", &MPCControllerCpp::set_path_data,
             py::arg("path_data"),
             "Set path data for general path following. path_data is list of [s, x, y, z] arrays.")
        .def("set_scan_attitude_context", &MPCControllerCpp::set_scan_attitude_context,
             py::arg("center"), py::arg("axis"), py::arg("direction"),
             "Set scan attitude context (center, axis, optional direction hint).")
        .def("clear_scan_attitude_context", &MPCControllerCpp::clear_scan_attitude_context,
             "Disable scan attitude context.")
        .def("project_onto_path", &MPCControllerCpp::project_onto_path,
             py::arg("position"),
             "Project a position onto the current path. Returns (s, point, distance, endpoint_error).")
        .def("get_reference_at_s", &MPCControllerCpp::get_reference_at_s,
             py::arg("s_query"), py::arg("q_current"),
             "Get path reference at arc-length s: returns (position, tangent, quaternion_ref).")
        .def_property_readonly("num_controls", &MPCControllerCpp::num_controls)
        .def_property_readonly("prediction_horizon", &MPCControllerCpp::prediction_horizon)
        .def_property_readonly("dt", &MPCControllerCpp::dt)
        .def_property_readonly("path_length", &MPCControllerCpp::path_length)
        .def_property_readonly("has_path", &MPCControllerCpp::has_path)
        .def_property_readonly("current_path_s", &MPCControllerCpp::current_path_s);
}

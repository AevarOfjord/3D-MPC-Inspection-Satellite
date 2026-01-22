
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
        .def_readwrite("com_offset", &SatelliteParams::com_offset);

    // Linearizer
    py::class_<Linearizer>(m, "Linearizer")
        .def(py::init<const SatelliteParams&>())
        .def("linearize", &Linearizer::linearize, "Compute Linearized Dynamics (A, B)");

    // MPC Parameters
    py::class_<MPCParams>(m, "MPCParams")
        .def(py::init<>())
        .def_readwrite("prediction_horizon", &MPCParams::prediction_horizon)
        .def_readwrite("dt", &MPCParams::dt)
        .def_readwrite("solver_time_limit", &MPCParams::solver_time_limit)
        .def_readwrite("Q_angvel", &MPCParams::Q_angvel)
        .def_readwrite("R_thrust", &MPCParams::R_thrust)
        .def_readwrite("R_rw_torque", &MPCParams::R_rw_torque)

        // Collision avoidance (V3.0.0)
        .def_readwrite("enable_collision_avoidance", &MPCParams::enable_collision_avoidance)
        .def_readwrite("obstacle_margin", &MPCParams::obstacle_margin)
        // Path Following (V4.0.1) - General Path MPCC
        .def_readwrite("Q_contour", &MPCParams::Q_contour)
        .def_readwrite("Q_progress", &MPCParams::Q_progress)
        .def_readwrite("Q_smooth", &MPCParams::Q_smooth)
        .def_readwrite("path_speed", &MPCParams::path_speed);

    // Control Result
    py::class_<ControlResult>(m, "ControlResult")
        .def(py::init<>())
        .def_readwrite("u", &ControlResult::u)
        .def_readwrite("status", &ControlResult::status)
        .def_readwrite("solve_time", &ControlResult::solve_time)
        .def_readwrite("timeout", &ControlResult::timeout);

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
        .def("set_path_data", &MPCControllerCpp::set_path_data,
             py::arg("path_data"),
             "Set path data for general path following. path_data is list of [s, x, y, z] arrays.")
        .def_property_readonly("num_controls", &MPCControllerCpp::num_controls)
        .def_property_readonly("prediction_horizon", &MPCControllerCpp::prediction_horizon)
        .def_property_readonly("dt", &MPCControllerCpp::dt);

    m.def("add", [](int i, int j) { return i + j; }, "A function that adds two numbers");
}

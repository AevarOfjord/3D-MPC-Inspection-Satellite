
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
        .def_readwrite("Q_pos", &MPCParams::Q_pos)
        .def_readwrite("Q_vel", &MPCParams::Q_vel)
        .def_readwrite("Q_ang", &MPCParams::Q_ang)
        .def_readwrite("Q_angvel", &MPCParams::Q_angvel)
        .def_readwrite("R_thrust", &MPCParams::R_thrust)
        .def_readwrite("R_rw_torque", &MPCParams::R_rw_torque)
        .def_readwrite("max_velocity", &MPCParams::max_velocity)
        .def_readwrite("max_angular_velocity", &MPCParams::max_angular_velocity)
        .def_readwrite("enable_z_tilt", &MPCParams::enable_z_tilt)
        .def_readwrite("z_tilt_gain", &MPCParams::z_tilt_gain)
        .def_readwrite("z_tilt_max_rad", &MPCParams::z_tilt_max_rad)
        // Collision avoidance (V3.0.0)
        .def_readwrite("enable_collision_avoidance", &MPCParams::enable_collision_avoidance)
        .def_readwrite("obstacle_margin", &MPCParams::obstacle_margin);

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
             py::arg("x_current"), py::arg("x_target"),
             "Compute optimal control action")
        .def("get_control_action_trajectory", &MPCControllerCpp::get_control_action_trajectory,
             py::arg("x_current"), py::arg("x_target_traj"),
             "Compute optimal control action using a trajectory reference")
        .def("set_obstacles", &MPCControllerCpp::set_obstacles, "Set obstacles for collision avoidance")
        .def("clear_obstacles", &MPCControllerCpp::clear_obstacles, "Clear all obstacles")
        .def_property_readonly("num_controls", &MPCControllerCpp::num_controls)
        .def_property_readonly("prediction_horizon", &MPCControllerCpp::prediction_horizon)
        .def_property_readonly("dt", &MPCControllerCpp::dt);

    m.def("add", [](int i, int j) { return i + j; }, "A function that adds two numbers");
}

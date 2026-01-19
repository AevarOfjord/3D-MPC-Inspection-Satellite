#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "simulation_engine.hpp"

namespace py = pybind11;
using namespace satellite_control;

PYBIND11_MODULE(_cpp_sim, m) {
    m.doc() = "C++ backend for Satellite Simulation Engine";

    // Re-expose SatelliteParams for use with SimulationEngine
    // Note: These are distinct types from _cpp_mpc.SatelliteParams in Python
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
        .def_readwrite("com_offset", &SatelliteParams::com_offset);

    py::class_<SimulationEngine>(m, "SimulationEngine")
        .def(py::init<const SatelliteParams&, double, double, double, bool>(), 
             py::arg("params"), py::arg("mean_motion"),
             py::arg("mu") = 3.986004418e14,
             py::arg("target_radius") = 6.778e6,
             py::arg("use_nonlinear") = true)
        .def("reset", &SimulationEngine::reset, 
             py::arg("state"), "Reset simulation state")
        .def("step", &SimulationEngine::step,
             py::arg("dt"), py::arg("thruster_cmds"), py::arg("rw_torques"),
             "Advance simulation by dt")
        .def("get_state", &SimulationEngine::get_state, 
             "Get current state vector [13]")
        .def("get_rw_speeds", &SimulationEngine::get_rw_speeds,
             "Get reaction wheel speeds [3]");
}

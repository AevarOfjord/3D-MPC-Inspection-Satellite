#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "orbital_dynamics.hpp"

namespace py = pybind11;
using namespace satellite_control;

PYBIND11_MODULE(_cpp_physics, m) {
    m.doc() = "C++ backend for Satellite Physics (Orbital Dynamics)";

    py::class_<CWDynamics>(m, "CWDynamics")
        .def(py::init<double>(), py::arg("mean_motion"))
        .def("compute_acceleration", &CWDynamics::compute_acceleration,
             py::arg("position"), py::arg("velocity"),
             "Compute CW gravitational acceleration [ax, ay, az]")
        .def("get_state_matrices", &CWDynamics::get_state_matrices,
             py::arg("dt"),
             "Get discrete-time state transition matrices (A[6x6], B[6x3])")
        .def("get_mpc_dynamics_matrices", &CWDynamics::get_mpc_dynamics_matrices,
             py::arg("dt"),
             "Get MPC dynamics matrices (A_cw[16x16], B_cw[16x9])");
}

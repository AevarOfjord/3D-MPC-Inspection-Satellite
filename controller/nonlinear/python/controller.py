"""Nonlinear controller profile adapter (Python orchestration + C++ SQP core)."""

from __future__ import annotations

from controller.configs.models import AppConfig
from controller.shared.python.control_common.mpc_controller import MPCController


class NonlinearMPCController(MPCController):
    """
    Nonlinear profile wired to the shared mixed runtime stack.

    Primary execution path:
    - Python: stage-wise exact CasADi linearization + nonlinear strategy orchestration
    - C++: profile-specific SQP/OSQP core (`_cpp_mpc_nonlinear`) for the control solve
    """

    controller_profile = "nonlinear"
    controller_core = "nonlinear-sqp"
    solver_type = "RTI-SQP-Nonlinear"
    solver_backend = "CasADi+OSQP"
    linearization_mode = "nonlinear_exact_stage"
    cpp_module_name = "_cpp_mpc_nonlinear"

    def __init__(self, cfg: AppConfig):
        super().__init__(cfg)

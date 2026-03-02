"""Nonlinear controller profile adapter."""

from __future__ import annotations

from controller.configs.models import AppConfig
from controller.shared.python.control_common.mpc_controller import MPCController


class NonlinearMPCController(MPCController):
    """Nonlinear profile built on the shared RTI-SQP execution stack."""

    controller_profile = "nonlinear"
    controller_core = "nonlinear-sqp"
    solver_type = "RTI-SQP-Nonlinear"
    solver_backend = "CasADi+OSQP"
    linearization_mode = "nonlinear_exact_stage"
    cpp_module_name = "_cpp_mpc_nonlinear"

    def __init__(self, cfg: AppConfig):
        super().__init__(cfg)

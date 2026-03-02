"""Linear controller profile adapter."""

from __future__ import annotations

from controller.configs.models import AppConfig
from controller.shared.python.control_common.mpc_controller import MPCController


class LinearMPCController(MPCController):
    """Linear profile wired to the shared MPC runtime stack."""

    controller_profile = "linear"
    controller_core = "linear-sqp"
    solver_type = "RTI-SQP-Linear"
    solver_backend = "CasADi+OSQP"
    linearization_mode = "linear_frozen_step"
    cpp_module_name = "_cpp_mpc_linear"

    def __init__(self, cfg: AppConfig):
        super().__init__(cfg)

"""Hybrid controller profile adapter."""

from __future__ import annotations

from controller.configs.models import AppConfig
from controller.shared.python.control_common.mpc_controller import MPCController


class HybridMPCController(MPCController):
    """Current production hybrid MPC profile."""

    controller_profile = "hybrid"
    controller_core = "v2-sqp"
    solver_type = "RTI-SQP"
    solver_backend = "CasADi+OSQP"
    linearization_mode = "hybrid_tolerant_stage"
    cpp_module_name = "_cpp_mpc"

    def __init__(self, cfg: AppConfig):
        super().__init__(cfg)

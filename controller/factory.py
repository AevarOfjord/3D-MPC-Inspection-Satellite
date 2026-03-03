"""Factory for selecting controller implementation by profile."""

from __future__ import annotations

from controller.configs.models import AppConfig
from controller.hybrid.python.controller import HybridMPCController
from controller.linear.python.controller import LinearMPCController
from controller.nonlinear.python.controller import NonlinearMPCController
from controller.registry import (
    ACADOS_RTI_PROFILE,
    ACADOS_SQP_PROFILE,
    LINEAR_PROFILE,
    NMPC_PROFILE,
    NONLINEAR_PROFILE,
    normalize_controller_profile,
)
from controller.shared.python.control_common.base import Controller


def create_controller(cfg: AppConfig) -> Controller:
    """Build a controller instance based on AppConfig profile selection."""
    mpc_core = getattr(cfg, "mpc_core", None)
    requested_profile = getattr(mpc_core, "controller_profile", None)
    profile = normalize_controller_profile(requested_profile)

    if profile == NONLINEAR_PROFILE:
        return NonlinearMPCController(cfg)
    if profile == LINEAR_PROFILE:
        return LinearMPCController(cfg)
    if profile == NMPC_PROFILE:
        from controller.nmpc.python.controller import NmpcController

        return NmpcController(cfg)
    if profile == ACADOS_RTI_PROFILE:
        from controller.acados_rti.python.controller import AcadosRtiController

        return AcadosRtiController(cfg)
    if profile == ACADOS_SQP_PROFILE:
        from controller.acados_sqp.python.controller import AcadosSqpController

        return AcadosSqpController(cfg)
    return HybridMPCController(cfg)


def resolve_controller_profile(cfg: AppConfig) -> str:
    """Resolve effective profile string for telemetry and diagnostics."""
    mpc_core = getattr(cfg, "mpc_core", None)
    return normalize_controller_profile(getattr(mpc_core, "controller_profile", None))


__all__ = ["create_controller", "resolve_controller_profile"]

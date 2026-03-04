"""Tests for controller profile registry/factory routing."""

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller, resolve_controller_profile
from controller.hybrid.python.controller import HybridMPCController
from controller.linear.python.controller import LinearMPCController
from controller.nonlinear.python.controller import NonlinearMPCController


def test_factory_default_profile_is_hybrid():
    cfg = SimulationConfig.create_default().app_config
    controller = create_controller(cfg)
    assert isinstance(controller, HybridMPCController)
    assert getattr(controller, "controller_profile", None) == "cpp_hybrid_rti_osqp"
    assert getattr(controller, "cpp_module_name", None) == "_cpp_mpc_runtime"


def test_factory_routes_nonlinear_profile():
    cfg = SimulationConfig.create_with_overrides(
        {"mpc_core": {"controller_profile": "cpp_nonlinear_rti_osqp"}}
    ).app_config
    controller = create_controller(cfg)
    assert isinstance(controller, NonlinearMPCController)
    assert getattr(controller, "controller_profile", None) == "cpp_nonlinear_rti_osqp"
    assert getattr(controller, "cpp_module_name", None) == "_cpp_mpc_runtime"


def test_factory_routes_linear_profile():
    cfg = SimulationConfig.create_with_overrides(
        {"mpc_core": {"controller_profile": "cpp_linearized_rti_osqp"}}
    ).app_config
    controller = create_controller(cfg)
    assert isinstance(controller, LinearMPCController)
    assert getattr(controller, "controller_profile", None) == "cpp_linearized_rti_osqp"
    assert getattr(controller, "cpp_module_name", None) == "_cpp_mpc_runtime"


def test_factory_profile_rejects_invalid_value():
    cfg = SimulationConfig.create_default().app_config.model_copy(deep=True)
    cfg.mpc_core.controller_profile = "unknown"
    try:
        resolve_controller_profile(cfg)
    except ValueError as exc:
        assert "Unsupported controller profile" in str(exc)
    else:  # pragma: no cover
        raise AssertionError("Expected ValueError for invalid controller profile")

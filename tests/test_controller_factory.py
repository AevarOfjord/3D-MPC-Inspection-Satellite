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
    assert getattr(controller, "controller_profile", None) == "hybrid"


def test_factory_routes_nonlinear_profile():
    cfg = SimulationConfig.create_with_overrides(
        {"mpc_core": {"controller_profile": "nonlinear"}}
    ).app_config
    controller = create_controller(cfg)
    assert isinstance(controller, NonlinearMPCController)
    assert getattr(controller, "controller_profile", None) == "nonlinear"


def test_factory_routes_linear_profile():
    cfg = SimulationConfig.create_with_overrides(
        {"mpc_core": {"controller_profile": "linear"}}
    ).app_config
    controller = create_controller(cfg)
    assert isinstance(controller, LinearMPCController)
    assert getattr(controller, "controller_profile", None) == "linear"


def test_factory_profile_falls_back_to_hybrid_for_invalid_value():
    cfg = SimulationConfig.create_default().app_config.model_copy(deep=True)
    cfg.mpc_core.controller_profile = "unknown"
    profile = resolve_controller_profile(cfg)
    assert profile == "hybrid"

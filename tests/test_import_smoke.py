"""Smoke tests for critical module imports."""

from importlib import import_module


def test_critical_modules_import() -> None:
    modules = [
        "satellite_control.dashboard.app",
        "satellite_control.control.mpc_controller",
        "satellite_control.mission.path_following",
    ]
    for module_name in modules:
        module = import_module(module_name)
        assert module is not None

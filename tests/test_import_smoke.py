"""Smoke tests for critical module imports."""

from importlib import import_module


def test_critical_modules_import() -> None:
    modules = [
        "src.satellite_control.dashboard.app",
        "src.satellite_control.control.mpc_controller",
        "src.satellite_control.control.rw_mpc_controller",
        "src.satellite_control.mission.path_following",
    ]
    for module_name in modules:
        module = import_module(module_name)
        assert module is not None

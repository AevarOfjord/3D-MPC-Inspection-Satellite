"""Tests for legacy controller_backend -> controller_profile migration."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.dashboard.runner_manager import RunnerManager


def test_runner_migrates_legacy_controller_backend_to_profile():
    manager = RunnerManager()
    envelope = manager._normalize_to_envelope(  # noqa: SLF001 - intentional private API test
        {
            "app_config": {
                "mpc_core": {
                    "controller_backend": "v1",
                    "solver_backend": "OSQP",
                },
                "simulation": {"dt": 0.001, "control_dt": 0.05, "max_duration": 0.0},
            }
        }
    )

    app_config = envelope["app_config"]
    mpc_core = app_config["mpc_core"]
    assert mpc_core["controller_profile"] == "cpp_linearized_rti_osqp"
    assert "controller_backend" not in mpc_core


def test_runner_preset_save_drops_legacy_controller_backend(tmp_path: Path):
    manager = RunnerManager()
    manager._presets_path = tmp_path / "runner_presets.json"  # noqa: SLF001
    saved = manager.save_preset(
        "legacy-backend",
        {
            "app_config": {
                "mpc_core": {
                    "controller_backend": "v2",
                    "solver_backend": "OSQP",
                },
                "simulation": {"dt": 0.001, "control_dt": 0.05, "max_duration": 0.0},
            }
        },
    )

    mpc_core = saved["config"]["app_config"]["mpc_core"]
    assert mpc_core["controller_profile"] == "cpp_hybrid_rti_osqp"
    assert "controller_backend" not in mpc_core


def test_runner_preserves_profile_overrides_on_save(tmp_path: Path):
    manager = RunnerManager()
    manager._presets_path = tmp_path / "runner_presets.json"  # noqa: SLF001

    saved = manager.save_preset(
        "profile-overrides",
        {
            "app_config": {
                "shared": {"parameters": False},
                "mpc_core": {
                    "controller_profile": "cpp_nonlinear_rti_osqp",
                    "solver_backend": "OSQP",
                },
                "mpc_profile_overrides": {
                    "cpp_nonlinear_rti_osqp": {
                        "base_overrides": {"Q_contour": 4321.0},
                        "profile_specific": {"sqp_max_iter": 3, "sqp_tol": 1e-5},
                    }
                },
                "simulation": {"dt": 0.001, "control_dt": 0.05, "max_duration": 0.0},
            }
        },
    )

    profile = saved["config"]["app_config"]["mpc_profile_overrides"][
        "cpp_nonlinear_rti_osqp"
    ]
    assert profile["base_overrides"]["Q_contour"] == 4321.0
    assert profile["profile_specific"]["sqp_max_iter"] == 3


def test_runner_rewrites_legacy_profile_ids_to_canonical():
    manager = RunnerManager()
    envelope = manager._normalize_to_envelope(  # noqa: SLF001 - intentional private API test
        {
            "app_config": {
                "mpc_core": {
                    "controller_profile": "nonlinear",
                    "solver_backend": "OSQP",
                },
                "simulation": {"dt": 0.001, "control_dt": 0.05, "max_duration": 0.0},
            }
        }
    )
    assert envelope["app_config"]["mpc_core"]["controller_profile"] == (
        "cpp_nonlinear_rti_osqp"
    )


def test_direct_runtime_rejects_legacy_profile_id():
    with pytest.raises(ValidationError):
        SimulationConfig.create_with_overrides(
            {"mpc_core": {"controller_profile": "nonlinear"}}
        )

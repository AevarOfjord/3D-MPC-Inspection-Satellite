"""Shared vs profile parameter mode tests."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.control_common.parameter_policy import (
    apply_profile_parameter_file_if_needed,
)


def test_shared_parameters_true_rejects_nonempty_profile_overrides() -> None:
    with pytest.raises(ValidationError):
        SimulationConfig.create_with_overrides(
            {
                "shared": {"parameters": True},
                "mpc_profile_overrides": {
                    "cpp_hybrid_rti_osqp": {
                        "base_overrides": {"Q_contour": 9999.0},
                        "profile_specific": {},
                    }
                },
            }
        )


def test_shared_parameters_false_allows_profile_overrides() -> None:
    cfg = SimulationConfig.create_with_overrides(
        {
            "shared": {"parameters": False},
            "mpc_profile_overrides": {
                "cpp_hybrid_rti_osqp": {
                    "base_overrides": {"Q_contour": 9999.0},
                    "profile_specific": {},
                }
            },
        }
    )
    value = cfg.app_config.mpc_profile_overrides.cpp_hybrid_rti_osqp.base_overrides[
        "Q_contour"
    ]
    assert value == pytest.approx(9999.0)


def test_cli_applies_profile_file_when_shared_parameters_disabled(
    tmp_path: Path,
) -> None:
    profile_file = tmp_path / "hybrid_profile.json"
    payload = {
        "mpc_profile_overrides": {
            "cpp_hybrid_rti_osqp": {
                "base_overrides": {"Q_contour": 4321.0},
                "profile_specific": {},
            }
        }
    }
    profile_file.write_text(json.dumps(payload), encoding="utf-8")

    overrides = {
        "shared": {
            "parameters": False,
            "profile_parameter_files": {
                "cpp_hybrid_rti_osqp": str(profile_file),
            },
        },
        "mpc_core": {"controller_profile": "cpp_hybrid_rti_osqp"},
    }

    merged, applied_path, shared_enabled, resolved_profile = (
        apply_profile_parameter_file_if_needed(
            config_overrides=overrides,
            default_profile="cpp_hybrid_rti_osqp",
        )
    )

    assert shared_enabled is False
    assert applied_path == str(profile_file.resolve())
    assert resolved_profile == "cpp_hybrid_rti_osqp"
    merged_overrides = merged["mpc_profile_overrides"]["cpp_hybrid_rti_osqp"][
        "base_overrides"
    ]
    assert merged_overrides["Q_contour"] == pytest.approx(4321.0)


def test_cli_does_not_apply_profile_file_when_shared_parameters_enabled(
    tmp_path: Path,
) -> None:
    profile_file = tmp_path / "hybrid_profile.json"
    profile_file.write_text("{}", encoding="utf-8")

    overrides = {
        "shared": {
            "parameters": True,
            "profile_parameter_files": {
                "cpp_hybrid_rti_osqp": str(profile_file),
            },
        },
        "mpc_core": {"controller_profile": "cpp_hybrid_rti_osqp"},
    }

    merged, applied_path, shared_enabled, resolved_profile = (
        apply_profile_parameter_file_if_needed(
            config_overrides=overrides,
            default_profile="cpp_hybrid_rti_osqp",
        )
    )

    assert merged == overrides
    assert applied_path is None
    assert shared_enabled is True
    assert resolved_profile == "cpp_hybrid_rti_osqp"


def test_cli_accepts_dotted_shared_parameters_alias() -> None:
    overrides = {
        "shared.parameters": True,
        "mpc_core": {"controller_profile": "cpp_hybrid_rti_osqp"},
    }

    merged, applied_path, shared_enabled, resolved_profile = (
        apply_profile_parameter_file_if_needed(
            config_overrides=overrides,
            default_profile="cpp_hybrid_rti_osqp",
        )
    )

    assert "shared.parameters" not in merged
    assert merged["shared"]["parameters"] is True
    assert applied_path is None
    assert shared_enabled is True
    assert resolved_profile == "cpp_hybrid_rti_osqp"


def test_cli_defaults_to_shared_parameter_mode_when_shared_block_missing() -> None:
    merged, applied_path, shared_enabled, resolved_profile = (
        apply_profile_parameter_file_if_needed(
            config_overrides={
                "mpc_core": {"controller_profile": "cpp_hybrid_rti_osqp"}
            },
            default_profile="cpp_hybrid_rti_osqp",
        )
    )

    assert merged["mpc_core"]["controller_profile"] == "cpp_hybrid_rti_osqp"
    assert applied_path is None
    assert shared_enabled is True
    assert resolved_profile == "cpp_hybrid_rti_osqp"

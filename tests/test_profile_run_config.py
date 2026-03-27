from __future__ import annotations

import json
from pathlib import Path

from controller.shared.python.control_common.parameter_policy import (
    default_profile_parameter_files,
)
from controller.shared.python.control_common.profile_run_config import (
    build_profile_sim_overrides,
    persist_profile_sweep_winner,
    write_profile_sim_config,
)


def test_build_profile_sim_overrides_enables_profile_parameter_mode() -> None:
    base = {
        "shared": {"parameters": True},
        "physics": {"random_disturbances_enabled": False},
    }

    payload = build_profile_sim_overrides(base, "hybrid")

    assert payload["shared"]["parameters"] is False
    assert payload["physics"]["random_disturbances_enabled"] is False
    assert payload["mpc_core"]["controller_profile"] == "cpp_hybrid_rti_osqp"
    assert (
        payload["shared"]["profile_parameter_files"]["cpp_hybrid_rti_osqp"]
        == default_profile_parameter_files()["cpp_hybrid_rti_osqp"]
    )


def test_write_profile_sim_config_writes_json_file(tmp_path: Path) -> None:
    base_path = tmp_path / "base.json"
    out_path = tmp_path / "runtime.json"
    base_path.write_text(json.dumps({"shared": {"parameters": True}}), encoding="utf-8")

    write_profile_sim_config(
        base_config_path=base_path,
        profile="cpp_nonlinear_rti_osqp",
        output_path=out_path,
    )

    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["shared"]["parameters"] is False
    assert payload["mpc_core"]["controller_profile"] == "cpp_nonlinear_rti_osqp"


def test_persist_profile_sweep_winner_updates_only_target_profile(
    tmp_path: Path,
) -> None:
    profile_path = tmp_path / "profile_parameters.json"
    profile_path.write_text(
        json.dumps(
            {
                "mpc_core": {"controller_profile": "cpp_hybrid_rti_osqp"},
                "mpc_profile_overrides": {
                    "cpp_hybrid_rti_osqp": {
                        "base_overrides": {"Q_contour": 123.0},
                        "profile_specific": {"allow_stale_stage_reuse": True},
                    },
                    "cpp_nonlinear_rti_osqp": {
                        "base_overrides": {"Q_contour": 999.0},
                        "profile_specific": {"strict_integrity": False},
                    },
                },
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    persist_profile_sweep_winner(
        profile="cpp_hybrid_rti_osqp",
        prediction_horizon=10,
        control_horizon=10,
        dt=0.01,
        solver_time_limit=0.008,
        profile_file_path=profile_path,
    )

    payload = json.loads(profile_path.read_text(encoding="utf-8"))
    hybrid = payload["mpc_profile_overrides"]["cpp_hybrid_rti_osqp"]
    nonlinear = payload["mpc_profile_overrides"]["cpp_nonlinear_rti_osqp"]

    assert payload["mpc"]["prediction_horizon"] == 10
    assert payload["mpc"]["control_horizon"] == 10
    assert payload["mpc"]["dt"] == 0.01
    assert payload["mpc"]["solver_time_limit"] == 0.008
    assert payload["simulation"]["control_dt"] == 0.01
    assert hybrid["base_overrides"]["prediction_horizon"] == 10
    assert hybrid["base_overrides"]["control_horizon"] == 10
    assert hybrid["base_overrides"]["dt"] == 0.01
    assert hybrid["base_overrides"]["solver_time_limit"] == 0.008
    assert hybrid["base_overrides"]["Q_contour"] == 123.0
    assert hybrid["profile_specific"]["allow_stale_stage_reuse"] is True
    assert nonlinear["base_overrides"]["Q_contour"] == 999.0
    assert nonlinear["profile_specific"]["strict_integrity"] is False

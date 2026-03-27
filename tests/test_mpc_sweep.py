from __future__ import annotations

import csv
import json
import subprocess
from pathlib import Path

import pytest

from controller.shared.python.benchmarks import mpc_sweep


def _base_row(**overrides):
    row = {
        "controller_profile": "cpp_hybrid_rti_osqp",
        "prediction_horizon": 20,
        "control_dt_s": 0.02,
        "control_horizon": 20,
        "solver_time_limit_s": 0.016,
        "return_code": 0,
        "run_dir": "run_dir",
        "run_id": "run_id",
        "process_succeeded": True,
        "path_completed": True,
        "timing_violation_count": 0,
        "solver_fallback_count": 0,
        "solver_hard_limit_breaches": 0,
        "path_position_error_p95_m": 0.1,
        "path_angular_error_p95_deg": 2.0,
        "terminal_position_error_m": 0.05,
        "terminal_angular_error_deg": 1.0,
        "mpc_mean_solve_time_ms": 4.0,
        "mpc_max_solve_time_ms": 8.0,
    }
    row.update(overrides)
    return row


def _write_run_artifacts(run_dir: Path) -> None:
    (run_dir / "Data" / "01_timeseries").mkdir(parents=True, exist_ok=True)
    (run_dir / "Data" / "03_diagnostics").mkdir(parents=True, exist_ok=True)

    with (run_dir / "Data" / "01_timeseries" / "mpc_step_stats.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["Path_Error_m", "Ang_Error_deg"],
        )
        writer.writeheader()
        for idx in range(5):
            writer.writerow(
                {
                    "Path_Error_m": f"{0.01 * (idx + 1):.5f}",
                    "Ang_Error_deg": f"{1.0 * (idx + 1):.5f}",
                }
            )

    (run_dir / "Data" / "03_diagnostics" / "kpi_summary.json").write_text(
        json.dumps(
            {
                "path_completed": True,
                "timing_violation_count": 0,
                "solver_fallback_count": 0,
                "solver_hard_limit_breaches": 0,
                "final_position_error_m": 0.03,
                "final_angle_error_deg": 1.2,
                "mpc_mean_solve_time_ms": 2.5,
                "mpc_max_solve_time_ms": 6.0,
            }
        ),
        encoding="utf-8",
    )


def test_score_sweep_rows_gates_ineligible_runs() -> None:
    eligible = _base_row(path_position_error_p95_m=0.3)
    ineligible = _base_row(
        path_position_error_p95_m=0.01,
        path_completed=False,
        prediction_horizon=10,
    )

    scored, winner, winner_eligible = mpc_sweep.score_sweep_rows([eligible, ineligible])

    assert len(scored) == 2
    assert winner is not None
    assert winner_eligible is True
    assert winner["path_completed"] is True
    assert winner["prediction_horizon"] == eligible["prediction_horizon"]


def test_score_sweep_rows_uses_rank_average_not_raw_units() -> None:
    rows = [
        _base_row(
            prediction_horizon=10,
            control_dt_s=0.01,
            path_position_error_p95_m=0.30,
            path_angular_error_p95_deg=0.50,
            terminal_position_error_m=0.30,
            terminal_angular_error_deg=0.50,
        ),
        _base_row(
            prediction_horizon=20,
            control_dt_s=0.02,
            path_position_error_p95_m=0.10,
            path_angular_error_p95_deg=3.00,
            terminal_position_error_m=0.10,
            terminal_angular_error_deg=3.00,
        ),
        _base_row(
            prediction_horizon=30,
            control_dt_s=0.03,
            path_position_error_p95_m=0.20,
            path_angular_error_p95_deg=1.50,
            terminal_position_error_m=0.20,
            terminal_angular_error_deg=1.50,
        ),
    ]

    scored, winner, winner_eligible = mpc_sweep.score_sweep_rows(rows)

    assert winner_eligible is True
    assert winner is not None
    assert [row["final_score"] for row in scored] == pytest.approx([2.0, 2.0, 2.0])
    assert winner["prediction_horizon"] == 10


def test_score_sweep_rows_tiebreaks_on_solve_time() -> None:
    fast = _base_row(mpc_mean_solve_time_ms=2.0, mpc_max_solve_time_ms=5.0)
    slow = _base_row(
        prediction_horizon=30,
        control_dt_s=0.03,
        mpc_mean_solve_time_ms=4.0,
        mpc_max_solve_time_ms=9.0,
    )

    _, winner, winner_eligible = mpc_sweep.score_sweep_rows([slow, fast])

    assert winner_eligible is True
    assert winner is not None
    assert winner["mpc_mean_solve_time_ms"] == 2.0


def test_build_sweep_point_overrides_clamps_timing_fields() -> None:
    payload = mpc_sweep.build_sweep_point_overrides(
        base_overrides={},
        profile="cpp_hybrid_rti_osqp",
        prediction_horizon=10,
        control_dt_s=0.01,
        base_control_horizon=40,
        base_solver_time_limit_s=0.035,
    )

    assert payload["shared"]["parameters"] is True
    assert payload["mpc"]["prediction_horizon"] == 10
    assert payload["mpc"]["control_horizon"] == 10
    assert payload["mpc"]["dt"] == 0.01
    assert payload["simulation"]["control_dt"] == 0.01
    assert payload["mpc"]["solver_time_limit"] == pytest.approx(0.008)


def test_execute_sweep_point_reads_synthetic_artifacts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    mission_path = tmp_path / "mission.json"
    mission_path.write_text("{}", encoding="utf-8")
    run_dir = tmp_path / "simulation_data" / "run_001"
    _write_run_artifacts(run_dir)

    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=0,
            stdout=f"Created data directory: {run_dir}\n",
            stderr="",
        )

    monkeypatch.setattr(mpc_sweep.subprocess, "run", fake_run)
    monkeypatch.setattr(
        mpc_sweep,
        "_read_latest_run_id",
        lambda: "run_001",
    )

    row = mpc_sweep._execute_sweep_point(
        "cpp_hybrid_rti_osqp",
        20,
        0.02,
        {},
        mission_path,
        "python",
        40,
        0.035,
    )

    assert row["process_succeeded"] is True
    assert row["path_completed"] is True
    assert row["path_position_error_p95_m"] == pytest.approx(0.05)
    assert row["path_angular_error_p95_deg"] == pytest.approx(5.0)
    assert row["terminal_position_error_m"] == pytest.approx(0.03)
    assert row["terminal_angular_error_deg"] == pytest.approx(1.2)


def test_run_controller_sweep_produces_100_rows_and_persists_winner(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    persisted = {}

    def fake_executor(profile, horizon, control_dt_s, *_args):
        return _base_row(
            controller_profile=profile,
            prediction_horizon=horizon,
            control_dt_s=control_dt_s,
            control_horizon=min(40, horizon),
            solver_time_limit_s=min(0.035, control_dt_s * 0.8),
            path_position_error_p95_m=horizon / 1000.0 + control_dt_s,
            path_angular_error_p95_deg=horizon / 100.0 + control_dt_s,
            terminal_position_error_m=horizon / 1000.0 + control_dt_s,
            terminal_angular_error_deg=horizon / 100.0 + control_dt_s,
        )

    def fake_persist(**kwargs):
        persisted.update(kwargs)
        return tmp_path / "profile_parameters.json"

    monkeypatch.setattr(mpc_sweep, "persist_profile_sweep_winner", fake_persist)

    summary = mpc_sweep.run_controller_sweep(
        controller_profile="cpp_hybrid_rti_osqp",
        mission_path=tmp_path / "mission.json",
        base_overrides={},
        python_executable="python",
        output_dir=tmp_path,
        executor=fake_executor,
    )

    assert len(summary["rows"]) == 100
    assert summary["winner"] is not None
    assert summary["winner_eligible"] is True
    assert summary["profile_updated"] is True
    assert persisted["profile"] == "cpp_hybrid_rti_osqp"
    assert (tmp_path / "data" / "matrix.csv").exists()
    assert (tmp_path / "data" / "matrix.json").exists()
    assert (tmp_path / "plots" / "heatmap.png").exists()
    assert (tmp_path / "plots" / "all_runs_comparison.png").exists()
    assert (tmp_path / "summary.md").exists()


def test_run_mpc_sweep_all_controllers_creates_batch_summary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    mission_path = tmp_path / "mission.json"
    mission_path.write_text("{}", encoding="utf-8")

    def fake_executor(profile, horizon, control_dt_s, *_args):
        return _base_row(
            controller_profile=profile,
            prediction_horizon=horizon,
            control_dt_s=control_dt_s,
            control_horizon=min(40, horizon),
            solver_time_limit_s=min(0.035, control_dt_s * 0.8),
            path_position_error_p95_m=horizon / 1000.0 + control_dt_s,
            path_angular_error_p95_deg=horizon / 100.0 + control_dt_s,
            terminal_position_error_m=horizon / 1000.0 + control_dt_s,
            terminal_angular_error_deg=horizon / 100.0 + control_dt_s,
        )

    monkeypatch.setattr(
        mpc_sweep,
        "persist_profile_sweep_winner",
        lambda **kwargs: tmp_path / f"{kwargs['profile']}.json",
    )

    out_dir = mpc_sweep.run_mpc_sweep(
        mission=str(mission_path),
        controller_profile=None,
        all_controllers=True,
        base_config_path=None,
        python_executable="python",
        output_root=tmp_path / "sweep_out",
        executor=fake_executor,
    )

    batch_payload = json.loads(
        (out_dir / "batch_summary.json").read_text(encoding="utf-8")
    )
    assert len(batch_payload["profiles"]) == len(
        mpc_sweep.SUPPORTED_CONTROLLER_PROFILES
    )
    assert batch_payload["schema_version"] == "mpc_sweep_v2"
    first_profile = batch_payload["profiles"][0]
    assert "rows" not in first_profile
    assert first_profile["artifact_paths"]["matrix_csv"].startswith("controllers/")
    assert (out_dir / "batch_summary.md").exists()
    assert (out_dir / "comparisons" / "winner_comparison.png").exists()
    assert (out_dir / "comparisons" / "winner_summary.csv").exists()

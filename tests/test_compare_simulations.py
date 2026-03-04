from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path


def _script_path() -> Path:
    return Path(__file__).resolve().parents[1] / "scripts" / "compare_simulations.py"


def _write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _make_run(
    root: Path,
    run_id: str,
    *,
    valid: bool = True,
    include_optional_columns: bool = True,
    fallback_count: int = 0,
) -> Path:
    run_dir = root / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    status_payload = {
        "status": "completed",
        "mission": {"name": f"mission_{run_id}"},
        "controller": {"profile": f"profile_{run_id}"},
    }
    _write(
        run_dir / "Data" / "02_metadata" / "run_status.json",
        json.dumps(status_payload),
    )

    kpi_payload = {
        "final_time_s": 12.3,
        "final_position_error_m": 0.04,
        "final_angle_error_deg": 1.2,
        "final_velocity_error_mps": 0.02,
        "final_angular_velocity_error_degps": 0.4,
        "mpc_mean_solve_time_ms": 2.1,
        "mpc_max_solve_time_ms": 4.7,
        "solver_fallback_count": fallback_count,
        "solver_hard_limit_breaches": 0,
        "path_completed": True,
    }
    _write(
        run_dir / "Data" / "03_diagnostics" / "kpi_summary.json",
        json.dumps(kpi_payload),
    )

    if valid:
        headers = [
            "Control_Time_s",
            "Pos_Error_m",
            "Ang_Error_deg",
            "Velocity_Error_mps",
            "Angular_Velocity_Error_degps",
            "Linear_Speed_mps",
            "Angular_Rate_radps",
            "Solve_Time_ms",
            "Timing_Violation",
            "MPC_Time_Limit_Exceeded",
            "Path_Error_m",
            "Path_Remaining_m",
        ]
        if not include_optional_columns:
            headers = [
                "Control_Time_s",
                "Pos_Error_m",
                "Solve_Time_ms",
                "Timing_Violation",
                "MPC_Time_Limit_Exceeded",
            ]

        csv_path = run_dir / "Data" / "01_timeseries" / "mpc_step_stats.csv"
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            for i in range(6):
                row = {
                    "Control_Time_s": f"{i * 0.5:.3f}",
                    "Pos_Error_m": f"{0.1 / (i + 1):.5f}",
                    "Ang_Error_deg": f"{2.0 / (i + 1):.5f}",
                    "Velocity_Error_mps": f"{0.05 / (i + 1):.5f}",
                    "Angular_Velocity_Error_degps": f"{0.3 / (i + 1):.5f}",
                    "Linear_Speed_mps": f"{0.2 + i * 0.01:.5f}",
                    "Angular_Rate_radps": f"{0.01 + i * 0.001:.6f}",
                    "Solve_Time_ms": f"{1.5 + i * 0.2:.4f}",
                    "Timing_Violation": "1" if i == 3 else "0",
                    "MPC_Time_Limit_Exceeded": "1" if i == 4 else "0",
                    "Path_Error_m": f"{0.15 / (i + 1):.5f}",
                    "Path_Remaining_m": f"{1.0 - i * 0.15:.5f}",
                }
                writer.writerow({k: v for k, v in row.items() if k in headers})

    return run_dir


def _run_compare(args: list[str]) -> subprocess.CompletedProcess[str]:
    cmd = [sys.executable, str(_script_path()), *args]
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _single_report_dir(out_root: Path) -> Path:
    report_dirs = [p for p in out_root.iterdir() if p.is_dir()]
    assert len(report_dirs) == 1
    return report_dirs[0]


def test_compare_generates_expected_report_files(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    _make_run(runs_root, "run_001")
    _make_run(runs_root, "run_002")
    out_root = tmp_path / "out"

    result = _run_compare(
        [
            "--runs-root",
            str(runs_root),
            "--run",
            "run_001",
            "--run",
            "run_002",
            "--output",
            str(out_root),
        ]
    )

    assert result.returncode == 0, result.stderr + result.stdout
    report_dir = _single_report_dir(out_root)

    assert (report_dir / "comparison_summary.csv").exists()
    assert (report_dir / "comparison_summary.md").exists()
    assert (report_dir / "comparison_meta.json").exists()
    meta = json.loads((report_dir / "comparison_meta.json").read_text(encoding="utf-8"))
    assert meta["plot_count_time"] == 16
    assert meta["plot_count_path_s"] == 9
    assert meta["plot_count_total"] == 25

    plots_dir = report_dir / "plots"
    expected = [f"{i:02d}_" for i in range(1, 17)]
    created = [p.name for p in plots_dir.glob("*.png")]
    assert len(created) >= 16
    for prefix in expected:
        assert any(name.startswith(prefix) for name in created)

    plots_path_s_dir = report_dir / "plots_path_s"
    expected_path_s = [f"{i:02d}_" for i in range(1, 10)]
    created_path_s = [p.name for p in plots_path_s_dir.glob("*.png")]
    assert len(created_path_s) >= 9
    for prefix in expected_path_s:
        assert any(name.startswith(prefix) for name in created_path_s)


def test_compare_handles_invalid_run_and_writes_warnings(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    _make_run(runs_root, "run_001", valid=True)
    _make_run(runs_root, "run_002", valid=True)
    _make_run(runs_root, "run_bad", valid=False)
    out_root = tmp_path / "out"

    result = _run_compare(
        [
            "--runs-root",
            str(runs_root),
            "--run",
            "run_001",
            "--run",
            "run_002",
            "--run",
            "run_bad",
            "--output",
            str(out_root),
        ]
    )

    assert result.returncode == 0, result.stderr + result.stdout
    report_dir = _single_report_dir(out_root)
    warnings_path = report_dir / "comparison_warnings.txt"
    assert warnings_path.exists()
    content = warnings_path.read_text(encoding="utf-8")
    assert "run_bad" in content


def test_compare_rejects_more_than_max_runs(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    for i in range(11):
        _make_run(runs_root, f"run_{i:03d}")

    out_root = tmp_path / "out"
    result = _run_compare(
        [
            "--runs-root",
            str(runs_root),
            "--latest",
            "11",
            "--max-runs",
            "10",
            "--output",
            str(out_root),
        ]
    )

    assert result.returncode == 2
    assert "Max allowed is 10" in (result.stdout + result.stderr)


def test_compare_tolerates_missing_optional_columns(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    _make_run(runs_root, "run_001", include_optional_columns=False)
    _make_run(runs_root, "run_002", include_optional_columns=False)

    out_root = tmp_path / "out"
    result = _run_compare(
        [
            "--runs-root",
            str(runs_root),
            "--run",
            "run_001",
            "--run",
            "run_002",
            "--output",
            str(out_root),
        ]
    )

    assert result.returncode == 0, result.stderr + result.stdout
    report_dir = _single_report_dir(out_root)
    summary_csv = report_dir / "comparison_summary.csv"
    rows = summary_csv.read_text(encoding="utf-8")
    assert "final_position_error_m" in rows


def test_compare_uses_run_status_config_and_control_fallback_metrics(
    tmp_path: Path,
) -> None:
    runs_root = tmp_path / "runs"
    run_a = _make_run(runs_root, "run_001")
    run_b = _make_run(runs_root, "run_002")

    # Simulate real run_status schema where controller profile is nested in config.
    for run_dir in (run_a, run_b):
        status_path = run_dir / "Data" / "02_metadata" / "run_status.json"
        status_payload = {
            "status": "completed",
            "mission": {"name": None, "path": None},
            "config": {"controller_profile": f"cfg_{run_dir.name}"},
        }
        _write(status_path, json.dumps(status_payload))
        _write(
            run_dir / "Data" / "02_metadata" / "mission_metadata.json",
            json.dumps({"mission_type": "path_following"}),
        )

        # Force comparison script to rebuild angular metrics from control_data.
        step_path = run_dir / "Data" / "01_timeseries" / "mpc_step_stats.csv"
        step_df = []
        with step_path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            headers = reader.fieldnames or []
            for row in reader:
                row["Ang_Error_deg"] = "0.0"
                row["Angular_Velocity_Error_degps"] = "0.0"
                step_df.append(row)
        with step_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            writer.writerows(step_df)

        control_headers = [
            "Control_Time",
            "Error_Roll",
            "Error_Pitch",
            "Error_Yaw",
            "Error_WX",
            "Error_WY",
            "Error_WZ",
            "MPC_Fallback_Used",
        ]
        control_path = run_dir / "Data" / "01_timeseries" / "control_data.csv"
        with control_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=control_headers)
            writer.writeheader()
            for i in range(6):
                writer.writerow(
                    {
                        "Control_Time": f"{i * 0.5:.3f}",
                        "Error_Roll": f"{0.01 * (i + 1):.6f}",
                        "Error_Pitch": f"{0.02 * (i + 1):.6f}",
                        "Error_Yaw": f"{0.03 * (i + 1):.6f}",
                        "Error_WX": f"{1.0 + i:.4f}",
                        "Error_WY": f"{0.5 + i:.4f}",
                        "Error_WZ": f"{0.25 + i:.4f}",
                        "MPC_Fallback_Used": "false",
                    }
                )

        # Ensure kpi angular metrics are zero so fallback path is exercised.
        kpi_path = run_dir / "Data" / "03_diagnostics" / "kpi_summary.json"
        kpi = json.loads(kpi_path.read_text(encoding="utf-8"))
        kpi["final_angle_error_deg"] = 0.0
        kpi["final_angular_velocity_error_degps"] = 0.0
        _write(kpi_path, json.dumps(kpi))

    out_root = tmp_path / "out"
    result = _run_compare(
        [
            "--runs-root",
            str(runs_root),
            "--run",
            "run_001",
            "--run",
            "run_002",
            "--output",
            str(out_root),
        ]
    )
    assert result.returncode == 0, result.stderr + result.stdout

    report_dir = _single_report_dir(out_root)
    summary_path = report_dir / "comparison_summary.csv"
    with summary_path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    assert rows
    assert {row["mission"] for row in rows} == {"path_following"}
    assert {row["controller_profile"] for row in rows} == {"cfg_run_001", "cfg_run_002"}
    assert all(float(row["final_angle_error_deg"]) > 0.0 for row in rows)

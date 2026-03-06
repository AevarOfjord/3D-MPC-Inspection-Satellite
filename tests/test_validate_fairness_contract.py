from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from controller.registry import SUPPORTED_CONTROLLER_PROFILES


def _script_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "validate_fairness_contract.py"
    )


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _empty_profile_overrides_payload() -> dict[str, dict[str, dict]]:
    return {
        profile: {"base_overrides": {}, "profile_specific": {}}
        for profile in SUPPORTED_CONTROLLER_PROFILES
    }


def _make_run(
    *,
    runs_root: Path,
    run_id: str,
    profile: str,
    shared_hash: str = "shared_hash_abc",
    override_diff: dict | None = None,
    mission_name: str = "FairMission",
    mission_path: str = "missions/FairMission.json",
    path_length_m: float = 42.0,
    path_waypoint_count: int = 123,
    git_commit: str = "commit_abc",
    platform: str = "macOS-26.3-arm64-arm-64bit",
    python_version: str = "3.11.14",
    profile_overrides: dict | None = None,
    random_disturbances_enabled: bool = False,
    noise_value: float = 0.0,
    shared_parameters: bool = True,
    profile_parameter_file: str | None = None,
) -> Path:
    run_dir = runs_root / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    status_payload = {
        "schema_version": "run_status_v1",
        "run_id": run_id,
        "status": "completed",
        "mission": {"name": mission_name, "path": mission_path},
        "config": {
            "controller_profile": profile,
            "shared_params_hash": shared_hash,
            "effective_params_hash": f"{shared_hash}_effective",
            "override_diff": {} if override_diff is None else override_diff,
            "shared_parameters": shared_parameters,
            "profile_parameter_file_applied": profile_parameter_file is not None,
            "profile_parameter_file": profile_parameter_file,
        },
    }
    _write_json(run_dir / "Data" / "02_metadata" / "run_status.json", status_payload)

    kpi_payload = {
        "schema_version": "kpi_summary_v1",
        "run_id": run_id,
        "path_length_m": path_length_m,
        "path_waypoint_count": path_waypoint_count,
    }
    _write_json(run_dir / "Data" / "03_diagnostics" / "kpi_summary.json", kpi_payload)

    repro_payload = {
        "schema_version": "run_reproducibility_manifest_v1",
        "run_id": run_id,
        "configuration": {
            "shared_parameters": shared_parameters,
            "profile_parameter_file": profile_parameter_file,
            "app_config": {
                "physics": {
                    "random_disturbances_enabled": random_disturbances_enabled,
                    "position_noise_std": noise_value,
                    "velocity_noise_std": noise_value,
                    "angle_noise_std": noise_value,
                    "angular_velocity_noise_std": noise_value,
                    "thrust_force_noise_percent": noise_value,
                    "disturbance_force_std": noise_value,
                    "disturbance_torque_std": noise_value,
                },
                "mpc_profile_overrides": profile_overrides
                if profile_overrides is not None
                else _empty_profile_overrides_payload(),
            },
        },
        "software": {
            "git": {"commit": git_commit},
            "platform": platform,
            "python_version": python_version,
        },
    }
    _write_json(
        run_dir / "Data" / "02_metadata" / "reproducibility_manifest.json",
        repro_payload,
    )

    _write_json(
        run_dir / "Data" / "02_metadata" / "mission_metadata.json",
        {"mission_name": mission_name},
    )
    return run_dir


def _run_validator(*args: str) -> subprocess.CompletedProcess[str]:
    cmd = [sys.executable, str(_script_path()), *args]
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _load_report(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_fairness_validator_pass_case(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        _make_run(runs_root=runs_root, run_id=run_id, profile=profile)
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 0, result.stdout + result.stderr
    report = _load_report(report_path)
    assert report["schema_version"] == "fairness_report_v1"
    assert report["pass"] is True
    assert report["fail_reasons"] == []


def test_fairness_validator_fails_on_nonempty_override_diff(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        override_diff = {"Q_contour": 1234.0} if idx == 3 else None
        _make_run(
            runs_root=runs_root,
            run_id=run_id,
            profile=profile,
            override_diff=override_diff,
        )
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 1
    report = _load_report(report_path)
    assert report["pass"] is False
    assert any("override_diff not empty" in reason for reason in report["fail_reasons"])


def test_fairness_validator_fails_when_shared_parameters_disabled(
    tmp_path: Path,
) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        _make_run(
            runs_root=runs_root,
            run_id=run_id,
            profile=profile,
            shared_parameters=False if idx == 2 else True,
        )
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 1
    report = _load_report(report_path)
    assert report["pass"] is False
    assert any(
        "shared.parameters must be true" in reason for reason in report["fail_reasons"]
    )


def test_fairness_validator_fails_when_profile_file_applied(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        _make_run(
            runs_root=runs_root,
            run_id=run_id,
            profile=profile,
            profile_parameter_file=(
                "controller/hybrid/profile_parameters.json" if idx == 1 else None
            ),
        )
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 1
    report = _load_report(report_path)
    assert report["pass"] is False
    assert any(
        "profile_parameter_file must not be applied" in reason
        for reason in report["fail_reasons"]
    )


def test_fairness_validator_fails_on_shared_hash_mismatch(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        shared_hash = "hash_A" if idx != 5 else "hash_B"
        _make_run(
            runs_root=runs_root,
            run_id=run_id,
            profile=profile,
            shared_hash=shared_hash,
        )
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 1
    report = _load_report(report_path)
    assert report["pass"] is False
    assert any(
        "shared_params_hash mismatch" in reason for reason in report["fail_reasons"]
    )


def test_fairness_validator_fails_on_mission_mismatch(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        path_length = 42.0 if idx != 6 else 41.5
        _make_run(
            runs_root=runs_root,
            run_id=run_id,
            profile=profile,
            mission_name="MissionA" if idx != 6 else "MissionB",
            mission_path="missions/MissionA.json"
            if idx != 6
            else "missions/MissionB.json",
            path_length_m=path_length,
        )
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 1
    report = _load_report(report_path)
    assert report["pass"] is False
    assert any("mission name mismatch" in reason for reason in report["fail_reasons"])
    assert any("path_length_m mismatch" in reason for reason in report["fail_reasons"])


def test_fairness_validator_fails_on_missing_or_duplicate_profiles(
    tmp_path: Path,
) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []

    subset_profiles = list(SUPPORTED_CONTROLLER_PROFILES[:5]) + [
        SUPPORTED_CONTROLLER_PROFILES[0]
    ]
    for idx, profile in enumerate(subset_profiles, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        _make_run(runs_root=runs_root, run_id=run_id, profile=profile)
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 1
    report = _load_report(report_path)
    assert report["pass"] is False
    assert any("missing profiles" in reason for reason in report["fail_reasons"])
    assert any("duplicate profiles" in reason for reason in report["fail_reasons"])


def test_fairness_validator_fails_on_software_snapshot_mismatch(tmp_path: Path) -> None:
    runs_root = tmp_path / "runs"
    selected_runs: list[str] = []
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        run_id = f"run_{idx:02d}_{profile}"
        git_commit = "commit_A" if idx != 4 else "commit_B"
        _make_run(
            runs_root=runs_root,
            run_id=run_id,
            profile=profile,
            git_commit=git_commit,
        )
        selected_runs.extend(["--run", run_id])

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        *selected_runs,
        "--output",
        str(report_path),
    )

    assert result.returncode == 1
    report = _load_report(report_path)
    assert report["pass"] is False
    assert any("git commit mismatch" in reason for reason in report["fail_reasons"])


def test_fairness_validator_mission_mode_picks_latest_per_profile(
    tmp_path: Path,
) -> None:
    runs_root = tmp_path / "runs"
    now = 1_700_000_000

    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        old_run = _make_run(
            runs_root=runs_root,
            run_id=f"old_{idx:02d}_{profile}",
            profile=profile,
            mission_name="MissionToken",
            mission_path="missions/MissionToken.json",
        )
        new_run = _make_run(
            runs_root=runs_root,
            run_id=f"new_{idx:02d}_{profile}",
            profile=profile,
            mission_name="MissionToken",
            mission_path="missions/MissionToken.json",
        )
        os.utime(old_run, (now + idx, now + idx))
        os.utime(new_run, (now + 100 + idx, now + 100 + idx))

    report_path = tmp_path / "fairness_report.json"
    result = _run_validator(
        "--runs-root",
        str(runs_root),
        "--mission",
        "MissionToken",
        "--output",
        str(report_path),
    )

    assert result.returncode == 0, result.stdout + result.stderr
    report = _load_report(report_path)
    assert report["pass"] is True
    assert all(run_id.startswith("new_") for run_id in report["selected_run_ids"])

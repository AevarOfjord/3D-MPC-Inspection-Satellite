"""MPC quality harness for V6 contract validation."""

from __future__ import annotations

import csv
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from satellite_control.config.paths import (
    MISSIONS_DIR,
    PROJECT_ROOT,
    SCRIPTS_DIR,
    SIMULATION_DATA_ROOT,
    resolve_repo_path,
)
from satellite_control.core.v6_controller_runtime import QualityContractReportV6

DATA_SIM_DIR = SIMULATION_DATA_ROOT
SIM_RUNNER = SCRIPTS_DIR / "run_simulation.py"

MISSION_PLANNER_M4 = PROJECT_ROOT / "missions" / "STARLINK-1008_M4_202602192133.json"
MISSION_PLANNER_2M = PROJECT_ROOT / "missions" / "Starlink2mScan.json"
ENV_MISSION_M4 = "SATCTRL_QUALITY_MISSION_M4"
ENV_MISSION_2M = "SATCTRL_QUALITY_MISSION_2M"
ENV_MISSION_FALLBACK = "SATCTRL_QUALITY_MISSION_FALLBACK"


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    duration_s: float
    contracts: dict[str, tuple[str, float | int | bool]]
    use_auto: bool = False
    mission_path: Path | None = None
    generate_manual_scurve: bool = False
    contract_run: bool = True
    optional: bool = False


@dataclass
class ScenarioResult:
    name: str
    command: list[str]
    return_code: int
    run_dir: str | None
    metrics: dict[str, Any]
    contracts: dict[str, dict[str, Any]]
    passed: bool
    breaches: list[str]
    skipped: bool = False
    skip_reason: str | None = None


@dataclass
class QualitySuiteResult:
    schema_version: str
    generated_at: str
    full: bool
    scenarios: list[ScenarioResult]
    passed: bool
    messages: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "generated_at": self.generated_at,
            "full": self.full,
            "passed": self.passed,
            "scenarios": [asdict(item) for item in self.scenarios],
        }


def _now_iso() -> str:
    return datetime.now(UTC).isoformat()


def _path_from_raw(raw: str | os.PathLike[str] | None) -> Path | None:
    if raw is None:
        return None
    candidate = resolve_repo_path(Path(raw).expanduser())
    if candidate.exists() and candidate.is_file():
        return candidate
    return None


def _discover_local_mission_candidates() -> list[Path]:
    if not MISSIONS_DIR.exists():
        return []
    return sorted(
        path.resolve() for path in MISSIONS_DIR.glob("*.json") if path.is_file()
    )


def _resolve_planner_missions(
    *,
    mission_m4: str | os.PathLike[str] | None = None,
    mission_2m: str | os.PathLike[str] | None = None,
) -> tuple[Path | None, Path | None, list[str]]:
    messages: list[str] = []

    fallback_candidates: list[Path] = []
    fallback_raw = os.environ.get(ENV_MISSION_FALLBACK, "").strip()
    if fallback_raw:
        fallback_path = _path_from_raw(fallback_raw)
        if fallback_path is not None:
            fallback_candidates.append(fallback_path)
        else:
            messages.append(
                f"{ENV_MISSION_FALLBACK} is set but missing/unreadable: {fallback_raw}"
            )
    for candidate in _discover_local_mission_candidates():
        if candidate not in fallback_candidates:
            fallback_candidates.append(candidate)

    def _resolve_one(
        *,
        scenario_name: str,
        explicit_raw: str | os.PathLike[str] | None,
        env_var: str,
        default_path: Path,
        fallback_index: int,
    ) -> Path | None:
        explicit_path = _path_from_raw(explicit_raw)
        if explicit_raw is not None and explicit_path is None:
            messages.append(
                f"{scenario_name}: explicit mission override missing/unreadable: {explicit_raw}"
            )
        if explicit_path is not None:
            return explicit_path

        env_raw = os.environ.get(env_var, "").strip()
        if env_raw:
            env_path = _path_from_raw(env_raw)
            if env_path is None:
                messages.append(
                    f"{scenario_name}: {env_var} is set but missing/unreadable: {env_raw}"
                )
            else:
                return env_path

        if default_path.exists():
            return default_path.resolve()

        if fallback_candidates:
            idx = min(fallback_index, len(fallback_candidates) - 1)
            fallback_path = fallback_candidates[idx]
            messages.append(
                f"{scenario_name}: default mission missing, falling back to {fallback_path}"
            )
            return fallback_path

        messages.append(
            f"{scenario_name}: no planner mission file available; scenario will be skipped"
        )
        return None

    resolved_m4 = _resolve_one(
        scenario_name="planner_m4",
        explicit_raw=mission_m4,
        env_var=ENV_MISSION_M4,
        default_path=MISSION_PLANNER_M4,
        fallback_index=0,
    )
    resolved_2m = _resolve_one(
        scenario_name="planner_2m",
        explicit_raw=mission_2m,
        env_var=ENV_MISSION_2M,
        default_path=MISSION_PLANNER_2M,
        fallback_index=1,
    )
    return resolved_m4, resolved_2m, messages


def _read_latest_run_id() -> str | None:
    latest_path = DATA_SIM_DIR / "latest_run.txt"
    if not latest_path.exists():
        return None
    raw = latest_path.read_text(encoding="utf-8").strip()
    return raw or None


def _resolve_run_dir(before_run_id: str | None, combined_output: str) -> Path | None:
    matches = re.findall(
        r"Created data directory:\s*(.+?)\s*$", combined_output, flags=re.MULTILINE
    )
    if matches:
        candidate = Path(matches[-1].strip().strip("'\""))
        candidate = resolve_repo_path(candidate)
        if candidate.exists() and candidate.is_dir():
            return candidate

    latest_run_id = _read_latest_run_id()
    if not latest_run_id or latest_run_id == before_run_id:
        return None
    candidate = (DATA_SIM_DIR / latest_run_id).resolve()
    if candidate.exists() and candidate.is_dir():
        return candidate
    return None


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _percentile(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    data = sorted(values)
    idx = int(round((len(data) - 1) * p))
    idx = max(0, min(idx, len(data) - 1))
    return data[idx]


def _extract_path_error_p95(step_stats_csv: Path) -> float:
    if not step_stats_csv.exists():
        return 0.0
    values: list[float] = []
    with step_stats_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            value = _to_float(
                row.get("Path_Error_m", row.get("Path_Error")), default=-1.0
            )
            if value >= 0.0:
                values.append(value)
    return _percentile(values, 0.95)


def _extract_metrics(run_dir: Path) -> dict[str, Any]:
    kpi = _read_json(run_dir / "kpi_summary.json")
    perf = _read_json(run_dir / "performance_metrics.json")

    mpc_steps = int(_to_float(kpi.get("mpc_control_steps"), 0.0))
    total_switches = int(_to_float(kpi.get("total_thruster_switches"), 0.0))
    switches_per_step = (
        (float(total_switches) / float(mpc_steps)) if mpc_steps > 0 else 0.0
    )

    timing = perf.get("mpc_timing_contract")
    hard_limit_breaches = 0
    if isinstance(timing, dict):
        hard_limit_breaches = int(_to_float(timing.get("hard_limit_breaches"), 0.0))
    elif "solver_time_limit_exceeded_count" in kpi:
        hard_limit_breaches = int(
            _to_float(kpi.get("solver_time_limit_exceeded_count"), 0.0)
        )

    return {
        "path_completed": bool(kpi.get("path_completed", False)),
        "final_position_error_m": _to_float(kpi.get("final_position_error_m")),
        "final_angle_error_deg": _to_float(kpi.get("final_angle_error_deg")),
        "final_velocity_error_mps": _to_float(kpi.get("final_velocity_error_mps")),
        "final_angular_velocity_error_degps": _to_float(
            kpi.get("final_angular_velocity_error_degps")
        ),
        "mean_solve_ms": _to_float(kpi.get("mpc_mean_solve_time_ms")),
        "max_solve_ms": _to_float(kpi.get("mpc_max_solve_time_ms")),
        "mpc_control_steps": mpc_steps,
        "total_thruster_switches": total_switches,
        "switches_per_step": switches_per_step,
        "path_error_p95_m": _extract_path_error_p95(run_dir / "mpc_step_stats.csv"),
        "mean_active_thrusters": _to_float(kpi.get("mean_active_thrusters")),
        "hard_limit_breaches": hard_limit_breaches,
    }


def _compare(actual: Any, op: str, expected: Any) -> bool:
    if op == "<=":
        return actual <= expected
    if op == ">=":
        return actual >= expected
    if op == "==":
        return actual == expected
    raise ValueError(f"Unsupported contract operator: {op}")


def _evaluate_contracts(
    metrics: dict[str, Any],
    contracts: dict[str, tuple[str, float | int | bool]],
) -> tuple[dict[str, dict[str, Any]], list[str], bool]:
    results: dict[str, dict[str, Any]] = {}
    breaches: list[str] = []

    for metric_name, (op, threshold) in contracts.items():
        actual = metrics.get(metric_name)
        passed = False
        if actual is not None:
            passed = _compare(actual, op, threshold)
        results[metric_name] = {
            "actual": actual,
            "operator": op,
            "threshold": threshold,
            "pass": passed,
        }
        if not passed:
            breaches.append(f"{metric_name} {op} {threshold} (actual={actual})")

    return results, breaches, not breaches


class QualityContractEngineV6:
    """Deterministic contract evaluator and report builder for V6 quality runs."""

    @staticmethod
    def evaluate(
        metrics: dict[str, Any],
        contracts: dict[str, tuple[str, float | int | bool]],
    ) -> tuple[dict[str, dict[str, Any]], list[str], bool]:
        return _evaluate_contracts(metrics, contracts)

    @staticmethod
    def build_report(
        *,
        scenario: str,
        run_dir: Path,
        command: list[str],
        return_code: int,
        metrics: dict[str, Any],
        contracts: dict[str, dict[str, Any]],
        passed: bool,
        breaches: list[str],
    ) -> QualityContractReportV6:
        return QualityContractReportV6(
            schema_version="contract_report_v6",
            generated_at=_now_iso(),
            scenario=scenario,
            run_id=run_dir.name,
            run_dir=str(run_dir),
            command=command,
            return_code=int(return_code),
            metrics=metrics,
            contracts=contracts,
            passed=bool(passed),
            breaches=list(breaches),
        )


def _default_scenarios(
    full: bool,
    *,
    planner_m4: Path | None,
    planner_2m: Path | None,
) -> list[ScenarioSpec]:
    auto_contracts = {
        "path_completed": ("==", True),
        "final_position_error_m": ("<=", 0.06),
        "final_angle_error_deg": ("<=", 4.0),
        "mean_solve_ms": ("<=", 5.0),
        "max_solve_ms": ("<=", 35.0),
        "switches_per_step": ("<=", 0.20),
    }
    planner_contracts = {
        "path_error_p95_m": ("<=", 0.20),
        "mean_active_thrusters": ("<=", 3.0),
        "switches_per_step": ("<=", 0.30),
        "mean_solve_ms": ("<=", 5.0),
        "max_solve_ms": ("<=", 35.0),
        "hard_limit_breaches": ("==", 0),
    }
    long_completion_contracts = {
        "path_completed": ("==", True),
        "final_position_error_m": ("<=", 0.10),
        "final_angle_error_deg": ("<=", 2.0),
        "final_velocity_error_mps": ("<=", 0.05),
        "final_angular_velocity_error_degps": ("<=", 2.0),
        "mean_solve_ms": ("<=", 5.0),
        "max_solve_ms": ("<=", 35.0),
        "hard_limit_breaches": ("==", 0),
    }

    scenarios = [
        ScenarioSpec(
            name="auto_short",
            duration_s=30.0,
            use_auto=True,
            contracts=auto_contracts,
        )
    ]

    if full:
        scenarios.extend(
            [
                ScenarioSpec(
                    name="planner_m4",
                    duration_s=60.0,
                    mission_path=planner_m4,
                    contracts=planner_contracts,
                    optional=True,
                ),
                ScenarioSpec(
                    name="planner_2m",
                    duration_s=60.0,
                    mission_path=planner_2m,
                    contracts=planner_contracts,
                    optional=True,
                ),
                ScenarioSpec(
                    name="stress_manual_scurve",
                    duration_s=35.0,
                    generate_manual_scurve=True,
                    contracts=auto_contracts,
                ),
                ScenarioSpec(
                    name="long_completion_tier",
                    duration_s=180.0,
                    mission_path=planner_m4,
                    contracts=long_completion_contracts,
                    optional=True,
                ),
            ]
        )

    return scenarios


def _build_manual_scurve_points(samples: int = 121) -> list[list[float]]:
    points: list[list[float]] = []
    for i in range(samples):
        t = i / max(1, samples - 1)
        x = 8.0 - 16.0 * t
        y = 2.0 * (1.0 - 2.0 * t) * (2.0 * t - 1.0) * (2.0 * t - 1.0)
        z = 0.8 * (2.0 * t - 1.0)
        points.append([round(x, 6), round(y, 6), round(z, 6)])
    return points


def _create_manual_scurve_mission(path: Path) -> None:
    manual_path = _build_manual_scurve_points()
    payload = {
        "schema_version": 2,
        "mission_id": "mission_v5_manual_scurve",
        "name": "V5_Manual_SCurve",
        "epoch": _now_iso(),
        "start_pose": {
            "frame": "LVLH",
            "position": manual_path[0],
            "orientation": None,
        },
        "segments": [],
        "obstacles": [],
        "overrides": {
            "spline_controls": [],
            "manual_path": manual_path,
            "path_density_multiplier": 1.0,
        },
        "metadata": {
            "version": 1,
            "created_at": _now_iso(),
            "updated_at": _now_iso(),
            "tags": ["v5", "quality", "stress"],
        },
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _run_scenario(
    scenario: ScenarioSpec,
    python_executable: str,
    manual_scurve_path: Path | None,
) -> ScenarioResult:
    if scenario.generate_manual_scurve:
        mission_path = manual_scurve_path
    else:
        mission_path = scenario.mission_path

    command = [
        python_executable,
        str(SIM_RUNNER),
        "--no-anim",
        "--duration",
        str(scenario.duration_s),
    ]
    if scenario.use_auto:
        command.append("--auto")
    elif mission_path is not None:
        command.extend(["--mission", str(mission_path)])
    elif scenario.optional:
        return ScenarioResult(
            name=scenario.name,
            command=command,
            return_code=0,
            run_dir=None,
            metrics={},
            contracts={},
            passed=True,
            breaches=[],
            skipped=True,
            skip_reason=(
                "Optional planner scenario skipped because no mission file was resolved."
            ),
        )

    if not SIM_RUNNER.exists():
        return ScenarioResult(
            name=scenario.name,
            command=command,
            return_code=1,
            run_dir=None,
            metrics={},
            contracts={},
            passed=False,
            breaches=[f"simulation_runner_missing ({SIM_RUNNER})"],
            skipped=False,
            skip_reason=None,
        )

    before_run_id = _read_latest_run_id()
    env = os.environ.copy()
    env["SATELLITE_HEADLESS"] = "1"
    env["SATCTRL_CONTRACT_SCENARIO"] = "1" if scenario.contract_run else "0"

    completed = subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    combined_output = f"{completed.stdout}\n{completed.stderr}"
    run_dir = _resolve_run_dir(before_run_id, combined_output)

    engine = QualityContractEngineV6()
    metrics: dict[str, Any] = {}
    contract_results: dict[str, dict[str, Any]] = {}
    breaches: list[str] = []
    passed = completed.returncode == 0

    if completed.returncode != 0:
        breaches.append(f"simulation_process_failed (exit_code={completed.returncode})")
        passed = False

    if run_dir is None:
        breaches.append("run_directory_not_found")
        passed = False
    else:
        metrics = _extract_metrics(run_dir)
        contract_results, contract_breaches, contracts_pass = engine.evaluate(
            metrics,
            scenario.contracts,
        )
        breaches.extend(contract_breaches)
        passed = passed and contracts_pass

        report = engine.build_report(
            scenario=scenario.name,
            run_dir=run_dir,
            command=command,
            return_code=completed.returncode,
            metrics=metrics,
            contracts=contract_results,
            passed=passed,
            breaches=breaches,
        )
        (run_dir / "contract_report_v6.json").write_text(
            json.dumps(report.to_dict(), indent=2),
            encoding="utf-8",
        )
        # Compatibility mirror for older tooling.
        (run_dir / "mpc_quality_report.json").write_text(
            json.dumps(report.to_dict(), indent=2),
            encoding="utf-8",
        )

    return ScenarioResult(
        name=scenario.name,
        command=command,
        return_code=int(completed.returncode),
        run_dir=str(run_dir) if run_dir is not None else None,
        metrics=metrics,
        contracts=contract_results,
        passed=passed,
        breaches=breaches,
    )


def run_mpc_quality_suite(
    *,
    full: bool = False,
    fail_on_breach: bool = False,
    python_executable: str | None = None,
    keep_temp_files: bool = False,
    mission_m4: str | os.PathLike[str] | None = None,
    mission_2m: str | os.PathLike[str] | None = None,
) -> QualitySuiteResult:
    """Execute MPC quality scenarios and evaluate contract thresholds."""
    py_exec = python_executable or sys.executable
    resolved_m4, resolved_2m, messages = _resolve_planner_missions(
        mission_m4=mission_m4,
        mission_2m=mission_2m,
    )
    scenarios = _default_scenarios(
        full=full,
        planner_m4=resolved_m4,
        planner_2m=resolved_2m,
    )

    manual_path: Path | None = None
    temp_dir_obj: tempfile.TemporaryDirectory[str] | None = None

    if any(item.generate_manual_scurve for item in scenarios):
        temp_dir_obj = tempfile.TemporaryDirectory(prefix="v5_mpc_quality_")
        manual_path = Path(temp_dir_obj.name) / "manual_scurve_quality.json"
        _create_manual_scurve_mission(manual_path)

    try:
        results = [_run_scenario(item, py_exec, manual_path) for item in scenarios]
    finally:
        if temp_dir_obj is not None and not keep_temp_files:
            temp_dir_obj.cleanup()

    passed = all(item.passed for item in results)
    suite = QualitySuiteResult(
        schema_version="mpc_quality_suite_v6",
        generated_at=_now_iso(),
        full=full,
        scenarios=results,
        passed=passed,
        messages=messages,
    )

    if fail_on_breach and not passed:
        raise RuntimeError("MPC quality suite failed one or more contracts")

    return suite

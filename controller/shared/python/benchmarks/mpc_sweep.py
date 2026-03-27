"""Grid-search sweep runner for MPC timing and horizon tuning."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import tempfile
from collections.abc import Callable
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from controller.configs.paths import DATA_ROOT, SCRIPTS_DIR, SIMULATION_DATA_ROOT
from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.control_common.profile_run_config import (
    load_json_object,
    normalize_profile_id,
    persist_profile_sweep_winner,
)
from controller.shared.python.mission.repository import (
    list_mission_entries,
    resolve_mission_file,
)
from controller.shared.python.simulation.artifact_paths import (
    artifact_path,
    resolve_existing_artifact_path,
)
from controller.shared.python.simulation.profile_prompt import (
    SUPPORTED_CONTROLLER_PROFILES,
    prompt_controller_profile,
)

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DT_GRID_SECONDS: tuple[float, ...] = tuple(
    round(step / 100.0, 2) for step in range(1, 11)
)
HORIZON_GRID: tuple[int, ...] = tuple(step * 10 for step in range(1, 11))
SIM_RUNNER = SCRIPTS_DIR / "run_simulation.py"
SWEEPS_ROOT = DATA_ROOT / "sweeps"
SWEEP_METRICS: tuple[str, ...] = (
    "path_position_error_p95_m",
    "path_angular_error_p95_deg",
    "terminal_position_error_m",
    "terminal_angular_error_deg",
)

SweepExecutor = Callable[
    [str, int, float, dict[str, Any], Path, str, int, float],
    dict[str, Any],
]


def _now_stamp() -> str:
    return datetime.now(UTC).strftime("%Y%m%d_%H%M%S")


def _slug(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    return cleaned.strip("._") or "sweep"


def _display_controller_name(profile: str) -> str:
    return profile[4:] if profile.startswith("cpp_") else profile


def _questionary_style():
    import questionary

    return questionary.Style(
        [
            ("qmark", ""),
            ("question", "bold"),
            ("pointer", "fg:#ffffff bg:#005f87 bold"),
            ("highlighted", "fg:#ffffff bg:#005f87 bold"),
            ("selected", "fg:#ffffff bg:#005f87"),
        ]
    )


def _coerce_float(value: Any, default: float = math.inf) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _coerce_int(value: Any, default: int = 10**9) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _load_base_overrides(base_config_path: Path | None) -> dict[str, Any]:
    if base_config_path is None:
        return {}
    return load_json_object(base_config_path)


def build_sweep_point_overrides(
    *,
    base_overrides: dict[str, Any],
    profile: str,
    prediction_horizon: int,
    control_dt_s: float,
    base_control_horizon: int,
    base_solver_time_limit_s: float,
) -> dict[str, Any]:
    """Return config overrides for one sweep point."""
    normalized_profile = normalize_profile_id(profile)
    payload = deepcopy(base_overrides)

    shared = payload.get("shared")
    if not isinstance(shared, dict):
        shared = {}
        payload["shared"] = shared
    shared["parameters"] = True

    mpc = payload.get("mpc")
    if not isinstance(mpc, dict):
        mpc = {}
        payload["mpc"] = mpc

    control_horizon = min(int(base_control_horizon), int(prediction_horizon))
    solver_time_limit = min(float(base_solver_time_limit_s), float(control_dt_s) * 0.8)
    mpc["prediction_horizon"] = int(prediction_horizon)
    mpc["control_horizon"] = int(control_horizon)
    mpc["dt"] = float(control_dt_s)
    mpc["solver_time_limit"] = float(solver_time_limit)

    simulation = payload.get("simulation")
    if not isinstance(simulation, dict):
        simulation = {}
        payload["simulation"] = simulation
    simulation["control_dt"] = float(control_dt_s)

    mpc_core = payload.get("mpc_core")
    if not isinstance(mpc_core, dict):
        mpc_core = {}
        payload["mpc_core"] = mpc_core
    mpc_core["controller_profile"] = normalized_profile

    return payload


def _read_latest_run_id() -> str | None:
    latest_path = SIMULATION_DATA_ROOT / "latest_run.txt"
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
        if candidate.exists() and candidate.is_dir():
            return candidate.resolve()

    latest_run_id = _read_latest_run_id()
    if not latest_run_id or latest_run_id == before_run_id:
        return None
    candidate = (SIMULATION_DATA_ROOT / latest_run_id).resolve()
    if candidate.exists() and candidate.is_dir():
        return candidate
    return None


def _percentile(values: list[float], p: float) -> float:
    if not values:
        return math.inf
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * p))
    index = max(0, min(index, len(ordered) - 1))
    return float(ordered[index])


def _extract_p95(csv_path: Path, *column_names: str) -> float:
    if not csv_path.exists():
        return math.inf
    values: list[float] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            for column_name in column_names:
                if column_name not in row:
                    continue
                value = _coerce_float(row.get(column_name), default=math.inf)
                if math.isfinite(value):
                    values.append(value)
                break
    return _percentile(values, 0.95)


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def _extract_run_metrics(run_dir: Path) -> dict[str, Any]:
    """Load KPI + step-stat metrics for a completed run directory."""
    kpi = _read_json(
        resolve_existing_artifact_path(run_dir, "kpi_summary.json")
        or artifact_path(run_dir, "kpi_summary.json")
    )
    step_stats_csv = resolve_existing_artifact_path(
        run_dir, "mpc_step_stats.csv"
    ) or artifact_path(run_dir, "mpc_step_stats.csv")
    return {
        "path_completed": bool(kpi.get("path_completed", False)),
        "timing_violation_count": _coerce_int(kpi.get("timing_violation_count"), 10**9),
        "solver_fallback_count": _coerce_int(kpi.get("solver_fallback_count"), 10**9),
        "solver_hard_limit_breaches": _coerce_int(
            kpi.get("solver_hard_limit_breaches"), 10**9
        ),
        "path_position_error_p95_m": _extract_p95(
            step_stats_csv, "Path_Error_m", "Path_Error"
        ),
        "path_angular_error_p95_deg": _extract_p95(step_stats_csv, "Ang_Error_deg"),
        "terminal_position_error_m": _coerce_float(
            kpi.get("final_position_error_m"), default=math.inf
        ),
        "terminal_angular_error_deg": _coerce_float(
            kpi.get("final_angle_error_deg"), default=math.inf
        ),
        "mpc_mean_solve_time_ms": _coerce_float(
            kpi.get("mpc_mean_solve_time_ms"), default=math.inf
        ),
        "mpc_max_solve_time_ms": _coerce_float(
            kpi.get("mpc_max_solve_time_ms"), default=math.inf
        ),
    }


def _build_base_config(base_overrides: dict[str, Any]) -> tuple[int, float]:
    sim_config = SimulationConfig.create_with_overrides(base_overrides)
    return (
        int(sim_config.app_config.mpc.control_horizon),
        float(sim_config.app_config.mpc.solver_time_limit),
    )


def _execute_sweep_point(
    profile: str,
    prediction_horizon: int,
    control_dt_s: float,
    base_overrides: dict[str, Any],
    mission_path: Path,
    python_executable: str,
    base_control_horizon: int,
    base_solver_time_limit_s: float,
) -> dict[str, Any]:
    overrides = build_sweep_point_overrides(
        base_overrides=base_overrides,
        profile=profile,
        prediction_horizon=prediction_horizon,
        control_dt_s=control_dt_s,
        base_control_horizon=base_control_horizon,
        base_solver_time_limit_s=base_solver_time_limit_s,
    )
    control_horizon = int(overrides["mpc"]["control_horizon"])
    solver_time_limit_s = float(overrides["mpc"]["solver_time_limit"])

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", encoding="utf-8", delete=False
    ) as tmp:
        temp_config_path = Path(tmp.name)
        json.dump(overrides, tmp, indent=2)
        tmp.write("\n")

    command = [
        python_executable,
        str(SIM_RUNNER),
        "--no-anim",
        "--mission",
        str(mission_path),
        "--config",
        str(temp_config_path),
    ]
    before_run_id = _read_latest_run_id()
    env = os.environ.copy()
    env["SATELLITE_HEADLESS"] = "1"

    try:
        completed = subprocess.run(
            command,
            cwd=str(SIM_RUNNER.parent.parent),
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
    finally:
        temp_config_path.unlink(missing_ok=True)

    combined_output = f"{completed.stdout}\n{completed.stderr}"
    run_dir = _resolve_run_dir(before_run_id, combined_output)
    metrics = _extract_run_metrics(run_dir) if run_dir is not None else {}
    process_succeeded = completed.returncode == 0 and run_dir is not None
    return {
        "controller_profile": normalize_profile_id(profile),
        "prediction_horizon": int(prediction_horizon),
        "control_dt_s": float(control_dt_s),
        "control_horizon": int(control_horizon),
        "solver_time_limit_s": float(solver_time_limit_s),
        "return_code": int(completed.returncode),
        "run_dir": str(run_dir) if run_dir is not None else "",
        "run_id": run_dir.name if run_dir is not None else "",
        "process_succeeded": bool(process_succeeded),
        "path_completed": bool(metrics.get("path_completed", False)),
        "timing_violation_count": _coerce_int(
            metrics.get("timing_violation_count"), 10**9
        ),
        "solver_fallback_count": _coerce_int(
            metrics.get("solver_fallback_count"), 10**9
        ),
        "solver_hard_limit_breaches": _coerce_int(
            metrics.get("solver_hard_limit_breaches"), 10**9
        ),
        "path_position_error_p95_m": _coerce_float(
            metrics.get("path_position_error_p95_m"), default=math.inf
        ),
        "path_angular_error_p95_deg": _coerce_float(
            metrics.get("path_angular_error_p95_deg"), default=math.inf
        ),
        "terminal_position_error_m": _coerce_float(
            metrics.get("terminal_position_error_m"), default=math.inf
        ),
        "terminal_angular_error_deg": _coerce_float(
            metrics.get("terminal_angular_error_deg"), default=math.inf
        ),
        "mpc_mean_solve_time_ms": _coerce_float(
            metrics.get("mpc_mean_solve_time_ms"), default=math.inf
        ),
        "mpc_max_solve_time_ms": _coerce_float(
            metrics.get("mpc_max_solve_time_ms"), default=math.inf
        ),
    }


def _winner_tiebreak_key(row: dict[str, Any]) -> tuple[float, float, int, float]:
    return (
        _coerce_float(row.get("mpc_mean_solve_time_ms"), default=math.inf),
        _coerce_float(row.get("mpc_max_solve_time_ms"), default=math.inf),
        _coerce_int(row.get("prediction_horizon"), default=10**9),
        -_coerce_float(row.get("control_dt_s"), default=0.0),
    )


def _metric_sort_key(
    metric_name: str, row: dict[str, Any]
) -> tuple[float, float, float, int, float]:
    return (
        _coerce_float(row.get(metric_name), default=math.inf),
    ) + _winner_tiebreak_key(row)


def _fallback_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        0 if row.get("process_succeeded") else 1,
        0 if row.get("path_completed") else 1,
        _coerce_int(row.get("timing_violation_count"), default=10**9),
        _coerce_int(row.get("solver_fallback_count"), default=10**9),
        _coerce_int(row.get("solver_hard_limit_breaches"), default=10**9),
        _coerce_float(row.get("path_position_error_p95_m"), default=math.inf),
        _coerce_float(row.get("path_angular_error_p95_deg"), default=math.inf),
        _coerce_float(row.get("terminal_position_error_m"), default=math.inf),
        _coerce_float(row.get("terminal_angular_error_deg"), default=math.inf),
    ) + _winner_tiebreak_key(row)


def score_sweep_rows(
    rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any] | None, bool]:
    """Apply eligibility gating, metric ranks, and winner selection."""
    scored_rows = [dict(row) for row in rows]
    for row in scored_rows:
        row["eligible"] = bool(
            row.get("process_succeeded")
            and row.get("path_completed")
            and _coerce_int(row.get("timing_violation_count"), 10**9) == 0
            and _coerce_int(row.get("solver_fallback_count"), 10**9) == 0
            and _coerce_int(row.get("solver_hard_limit_breaches"), 10**9) == 0
        )
        row["final_score"] = None
        row["fallback_rank"] = None
        row["winner_reason"] = ""

    eligible_rows = [row for row in scored_rows if row["eligible"]]
    if eligible_rows:
        for metric_name in SWEEP_METRICS:
            ordered = sorted(
                eligible_rows, key=lambda item: _metric_sort_key(metric_name, item)
            )
            for rank, row in enumerate(ordered, start=1):
                row[f"{metric_name}_rank"] = float(rank)
        for row in scored_rows:
            if not row["eligible"]:
                for metric_name in SWEEP_METRICS:
                    row[f"{metric_name}_rank"] = None
                row["heatmap_score"] = None
                continue
            ranks = [float(row[f"{metric_name}_rank"]) for metric_name in SWEEP_METRICS]
            row["final_score"] = float(sum(ranks) / len(ranks))
            row["heatmap_score"] = row["final_score"]
        winner = min(
            eligible_rows,
            key=lambda item: (
                _coerce_float(item.get("final_score"), default=math.inf),
                *_winner_tiebreak_key(item),
            ),
        )
        winner["winner_reason"] = "eligible_best"
        return scored_rows, winner, True

    ordered_fallback = sorted(scored_rows, key=_fallback_sort_key)
    for rank, row in enumerate(ordered_fallback, start=1):
        row["fallback_rank"] = int(rank)
        row["heatmap_score"] = float(rank)
        for metric_name in SWEEP_METRICS:
            row[f"{metric_name}_rank"] = None
    winner = ordered_fallback[0] if ordered_fallback else None
    if winner is not None:
        winner["winner_reason"] = "fallback_best"
    return scored_rows, winner, False


def _write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_json(payload: dict[str, Any], path: Path) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_heatmap(
    rows: list[dict[str, Any]],
    *,
    profile: str,
    out_path: Path,
    eligible_exists: bool,
) -> None:
    matrix = np.full((len(HORIZON_GRID), len(DT_GRID_SECONDS)), np.nan, dtype=float)
    for row in rows:
        try:
            y_index = HORIZON_GRID.index(int(row["prediction_horizon"]))
            x_index = DT_GRID_SECONDS.index(round(float(row["control_dt_s"]), 2))
        except ValueError:
            continue
        heatmap_score = row.get("heatmap_score")
        if heatmap_score is None:
            continue
        matrix[y_index, x_index] = float(heatmap_score)

    fig, ax = plt.subplots(figsize=(9, 7))
    cmap = plt.cm.viridis.copy()
    cmap.set_bad(color="#d9d9d9")
    image = ax.imshow(matrix, aspect="auto", origin="lower", cmap=cmap)
    ax.set_xticks(range(len(DT_GRID_SECONDS)))
    ax.set_xticklabels([f"{int(dt * 1000)}" for dt in DT_GRID_SECONDS])
    ax.set_yticks(range(len(HORIZON_GRID)))
    ax.set_yticklabels([str(horizon) for horizon in HORIZON_GRID])
    ax.set_xlabel("Control Interval (ms)")
    ax.set_ylabel("Prediction Horizon")
    ax.set_title(
        f"Sweep Score Heatmap: {_display_controller_name(profile)}"
        + (" (eligible only)" if eligible_exists else " (fallback ranking)")
    )
    colorbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
    colorbar.set_label("Score" if eligible_exists else "Fallback Rank")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def _write_controller_summary_markdown(
    *,
    profile: str,
    rows: list[dict[str, Any]],
    winner: dict[str, Any] | None,
    profile_updated: bool,
    profile_path: Path | None,
    out_path: Path,
) -> None:
    lines = [
        f"# Sweep Summary: {_display_controller_name(profile)}",
        "",
        f"- Controller profile: `{profile}`",
        f"- Sweep points: `{len(rows)}`",
        f"- Eligible runs: `{sum(1 for row in rows if row.get('eligible'))}`",
        f"- Profile updated: `{profile_updated}`",
    ]
    if profile_path is not None:
        lines.append(f"- Profile file: `{profile_path}`")
    if winner is not None:
        lines.extend(
            [
                "",
                "## Winner",
                "",
                f"- Reason: `{winner.get('winner_reason')}`",
                f"- Prediction horizon: `{winner.get('prediction_horizon')}`",
                f"- Control interval: `{int(round(float(winner.get('control_dt_s', 0.0)) * 1000.0))} ms`",
                f"- Control horizon: `{winner.get('control_horizon')}`",
                f"- Solver time limit: `{float(winner.get('solver_time_limit_s', 0.0)):.6f} s`",
                f"- Path position p95: `{float(winner.get('path_position_error_p95_m', math.inf)):.6f}`",
                f"- Path angular p95: `{float(winner.get('path_angular_error_p95_deg', math.inf)):.6f}`",
                f"- Terminal position error: `{float(winner.get('terminal_position_error_m', math.inf)):.6f}`",
                f"- Terminal angular error: `{float(winner.get('terminal_angular_error_deg', math.inf)):.6f}`",
            ]
        )
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_batch_summary_markdown(
    *,
    mission_path: Path,
    summaries: list[dict[str, Any]],
    out_path: Path,
) -> None:
    lines = [
        "# MPC Sweep Batch Summary",
        "",
        f"- Mission: `{mission_path}`",
        f"- Generated: `{datetime.now(UTC).isoformat()}`",
        "",
        "| controller_profile | winner_reason | eligible | N | dt_ms | updated_profile | profile_file |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for summary in summaries:
        winner = summary.get("winner") or {}
        dt_ms = int(round(float(winner.get("control_dt_s", 0.0)) * 1000.0))
        lines.append(
            "| "
            + " | ".join(
                [
                    str(summary.get("controller_profile", "")),
                    str(winner.get("winner_reason", "")),
                    str(bool(summary.get("winner_eligible", False))),
                    str(winner.get("prediction_horizon", "")),
                    str(dt_ms),
                    str(bool(summary.get("profile_updated", False))),
                    str(summary.get("profile_file_path", "")),
                ]
            )
            + " |"
        )
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _prompt_sweep_scope() -> str:
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        return "single"
    try:
        import questionary

        selection = questionary.select(
            "Run sweep for a single controller or all controllers?",
            choices=[
                questionary.Choice("Single controller", value="single"),
                questionary.Choice("All controllers", value="all"),
            ],
            qmark="",
            style=_questionary_style(),
        ).ask()
        return selection or "single"
    except Exception:
        raw = input("Sweep scope [1=single, 2=all, blank=1]: ").strip()
        return "all" if raw == "2" else "single"


def _prompt_mission_path() -> Path:
    entries = list_mission_entries(source_priority=("local",))
    if not entries:
        raise SystemExit("No saved missions found in missions/.")
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        return entries[0].path
    try:
        import questionary

        selection = questionary.select(
            "Select mission to sweep:",
            choices=[
                questionary.Choice(title=entry.path.name, value=str(entry.path))
                for entry in entries
            ],
            qmark="",
            style=_questionary_style(),
        ).ask()
        if not selection:
            raise SystemExit("Mission selection cancelled.")
        return Path(selection).resolve()
    except Exception:
        for idx, entry in enumerate(entries, start=1):
            print(f"  {idx}. {entry.path.name}")
        raw = input("Select mission number (blank=1): ").strip()
        selected = int(raw) if raw.isdigit() else 1
        selected = max(1, min(selected, len(entries)))
        return entries[selected - 1].path.resolve()


def _resolve_mission_arg(mission_arg: str | None) -> Path:
    if mission_arg:
        candidate = Path(mission_arg)
        if candidate.exists():
            return candidate.resolve()
        return resolve_mission_file(mission_arg).resolve()
    return _prompt_mission_path()


def _resolve_profiles(
    *,
    controller_profile: str | None,
    all_controllers: bool,
) -> list[str]:
    if all_controllers:
        return list(SUPPORTED_CONTROLLER_PROFILES)
    if controller_profile:
        return [normalize_profile_id(controller_profile)]
    if _prompt_sweep_scope() == "all":
        return list(SUPPORTED_CONTROLLER_PROFILES)
    return [prompt_controller_profile()]


def run_controller_sweep(
    *,
    controller_profile: str,
    mission_path: Path,
    base_overrides: dict[str, Any],
    python_executable: str,
    output_dir: Path,
    executor: SweepExecutor | None = None,
) -> dict[str, Any]:
    """Run the 10x10 sweep for one controller and write artifacts."""
    output_dir.mkdir(parents=True, exist_ok=True)
    base_control_horizon, base_solver_time_limit_s = _build_base_config(base_overrides)
    run_point = _execute_sweep_point if executor is None else executor

    rows: list[dict[str, Any]] = []
    for control_dt_s in DT_GRID_SECONDS:
        for prediction_horizon in HORIZON_GRID:
            row = run_point(
                controller_profile,
                prediction_horizon,
                control_dt_s,
                base_overrides,
                mission_path,
                python_executable,
                base_control_horizon,
                base_solver_time_limit_s,
            )
            rows.append(row)

    scored_rows, winner, winner_eligible = score_sweep_rows(rows)
    profile_updated = False
    profile_path: Path | None = None
    if winner is not None and winner_eligible:
        profile_path = persist_profile_sweep_winner(
            profile=controller_profile,
            prediction_horizon=int(winner["prediction_horizon"]),
            control_horizon=int(winner["control_horizon"]),
            dt=float(winner["control_dt_s"]),
            solver_time_limit=float(winner["solver_time_limit_s"]),
        )
        profile_updated = True

    summary_payload = {
        "controller_profile": controller_profile,
        "mission_path": str(mission_path),
        "rows": scored_rows,
        "winner": winner,
        "winner_eligible": bool(winner_eligible),
        "profile_updated": bool(profile_updated),
        "profile_file_path": str(profile_path) if profile_path is not None else "",
    }
    _write_csv(
        scored_rows,
        output_dir / f"{_slug(controller_profile)}_matrix.csv",
    )
    _write_json(
        summary_payload,
        output_dir / f"{_slug(controller_profile)}_matrix.json",
    )
    _write_heatmap(
        scored_rows,
        profile=controller_profile,
        out_path=output_dir / f"{_slug(controller_profile)}_heatmap.png",
        eligible_exists=winner_eligible,
    )
    _write_controller_summary_markdown(
        profile=controller_profile,
        rows=scored_rows,
        winner=winner,
        profile_updated=profile_updated,
        profile_path=profile_path,
        out_path=output_dir / f"{_slug(controller_profile)}_summary.md",
    )
    return summary_payload


def run_mpc_sweep(
    *,
    mission: str | None,
    controller_profile: str | None,
    all_controllers: bool,
    base_config_path: Path | None,
    python_executable: str | None = None,
    output_root: Path | None = None,
    executor: SweepExecutor | None = None,
) -> Path:
    """Run the requested sweep and return the batch output directory."""
    python_executable = python_executable or sys.executable
    mission_path = _resolve_mission_arg(mission)
    profiles = _resolve_profiles(
        controller_profile=controller_profile,
        all_controllers=all_controllers,
    )
    base_overrides = _load_base_overrides(base_config_path)

    batch_root = output_root or (
        SWEEPS_ROOT / f"{_now_stamp()}_{_slug(mission_path.stem)}"
    )
    batch_root.mkdir(parents=True, exist_ok=True)

    summaries: list[dict[str, Any]] = []
    for profile in profiles:
        summary = run_controller_sweep(
            controller_profile=profile,
            mission_path=mission_path,
            base_overrides=base_overrides,
            python_executable=python_executable,
            output_dir=batch_root,
            executor=executor,
        )
        summaries.append(summary)

    batch_payload = {
        "schema_version": "mpc_sweep_v1",
        "generated_at": datetime.now(UTC).isoformat(),
        "mission_path": str(mission_path),
        "base_config_path": str(base_config_path)
        if base_config_path is not None
        else "",
        "profiles": summaries,
    }
    _write_json(batch_payload, batch_root / "batch_summary.json")
    _write_batch_summary_markdown(
        mission_path=mission_path,
        summaries=summaries,
        out_path=batch_root / "batch_summary.md",
    )
    return batch_root


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run MPC sweep grid search.")
    parser.add_argument(
        "--mission",
        help="Mission filename or path. Omit for interactive mission selection.",
    )
    parser.add_argument(
        "--controller-profile",
        help="Single controller profile to sweep. Omit for interactive selection.",
    )
    parser.add_argument(
        "--all-controllers",
        action="store_true",
        help="Sweep all supported controller profiles.",
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        default=None,
        help="Baseline config JSON file to layer sweep overrides onto.",
    )
    parser.add_argument(
        "--python-executable",
        default=sys.executable,
        help="Python executable used to launch child simulation runs.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Optional explicit output directory for sweep artifacts.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(list(sys.argv[1:] if argv is None else argv))
    output_dir = run_mpc_sweep(
        mission=args.mission,
        controller_profile=args.controller_profile,
        all_controllers=bool(args.all_controllers),
        base_config_path=args.base_config,
        python_executable=args.python_executable,
        output_root=args.output_root,
    )
    print(f"Sweep complete. Artifacts saved to: {output_dir}")
    return 0


__all__ = [
    "DT_GRID_SECONDS",
    "HORIZON_GRID",
    "build_sweep_point_overrides",
    "main",
    "run_controller_sweep",
    "run_mpc_sweep",
    "score_sweep_rows",
]


if __name__ == "__main__":
    raise SystemExit(main())

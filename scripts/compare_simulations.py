#!/usr/bin/env python3
"""Compare up to 10 simulation runs with time-based overlay plots."""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from controller.configs.paths import SIMULATION_DATA_ROOT
from controller.shared.python.simulation.artifact_paths import (
    artifact_path,
    resolve_existing_artifact_path,
)

MAX_RUNS_DEFAULT = 10
PLOT_DPI = 140


def _display_controller_name(value: Any) -> str:
    text = str(value or "").strip() or "unknown"
    return text[4:] if text.startswith("cpp_") else text


@dataclass(frozen=True)
class RunCandidate:
    run_dir: Path
    run_id: str
    mission: str
    controller_profile: str
    status: str
    final_time_s: float


@dataclass
class RunBundle:
    run_dir: Path
    run_id: str
    mission: str
    controller_profile: str
    status: str
    step_df: pd.DataFrame
    kpi: dict[str, Any]

    @property
    def legend_label(self) -> str:
        return _display_controller_name(self.controller_profile)


def _now_utc_stamp() -> str:
    return datetime.now(UTC).strftime("%Y%m%d_%H%M%S")


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        return payload if isinstance(payload, dict) else {}
    except Exception:
        return {}


def _status_path(run_dir: Path) -> Path:
    return resolve_existing_artifact_path(run_dir, "run_status.json") or artifact_path(
        run_dir, "run_status.json"
    )


def _kpi_path(run_dir: Path) -> Path:
    return resolve_existing_artifact_path(run_dir, "kpi_summary.json") or artifact_path(
        run_dir, "kpi_summary.json"
    )


def _steps_path(run_dir: Path) -> Path:
    return resolve_existing_artifact_path(
        run_dir, "mpc_step_stats.csv"
    ) or artifact_path(run_dir, "mpc_step_stats.csv")


def _control_path(run_dir: Path) -> Path:
    return resolve_existing_artifact_path(run_dir, "control_data.csv") or artifact_path(
        run_dir, "control_data.csv"
    )


def _mission_metadata_path(run_dir: Path) -> Path:
    return resolve_existing_artifact_path(
        run_dir, "mission_metadata.json"
    ) or artifact_path(run_dir, "mission_metadata.json")


def _coalesce_text(*values: Any) -> str | None:
    for value in values:
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return None


def _extract_run_identity(
    run_dir: Path, status: dict[str, Any]
) -> tuple[str, str, str]:
    mission_payload = status.get("mission")
    mission_dict = mission_payload if isinstance(mission_payload, dict) else {}

    config_payload = status.get("config")
    config_dict = config_payload if isinstance(config_payload, dict) else {}

    controller_payload = status.get("controller")
    controller_dict = controller_payload if isinstance(controller_payload, dict) else {}

    mission_metadata = _load_json(_mission_metadata_path(run_dir))
    mission_path_raw = mission_dict.get("path")
    mission_path_stem = None
    if isinstance(mission_path_raw, str) and mission_path_raw.strip():
        mission_path_stem = Path(mission_path_raw).stem

    mission = (
        _coalesce_text(
            mission_dict.get("name"),
            mission_path_stem,
            mission_metadata.get("mission_name"),
            mission_metadata.get("name"),
            mission_metadata.get("mission_type"),
        )
        or "unknown"
    )
    controller_profile = (
        _coalesce_text(
            config_dict.get("controller_profile"),
            status.get("controller_profile"),
            controller_dict.get("profile"),
        )
        or "unknown"
    )
    run_status = _coalesce_text(status.get("status")) or "unknown"
    return mission, controller_profile, run_status


def _series_has_signal(values: pd.Series, eps: float = 1e-12) -> bool:
    arr = pd.to_numeric(values, errors="coerce").to_numpy(dtype=float)
    finite = np.isfinite(arr)
    if not np.any(finite):
        return False
    return bool(np.nanmax(np.abs(arr[finite])) > eps)


def _align_control_series(
    step_df: pd.DataFrame, control_df: pd.DataFrame, column: str
) -> pd.Series | None:
    if column not in control_df.columns:
        return None

    values = pd.to_numeric(control_df[column], errors="coerce")
    if len(values) == len(step_df):
        return values.reset_index(drop=True)

    control_time_col = "Control_Time" if "Control_Time" in control_df.columns else None
    if control_time_col is None and "Time" in control_df.columns:
        control_time_col = "Time"
    if control_time_col is None:
        return None

    control_time = pd.to_numeric(control_df[control_time_col], errors="coerce")
    step_time = pd.to_numeric(step_df["time_s"], errors="coerce")

    valid = np.isfinite(control_time.to_numpy()) & np.isfinite(values.to_numpy())
    if int(np.sum(valid)) < 2:
        return None

    ct = control_time.to_numpy()[valid]
    cv = values.to_numpy()[valid]
    order = np.argsort(ct)
    ct = ct[order]
    cv = cv[order]

    uniq_t, uniq_idx = np.unique(ct, return_index=True)
    if uniq_t.size < 2:
        return None
    uniq_v = cv[uniq_idx]

    st = step_time.to_numpy(dtype=float)
    interp_vals = np.interp(st, uniq_t, uniq_v, left=np.nan, right=np.nan)
    outside = (st < uniq_t[0]) | (st > uniq_t[-1]) | ~np.isfinite(st)
    interp_vals[outside] = np.nan
    return pd.Series(interp_vals)


def _align_control_bool_series(
    step_df: pd.DataFrame, control_df: pd.DataFrame, column: str
) -> pd.Series | None:
    if column not in control_df.columns:
        return None
    raw = control_df[column]
    mapped = (
        raw.astype(str)
        .str.strip()
        .str.lower()
        .map(
            {
                "1": 1,
                "true": 1,
                "yes": 1,
                "y": 1,
                "on": 1,
                "0": 0,
                "false": 0,
                "no": 0,
                "n": 0,
                "off": 0,
            }
        )
    )
    numeric = pd.to_numeric(mapped.fillna(0), errors="coerce").fillna(0).astype(int)
    if len(numeric) == len(step_df):
        return numeric.reset_index(drop=True)
    return None


def _apply_control_fallbacks(
    *,
    step_df: pd.DataFrame,
    control_df: pd.DataFrame | None,
) -> pd.DataFrame:
    if control_df is None or control_df.empty:
        return step_df

    fallback_series = _align_control_bool_series(
        step_df, control_df, "MPC_Fallback_Used"
    )
    if fallback_series is not None:
        step_df["fallback_flag"] = fallback_series.astype(int)

    # Rebuild angular error when mpc_step_stats contains only zeros.
    if not _series_has_signal(step_df["ang_error_deg"]):
        ang_error_candidate: pd.Series | None = _align_control_series(
            step_df, control_df, "Error_Angle_Rad"
        )
        if ang_error_candidate is not None:
            ang_error_candidate = np.degrees(
                pd.to_numeric(ang_error_candidate, errors="coerce").abs()
            )
        else:
            er = _align_control_series(step_df, control_df, "Error_Roll")
            ep = _align_control_series(step_df, control_df, "Error_Pitch")
            ey = _align_control_series(step_df, control_df, "Error_Yaw")
            if er is not None and ep is not None and ey is not None:
                ang_error_candidate = np.degrees(
                    np.sqrt(
                        pd.to_numeric(er, errors="coerce") ** 2
                        + pd.to_numeric(ep, errors="coerce") ** 2
                        + pd.to_numeric(ey, errors="coerce") ** 2
                    )
                )
            else:
                ang_error_candidate = None

        if ang_error_candidate is not None and _series_has_signal(ang_error_candidate):
            step_df["ang_error_deg"] = pd.to_numeric(
                ang_error_candidate, errors="coerce"
            )

    # Rebuild angular velocity error from control-data components.
    ewx = _align_control_series(step_df, control_df, "Error_WX")
    ewy = _align_control_series(step_df, control_df, "Error_WY")
    ewz = _align_control_series(step_df, control_df, "Error_WZ")
    if ewx is not None and ewy is not None and ewz is not None:
        # Error_W* columns are logged in deg/s in this pipeline.
        ang_vel_candidate = np.sqrt(
            pd.to_numeric(ewx, errors="coerce") ** 2
            + pd.to_numeric(ewy, errors="coerce") ** 2
            + pd.to_numeric(ewz, errors="coerce") ** 2
        )
        existing = pd.to_numeric(step_df["ang_vel_error_degps"], errors="coerce")
        cand_finite = np.isfinite(ang_vel_candidate.to_numpy(dtype=float))
        ex_finite = np.isfinite(existing.to_numpy(dtype=float))
        replace = False
        if np.any(cand_finite):
            if not np.any(ex_finite):
                replace = True
            else:
                ex_q95 = float(
                    np.nanquantile(existing.to_numpy(dtype=float)[ex_finite], 0.95)
                )
                cand_q95 = float(
                    np.nanquantile(
                        ang_vel_candidate.to_numpy(dtype=float)[cand_finite], 0.95
                    )
                )
                # Existing series in mpc_step_stats may be inflated by unit mismatch.
                replace = ex_q95 > max(200.0, 3.0 * max(cand_q95, 1e-6))
            if replace:
                step_df["ang_vel_error_degps"] = pd.to_numeric(
                    ang_vel_candidate, errors="coerce"
                )

    # If path_error is flat zero (missing telemetry), fall back to position error.
    if not _series_has_signal(step_df["path_error_m"]) and _series_has_signal(
        step_df["pos_error_m"]
    ):
        step_df["path_error_m"] = pd.to_numeric(step_df["pos_error_m"], errors="coerce")

    return step_df


def discover_run_candidates(runs_root: Path) -> list[RunCandidate]:
    if not runs_root.exists():
        return []

    out: list[RunCandidate] = []
    for run_dir in sorted(
        (path for path in runs_root.iterdir() if path.is_dir()),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    ):
        # Skip aggregate/non-run directories at the root.
        if run_dir.name.lower() in {"comparisons", "comparison"}:
            continue
        status = _load_json(_status_path(run_dir))
        kpi = _load_json(_kpi_path(run_dir))
        if not status:
            continue

        mission, controller_profile, run_status = _extract_run_identity(run_dir, status)
        final_time_s = float(kpi.get("final_time_s") or 0.0)

        out.append(
            RunCandidate(
                run_dir=run_dir,
                run_id=run_dir.name,
                mission=mission,
                controller_profile=controller_profile,
                status=run_status,
                final_time_s=final_time_s,
            )
        )
    return out


def _normalize_steps_df(df: pd.DataFrame, kpi: dict[str, Any]) -> pd.DataFrame:
    mapping = {
        "Control_Time_s": "time_s",
        "Pos_Error_m": "pos_error_m",
        "Ang_Error_deg": "ang_error_deg",
        "Velocity_Error_mps": "vel_error_mps",
        "Angular_Velocity_Error_degps": "ang_vel_error_degps",
        "Linear_Speed_mps": "linear_speed_mps",
        "Angular_Rate_radps": "angular_rate_radps",
        "Solve_Time_ms": "solve_time_ms",
        "Timing_Violation": "timing_violation",
        "MPC_Time_Limit_Exceeded": "time_limit_exceeded",
        "Path_S_m": "path_s_m",
        "Path_Progress": "path_progress",
        "Path_Error_m": "path_error_m",
        "Path_Remaining_m": "path_remaining_m",
    }
    out = df.rename(columns=mapping).copy()

    required = list(mapping.values())
    for col in required:
        if col not in out.columns:
            out[col] = np.nan

    for col in required:
        out[col] = pd.to_numeric(out[col], errors="coerce")

    out["time_s"] = out["time_s"].ffill()
    out["time_s"] = out["time_s"].fillna(0.0)
    out = out.sort_values("time_s").reset_index(drop=True)

    out["timing_violation"] = out["timing_violation"].fillna(0).astype(int)
    out["time_limit_exceeded"] = out["time_limit_exceeded"].fillna(0).astype(int)

    out["angular_rate_degps"] = out["angular_rate_radps"] * (180.0 / np.pi)
    out["solve_time_ms_smooth"] = (
        out["solve_time_ms"].rolling(window=5, min_periods=1).median()
    )
    path_length_hint = float(kpi.get("path_length_m") or 0.0)
    if path_length_hint <= 0.0:
        s_plus_rem = pd.to_numeric(out["path_s_m"], errors="coerce") + pd.to_numeric(
            out["path_remaining_m"], errors="coerce"
        )
        s_plus_rem = s_plus_rem[np.isfinite(s_plus_rem.to_numpy(dtype=float))]
        if not s_plus_rem.empty:
            path_length_hint = float(np.nanmedian(s_plus_rem.to_numpy(dtype=float)))

    path_s_axis = pd.to_numeric(out["path_s_m"], errors="coerce")
    if not _series_has_signal(path_s_axis) and path_length_hint > 0.0:
        path_s_axis = path_length_hint - pd.to_numeric(
            out["path_remaining_m"], errors="coerce"
        )

    path_s_axis = pd.to_numeric(path_s_axis, errors="coerce")
    path_s_axis = (
        path_s_axis.where(np.isfinite(path_s_axis), np.nan).ffill().fillna(0.0)
    )
    path_s_axis = path_s_axis.clip(lower=0.0)
    out["path_s_plot"] = np.maximum.accumulate(path_s_axis.to_numpy(dtype=float))

    fallback_flag = 1 if float(kpi.get("solver_fallback_count", 0.0) or 0.0) > 0 else 0
    out["fallback_flag"] = fallback_flag
    return out


def _load_run_bundle(run_dir: Path, warnings: list[str]) -> RunBundle | None:
    steps_path = _steps_path(run_dir)
    kpi_path = _kpi_path(run_dir)
    status_path = _status_path(run_dir)

    if not steps_path.exists():
        warnings.append(f"{run_dir.name}: missing mpc_step_stats.csv")
        return None
    if not kpi_path.exists():
        warnings.append(f"{run_dir.name}: missing kpi_summary.json")
        return None

    try:
        step_df = pd.read_csv(steps_path)
    except Exception as exc:
        warnings.append(f"{run_dir.name}: failed to read mpc_step_stats.csv ({exc})")
        return None

    kpi = _load_json(kpi_path)
    status = _load_json(status_path)

    mission, controller_profile, run_status = _extract_run_identity(run_dir, status)

    step_df = _normalize_steps_df(step_df, kpi)

    control_df: pd.DataFrame | None = None
    control_path = _control_path(run_dir)
    if control_path.exists():
        try:
            control_df = pd.read_csv(control_path)
        except Exception:
            control_df = None

    step_df = _apply_control_fallbacks(step_df=step_df, control_df=control_df)

    return RunBundle(
        run_dir=run_dir,
        run_id=run_dir.name,
        mission=mission,
        controller_profile=controller_profile,
        status=run_status,
        step_df=step_df,
        kpi=kpi,
    )


def _palette(count: int) -> list[Any]:
    cmap = plt.get_cmap("tab10")
    return [cmap(i % 10) for i in range(count)]


def _plot_overlay(
    bundles: list[RunBundle],
    *,
    x_col: str,
    xlabel: str,
    y_col: str,
    ylabel: str,
    title: str,
    filename: str,
    plots_dir: Path,
    mark_final: bool = False,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 6.5), constrained_layout=True)
    colors = _palette(len(bundles))

    for idx, bundle in enumerate(bundles):
        df = bundle.step_df
        if y_col not in df.columns or x_col not in df.columns:
            continue
        x = pd.to_numeric(df[x_col], errors="coerce").to_numpy()
        y = pd.to_numeric(df[y_col], errors="coerce").to_numpy()

        valid = np.isfinite(x) & np.isfinite(y)
        if not np.any(valid):
            continue

        ax.plot(
            x[valid],
            y[valid],
            color=colors[idx],
            linewidth=1.8,
            alpha=0.88,
            label=bundle.legend_label,
        )
        if mark_final:
            ax.scatter(x[valid][-1], y[valid][-1], color=colors[idx], s=26, zorder=3)

    # Keep plots readable when a few rows contain extreme spikes.
    all_vals: list[np.ndarray] = []
    for bundle in bundles:
        arr = pd.to_numeric(bundle.step_df.get(y_col), errors="coerce").to_numpy()
        finite = arr[np.isfinite(arr)]
        if finite.size > 0:
            all_vals.append(finite)
    if all_vals and y_col in {"ang_vel_error_degps", "solve_time_ms_smooth"}:
        merged = np.concatenate(all_vals)
        hi = float(np.nanquantile(merged, 0.995))
        lo = float(np.nanmin(merged))
        if np.isfinite(hi) and hi > lo:
            ax.set_ylim(bottom=max(0.0, lo), top=hi * 1.05)

    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.savefig(plots_dir / filename, dpi=PLOT_DPI)
    plt.close(fig)


def _plot_event_raster(
    bundles: list[RunBundle],
    *,
    event_col: str,
    title: str,
    filename: str,
    plots_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 6.2), constrained_layout=True)
    colors = _palette(len(bundles))

    yticks: list[int] = []
    ylabels: list[str] = []

    total_events = 0
    max_time_s = 0.0

    for idx, bundle in enumerate(bundles):
        y_level = idx + 1
        yticks.append(y_level)
        ylabels.append(bundle.legend_label)

        df = bundle.step_df
        run_max_t = pd.to_numeric(df["time_s"], errors="coerce").max()
        if pd.notna(run_max_t):
            max_time_s = max(max_time_s, float(run_max_t))

        events = df[df[event_col] > 0]
        if events.empty:
            continue

        x = pd.to_numeric(events["time_s"], errors="coerce").to_numpy()
        x = x[np.isfinite(x)]
        if x.size == 0:
            continue

        total_events += int(x.size)
        ax.vlines(
            x=x,
            ymin=y_level - 0.35,
            ymax=y_level + 0.35,
            color=colors[idx],
            linewidth=1.4,
            alpha=0.9,
        )
        ax.scatter(x, np.full_like(x, y_level, dtype=float), color=colors[idx], s=20)

    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Run")
    ax.set_yticks(yticks)
    ax.set_yticklabels(ylabels, fontsize=8)
    ax.grid(True, axis="x", alpha=0.25)
    if max_time_s > 0:
        ax.set_xlim(0.0, max_time_s * 1.02)
    if total_events == 0:
        ax.text(
            0.5,
            0.55,
            "No events recorded",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=11,
            alpha=0.75,
        )
    fig.savefig(plots_dir / filename, dpi=PLOT_DPI)
    plt.close(fig)


def _plot_summary_bars(
    summary_df: pd.DataFrame,
    *,
    metric: str,
    ylabel: str,
    title: str,
    filename: str,
    plots_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 6.2), constrained_layout=True)
    colors = _palette(len(summary_df))

    values = pd.to_numeric(summary_df[metric], errors="coerce").fillna(0.0).to_numpy()
    labels = (
        summary_df["controller_profile"]
        .map(_display_controller_name)
        .astype(str)
        .tolist()
    )

    ax.bar(np.arange(len(values)), values, color=colors, alpha=0.9)
    ax.set_xticks(np.arange(len(values)))
    ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.25)

    fig.savefig(plots_dir / filename, dpi=PLOT_DPI)
    plt.close(fig)


def _build_summary_rows(bundles: list[RunBundle]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for bundle in bundles:
        kpi = bundle.kpi
        step_df = bundle.step_df

        final_ang_from_series = float(
            pd.to_numeric(step_df["ang_error_deg"], errors="coerce")
            .dropna()
            .tail(1)
            .squeeze()
            if "ang_error_deg" in step_df.columns
            and not pd.to_numeric(step_df["ang_error_deg"], errors="coerce")
            .dropna()
            .empty
            else 0.0
        )
        final_ang_vel_from_series = float(
            pd.to_numeric(step_df["ang_vel_error_degps"], errors="coerce")
            .dropna()
            .tail(1)
            .squeeze()
            if "ang_vel_error_degps" in step_df.columns
            and not pd.to_numeric(step_df["ang_vel_error_degps"], errors="coerce")
            .dropna()
            .empty
            else 0.0
        )

        final_angle_error_deg = float(kpi.get("final_angle_error_deg") or 0.0)
        if abs(final_angle_error_deg) <= 1e-12 and abs(final_ang_from_series) > 1e-12:
            final_angle_error_deg = final_ang_from_series

        final_ang_vel_degps = float(
            kpi.get("final_angular_velocity_error_degps") or 0.0
        )
        if abs(final_ang_vel_degps) <= 1e-12 and abs(final_ang_vel_from_series) > 1e-12:
            final_ang_vel_degps = final_ang_vel_from_series

        rows.append(
            {
                "run_id": bundle.run_id,
                "mission": bundle.mission,
                "controller_profile": bundle.controller_profile,
                "status": bundle.status,
                "final_time_s": float(kpi.get("final_time_s") or 0.0),
                "final_position_error_m": float(
                    kpi.get("final_position_error_m") or 0.0
                ),
                "final_angle_error_deg": final_angle_error_deg,
                "final_velocity_error_mps": float(
                    kpi.get("final_velocity_error_mps") or 0.0
                ),
                "final_angular_velocity_error_degps": final_ang_vel_degps,
                "mpc_mean_solve_time_ms": float(
                    kpi.get("mpc_mean_solve_time_ms") or 0.0
                ),
                "mpc_max_solve_time_ms": float(kpi.get("mpc_max_solve_time_ms") or 0.0),
                "solver_fallback_count": int(kpi.get("solver_fallback_count") or 0),
                "solver_hard_limit_breaches": int(
                    kpi.get("solver_hard_limit_breaches") or 0
                ),
                "path_completed": bool(kpi.get("path_completed", False)),
            }
        )
    return rows


def _write_summary_markdown(summary_df: pd.DataFrame, out_path: Path) -> None:
    cols = [
        "run_id",
        "mission",
        "controller_profile",
        "status",
        "final_time_s",
        "final_position_error_m",
        "final_angle_error_deg",
        "final_velocity_error_mps",
        "final_angular_velocity_error_degps",
        "mpc_mean_solve_time_ms",
        "mpc_max_solve_time_ms",
        "solver_fallback_count",
        "solver_hard_limit_breaches",
        "path_completed",
    ]

    lines = [
        "# Simulation Comparison Summary",
        "",
        f"Generated: {datetime.now(UTC).isoformat()}",
        "",
        "| " + " | ".join(cols) + " |",
        "|" + "|".join(["---"] * len(cols)) + "|",
    ]

    for _, row in summary_df.iterrows():
        values = [str(row.get(col, "")) for col in cols]
        lines.append("| " + " | ".join(values) + " |")

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _generate_plots(
    bundles: list[RunBundle],
    summary_df: pd.DataFrame,
    plots_time_dir: Path,
    plots_path_s_dir: Path,
) -> None:
    plots_time_dir.mkdir(parents=True, exist_ok=True)
    plots_path_s_dir.mkdir(parents=True, exist_ok=True)

    # Time-domain plots (existing set).
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="pos_error_m",
        ylabel="Position Error (m)",
        title="Position Error vs Time",
        filename="01_position_error_vs_time.png",
        plots_dir=plots_time_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="ang_error_deg",
        ylabel="Angular Error (deg)",
        title="Angular Error vs Time",
        filename="02_angular_error_vs_time.png",
        plots_dir=plots_time_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="vel_error_mps",
        ylabel="Velocity Error (m/s)",
        title="Velocity Error vs Time",
        filename="03_velocity_error_vs_time.png",
        plots_dir=plots_time_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="ang_vel_error_degps",
        ylabel="Angular Velocity Error (deg/s)",
        title="Angular Velocity Error vs Time",
        filename="04_angular_velocity_error_vs_time.png",
        plots_dir=plots_time_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="linear_speed_mps",
        ylabel="Linear Speed (m/s)",
        title="Linear Speed vs Time",
        filename="05_linear_speed_vs_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="angular_rate_degps",
        ylabel="Angular Rate (deg/s)",
        title="Angular Rate vs Time",
        filename="06_angular_rate_vs_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="path_error_m",
        ylabel="Path Error (m)",
        title="Path Error vs Time",
        filename="07_path_error_vs_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="path_remaining_m",
        ylabel="Path Remaining (m)",
        title="Path Remaining vs Time",
        filename="08_path_remaining_vs_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_overlay(
        bundles,
        x_col="time_s",
        xlabel="Time (s)",
        y_col="solve_time_ms_smooth",
        ylabel="MPC Solve Time (ms)",
        title="MPC Solve Time vs Time (Rolling Median)",
        filename="09_mpc_solve_time_vs_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_event_raster(
        bundles,
        event_col="timing_violation",
        title="Control Timing Violation Events vs Time",
        filename="10_timing_violation_events_vs_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_event_raster(
        bundles,
        event_col="time_limit_exceeded",
        title="Solver Time-Limit Exceeded Events vs Time",
        filename="11_solver_time_limit_exceeded_events_vs_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_summary_bars(
        summary_df,
        metric="final_position_error_m",
        ylabel="m",
        title="Final Position Error by Run",
        filename="12_kpi_summary_bar_final_position_error.png",
        plots_dir=plots_time_dir,
    )
    _plot_summary_bars(
        summary_df,
        metric="final_angle_error_deg",
        ylabel="deg",
        title="Final Angular Error by Run",
        filename="13_kpi_summary_bar_final_angle_error.png",
        plots_dir=plots_time_dir,
    )
    _plot_summary_bars(
        summary_df,
        metric="mpc_mean_solve_time_ms",
        ylabel="ms",
        title="MPC Mean Solve Time by Run",
        filename="14_kpi_summary_bar_mean_solve_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_summary_bars(
        summary_df,
        metric="mpc_max_solve_time_ms",
        ylabel="ms",
        title="MPC Max Solve Time by Run",
        filename="15_kpi_summary_bar_max_solve_time.png",
        plots_dir=plots_time_dir,
    )
    _plot_summary_bars(
        summary_df,
        metric="solver_fallback_count",
        ylabel="count",
        title="Solver Fallback Count by Run",
        filename="16_kpi_summary_bar_fallback_count.png",
        plots_dir=plots_time_dir,
    )

    # Path-distance-domain plots.
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="pos_error_m",
        ylabel="Position Error (m)",
        title="Position Error vs Path Distance",
        filename="01_position_error_vs_path_s.png",
        plots_dir=plots_path_s_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="ang_error_deg",
        ylabel="Angular Error (deg)",
        title="Angular Error vs Path Distance",
        filename="02_angular_error_vs_path_s.png",
        plots_dir=plots_path_s_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="vel_error_mps",
        ylabel="Velocity Error (m/s)",
        title="Velocity Error vs Path Distance",
        filename="03_velocity_error_vs_path_s.png",
        plots_dir=plots_path_s_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="ang_vel_error_degps",
        ylabel="Angular Velocity Error (deg/s)",
        title="Angular Velocity Error vs Path Distance",
        filename="04_angular_velocity_error_vs_path_s.png",
        plots_dir=plots_path_s_dir,
        mark_final=True,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="linear_speed_mps",
        ylabel="Linear Speed (m/s)",
        title="Linear Speed vs Path Distance",
        filename="05_linear_speed_vs_path_s.png",
        plots_dir=plots_path_s_dir,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="angular_rate_degps",
        ylabel="Angular Rate (deg/s)",
        title="Angular Rate vs Path Distance",
        filename="06_angular_rate_vs_path_s.png",
        plots_dir=plots_path_s_dir,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="path_error_m",
        ylabel="Path Error (m)",
        title="Path Error vs Path Distance",
        filename="07_path_error_vs_path_s.png",
        plots_dir=plots_path_s_dir,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="path_remaining_m",
        ylabel="Path Remaining (m)",
        title="Path Remaining vs Path Distance",
        filename="08_path_remaining_vs_path_s.png",
        plots_dir=plots_path_s_dir,
    )
    _plot_overlay(
        bundles,
        x_col="path_s_plot",
        xlabel="Path Distance s (m)",
        y_col="solve_time_ms_smooth",
        ylabel="MPC Solve Time (ms)",
        title="MPC Solve Time vs Path Distance (Rolling Median)",
        filename="09_mpc_solve_time_vs_path_s.png",
        plots_dir=plots_path_s_dir,
    )


def _candidate_label(candidate: RunCandidate) -> str:
    return (
        f"{candidate.run_id} | {candidate.controller_profile} | "
        f"{candidate.mission} | {candidate.status} | t={candidate.final_time_s:.2f}s"
    )


def _prompt_select_run_dirs(
    candidates: list[RunCandidate], max_runs: int
) -> list[Path]:
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise RuntimeError(
            "Interactive selection requires a TTY. Use --run or --latest."
        )

    try:
        import questionary

        selected: list[RunCandidate] = []
        remaining = list(candidates)

        while True:
            if len(selected) >= max_runs:
                return [item.run_dir for item in selected]

            choices = [
                questionary.Choice(title=_candidate_label(item), value=item)
                for item in remaining
            ]
            choices.append(questionary.Separator())
            choices.append(
                questionary.Choice(
                    title=f"Done ({len(selected)} selected) - run comparison",
                    value="__done__",
                )
            )
            choices.append(questionary.Choice(title="Cancel", value="__cancel__"))

            picked = questionary.select(
                f"Select simulation runs to compare (selected {len(selected)}/{max_runs}):",
                choices=choices,
                qmark="",
            ).ask()

            if picked in (None, "__cancel__"):
                return []

            if picked == "__done__":
                if len(selected) < 2:
                    print("Select at least 2 runs before finishing.")
                    continue
                return [item.run_dir for item in selected]

            if not isinstance(picked, RunCandidate):
                print("Invalid selection.")
                continue

            selected.append(picked)
            remaining = [item for item in remaining if item.run_dir != picked.run_dir]

            if not remaining and len(selected) >= 2:
                return [item.run_dir for item in selected]
    except ImportError:
        print("questionary unavailable; using numbered input.")

    selected: list[RunCandidate] = []
    remaining = list(candidates)
    while True:
        if len(selected) >= max_runs:
            return [item.run_dir for item in selected]
        if not remaining and len(selected) >= 2:
            return [item.run_dir for item in selected]

        print("")
        for idx, item in enumerate(remaining, start=1):
            print(f"{idx:2d}. {_candidate_label(item)}")
        print(" d. Done - run comparison")
        print(" c. Cancel")

        raw = (
            input(f"Select one run (selected {len(selected)}/{max_runs}): ")
            .strip()
            .lower()
        )

        if raw in {"", "c", "cancel", "q", "quit"}:
            return []
        if raw in {"d", "done", "finished", "finish"}:
            if len(selected) < 2:
                print("Select at least 2 runs before finishing.")
                continue
            return [item.run_dir for item in selected]
        if not raw.isdigit():
            print(f"Invalid selection: {raw}")
            continue

        idx = int(raw)
        if idx < 1 or idx > len(remaining):
            print(f"Selection out of range: {idx}")
            continue

        picked = remaining.pop(idx - 1)
        selected.append(picked)


def _resolve_run_arg(token: str, runs_root: Path) -> Path:
    raw = Path(token).expanduser()
    if raw.exists() and raw.is_dir():
        return raw.resolve()

    candidate = (runs_root / token).resolve()
    if candidate.exists() and candidate.is_dir():
        return candidate

    raise FileNotFoundError(f"Run not found: {token}")


def _select_run_dirs(args: argparse.Namespace, runs_root: Path) -> list[Path]:
    selected: list[Path] = []

    if args.run:
        for token in args.run:
            selected.append(_resolve_run_arg(token, runs_root))

    if args.latest is not None:
        candidates = discover_run_candidates(runs_root)
        latest_dirs = [item.run_dir for item in candidates[: args.latest]]
        selected.extend(latest_dirs)

    if not selected:
        candidates = discover_run_candidates(runs_root)
        if not candidates:
            raise RuntimeError(f"No simulation runs found in: {runs_root}")
        selected = _prompt_select_run_dirs(candidates, args.max_runs)

    dedup: list[Path] = []
    seen: set[Path] = set()
    for path in selected:
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        dedup.append(resolved)

    if len(dedup) < 2:
        raise RuntimeError("At least 2 runs are required for comparison.")
    if len(dedup) > args.max_runs:
        raise RuntimeError(
            f"Selected {len(dedup)} runs. Max allowed is {args.max_runs}."
        )
    return dedup


def _write_warnings(warnings: list[str], out_path: Path) -> None:
    if not warnings:
        return
    out_path.write_text("\n".join(warnings) + "\n", encoding="utf-8")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare up to 10 simulation runs with time-based plots."
    )
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        help="Run directory path or run id (repeatable)",
    )
    parser.add_argument(
        "--latest",
        type=int,
        default=None,
        help="Auto-select latest N runs from runs root",
    )
    parser.add_argument(
        "--runs-root",
        type=Path,
        default=SIMULATION_DATA_ROOT,
        help="Root directory containing simulation run folders",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=SIMULATION_DATA_ROOT / "comparisons",
        help="Output directory root for comparison report",
    )
    parser.add_argument("--title", default="Simulation Comparison Report")
    parser.add_argument("--max-runs", type=int, default=MAX_RUNS_DEFAULT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(sys.argv[1:] if argv is None else argv))
    if args.max_runs < 2:
        print("--max-runs must be >= 2")
        return 2

    runs_root = args.runs_root.resolve()
    out_root = args.output.resolve()

    try:
        selected_dirs = _select_run_dirs(args, runs_root)
    except Exception as exc:
        print(str(exc))
        return 2

    warnings: list[str] = []
    bundles: list[RunBundle] = []
    for run_dir in selected_dirs:
        bundle = _load_run_bundle(run_dir, warnings)
        if bundle is not None:
            bundles.append(bundle)

    if len(bundles) < 2:
        print("Need at least 2 valid runs after loading artifacts.")
        if warnings:
            for line in warnings:
                print(f"[WARN] {line}")
        return 2

    report_dir = out_root / f"{_now_utc_stamp()}_compare_{len(bundles)}runs"
    plots_time_dir = report_dir / "plots"
    plots_path_s_dir = report_dir / "plots_path_s"
    report_dir.mkdir(parents=True, exist_ok=True)

    summary_rows = _build_summary_rows(bundles)
    summary_df = pd.DataFrame(summary_rows)

    _generate_plots(
        bundles=bundles,
        summary_df=summary_df,
        plots_time_dir=plots_time_dir,
        plots_path_s_dir=plots_path_s_dir,
    )

    summary_csv_path = report_dir / "comparison_summary.csv"
    summary_md_path = report_dir / "comparison_summary.md"
    summary_df.to_csv(summary_csv_path, index=False)
    _write_summary_markdown(summary_df, summary_md_path)

    meta_payload = {
        "schema_version": "compare_report_v1",
        "generated_at": datetime.now(UTC).isoformat(),
        "title": str(args.title),
        "runs_root": str(runs_root),
        "selected_inputs": [str(path) for path in selected_dirs],
        "included_runs": [bundle.run_id for bundle in bundles],
        "excluded_count": len(selected_dirs) - len(bundles),
        "plot_count_time": 16,
        "plot_count_path_s": 9,
        "plot_count_total": 25,
        "output_dir": str(report_dir),
    }
    (report_dir / "comparison_meta.json").write_text(
        json.dumps(meta_payload, indent=2), encoding="utf-8"
    )

    _write_warnings(warnings, report_dir / "comparison_warnings.txt")

    print(f"Comparison report written: {report_dir}")
    if warnings:
        print(f"Warnings written: {report_dir / 'comparison_warnings.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

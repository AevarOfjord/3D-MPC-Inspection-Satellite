"""Diagnostics plotting helpers for PlotGenerator."""

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from utils.orientation_utils import quat_angle_error
from visualization.plot_data_utils import (
    get_control_time_axis,
    resolve_data_frame_and_columns,
)
from visualization.plot_style import PlotStyle


def generate_solver_health_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate solver health summary plot."""
    fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Solver Health - {plot_gen.system_title}")

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)

    if df is not None and "MPC_Status" in cols:
        status_vals = df["MPC_Status"].values
    else:
        status_vals = plot_gen._col("MPC_Status")

    status_counts = {}
    for val in status_vals:
        if val is None:
            continue
        label = str(val).strip()
        if label == "" or label.lower() == "nan":
            continue
        status_counts[label] = status_counts.get(label, 0) + 1

    if status_counts:
        labels = list(status_counts.keys())
        counts = [status_counts[k] for k in labels]
        axes[0].bar(labels, counts, color=PlotStyle.COLOR_SIGNAL_POS, alpha=0.8)
        axes[0].set_ylabel("Count", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].set_title("Solver Status Counts")
        axes[0].grid(True, axis="y", alpha=PlotStyle.GRID_ALPHA)
    else:
        axes[0].text(
            0.5,
            0.5,
            "Solver status data\nnot available",
            ha="center",
            va="center",
            transform=axes[0].transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )

    if df is not None and "MPC_Solve_Time" in cols:
        solve_times = df["MPC_Solve_Time"].values
    else:
        solve_times = plot_gen._col("MPC_Solve_Time")
        if len(solve_times) == 0:
            solve_times = plot_gen._col("MPC_Computation_Time")

    solve_ms = []
    for val in solve_times:
        try:
            num = float(val) * 1000.0
            if num > 0:
                solve_ms.append(num)
        except (ValueError, TypeError):
            continue
    solve_ms = np.array(solve_ms)

    if solve_ms.size > 0:
        axes[1].hist(solve_ms, bins=30, color=PlotStyle.COLOR_SIGNAL_ANG, alpha=0.8)
        axes[1].set_xlabel("Solve Time (ms)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_ylabel("Count", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_title("Solve Time Distribution")
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
    else:
        axes[1].text(
            0.5,
            0.5,
            "Solve time data\nnot available",
            ha="center",
            va="center",
            transform=axes[1].transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )

    PlotStyle.save_figure(fig, plot_dir / "solver_health.png")


def generate_waypoint_progress_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate waypoint/mission phase progress plot."""
    fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Waypoint Progress - {plot_gen.system_title}")

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)
    time = get_control_time_axis(
        df=df,
        cols=cols,
        fallback_len=plot_gen._get_len(),
        dt=float(plot_gen.dt),
    )

    if df is not None and "Waypoint_Number" in cols:
        waypoint_vals = df["Waypoint_Number"].values
    else:
        waypoint_vals = plot_gen._col("Waypoint_Number")

    if df is not None and "Mission_Phase" in cols:
        phase_vals = df["Mission_Phase"].values
    else:
        phase_vals = plot_gen._col("Mission_Phase")

    if len(waypoint_vals) == 0 or len(phase_vals) == 0:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "Waypoint/phase data\nnot available",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=PlotStyle.ANNOTATION_SIZE,
            )
        PlotStyle.save_figure(fig, plot_dir / "waypoint_progress.png")
        return

    min_len = min(len(time), len(waypoint_vals), len(phase_vals))
    time = time[:min_len]
    waypoint_vals = waypoint_vals[:min_len]
    phase_vals = phase_vals[:min_len]

    axes[0].step(time, waypoint_vals, where="post", color=PlotStyle.COLOR_SIGNAL_POS)
    axes[0].set_ylabel("Waypoint #", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)

    phase_order = []
    phase_codes = []
    for val in phase_vals:
        label = str(val)
        if label not in phase_order:
            phase_order.append(label)
        phase_codes.append(phase_order.index(label))

    axes[1].step(time, phase_codes, where="post", color=PlotStyle.COLOR_SIGNAL_ANG)
    axes[1].set_yticks(range(len(phase_order)))
    axes[1].set_yticklabels(phase_order)
    axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].set_ylabel("Mission Phase", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)

    PlotStyle.save_figure(fig, plot_dir / "waypoint_progress.png")


def generate_mpc_performance_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate MPC performance plot."""
    fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)

    if "MPC_Computation_Time" in cols or "MPC_Solve_Time" in cols:
        time = get_control_time_axis(
            df=df,
            cols=cols,
            fallback_len=plot_gen._get_len(),
            dt=float(plot_gen.dt),
        )

        if df is not None:
            raw_comp_times = df.get(
                "MPC_Solve_Time", df.get("MPC_Computation_Time", [])
            ).values
        else:
            raw_comp_times = (
                plot_gen._col("MPC_Solve_Time")
                if "MPC_Solve_Time" in cols
                else plot_gen._col("MPC_Computation_Time")
            )

        comp_times = []
        for val in raw_comp_times:
            try:
                comp_times.append(float(val) * 1000)
            except (ValueError, TypeError):
                comp_times.append(0.0)
        comp_times = np.array(comp_times)

        limit_ms = None
        if "MPC_Solver_Time_Limit" in cols:
            if df is not None:
                raw_limits = df["MPC_Solver_Time_Limit"].values
            else:
                raw_limits = plot_gen._col("MPC_Solver_Time_Limit")
            limits = []
            for val in raw_limits:
                try:
                    limits.append(float(val) * 1000)
                except (ValueError, TypeError):
                    limits.append(0.0)
            limit_ms = np.array(limits)

        ax.plot(
            time,
            comp_times,
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Computation Time",
        )
        mean_ms = float(np.mean(comp_times))
        max_ms = float(np.max(comp_times))
        ax.axhline(
            y=mean_ms,
            color=PlotStyle.COLOR_WARNING,
            linestyle="--",
            alpha=0.7,
            label=f"Mean: {mean_ms:.1f} ms",
        )
        ax.axhline(
            y=max_ms,
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linestyle=":",
            alpha=0.7,
            label=f"Max: {max_ms:.1f} ms",
        )
        if limit_ms is not None and np.any(limit_ms > 0):
            if len(np.unique(limit_ms)) > 1:
                ax.plot(
                    time,
                    limit_ms,
                    color=PlotStyle.COLOR_THRESHOLD,
                    linestyle=":",
                    alpha=0.6,
                    label="Time Limit",
                )
            else:
                limit_val = float(
                    limit_ms[0] if isinstance(limit_ms, np.ndarray) else limit_ms
                )
                if limit_val > 0:
                    ax.axhline(
                        y=limit_val,
                        color=PlotStyle.COLOR_THRESHOLD,
                        linestyle=":",
                        alpha=0.6,
                        label="Time Limit",
                    )
            if "MPC_Time_Limit_Exceeded" in cols:
                exceeded_vals = plot_gen._col("MPC_Time_Limit_Exceeded")
                exceeded_idx = []
                for i, val in enumerate(exceeded_vals):
                    try:
                        if bool(val) or (
                            isinstance(val, str) and val.lower() == "true"
                        ):
                            exceeded_idx.append(i)
                    except Exception:
                        pass
                exceeded_idx_arr = np.array(exceeded_idx)
                if len(exceeded_idx_arr) > 0:
                    ax.scatter(
                        np.array(time)[exceeded_idx_arr],
                        comp_times[exceeded_idx_arr],
                        color=PlotStyle.COLOR_ERROR,
                        s=25,
                        label="Exceeded",
                        zorder=5,
                    )
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Computation Time (ms)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title(f"MPC Computation Time - {plot_gen.system_title}")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
    else:
        ax.text(
            0.5,
            0.5,
            "MPC Computation Time\nData Not Available",
            ha="center",
            va="center",
            transform=ax.transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )
        ax.set_title(f"MPC Computation Time - {plot_gen.system_title}")

    PlotStyle.save_figure(fig, plot_dir / "mpc_performance.png")


def generate_timing_intervals_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate timing intervals plot."""
    fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)

    if "Actual_Time_Interval" in cols:
        time = get_control_time_axis(
            df=df,
            cols=cols,
            fallback_len=plot_gen._get_len(),
            dt=float(plot_gen.dt),
        )

        if df is not None:
            intervals = df["Actual_Time_Interval"].values
        else:
            intervals = plot_gen._col("Actual_Time_Interval")

        reference_dt = plot_gen.dt
        if df is not None and "CONTROL_DT" in cols:
            reference_dt = float(df["CONTROL_DT"].iloc[0])

        ax.plot(
            time,
            intervals,
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Actual Intervals",
        )
        ax.axhline(
            y=reference_dt,
            color=PlotStyle.COLOR_THRESHOLD,
            linestyle="--",
            alpha=0.7,
            label=f"Reference: {reference_dt:.3f}s",
        )
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Time Interval (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title(f"Timing Intervals - {plot_gen.system_title}")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
    else:
        ax.text(
            0.5,
            0.5,
            "Timing Interval\nData Not Available",
            ha="center",
            va="center",
            transform=ax.transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )
        ax.set_title(f"Timing Intervals - {plot_gen.system_title}")

    PlotStyle.save_figure(fig, plot_dir / "timing_intervals.png")


def generate_path_shaping_note_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Render compatibility panel for manual path shaping."""
    fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
    ax.text(
        0.5,
        0.5,
        "Manual path shaping active\n(no autonomous avoidance)",
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=PlotStyle.ANNOTATION_SIZE,
    )
    ax.set_title(f"Path Shaping Note - {plot_gen.system_title}")
    PlotStyle.save_figure(fig, plot_dir / "path_shaping_note.png")


def generate_solver_iterations_and_status_timeline_plot(
    plot_gen: Any, plot_dir: Path
) -> None:
    """Generate solver iterations/status timeline with limit and fallback markers."""
    fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
    fig.suptitle(f"Solver Iterations & Status Timeline - {plot_gen.system_title}")

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)
    time = get_control_time_axis(
        df=df,
        cols=cols,
        fallback_len=plot_gen._get_len(),
        dt=float(plot_gen.dt),
    )

    if len(time) == 0:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "Solver timeline data unavailable",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
        PlotStyle.save_figure(
            fig, plot_dir / "solver_iterations_and_status_timeline.png"
        )
        return

    def get_col(name: str) -> np.ndarray:
        if df is not None and name in cols:
            return np.array(df[name].values)
        return np.array(plot_gen._col(name))

    iterations_raw = (
        get_col("MPC_Iterations") if "MPC_Iterations" in cols else np.array([])
    )
    if len(iterations_raw) == 0:
        iterations = np.zeros(len(time), dtype=float)
    else:
        iterations = np.zeros(len(time), dtype=float)
        for i in range(min(len(time), len(iterations_raw))):
            try:
                iterations[i] = float(iterations_raw[i])
            except (TypeError, ValueError):
                iterations[i] = 0.0

    statuses = (
        get_col("MPC_Status")
        if "MPC_Status" in cols
        else np.array(["UNKNOWN"] * len(time))
    )
    status_labels = []
    status_codes = []
    for raw in statuses[: len(time)]:
        label = str(raw).strip() or "UNKNOWN"
        if label not in status_labels:
            status_labels.append(label)
        status_codes.append(status_labels.index(label))
    status_codes_arr = np.array(status_codes, dtype=float)

    limit_flags = np.zeros(len(time), dtype=float)
    fallback_flags = np.zeros(len(time), dtype=float)
    if "MPC_Time_Limit_Exceeded" in cols:
        raw = get_col("MPC_Time_Limit_Exceeded")
        for i in range(min(len(time), len(raw))):
            val = str(raw[i]).strip().lower()
            limit_flags[i] = 1.0 if val in {"1", "true", "yes", "y", "on"} else 0.0
    if "MPC_Fallback_Used" in cols:
        raw = get_col("MPC_Fallback_Used")
        for i in range(min(len(time), len(raw))):
            val = str(raw[i]).strip().lower()
            fallback_flags[i] = 1.0 if val in {"1", "true", "yes", "y", "on"} else 0.0

    axes[0].plot(
        time,
        iterations,
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH,
        label="Iterations",
    )
    axes[0].set_ylabel("Iterations", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[1].step(
        time[: len(status_codes_arr)],
        status_codes_arr,
        where="post",
        color=PlotStyle.COLOR_SIGNAL_ANG,
        linewidth=PlotStyle.LINEWIDTH,
    )
    axes[1].set_yticks(range(len(status_labels)))
    axes[1].set_yticklabels(status_labels)
    axes[1].set_ylabel("Status", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)

    axes[2].step(
        time,
        limit_flags,
        where="post",
        color=PlotStyle.COLOR_ERROR,
        linewidth=PlotStyle.LINEWIDTH,
        label="Time Limit Exceeded",
    )
    axes[2].step(
        time,
        fallback_flags,
        where="post",
        color=PlotStyle.COLOR_SIGNAL_ANG,
        linewidth=PlotStyle.LINEWIDTH,
        label="Fallback Used",
    )
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_yticks([0, 1])
    axes[2].set_ylabel("Flag", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "solver_iterations_and_status_timeline.png")


def generate_error_vs_solve_time_scatter_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate scatter plot of tracking error vs MPC solve time."""
    fig, axes = plt.subplots(1, 2, figsize=PlotStyle.FIGSIZE_WIDE)
    fig.suptitle(f"Error vs Solve Time - {plot_gen.system_title}")

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)
    if df is None:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "Control data required\nfor scatter plot",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
        PlotStyle.save_figure(fig, plot_dir / "error_vs_solve_time_scatter.png")
        return

    if "MPC_Solve_Time" in cols:
        solve_raw = np.array(df["MPC_Solve_Time"].values, dtype=object)
    elif "MPC_Computation_Time" in cols:
        solve_raw = np.array(df["MPC_Computation_Time"].values, dtype=object)
    else:
        solve_raw = np.array([], dtype=object)

    if solve_raw.size == 0:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "Solve-time column missing",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
        PlotStyle.save_figure(fig, plot_dir / "error_vs_solve_time_scatter.png")
        return

    solve_ms = np.zeros(len(solve_raw), dtype=float)
    for i, value in enumerate(solve_raw):
        try:
            solve_ms[i] = float(value) * 1000.0
        except (TypeError, ValueError):
            solve_ms[i] = np.nan

    def col_or_zeros(name: str) -> np.ndarray:
        if name not in cols:
            return np.zeros(len(solve_ms), dtype=float)
        arr = np.array(df[name].values, dtype=object)
        out = np.zeros(len(solve_ms), dtype=float)
        for i, value in enumerate(arr[: len(solve_ms)]):
            try:
                out[i] = float(value)
            except (TypeError, ValueError):
                out[i] = 0.0
        return out

    ex = col_or_zeros("Error_X")
    ey = col_or_zeros("Error_Y")
    ez = col_or_zeros("Error_Z")

    pos_err = np.sqrt(ex**2 + ey**2 + ez**2)
    has_q_cols = all(
        name in cols
        for name in (
            "Current_QW",
            "Current_QX",
            "Current_QY",
            "Current_QZ",
            "Reference_QW",
            "Reference_QX",
            "Reference_QY",
            "Reference_QZ",
        )
    )
    if has_q_cols:
        cq = np.column_stack(
            [
                col_or_zeros("Current_QW"),
                col_or_zeros("Current_QX"),
                col_or_zeros("Current_QY"),
                col_or_zeros("Current_QZ"),
            ]
        )
        rq = np.column_stack(
            [
                col_or_zeros("Reference_QW"),
                col_or_zeros("Reference_QX"),
                col_or_zeros("Reference_QY"),
                col_or_zeros("Reference_QZ"),
            ]
        )
        c_norm = np.linalg.norm(cq, axis=1, keepdims=True)
        r_norm = np.linalg.norm(rq, axis=1, keepdims=True)
        c_norm[c_norm <= 1e-12] = 1.0
        r_norm[r_norm <= 1e-12] = 1.0
        cq = cq / c_norm
        rq = rq / r_norm
        ang_err_deg = np.degrees(
            np.array([quat_angle_error(rq[i], cq[i]) for i in range(len(solve_ms))])
        )
    elif "Error_Angle_Rad" in cols:
        ang_err_deg = np.degrees(col_or_zeros("Error_Angle_Rad"))
    else:
        ang_err_deg = np.zeros(len(solve_ms), dtype=float)

    valid = np.isfinite(solve_ms)
    if np.any(valid):
        solve_ms = solve_ms[valid]
        pos_err = pos_err[valid]
        ang_err_deg = ang_err_deg[valid]
    else:
        solve_ms = np.array([])

    if solve_ms.size == 0:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "No valid solve-time samples",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
        PlotStyle.save_figure(fig, plot_dir / "error_vs_solve_time_scatter.png")
        return

    color_idx = np.arange(len(solve_ms))
    sc0 = axes[0].scatter(
        solve_ms,
        pos_err,
        c=color_idx,
        cmap="cividis",
        s=16,
        alpha=0.75,
    )
    axes[0].set_xlabel("Solve Time (ms)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].set_ylabel("Position Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)

    axes[1].scatter(
        solve_ms,
        ang_err_deg,
        c=color_idx,
        cmap="cividis",
        s=16,
        alpha=0.75,
    )
    axes[1].set_xlabel("Solve Time (ms)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].set_ylabel("Attitude Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)

    cbar = fig.colorbar(sc0, ax=axes, fraction=0.02, pad=0.02)
    cbar.set_label("Sample Index")

    PlotStyle.save_figure(fig, plot_dir / "error_vs_solve_time_scatter.png")

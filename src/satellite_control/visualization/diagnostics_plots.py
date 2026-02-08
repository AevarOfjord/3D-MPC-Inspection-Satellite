"""Diagnostics plotting helpers for PlotGenerator."""

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from satellite_control.visualization.plot_data_utils import (
    get_control_time_axis,
    resolve_data_frame_and_columns,
)
from satellite_control.visualization.plot_style import PlotStyle


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

    PlotStyle.save_figure(fig, plot_dir / "06_solver_health.png")


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
        PlotStyle.save_figure(fig, plot_dir / "01_waypoint_progress.png")
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

    PlotStyle.save_figure(fig, plot_dir / "01_waypoint_progress.png")


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
            color="purple",
            linewidth=PlotStyle.LINEWIDTH,
            label="Computation Time",
        )
        mean_ms = float(np.mean(comp_times))
        max_ms = float(np.max(comp_times))
        ax.axhline(
            y=mean_ms,
            color="r",
            linestyle="--",
            alpha=0.7,
            label=f"Mean: {mean_ms:.1f} ms",
        )
        ax.axhline(
            y=max_ms, color="g", linestyle=":", alpha=0.7, label=f"Max: {max_ms:.1f} ms"
        )
        if limit_ms is not None and np.any(limit_ms > 0):
            if len(np.unique(limit_ms)) > 1:
                ax.plot(
                    time,
                    limit_ms,
                    color="black",
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
                        color="black",
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
                        color="red",
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

    PlotStyle.save_figure(fig, plot_dir / "06_mpc_performance.png")


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
            color="orange",
            linewidth=PlotStyle.LINEWIDTH,
            label="Actual Intervals",
        )
        ax.axhline(
            y=reference_dt,
            color="r",
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

    PlotStyle.save_figure(fig, plot_dir / "06_timing_intervals.png")

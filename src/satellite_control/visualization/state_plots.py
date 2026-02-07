"""State and kinematics plotting helpers."""

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from src.satellite_control.visualization.plot_style import PlotStyle


def generate_constraint_violations_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate constraint violation plot."""
    fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Constraint Violations - {plot_gen.system_title}")

    time = np.arange(plot_gen._get_len()) * float(plot_gen.dt)

    pos_bound = 5.0
    max_vel = 1.0
    max_ang_vel = float(np.pi)

    x = plot_gen._col("Current_X")
    y = plot_gen._col("Current_Y")
    z = plot_gen._col("Current_Z")
    vx = plot_gen._col("Current_VX")
    vy = plot_gen._col("Current_VY")
    vz = plot_gen._col("Current_VZ")
    wx = plot_gen._col("Current_WX")
    wy = plot_gen._col("Current_WY")
    wz = plot_gen._col("Current_WZ")

    min_len = min(
        len(time),
        len(x),
        len(y),
        len(z),
        len(vx),
        len(vy),
        len(vz),
        len(wx),
        len(wy),
        len(wz),
    )
    if min_len == 0:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "Constraint data\nnot available",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=PlotStyle.ANNOTATION_SIZE,
            )
        PlotStyle.save_figure(fig, plot_dir / "06_constraint_violations.png")
        return

    pos_violation = np.maximum(
        np.max(np.abs(np.vstack([x[:min_len], y[:min_len], z[:min_len]])), axis=0)
        - pos_bound,
        0.0,
    )
    vel_mag = np.sqrt(vx[:min_len] ** 2 + vy[:min_len] ** 2 + vz[:min_len] ** 2)
    vel_violation = np.maximum(vel_mag - max_vel, 0.0)
    ang_vel_mag = np.sqrt(wx[:min_len] ** 2 + wy[:min_len] ** 2 + wz[:min_len] ** 2)
    ang_vel_violation = np.maximum(ang_vel_mag - max_ang_vel, 0.0)

    axes[0].plot(
        time[:min_len],
        pos_violation,
        color=PlotStyle.COLOR_ERROR,
        linewidth=PlotStyle.LINEWIDTH,
        label="Position Bound Violation",
    )
    axes[0].set_ylabel("Meters", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[1].plot(
        time[:min_len],
        vel_violation,
        color=PlotStyle.COLOR_ERROR,
        linewidth=PlotStyle.LINEWIDTH,
        label="Velocity Limit Violation",
    )
    axes[1].set_ylabel("m/s", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[2].plot(
        time[:min_len],
        ang_vel_violation,
        color=PlotStyle.COLOR_ERROR,
        linewidth=PlotStyle.LINEWIDTH,
        label="Angular Velocity Limit Violation",
    )
    axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].set_ylabel("rad/s", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "06_constraint_violations.png")


def generate_z_tilt_coupling_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate Z-tilt coupling plot."""
    fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Z Tilt Coupling - {plot_gen.system_title}")

    time = np.arange(plot_gen._get_len()) * float(plot_gen.dt)

    err_z = plot_gen._col("Error_Z")
    if len(err_z) == 0:
        current_z = plot_gen._col("Current_Z")
        reference_z = plot_gen._col("Reference_Z")
        min_len = min(len(current_z), len(reference_z))
        err_z = current_z[:min_len] - reference_z[:min_len]

    roll = np.degrees(plot_gen._col("Current_Roll"))
    pitch = np.degrees(plot_gen._col("Current_Pitch"))
    vz = plot_gen._col("Current_VZ")

    min_len = min(len(time), len(err_z), len(roll), len(pitch), len(vz))
    if min_len == 0:
        for ax in axes:
            ax.text(
                0.5,
                0.5,
                "Z tilt data\nnot available",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=PlotStyle.ANNOTATION_SIZE,
            )
        PlotStyle.save_figure(fig, plot_dir / "04_z_tilt_coupling.png")
        return

    axes[0].plot(
        time[:min_len],
        err_z[:min_len],
        color=PlotStyle.COLOR_ERROR,
        linewidth=PlotStyle.LINEWIDTH,
        label="Z Error",
    )
    axes[0].set_ylabel("Z Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[1].plot(
        time[:min_len],
        roll[:min_len],
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH,
        label="Roll",
    )
    axes[1].plot(
        time[:min_len],
        pitch[:min_len],
        color=PlotStyle.COLOR_SIGNAL_ANG,
        linewidth=PlotStyle.LINEWIDTH,
        label="Pitch",
    )
    axes[1].set_ylabel("Angle (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[2].plot(
        time[:min_len],
        vz[:min_len],
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH,
        label="VZ",
    )
    axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].set_ylabel("Vertical Velocity (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "04_z_tilt_coupling.png")


def generate_phase_position_velocity_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate position vs velocity phase plots."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    fig.suptitle(f"Phase Plot: Position vs Velocity - {plot_gen.system_title}")

    x = plot_gen._col("Current_X")
    y = plot_gen._col("Current_Y")
    z = plot_gen._col("Current_Z")
    vx = plot_gen._col("Current_VX")
    vy = plot_gen._col("Current_VY")
    vz = plot_gen._col("Current_VZ")

    axes[0].plot(x, vx, color=PlotStyle.COLOR_SIGNAL_POS, linewidth=PlotStyle.LINEWIDTH)
    axes[0].set_xlabel("X (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].set_ylabel("VX (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)

    axes[1].plot(y, vy, color=PlotStyle.COLOR_SIGNAL_POS, linewidth=PlotStyle.LINEWIDTH)
    axes[1].set_xlabel("Y (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].set_ylabel("VY (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)

    axes[2].plot(z, vz, color=PlotStyle.COLOR_SIGNAL_POS, linewidth=PlotStyle.LINEWIDTH)
    axes[2].set_xlabel("Z (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].set_ylabel("VZ (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)

    PlotStyle.save_figure(fig, plot_dir / "05_phase_plot_pos_vel.png")


def generate_phase_attitude_rate_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate attitude vs rate phase plots."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    fig.suptitle(f"Phase Plot: Attitude vs Rates - {plot_gen.system_title}")

    roll = np.degrees(plot_gen._col("Current_Roll"))
    pitch = np.degrees(plot_gen._col("Current_Pitch"))
    yaw = np.degrees(plot_gen._col("Current_Yaw"))
    wx = np.degrees(plot_gen._col("Current_WX"))
    wy = np.degrees(plot_gen._col("Current_WY"))
    wz = np.degrees(plot_gen._col("Current_WZ"))

    axes[0].plot(
        roll, wx, color=PlotStyle.COLOR_SIGNAL_ANG, linewidth=PlotStyle.LINEWIDTH
    )
    axes[0].set_xlabel("Roll (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].set_ylabel("WX (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)

    axes[1].plot(
        pitch, wy, color=PlotStyle.COLOR_SIGNAL_ANG, linewidth=PlotStyle.LINEWIDTH
    )
    axes[1].set_xlabel("Pitch (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].set_ylabel("WY (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)

    axes[2].plot(yaw, wz, color=PlotStyle.COLOR_SIGNAL_ANG, linewidth=PlotStyle.LINEWIDTH)
    axes[2].set_xlabel("Yaw (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].set_ylabel("WZ (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)

    PlotStyle.save_figure(fig, plot_dir / "05_phase_plot_att_rate.png")


def generate_velocity_tracking_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate velocity tracking over time plot."""
    fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Velocity Tracking - {plot_gen.system_title}")

    time = np.arange(plot_gen._get_len()) * float(plot_gen.dt)

    def plot_velocity(ax, axis_label, current_col, reference_col):
        current_vals = plot_gen._col(current_col)
        min_len = min(len(time), len(current_vals))
        if min_len == 0:
            return
        ax.plot(
            time[:min_len],
            current_vals[:min_len],
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label=f"Current {axis_label}",
        )
        reference_vals = plot_gen._col(reference_col)
        if len(reference_vals) > 0:
            tgt_len = min(len(time), len(reference_vals))
            ax.plot(
                time[:tgt_len],
                reference_vals[:tgt_len],
                color=PlotStyle.COLOR_TARGET,
                linestyle="--",
                linewidth=PlotStyle.LINEWIDTH,
                label=f"Reference {axis_label}",
            )
        ax.set_ylabel(f"{axis_label} Velocity (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)

    plot_velocity(axes[0], "X", "Current_VX", "Reference_VX")
    plot_velocity(axes[1], "Y", "Current_VY", "Reference_VY")
    plot_velocity(axes[2], "Z", "Current_VZ", "Reference_VZ")
    axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "02_tracking_velocity.png")


def generate_velocity_magnitude_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate velocity magnitude over time plot (speed vs time)."""
    fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)

    n = plot_gen._get_len()
    if n < 2:
        ax.text(
            0.5,
            0.5,
            "Insufficient data for velocity magnitude plot",
            ha="center",
            va="center",
            transform=ax.transAxes,
        )
        PlotStyle.save_figure(fig, plot_dir / "04_velocity_magnitude.png")
        return

    time = np.arange(n) * float(plot_gen.dt)

    vx = plot_gen._col("Current_VX")
    vy = plot_gen._col("Current_VY")
    vz = plot_gen._col("Current_VZ")

    min_len = min(len(time), len(vx), len(vy), len(vz))
    if min_len == 0:
        return

    velocity_magnitude = np.sqrt(vx[:min_len] ** 2 + vy[:min_len] ** 2 + vz[:min_len] ** 2)

    ax.plot(
        time[:min_len],
        velocity_magnitude,
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH,
        label="Velocity Magnitude",
    )
    ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax.set_ylabel("Speed (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax.set_title(f"Velocity Magnitude Over Time - {plot_gen.system_title}")
    ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
    ax.legend(fontsize=PlotStyle.LEGEND_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "04_velocity_magnitude.png")

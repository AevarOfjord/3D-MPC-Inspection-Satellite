"""Actuator- and thruster-focused plotting helpers for PlotGenerator."""

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from satellite_control.visualization.command_utils import (
    get_thruster_count,
    parse_command_vector,
)
from satellite_control.visualization.plot_data_utils import (
    get_control_time_axis,
    get_series,
    has_valve_data,
    normalize_series,
    resolve_data_frame_and_columns,
)
from satellite_control.visualization.plot_style import PlotStyle


def generate_thruster_usage_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate thruster usage plot using actual valve states."""
    fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)

    thruster_count = get_thruster_count(plot_gen.data_accessor, plot_gen.app_config)
    thruster_ids = np.arange(1, thruster_count + 1)
    total_activation_time = np.zeros(thruster_count)
    data_source = "commanded"

    if has_valve_data(plot_gen.data_accessor):
        data_source = "actual valve"
        for i in range(thruster_count):
            col_name = f"Thruster_{i + 1}_Val"
            vals = plot_gen._col(col_name)
            try:
                vals = np.array([float(x) for x in vals])
            except (ValueError, TypeError):
                vals = np.zeros(len(vals), dtype=float)
            total_activation_time[i] = np.sum(vals) * float(plot_gen.dt)
    else:
        command_data = []
        for idx in range(plot_gen._get_len()):
            row = plot_gen._row(idx)
            cmd_vec = parse_command_vector(
                row["Command_Vector"], plot_gen.data_accessor, plot_gen.app_config
            )
            command_data.append(cmd_vec)
        command_matrix = np.array(command_data)
        if command_matrix.ndim == 2:
            if command_matrix.shape[1] >= thruster_count:
                command_matrix = command_matrix[:, :thruster_count]
            else:
                pad = np.zeros((command_matrix.shape[0], thruster_count))
                pad[:, : command_matrix.shape[1]] = command_matrix
                command_matrix = pad
        total_activation_time = np.sum(command_matrix, axis=0) * float(plot_gen.dt)

    bars = ax.bar(
        thruster_ids,
        total_activation_time,
        color=PlotStyle.COLOR_BARS,
        alpha=0.8,
        edgecolor="black",
        linewidth=1.2,
    )

    for bar in bars:
        height = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            height + 0.01,
            f"{height:.2f}s",
            ha="center",
            va="bottom",
            fontsize=PlotStyle.ANNOTATION_SIZE,
            fontfamily=(
                PlotStyle.FONT_FAMILY if hasattr(PlotStyle, "FONT_FAMILY") else "serif"
            ),
        )

    ax.set_xlabel("Thruster ID", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax.set_ylabel("Total Active Time (seconds)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax.set_title(
        f"Thruster Usage Summary - {plot_gen.system_title}",
        fontsize=PlotStyle.TITLE_SIZE,
        fontweight="bold",
    )
    ax.grid(True, axis="y", alpha=PlotStyle.GRID_ALPHA)
    ax.set_xticks(thruster_ids)

    total_thruster_seconds = float(np.sum(total_activation_time))
    ax.text(
        0.98,
        0.95,
        f"Total active time ({data_source}): {total_thruster_seconds:.2f}s",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=PlotStyle.ANNOTATION_SIZE,
        bbox=PlotStyle.TEXTBOX_STYLE,
    )

    PlotStyle.save_figure(fig, plot_dir / "03_thruster_usage.png")


def generate_thruster_valve_activity_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate detailed valve activity plot for each thruster (0.0 to 1.0)."""
    if not has_valve_data(plot_gen.data_accessor):
        return

    thruster_count = get_thruster_count(plot_gen.data_accessor, plot_gen.app_config)
    cols = 3 if thruster_count > 8 else 2
    rows = int(np.ceil(thruster_count / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(15, 4 * rows), sharex=True)
    axes = axes.flatten()

    time = np.arange(plot_gen._get_len()) * float(plot_gen.dt)

    for i in range(thruster_count):
        thruster_id = i + 1
        col_name_val = f"Thruster_{thruster_id}_Val"
        col_name_cmd = f"Thruster_{thruster_id}_Cmd"

        vals = plot_gen._col(col_name_val)
        try:
            vals = np.array([float(x) for x in vals])
        except (ValueError, TypeError):
            vals = np.zeros_like(vals, dtype=float)

        cmds = plot_gen._col(col_name_cmd)
        try:
            cmds = np.array([float(x) for x in cmds])
        except (ValueError, TypeError):
            cmds = np.zeros_like(vals, dtype=float)

        ax = axes[i]
        ax.fill_between(time, vals, color="tab:blue", alpha=0.3, label="Valve")
        ax.plot(time, vals, color="tab:blue", linewidth=1)
        ax.plot(
            time,
            cmds,
            color="red",
            linestyle="--",
            linewidth=1.0,
            label="Command",
        )

        ax.set_ylim(-0.1, 1.1)
        ax.set_yticks([0, 0.5, 1.0])
        ax.grid(True, alpha=0.3)
        ax.set_ylabel(f"T{thruster_id}")
        ax.set_title(f"Thruster {thruster_id}", fontsize=10, pad=2)

        if i // cols == rows - 1:
            ax.set_xlabel("Time (s)")

        if i == 0:
            ax.legend(loc="upper right", fontsize=8)

        fig_ind, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)

        ax_top.fill_between(time, vals, color="tab:blue", alpha=0.3)
        ax_top.plot(time, vals, color="tab:blue", linewidth=1)
        ax_top.set_ylim(-0.1, 1.1)
        ax_top.set_yticks([0, 1])
        ax_top.set_yticklabels(["OFF", "ON"])
        ax_top.grid(True, alpha=0.3)
        ax_top.set_ylabel("Valve State")
        ax_top.set_title(
            f"Thruster {thruster_id} - {plot_gen.system_title}", fontsize=12
        )

        ax_bot.plot(time, cmds, color="red", linewidth=1.5)
        ax_bot.fill_between(time, cmds, color="red", alpha=0.2)
        ax_bot.set_ylim(-0.05, 1.05)
        ax_bot.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
        ax_bot.grid(True, alpha=0.3)
        ax_bot.set_ylabel("Cmd Duty Cycle")
        ax_bot.set_xlabel("Time (s)")

        plt.tight_layout()
        plt.savefig(
            plot_dir / f"03_thruster_{thruster_id}_valve_activity.png",
            dpi=300,
            bbox_inches="tight",
        )
        plt.close(fig_ind)

    fig.suptitle(
        f"Thruster Valve Activity (0.0 - 1.0) - {plot_gen.system_title}",
        fontsize=16,
    )
    for idx in range(thruster_count, len(axes)):
        axes[idx].axis("off")
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    plt.savefig(
        plot_dir / "03_thruster_valve_activity.png",
        dpi=300,
        bbox_inches="tight",
    )
    plt.close()


def generate_pwm_quantization_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate PWM duty cycle plot showing MPC u-values vs time."""
    control_df = getattr(plot_gen.data_accessor, "control_data", None)

    if control_df is None or "Command_Vector" not in control_df.columns:
        return plot_gen.generate_pwm_duty_cycles_from_physics(plot_dir)

    time = (
        control_df["Control_Time"].values
        if "Control_Time" in control_df.columns
        else np.arange(len(control_df)) * float(plot_gen.dt)
    )

    thruster_count = get_thruster_count(plot_gen.data_accessor, plot_gen.app_config)
    duty_cycles_per_thruster: dict[int, list[float]] = {
        i: [] for i in range(thruster_count)
    }

    for cmd_str in control_df["Command_Vector"]:
        try:
            values = cmd_str.strip('[]"').split(",")
            for i in range(thruster_count):
                if i < len(values):
                    duty_cycles_per_thruster[i].append(float(values[i].strip()))
                else:
                    duty_cycles_per_thruster[i].append(0.0)
        except BaseException:
            for i in range(thruster_count):
                duty_cycles_per_thruster[i].append(0.0)

    cols = 3 if thruster_count > 8 else 2
    rows = int(np.ceil(thruster_count / cols))
    fig, axes = plt.subplots(
        rows, cols, figsize=(14, 3.5 * rows), sharex=True, sharey=True
    )
    axes = axes.flatten()
    fig.suptitle(
        f"PWM MPC Duty Cycle Output (u) vs Time - {plot_gen.system_title}",
        fontsize=14,
        fontweight="bold",
    )

    colors = plt.cm.tab20(np.linspace(0, 1, thruster_count))
    all_intermediate_values = []

    for i, ax in enumerate(axes):
        if i >= thruster_count:
            ax.axis("off")
            continue
        thruster_id = i + 1
        duty_cycles = np.array(duty_cycles_per_thruster[i])

        ax.step(
            time,
            duty_cycles,
            where="post",
            color=colors[i],
            linewidth=1.5,
            alpha=0.9,
        )
        ax.fill_between(time, 0, duty_cycles, step="post", color=colors[i], alpha=0.3)

        ax.axhline(y=0.0, color="gray", linestyle="-", alpha=0.3, linewidth=0.5)
        ax.axhline(y=0.5, color="gray", linestyle="--", alpha=0.3, linewidth=0.5)
        ax.axhline(y=1.0, color="gray", linestyle="-", alpha=0.3, linewidth=0.5)

        intermediate = [(t, u) for t, u in zip(time, duty_cycles) if 0.01 < u < 0.99]
        if intermediate:
            all_intermediate_values.extend([u for _, u in intermediate])
            for t_val, u_val in intermediate:
                ax.plot(t_val, u_val, "ko", markersize=4)

        ax.set_ylabel(f"T{thruster_id}", fontsize=10, fontweight="bold")
        ax.set_ylim(-0.05, 1.05)
        ax.set_yticks([0, 0.5, 1.0])
        ax.grid(True, alpha=0.3)

        intermediate_count = len(intermediate)
        if intermediate_count > 0:
            ax.text(
                0.98,
                0.95,
                f"Intermediate: {intermediate_count}",
                transform=ax.transAxes,
                fontsize=8,
                ha="right",
                va="top",
                bbox=dict(
                    facecolor="lightgreen",
                    alpha=0.7,
                    boxstyle="round,pad=0.2",
                ),
            )

    for i in range(thruster_count):
        if i // cols == rows - 1:
            axes[i].set_xlabel("Time (s)", fontsize=12)

    if all_intermediate_values:
        unique_values = sorted(set([round(v, 3) for v in all_intermediate_values]))
        n_instances = len(all_intermediate_values)
        summary = f"Intermediate u-values found: {n_instances} instances\n"
        ellipsis = "..." if len(unique_values) > 10 else ""
        summary += f"Unique values: {unique_values[:10]}{ellipsis}\n"
        summary += "✓ PWM MPC outputs continuous duty cycles [0, 1]"
        fig.text(
            0.5,
            0.01,
            summary,
            ha="center",
            fontsize=10,
            bbox=dict(facecolor="lightgreen", alpha=0.8, boxstyle="round,pad=0.5"),
        )
    else:
        fig.text(
            0.5,
            0.01,
            "No intermediate u-values found (all 0 or 1)\n"
            "MPC chose full thrust for this scenario",
            ha="center",
            fontsize=10,
            bbox=dict(
                facecolor="lightyellow",
                alpha=0.8,
                boxstyle="round,pad=0.5",
            ),
        )

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.1)
    plt.savefig(plot_dir / "03_pwm_duty_cycles.png", dpi=300, bbox_inches="tight")
    plt.close()


def generate_control_effort_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate control effort plot."""
    fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)

    time = np.arange(plot_gen._get_len()) * float(plot_gen.dt)

    command_data = []
    for idx in range(plot_gen._get_len()):
        row = plot_gen._row(idx)
        cmd_vec = parse_command_vector(
            row["Command_Vector"], plot_gen.data_accessor, plot_gen.app_config
        )
        command_data.append(cmd_vec)
    command_matrix = np.array(command_data)

    total_effort_per_step = np.sum(command_matrix, axis=1)
    ax.plot(
        time,
        total_effort_per_step,
        color="c",
        linewidth=PlotStyle.LINEWIDTH,
        label="Total Control Effort",
    )
    ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax.set_ylabel("Total Control Effort", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax.set_title(f"Control Effort Over Time - {plot_gen.system_title}")
    ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
    ax.legend(fontsize=PlotStyle.LEGEND_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "03_control_effort.png")


def generate_reaction_wheel_output_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate reaction wheel torque output plot."""
    fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Reaction Wheel Output - {plot_gen.system_title}")

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)
    time = get_control_time_axis(
        df=df,
        cols=cols,
        fallback_len=plot_gen._get_len(),
        dt=float(plot_gen.dt),
    )

    base_len = len(time)
    if base_len == 0:
        axes[0].text(
            0.5,
            0.5,
            "Reaction wheel data\nnot available",
            ha="center",
            va="center",
            transform=axes[0].transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )
        axes[0].set_title(f"Reaction Wheel Output - {plot_gen.system_title}")
        for ax in axes[1:]:
            ax.axis("off")
        PlotStyle.save_figure(fig, plot_dir / "03_reaction_wheel_output.png")
        return

    rw_x = normalize_series(get_series(plot_gen, "RW_Torque_X", df, cols), base_len)
    rw_y = normalize_series(get_series(plot_gen, "RW_Torque_Y", df, cols), base_len)
    rw_z = normalize_series(get_series(plot_gen, "RW_Torque_Z", df, cols), base_len)

    axes[0].plot(
        time,
        rw_x,
        color=PlotStyle.COLOR_SIGNAL_ANG,
        linewidth=PlotStyle.LINEWIDTH,
        label="RW X",
    )
    axes[0].set_ylabel("Torque X (N·m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[1].plot(
        time,
        rw_y,
        color=PlotStyle.COLOR_SIGNAL_ANG,
        linewidth=PlotStyle.LINEWIDTH,
        label="RW Y",
    )
    axes[1].set_ylabel("Torque Y (N·m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[2].plot(
        time,
        rw_z,
        color=PlotStyle.COLOR_SIGNAL_ANG,
        linewidth=PlotStyle.LINEWIDTH,
        label="RW Z",
    )
    axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].set_ylabel("Torque Z (N·m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "03_reaction_wheel_output.png")


def generate_actuator_limits_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate actuator outputs with limit overlays."""
    fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Actuator Limits - {plot_gen.system_title}")

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)
    time = get_control_time_axis(
        df=df,
        cols=cols,
        fallback_len=plot_gen._get_len(),
        dt=float(plot_gen.dt),
    )

    command_vectors = []
    if df is not None and "Command_Vector" in cols:
        for cmd_str in df["Command_Vector"].values:
            command_vectors.append(
                parse_command_vector(
                    cmd_str, plot_gen.data_accessor, plot_gen.app_config
                )
            )
    else:
        for idx in range(plot_gen._get_len()):
            row = plot_gen._row(idx)
            command_vectors.append(
                parse_command_vector(
                    row.get("Command_Vector"),
                    plot_gen.data_accessor,
                    plot_gen.app_config,
                )
            )
    command_matrix = np.array(command_vectors) if command_vectors else np.zeros((0, 0))

    if command_matrix.size > 0:
        max_u = np.max(command_matrix, axis=1)
        sum_u = np.sum(command_matrix, axis=1)
        min_len = min(len(time), len(max_u), len(sum_u))
        axes[0].plot(
            time[:min_len],
            max_u[:min_len],
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Max Thruster Command",
        )
        axes[0].plot(
            time[:min_len],
            sum_u[:min_len],
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linewidth=PlotStyle.LINEWIDTH,
            label="Sum Thruster Command",
        )
        axes[0].axhline(
            y=1.0, color="black", linestyle="--", alpha=0.6, label="Max Limit"
        )
        axes[0].set_ylabel("Command (0-1)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
    else:
        axes[0].text(
            0.5,
            0.5,
            "Thruster command data\nnot available",
            ha="center",
            va="center",
            transform=axes[0].transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )

    base_len = len(time)

    rw_x = normalize_series(get_series(plot_gen, "RW_Torque_X", df, cols), base_len)
    rw_y = normalize_series(get_series(plot_gen, "RW_Torque_Y", df, cols), base_len)
    rw_z = normalize_series(get_series(plot_gen, "RW_Torque_Z", df, cols), base_len)

    try:
        from satellite_control.config.reaction_wheel_config import (
            get_reaction_wheel_config,
        )

        max_rw = float(get_reaction_wheel_config().wheel_x.max_torque)
    except Exception:
        max_rw = 0.0

    if base_len > 0:
        axes[1].plot(
            time,
            rw_x,
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linewidth=PlotStyle.LINEWIDTH,
            label="RW X",
        )
        axes[1].plot(
            time,
            rw_y,
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="RW Y",
        )
        axes[1].plot(
            time,
            rw_z,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="RW Z",
        )
        if max_rw > 0:
            axes[1].axhline(y=max_rw, color="black", linestyle="--", alpha=0.6)
            axes[1].axhline(
                y=-max_rw,
                color="black",
                linestyle="--",
                alpha=0.6,
                label="RW Limit",
            )
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_ylabel("Torque (N*m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
    else:
        axes[1].text(
            0.5,
            0.5,
            "Reaction wheel data\nnot available",
            ha="center",
            va="center",
            transform=axes[1].transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "03_actuator_limits.png")


def generate_thruster_impulse_proxy_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate thruster impulse proxy plot."""
    fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
    fig.suptitle(f"Thruster Impulse Proxy - {plot_gen.system_title}")

    df, cols = resolve_data_frame_and_columns(plot_gen.data_accessor)
    time = get_control_time_axis(
        df=df,
        cols=cols,
        fallback_len=plot_gen._get_len(),
        dt=float(plot_gen.dt),
    )

    thruster_forces = {}
    thruster_dirs = {}
    try:
        if plot_gen.app_config and plot_gen.app_config.physics:
            thruster_forces = plot_gen.app_config.physics.thruster_forces
            thruster_dirs = plot_gen.app_config.physics.thruster_directions
        else:
            from satellite_control.config.simulation_config import SimulationConfig

            cfg = SimulationConfig.create_default().app_config.physics
            thruster_forces = cfg.thruster_forces
            thruster_dirs = cfg.thruster_directions
    except Exception:
        thruster_forces = {}
        thruster_dirs = {}

    thruster_ids = sorted(thruster_forces.keys())
    if not thruster_ids:
        axes[0].text(
            0.5,
            0.5,
            "Thruster configuration\nnot available",
            ha="center",
            va="center",
            transform=axes[0].transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )
        PlotStyle.save_figure(fig, plot_dir / "03_thruster_impulse_proxy.png")
        return

    force_matrix = []
    for tid in thruster_ids:
        direction = np.array(thruster_dirs[tid], dtype=float)
        force_matrix.append(float(thruster_forces[tid]) * direction)
    force_matrix = np.array(force_matrix)

    command_vectors = []
    if df is not None and "Command_Vector" in cols:
        for cmd_str in df["Command_Vector"].values:
            command_vectors.append(
                parse_command_vector(
                    cmd_str, plot_gen.data_accessor, plot_gen.app_config
                )
            )
    else:
        for idx in range(plot_gen._get_len()):
            row = plot_gen._row(idx)
            command_vectors.append(
                parse_command_vector(
                    row.get("Command_Vector"),
                    plot_gen.data_accessor,
                    plot_gen.app_config,
                )
            )
    if not command_vectors:
        axes[0].text(
            0.5,
            0.5,
            "Command data\nnot available",
            ha="center",
            va="center",
            transform=axes[0].transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
        )
        PlotStyle.save_figure(fig, plot_dir / "03_thruster_impulse_proxy.png")
        return

    commands = []
    for cmd in command_vectors:
        if len(cmd) < len(thruster_ids):
            padded = np.zeros(len(thruster_ids))
            padded[: len(cmd)] = cmd
            commands.append(padded)
        else:
            commands.append(cmd[: len(thruster_ids)])
    commands = np.array(commands)

    min_len = min(len(time), commands.shape[0])
    if min_len == 0:
        return

    net_forces = commands[:min_len] @ force_matrix
    force_mag = np.linalg.norm(net_forces, axis=1)

    dt_steps = np.diff(time[:min_len], prepend=time[0])
    if min_len > 1:
        fallback_dt = (
            float(np.median(dt_steps[1:]))
            if np.any(dt_steps[1:])
            else float(plot_gen.dt)
        )
    else:
        fallback_dt = float(plot_gen.dt)
    if dt_steps[0] == 0:
        dt_steps[0] = fallback_dt

    impulse = np.cumsum(force_mag * dt_steps)

    axes[0].plot(
        time[:min_len],
        net_forces[:, 0],
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH,
        label="Fx (body)",
    )
    axes[0].plot(
        time[:min_len],
        net_forces[:, 1],
        color=PlotStyle.COLOR_SIGNAL_ANG,
        linewidth=PlotStyle.LINEWIDTH,
        label="Fy (body)",
    )
    axes[0].plot(
        time[:min_len],
        net_forces[:, 2],
        color=PlotStyle.COLOR_ERROR,
        linewidth=PlotStyle.LINEWIDTH,
        label="Fz (body)",
    )
    axes[0].plot(
        time[:min_len],
        force_mag,
        color="black",
        linestyle="--",
        linewidth=PlotStyle.LINEWIDTH,
        label="|F|",
    )
    axes[0].set_ylabel("Force (N)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)

    axes[1].plot(
        time[:min_len],
        impulse,
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH,
        label="Cumulative Impulse",
    )
    axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].set_ylabel("Impulse (N*s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
    axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)

    PlotStyle.save_figure(fig, plot_dir / "03_thruster_impulse_proxy.png")

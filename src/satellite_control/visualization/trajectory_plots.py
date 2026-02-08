"""Trajectory plotting helpers for simulation analysis."""

from pathlib import Path
from typing import Any, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from satellite_control.visualization.plot_style import PlotStyle


def _reference_point(plot_gen: Any) -> Tuple[float, float, float]:
    reference_x_col = plot_gen._col("Reference_X")
    reference_y_col = plot_gen._col("Reference_Y")
    reference_z_col = plot_gen._col("Reference_Z")

    reference_x = float(reference_x_col[0]) if len(reference_x_col) > 0 else 0.0
    reference_y = float(reference_y_col[0]) if len(reference_y_col) > 0 else 0.0
    reference_z = float(reference_z_col[0]) if len(reference_z_col) > 0 else 0.0
    return reference_x, reference_y, reference_z


def generate_trajectory_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate 2D trajectory overview plots (XY and XZ)."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    x_pos = plot_gen._col("Current_X")
    y_pos = plot_gen._col("Current_Y")
    z_pos = plot_gen._col("Current_Z")
    reference_x, reference_y, reference_z = _reference_point(plot_gen)

    # Plot trajectory X-Y
    ax_xy = axes[0]
    ax_xy.plot(
        x_pos,
        y_pos,
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH_THICK,
        alpha=0.8,
        label="Satellite Path",
    )
    if len(x_pos) > 0:
        ax_xy.plot(
            x_pos[0],
            y_pos[0],
            "o",
            color=PlotStyle.COLOR_SUCCESS,
            markersize=PlotStyle.MARKER_SIZE,
            label="Start Position",
        )
        ax_xy.plot(
            x_pos[-1],
            y_pos[-1],
            "o",
            color=PlotStyle.COLOR_ERROR,
            markersize=PlotStyle.MARKER_SIZE,
            label="Final Position",
        )
    ax_xy.plot(reference_x, reference_y, "r*", markersize=20, label="Reference")

    circle = Circle(
        (reference_x, reference_y),
        0.1,
        color=PlotStyle.COLOR_TARGET,
        fill=False,
        linewidth=PlotStyle.LINEWIDTH,
        linestyle="--",
        alpha=0.7,
        label="Reference Zone (±0.1m)",
    )
    ax_xy.add_patch(circle)

    ax_xy.set_xlim(-3, 3)
    ax_xy.set_ylim(-3, 3)
    ax_xy.set_xlabel("X Position (meters)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax_xy.set_ylabel("Y Position (meters)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax_xy.set_title("Trajectory (X-Y)")
    ax_xy.grid(True, alpha=PlotStyle.GRID_ALPHA)
    ax_xy.legend(fontsize=PlotStyle.LEGEND_SIZE)
    ax_xy.set_aspect("equal")

    # Plot trajectory X-Z
    ax_xz = axes[1]
    ax_xz.plot(
        x_pos,
        z_pos,
        color=PlotStyle.COLOR_SIGNAL_POS,
        linewidth=PlotStyle.LINEWIDTH_THICK,
        alpha=0.8,
        label="Satellite Path",
    )
    if len(x_pos) > 0 and len(z_pos) > 0:
        ax_xz.plot(
            x_pos[0],
            z_pos[0],
            "o",
            color=PlotStyle.COLOR_SUCCESS,
            markersize=PlotStyle.MARKER_SIZE,
            label="Start Position",
        )
        ax_xz.plot(
            x_pos[-1],
            z_pos[-1],
            "o",
            color=PlotStyle.COLOR_ERROR,
            markersize=PlotStyle.MARKER_SIZE,
            label="Final Position",
        )
    ax_xz.plot(reference_x, reference_z, "r*", markersize=20, label="Reference")

    circle_xz = Circle(
        (reference_x, reference_z),
        0.1,
        color=PlotStyle.COLOR_TARGET,
        fill=False,
        linewidth=PlotStyle.LINEWIDTH,
        linestyle="--",
        alpha=0.7,
        label="Reference Zone (±0.1m)",
    )
    ax_xz.add_patch(circle_xz)

    ax_xz.set_xlim(-3, 3)
    ax_xz.set_ylim(-3, 3)
    ax_xz.set_xlabel("X Position (meters)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax_xz.set_ylabel("Z Position (meters)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
    ax_xz.set_title("Trajectory (X-Z)")
    ax_xz.grid(True, alpha=PlotStyle.GRID_ALPHA)
    ax_xz.legend(fontsize=PlotStyle.LEGEND_SIZE)
    ax_xz.set_aspect("equal")

    if len(x_pos) > 0 and len(y_pos) > 0 and len(z_pos) > 0:
        final_distance = np.sqrt(
            (x_pos[-1] - reference_x) ** 2
            + (y_pos[-1] - reference_y) ** 2
            + (z_pos[-1] - reference_z) ** 2
        )
    else:
        final_distance = 0.0
    ax_xy.text(
        0.02,
        0.98,
        f"Final Distance to Reference: {final_distance:.3f}m",
        transform=ax_xy.transAxes,
        fontsize=PlotStyle.ANNOTATION_SIZE,
        verticalalignment="top",
        bbox=PlotStyle.TEXTBOX_STYLE,
    )

    PlotStyle.save_figure(fig, plot_dir / "01_trajectory_2d.png")


def generate_trajectory_3d_interactive_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate interactive 3D trajectory plot (HTML)."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        import sys

        print(
            "Plotly not installed; skipping interactive 3D trajectory plot.",
            file=sys.stderr,
        )
        return

    x_pos = plot_gen._col("Current_X")
    y_pos = plot_gen._col("Current_Y")
    z_pos = plot_gen._col("Current_Z")
    if len(x_pos) == 0 or len(y_pos) == 0 or len(z_pos) == 0:
        print("No trajectory data available for interactive 3D plot.")
        return

    reference_x_col = plot_gen._col("Reference_X")
    reference_y_col = plot_gen._col("Reference_Y")
    reference_z_col = plot_gen._col("Reference_Z")
    reference_x, reference_y, reference_z = _reference_point(plot_gen)

    fig = go.Figure()
    if (
        len(reference_x_col) > 1
        and len(reference_y_col) > 1
        and len(reference_z_col) > 1
    ):
        target_x: List[float] = []
        target_y: List[float] = []
        target_z: List[float] = []
        last = None
        min_step = 0.02  # meters
        for rx, ry, rz in zip(reference_x_col, reference_y_col, reference_z_col):
            if last is None:
                target_x.append(float(rx))
                target_y.append(float(ry))
                target_z.append(float(rz))
                last = (float(rx), float(ry), float(rz))
                continue
            dx = float(rx) - last[0]
            dy = float(ry) - last[1]
            dz = float(rz) - last[2]
            if (dx * dx + dy * dy + dz * dz) ** 0.5 >= min_step:
                target_x.append(float(rx))
                target_y.append(float(ry))
                target_z.append(float(rz))
                last = (float(rx), float(ry), float(rz))

        if len(target_x) > 1:
            fig.add_trace(
                go.Scatter3d(
                    x=target_x,
                    y=target_y,
                    z=target_z,
                    mode="lines",
                    line=dict(color="#facc15", width=3, dash="dash"),
                    name="Target Path",
                )
            )

    fig.add_trace(
        go.Scatter3d(
            x=x_pos,
            y=y_pos,
            z=z_pos,
            mode="lines",
            line=dict(color=PlotStyle.COLOR_SIGNAL_POS, width=4),
            name="Trajectory",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=[x_pos[0]],
            y=[y_pos[0]],
            z=[z_pos[0]],
            mode="markers",
            marker=dict(size=5, color=PlotStyle.COLOR_SUCCESS),
            name="Start",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=[x_pos[-1]],
            y=[y_pos[-1]],
            z=[z_pos[-1]],
            mode="markers",
            marker=dict(size=5, color=PlotStyle.COLOR_ERROR),
            name="Final",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=[reference_x],
            y=[reference_y],
            z=[reference_z],
            mode="markers",
            marker=dict(size=6, color=PlotStyle.COLOR_TARGET, symbol="x"),
            name="Reference",
        )
    )

    fig.update_layout(
        title=f"Interactive 3D Trajectory - {plot_gen.system_title}",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(itemsizing="constant"),
        margin=dict(l=0, r=0, t=60, b=0),
    )

    fig.add_annotation(
        text=(
            "Double-click this file to open in browser. Scroll to zoom, drag to rotate."
        ),
        xref="paper",
        yref="paper",
        x=0.5,
        y=0.98,
        showarrow=False,
        font=dict(size=14, color="gray"),
        bgcolor="white",
        opacity=0.8,
    )

    output_path = plot_dir.parent / "01_trajectory_3d_interactive.html"
    fig.write_html(output_path, include_plotlyjs="cdn")
    print(f"Interactive 3D plot saved to: {output_path}")

    output_path = plot_dir / "00_mission_dashboard.html"
    fig.write_html(output_path, include_plotlyjs="cdn")
    print(f"Interactive mission dashboard saved to: {output_path}")


def generate_trajectory_3d_orientation_plot(plot_gen: Any, plot_dir: Path) -> None:
    """Generate 3D trajectory plot with orientation arrows."""
    x_pos = plot_gen._col("Current_X")
    y_pos = plot_gen._col("Current_Y")
    z_pos = plot_gen._col("Current_Z")
    if len(x_pos) == 0 or len(y_pos) == 0 or len(z_pos) == 0:
        return

    reference_x, reference_y, reference_z = _reference_point(plot_gen)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(x_pos, y_pos, z_pos, color=PlotStyle.COLOR_SIGNAL_POS, linewidth=2)
    ax.scatter(
        x_pos[0],
        y_pos[0],
        z_pos[0],
        color=PlotStyle.COLOR_SUCCESS,
        s=40,
        label="Start",
    )
    ax.scatter(
        x_pos[-1],
        y_pos[-1],
        z_pos[-1],
        color=PlotStyle.COLOR_ERROR,
        s=40,
        label="End",
    )
    ax.scatter(
        reference_x,
        reference_y,
        reference_z,
        color=PlotStyle.COLOR_TARGET,
        s=60,
        marker="*",
    )

    roll = plot_gen._col("Current_Roll")
    pitch = plot_gen._col("Current_Pitch")
    yaw = plot_gen._col("Current_Yaw")

    n = len(x_pos)
    step = max(n // 50, 1)
    idxs = np.arange(0, n, step)

    arrow_len = 0.06
    try:
        if plot_gen.app_config and plot_gen.app_config.physics:
            arrow_len = float(plot_gen.app_config.physics.satellite_size) * 0.2
        else:
            from satellite_control.config.simulation_config import SimulationConfig

            arrow_len = (
                float(
                    SimulationConfig.create_default().app_config.physics.satellite_size
                )
                * 0.2
            )
    except Exception:
        pass

    try:
        from scipy.spatial.transform import Rotation

        eulers = np.vstack([roll[idxs], pitch[idxs], yaw[idxs]]).T
        dirs = Rotation.from_euler("xyz", eulers, degrees=False).apply(
            np.array([1.0, 0.0, 0.0])
        )
        ax.quiver(
            x_pos[idxs],
            y_pos[idxs],
            z_pos[idxs],
            dirs[:, 0],
            dirs[:, 1],
            dirs[:, 2],
            length=arrow_len,
            normalize=True,
            color="gray",
            alpha=0.6,
        )
    except Exception:
        pass

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"3D Trajectory with Orientation - {plot_gen.system_title}")
    ax.legend()

    PlotStyle.save_figure(fig, plot_dir / "01_trajectory_3d_orientation.png")

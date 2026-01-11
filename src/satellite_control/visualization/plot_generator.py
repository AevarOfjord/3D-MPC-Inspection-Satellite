"""
Plot Generator for Visualization

Provides plotting functionality for performance analysis plots.
Extracted from unified_visualizer.py to improve modularity.

This module handles all static plot generation (not animation frames).
"""

from pathlib import Path
from typing import Any, Dict, List
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from typing import Optional

from src.satellite_control.config.models import AppConfig
from src.satellite_control.visualization.unified_visualizer import PlotStyle


class PlotGenerator:
    """
    Generates performance analysis plots from simulation data.
    
    This class handles all static plot generation, separating plotting
    logic from data management and animation generation.
    """
    
    def __init__(
        self,
        data_accessor: Any,
        dt: float,
        system_title: str = "Satellite Control System",
        app_config: Optional[AppConfig] = None,
    ):
        """
        Initialize plot generator.
        
        Args:
            data_accessor: Object with data access methods (_col, _row, _get_len)
            dt: Simulation timestep in seconds
            system_title: Title for plots
            app_config: Optional AppConfig for accessing configuration (v3.0.0)
        """
        self.data_accessor = data_accessor
        self.dt = dt
        self.system_title = system_title
        self.app_config = app_config
    
    def _col(self, name: str) -> np.ndarray:
        """Get column data from data accessor."""
        return self.data_accessor._col(name)
    
    def _row(self, idx: int) -> Dict[str, Any]:
        """Get row data from data accessor."""
        return self.data_accessor._row(idx)
    
    def _get_len(self) -> int:
        """Get data length from data accessor."""
        return self.data_accessor._get_len()
    
    def generate_all_plots(self, plot_dir: Path) -> None:
        """
        Generate all performance analysis plots.
        
        Args:
            plot_dir: Directory to save plots
        """
        print("Generating performance analysis plots...")
        plot_dir.mkdir(exist_ok=True)
        print(f" Created Plots directory: {plot_dir}")
        
        # Generate specific performance plots
        self.generate_position_tracking_plot(plot_dir)
        self.generate_position_error_plot(plot_dir)
        self.generate_angular_tracking_plot(plot_dir)
        self.generate_angular_error_plot(plot_dir)
        self.generate_error_norms_plot(plot_dir)
        self.generate_trajectory_plot(plot_dir)
        self.generate_trajectory_3d_interactive_plot(plot_dir)
        self.generate_trajectory_3d_orientation_plot(plot_dir)
        self.generate_thruster_usage_plot(plot_dir)
        self.generate_thruster_valve_activity_plot(plot_dir)
        self.generate_pwm_quantization_plot(plot_dir)
        self.generate_control_effort_plot(plot_dir)
        self.generate_actuator_limits_plot(plot_dir)
        self.generate_constraint_violations_plot(plot_dir)
        self.generate_reaction_wheel_output_plot(plot_dir)
        self.generate_z_tilt_coupling_plot(plot_dir)
        self.generate_thruster_impulse_proxy_plot(plot_dir)
        self.generate_phase_position_velocity_plot(plot_dir)
        self.generate_phase_attitude_rate_plot(plot_dir)
        self.generate_velocity_tracking_plot(plot_dir)
        self.generate_velocity_magnitude_plot(plot_dir)
        self.generate_mpc_performance_plot(plot_dir)
        self.generate_solver_health_plot(plot_dir)
        self.generate_waypoint_progress_plot(plot_dir)
        self.generate_timing_intervals_plot(plot_dir)
        
        print(f"Performance plots saved to: {plot_dir}")
    
    # Plot generation methods will be moved here from UnifiedVisualizationGenerator
    # For now, these are placeholder methods that delegate to the original class
    # They will be fully implemented in subsequent steps
    
    def generate_position_tracking_plot(self, plot_dir: Path) -> None:
        """Generate position tracking over time plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Position Tracking - {self.system_title}")
        
        time = np.arange(self._get_len()) * float(self.dt)
        
        # X position tracking
        axes[0].plot(
            time,
            self._col("Current_X"),
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current X",
        )
        axes[0].plot(
            time,
            self._col("Target_X"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Target X",
        )
        axes[0].set_ylabel("X Position (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("X Position Tracking")
        
        # Y position tracking
        axes[1].plot(
            time,
            self._col("Current_Y"),
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current Y",
        )
        axes[1].plot(
            time,
            self._col("Target_Y"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Target Y",
        )
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_ylabel("Y Position (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[1].set_title("Y Position Tracking")
        
        # Z position tracking
        axes[2].plot(
            time,
            self._col("Current_Z"),
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current Z",
        )
        axes[2].plot(
            time,
            self._col("Target_Z"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Target Z",
        )
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Z Position (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Z Position Tracking")
        
        PlotStyle.save_figure(fig, plot_dir / "position_tracking.png")
    
    def generate_position_error_plot(self, plot_dir: Path) -> None:
        """Generate position error plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Position Error - {self.system_title}")
        
        time = np.arange(self._get_len()) * float(self.dt)
        
        # Calculate errors
        error_x = self._col("Current_X") - self._col("Target_X")
        error_y = self._col("Current_Y") - self._col("Target_Y")
        error_z = self._col("Current_Z") - self._col("Target_Z")
        
        # X error
        axes[0].plot(
            time,
            error_x,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="X Error",
        )
        axes[0].axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
        axes[0].set_ylabel("X Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("X Position Error")
        
        # Y error
        axes[1].plot(
            time,
            error_y,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Y Error",
        )
        axes[1].axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
        axes[1].set_ylabel("Y Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[1].set_title("Y Position Error")
        
        # Z error
        axes[2].plot(
            time,
            error_z,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Z Error",
        )
        axes[2].axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Z Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Z Position Error")
        
        PlotStyle.save_figure(fig, plot_dir / "position_error.png")
    
    def generate_angular_tracking_plot(self, plot_dir: Path) -> None:
        """Generate angular tracking plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Angular Tracking - {self.system_title}")
        
        time = np.arange(self._get_len()) * float(self.dt)
        
        # Roll tracking
        axes[0].plot(
            time,
            np.degrees(self._col("Current_Roll")),
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current Roll",
        )
        axes[0].plot(
            time,
            np.degrees(self._col("Target_Roll")),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Target Roll",
        )
        axes[0].set_ylabel("Roll (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("Roll Tracking")
        
        # Pitch tracking
        axes[1].plot(
            time,
            np.degrees(self._col("Current_Pitch")),
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current Pitch",
        )
        axes[1].plot(
            time,
            np.degrees(self._col("Target_Pitch")),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Target Pitch",
        )
        axes[1].set_ylabel("Pitch (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[1].set_title("Pitch Tracking")
        
        # Yaw tracking
        axes[2].plot(
            time,
            np.degrees(self._col("Current_Yaw")),
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current Yaw",
        )
        axes[2].plot(
            time,
            np.degrees(self._col("Target_Yaw")),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Target Yaw",
        )
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Yaw (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Yaw Tracking")
        
        PlotStyle.save_figure(fig, plot_dir / "angular_tracking.png")
    
    def generate_angular_error_plot(self, plot_dir: Path) -> None:
        """Generate angular error plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Angular Error - {self.system_title}")
        
        time = np.arange(self._get_len()) * float(self.dt)
        
        # Calculate errors
        error_roll = self._col("Current_Roll") - self._col("Target_Roll")
        error_pitch = self._col("Current_Pitch") - self._col("Target_Pitch")
        error_yaw = self._col("Current_Yaw") - self._col("Target_Yaw")
        
        # Normalize angles to [-180, 180] degrees
        error_roll = np.degrees(np.arctan2(np.sin(error_roll), np.cos(error_roll)))
        error_pitch = np.degrees(np.arctan2(np.sin(error_pitch), np.cos(error_pitch)))
        error_yaw = np.degrees(np.arctan2(np.sin(error_yaw), np.cos(error_yaw)))
        
        # Roll error
        axes[0].plot(
            time,
            error_roll,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Roll Error",
        )
        axes[0].axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
        axes[0].set_ylabel("Roll Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("Roll Error")
        
        # Pitch error
        axes[1].plot(
            time,
            error_pitch,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Pitch Error",
        )
        axes[1].axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
        axes[1].set_ylabel("Pitch Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[1].set_title("Pitch Error")
        
        # Yaw error
        axes[2].plot(
            time,
            error_yaw,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Yaw Error",
        )
        axes[2].axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Yaw Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Yaw Error")
        
        PlotStyle.save_figure(fig, plot_dir / "angular_error.png")

    def generate_error_norms_plot(self, plot_dir: Path) -> None:
        """Generate error norm summary plot."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"Error Norms - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        def get_series(name: str) -> np.ndarray:
            vals = self._col(name)
            return vals if len(vals) else np.zeros_like(time)

        err_x = get_series("Error_X")
        err_y = get_series("Error_Y")
        err_z = get_series("Error_Z")
        err_vx = get_series("Error_VX")
        err_vy = get_series("Error_VY")
        err_vz = get_series("Error_VZ")
        err_roll = get_series("Error_Roll")
        err_pitch = get_series("Error_Pitch")
        err_yaw = get_series("Error_Yaw")
        err_wx = get_series("Error_WX")
        err_wy = get_series("Error_WY")
        err_wz = get_series("Error_WZ")

        pos_err_norm = np.sqrt(err_x**2 + err_y**2 + err_z**2)
        vel_err_norm = np.sqrt(err_vx**2 + err_vy**2 + err_vz**2)
        ang_err_norm = np.degrees(np.sqrt(err_roll**2 + err_pitch**2 + err_yaw**2))
        angvel_err_norm = np.degrees(np.sqrt(err_wx**2 + err_wy**2 + err_wz**2))

        axes[0, 0].plot(
            time,
            pos_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Position Error Norm",
        )
        axes[0, 0].set_ylabel("Position Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0, 0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0, 0].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[0, 1].plot(
            time,
            vel_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Velocity Error Norm",
        )
        axes[0, 1].set_ylabel("Velocity Error (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0, 1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0, 1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[1, 0].plot(
            time,
            ang_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Attitude Error Norm",
        )
        axes[1, 0].set_ylabel("Attitude Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1, 0].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1, 0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1, 0].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[1, 1].plot(
            time,
            angvel_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Angular Rate Error Norm",
        )
        axes[1, 1].set_ylabel("Angular Rate Error (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1, 1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1, 1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1, 1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "error_norms.png")
    
    def generate_trajectory_plot(self, plot_dir: Path) -> None:
        """Generate trajectory plot."""
        fig, axes = plt.subplots(1, 2, figsize=(12, 6))
        
        x_pos = self._col("Current_X")
        y_pos = self._col("Current_Y")
        z_pos = self._col("Current_Z")
        target_x_col = self._col("Target_X")
        target_y_col = self._col("Target_Y")
        target_z_col = self._col("Target_Z")
        target_x = target_x_col[0] if len(target_x_col) > 0 else 0.0
        target_y = target_y_col[0] if len(target_y_col) > 0 else 0.0
        target_z = target_z_col[0] if len(target_z_col) > 0 else 0.0
        
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
        ax_xy.plot(target_x, target_y, "r*", markersize=20, label="Target")
        
        # Add target circle
        circle = Circle(
            (target_x, target_y),
            0.1,
            color=PlotStyle.COLOR_TARGET,
            fill=False,
            linewidth=PlotStyle.LINEWIDTH,
            linestyle="--",
            alpha=0.7,
            label="Target Zone (±0.1m)",
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
        ax_xz.plot(target_x, target_z, "r*", markersize=20, label="Target")
        
        circle_xz = Circle(
            (target_x, target_z),
            0.1,
            color=PlotStyle.COLOR_TARGET,
            fill=False,
            linewidth=PlotStyle.LINEWIDTH,
            linestyle="--",
            alpha=0.7,
            label="Target Zone (±0.1m)",
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
        
        # Add distance info
        if len(x_pos) > 0 and len(y_pos) > 0 and len(z_pos) > 0:
            final_distance = np.sqrt(
                (x_pos[-1] - target_x) ** 2
                + (y_pos[-1] - target_y) ** 2
                + (z_pos[-1] - target_z) ** 2
            )
        else:
            final_distance = 0.0
        ax_xy.text(
            0.02,
            0.98,
            f"Final Distance to Target: {final_distance:.3f}m",
            transform=ax_xy.transAxes,
            fontsize=PlotStyle.ANNOTATION_SIZE,
            verticalalignment="top",
            bbox=PlotStyle.TEXTBOX_STYLE,
        )
        
        PlotStyle.save_figure(fig, plot_dir / "trajectory.png")
    
    def generate_trajectory_3d_interactive_plot(self, plot_dir: Path) -> None:
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

        x_pos = self._col("Current_X")
        y_pos = self._col("Current_Y")
        z_pos = self._col("Current_Z")
        if len(x_pos) == 0 or len(y_pos) == 0 or len(z_pos) == 0:
            print("No trajectory data available for interactive 3D plot.")
            return

        target_x_col = self._col("Target_X")
        target_y_col = self._col("Target_Y")
        target_z_col = self._col("Target_Z")
        target_x = float(target_x_col[0]) if len(target_x_col) > 0 else 0.0
        target_y = float(target_y_col[0]) if len(target_y_col) > 0 else 0.0
        target_z = float(target_z_col[0]) if len(target_z_col) > 0 else 0.0

        fig = go.Figure()
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
                x=[target_x],
                y=[target_y],
                z=[target_z],
                mode="markers",
                marker=dict(size=6, color=PlotStyle.COLOR_TARGET, symbol="x"),
                name="Target",
            )
        )

        fig.update_layout(
            title=f"Interactive 3D Trajectory - {self.system_title}",
            scene=dict(
                xaxis_title="X (m)",
                yaxis_title="Y (m)",
                zaxis_title="Z (m)",
                aspectmode="data",
            ),
            legend=dict(itemsizing="constant"),
            margin=dict(l=0, r=0, t=40, b=0),
        )

        output_path = plot_dir / "trajectory_3d_interactive.html"
        fig.write_html(output_path, include_plotlyjs="cdn")

    def generate_trajectory_3d_orientation_plot(self, plot_dir: Path) -> None:
        """Generate 3D trajectory plot with orientation arrows."""
        x_pos = self._col("Current_X")
        y_pos = self._col("Current_Y")
        z_pos = self._col("Current_Z")
        if len(x_pos) == 0 or len(y_pos) == 0 or len(z_pos) == 0:
            return

        target_x_col = self._col("Target_X")
        target_y_col = self._col("Target_Y")
        target_z_col = self._col("Target_Z")
        target_x = float(target_x_col[0]) if len(target_x_col) > 0 else 0.0
        target_y = float(target_y_col[0]) if len(target_y_col) > 0 else 0.0
        target_z = float(target_z_col[0]) if len(target_z_col) > 0 else 0.0

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        ax.plot(x_pos, y_pos, z_pos, color=PlotStyle.COLOR_SIGNAL_POS, linewidth=2)
        ax.scatter(x_pos[0], y_pos[0], z_pos[0], color=PlotStyle.COLOR_SUCCESS, s=40, label="Start")
        ax.scatter(x_pos[-1], y_pos[-1], z_pos[-1], color=PlotStyle.COLOR_ERROR, s=40, label="End")
        ax.scatter(target_x, target_y, target_z, color=PlotStyle.COLOR_TARGET, s=60, marker="*")

        roll = self._col("Current_Roll")
        pitch = self._col("Current_Pitch")
        yaw = self._col("Current_Yaw")

        n = len(x_pos)
        step = max(n // 50, 1)
        idxs = np.arange(0, n, step)

        arrow_len = 0.06
        try:
            if self.app_config and self.app_config.physics:
                arrow_len = float(self.app_config.physics.satellite_size) * 0.2
            else:
                from src.satellite_control.config.simulation_config import SimulationConfig

                arrow_len = float(
                    SimulationConfig.create_default().app_config.physics.satellite_size
                ) * 0.2
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
        ax.set_title(f"3D Trajectory with Orientation - {self.system_title}")
        ax.legend()

        PlotStyle.save_figure(fig, plot_dir / "trajectory_3d_orientation.png")
    
    def generate_thruster_usage_plot(self, plot_dir: Path) -> None:
        """Generate thruster usage plot using actual valve states."""
        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)

        thruster_count = self._get_thruster_count()
        thruster_ids = np.arange(1, thruster_count + 1)
        total_activation_time = np.zeros(thruster_count)
        data_source = "commanded"

        # Try to use actual valve states (Thruster_X_Val) for accurate PWM tracking
        has_valve_data = False
        if (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
                and "Thruster_1_Val" in self.data_accessor.data.columns
            ):
                has_valve_data = True
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
            and "Thruster_1_Val" in self.data_accessor._col_data
        ):
            has_valve_data = True

        if has_valve_data:
            # Use actual valve states - most accurate for PWM
            data_source = "actual valve"
            for i in range(thruster_count):
                col_name = f"Thruster_{i+1}_Val"
                vals = self._col(col_name)
                try:
                    vals = np.array([float(x) for x in vals])
                except (ValueError, TypeError):
                    vals = np.zeros(len(vals), dtype=float)
                # Sum valve states and multiply by physics timestep
                total_activation_time[i] = np.sum(vals) * float(self.dt)
        else:
            # Fallback: Use commanded duty cycles from Command_Vector
            command_data = []
            for idx in range(self._get_len()):
                row = self._row(idx)
                cmd_vec = self._parse_command_vector(row["Command_Vector"])
                command_data.append(cmd_vec)
            command_matrix = np.array(command_data)
            if command_matrix.ndim == 2:
                if command_matrix.shape[1] >= thruster_count:
                    command_matrix = command_matrix[:, :thruster_count]
                else:
                    pad = np.zeros((command_matrix.shape[0], thruster_count))
                    pad[:, : command_matrix.shape[1]] = command_matrix
                    command_matrix = pad
            total_activation_time = np.sum(command_matrix, axis=0) * float(self.dt)

        # Create bar plot
        bars = ax.bar(
            thruster_ids,
            total_activation_time,
            color=PlotStyle.COLOR_BARS,
            alpha=0.8,
            edgecolor="black",
            linewidth=1.2,
        )

        # Add value labels on top of bars
        for _i, bar in enumerate(bars):
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
            f"Thruster Usage Summary - {self.system_title}",
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

        PlotStyle.save_figure(fig, plot_dir / "thruster_usage.png")
    
    def generate_thruster_valve_activity_plot(self, plot_dir: Path) -> None:
        """Generate detailed valve activity plot for each thruster (0.0 to 1.0)."""
        # Check if we have valve data
        has_valve_data = False
        if (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
                and "Thruster_1_Val" in self.data_accessor.data.columns
            ):
                has_valve_data = True
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
            and "Thruster_1_Val" in self.data_accessor._col_data
        ):
            has_valve_data = True

        if not has_valve_data:
            return

        thruster_count = self._get_thruster_count()
        cols = 3 if thruster_count > 8 else 2
        rows = int(np.ceil(thruster_count / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(15, 4 * rows), sharex=True)
        axes = axes.flatten()

        time = np.arange(self._get_len()) * float(self.dt)

        for i in range(thruster_count):
            thruster_id = i + 1
            col_name_val = f"Thruster_{thruster_id}_Val"
            col_name_cmd = f"Thruster_{thruster_id}_Cmd"

            # Get Valve data
            vals = self._col(col_name_val)
            try:
                vals = np.array([float(x) for x in vals])
            except (ValueError, TypeError):
                vals = np.zeros_like(vals, dtype=float)

            # Get Command data (Duty Cycle)
            cmds = self._col(col_name_cmd)
            try:
                cmds = np.array([float(x) for x in cmds])
            except (ValueError, TypeError):
                # If command column missing or invalid, default to zeros
                cmds = np.zeros_like(vals, dtype=float)

            # Update Combined Plot
            ax = axes[i]
            # Fill area for actual valve state
            ax.fill_between(time, vals, color="tab:blue", alpha=0.3, label="Valve")
            ax.plot(time, vals, color="tab:blue", linewidth=1)
            # Overlay Commanded Duty Cycle
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

            # Generate Separate Plot for this Thruster (Two subplots)
            fig_ind, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)

            # Top plot: Binary valve state (0 or 1)
            ax_top.fill_between(
                time,
                vals,
                color="tab:blue",
                alpha=0.3,
            )
            ax_top.plot(time, vals, color="tab:blue", linewidth=1)
            ax_top.set_ylim(-0.1, 1.1)
            ax_top.set_yticks([0, 1])
            ax_top.set_yticklabels(["OFF", "ON"])
            ax_top.grid(True, alpha=0.3)
            ax_top.set_ylabel("Valve State")
            ax_top.set_title(f"Thruster {thruster_id} - {self.system_title}", fontsize=12)

            # Bottom plot: Commanded duty cycle (u value)
            ax_bot.plot(
                time,
                cmds,
                color="red",
                linewidth=1.5,
            )
            ax_bot.fill_between(time, cmds, color="red", alpha=0.2)
            ax_bot.set_ylim(-0.05, 1.05)
            ax_bot.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
            ax_bot.grid(True, alpha=0.3)
            ax_bot.set_ylabel("Cmd Duty Cycle")
            ax_bot.set_xlabel("Time (s)")

            plt.tight_layout()
            # Save individual plot
            plt.savefig(
                plot_dir / f"thruster_{thruster_id}_valve_activity.png",
                dpi=300,
                bbox_inches="tight",
            )
            plt.close(fig_ind)

        fig.suptitle(
            f"Thruster Valve Activity (0.0 - 1.0) - {self.system_title}",
            fontsize=16,
        )
        for idx in range(thruster_count, len(axes)):
            axes[idx].axis("off")
        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        plt.savefig(
            plot_dir / "thruster_valve_activity.png",
            dpi=300,
            bbox_inches="tight",
        )
        plt.close()
    
    def generate_pwm_quantization_plot(self, plot_dir: Path) -> None:
        """Generate PWM duty cycle plot showing MPC u-values vs time."""
        # Use control_data which has the actual MPC outputs per control step
        control_df = None
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            control_df = self.data_accessor.control_data

        if control_df is None or "Command_Vector" not in control_df.columns:
            # Fallback: check if physics_data has the cmd columns
            return self.generate_pwm_duty_cycles_from_physics(plot_dir)

        # Parse Command_Vector column to get duty cycles per thruster
        time = control_df["Control_Time"].values

        # Parse the command vectors
        thruster_count = self._get_thruster_count()
        duty_cycles_per_thruster: Dict[int, List[float]] = {
            i: [] for i in range(thruster_count)
        }

        for cmd_str in control_df["Command_Vector"]:
            try:
                # Parse "[0.000, 1.000, ...]" format
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
            f"PWM MPC Duty Cycle Output (u) vs Time - {self.system_title}",
            fontsize=14,
            fontweight="bold",
        )

        colors = plt.cm.tab20(np.linspace(0, 1, thruster_count))

        # Track if we find any intermediate values
        all_intermediate_values = []

        for i, ax in enumerate(axes):
            if i >= thruster_count:
                ax.axis("off")
                continue
            thruster_id = i + 1
            duty_cycles = np.array(duty_cycles_per_thruster[i])

            # Plot duty cycle over time as step function
            ax.step(
                time,
                duty_cycles,
                where="post",
                color=colors[i],
                linewidth=1.5,
                alpha=0.9,
            )
            ax.fill_between(time, 0, duty_cycles, step="post", color=colors[i], alpha=0.3)

            # Add horizontal lines at key duty cycle levels
            ax.axhline(y=0.0, color="gray", linestyle="-", alpha=0.3, linewidth=0.5)
            ax.axhline(y=0.5, color="gray", linestyle="--", alpha=0.3, linewidth=0.5)
            ax.axhline(y=1.0, color="gray", linestyle="-", alpha=0.3, linewidth=0.5)

            # Find intermediate values (not 0 or 1)
            intermediate = [(t, u) for t, u in zip(time, duty_cycles) if 0.01 < u < 0.99]
            if intermediate:
                all_intermediate_values.extend([u for _, u in intermediate])
                # Mark intermediate points with dots
                for t_val, u_val in intermediate:
                    ax.plot(t_val, u_val, "ko", markersize=4)

            # Labels
            ax.set_ylabel(f"T{thruster_id}", fontsize=10, fontweight="bold")
            ax.set_ylim(-0.05, 1.05)
            ax.set_yticks([0, 0.5, 1.0])
            ax.grid(True, alpha=0.3)

            # Stats annotation
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

        # X-axis label on bottom row
        for i in range(thruster_count):
            if i // cols == rows - 1:
                axes[i].set_xlabel("Time (s)", fontsize=12)

        # Add summary annotation
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
        plt.savefig(plot_dir / "pwm_duty_cycles.png", dpi=300, bbox_inches="tight")
        plt.close()

    def generate_pwm_duty_cycles_from_physics(self, plot_dir: Path) -> None:
        """Fallback: Generate PWM plot from physics_data Thruster_X_Cmd columns."""
        # This shows binary valve states, not continuous duty cycles
        pass
    
    def _get_thruster_count(self) -> int:
        """Determine thruster count based on available data or config."""
        cols: list[str] = []
        if (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if hasattr(self.data_accessor, "data") and self.data_accessor.data is not None:
                cols = list(self.data_accessor.data.columns)
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())

        max_id = 0
        if cols:
            for i in range(1, 33):
                if f"Thruster_{i}_Val" in cols or f"Thruster_{i}_Cmd" in cols:
                    max_id = i

        if max_id > 0:
            return max_id

        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            if (
                "Command_Vector" in self.data_accessor.control_data.columns
                and len(self.data_accessor.control_data) > 0
            ):
                sample_vec = self._parse_command_vector(
                    self.data_accessor.control_data["Command_Vector"].iloc[0]
                )
                if sample_vec.size > 0:
                    return int(sample_vec.size)

        # Fallback to default
        try:
            # Get thruster count from app_config if available, otherwise fallback
            if self.app_config and self.app_config.physics:
                return len(self.app_config.physics.thruster_positions)
            else:
                # V4.0.0: Use default config if app_config not available
                from src.satellite_control.config.simulation_config import SimulationConfig
                default_config = SimulationConfig.create_default()
                return len(default_config.app_config.physics.thruster_positions)
        except Exception:
            return 8

    def _parse_command_vector(self, command_str: Any) -> np.ndarray:
        """Parse command vector string to numpy array.
        
        Args:
            command_str: String representation of command vector (or any type)
        
        Returns:
            numpy array of thruster commands
        """
        try:
            # Handle None or empty
            if command_str is None or command_str == "":
                return np.zeros(self._get_thruster_count())
            
            # Convert to string if not already
            command_str = str(command_str)
            
            # Remove brackets and split
            command_str = command_str.strip("[]")
            values = [float(x.strip()) for x in command_str.split(",")]
            return np.array(values)
        except Exception:
            return np.zeros(self._get_thruster_count())
    
    def generate_control_effort_plot(self, plot_dir: Path) -> None:
        """Generate control effort plot."""
        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        
        time = np.arange(self._get_len()) * float(self.dt)
        
        # Parse command vectors
        command_data = []
        for idx in range(self._get_len()):
            row = self._row(idx)
            cmd_vec = self._parse_command_vector(row["Command_Vector"])
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
        ax.set_title(f"Control Effort Over Time - {self.system_title}")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        
        PlotStyle.save_figure(fig, plot_dir / "control_effort.png")

    def generate_reaction_wheel_output_plot(self, plot_dir: Path) -> None:
        """Generate reaction wheel torque output plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Reaction Wheel Output - {self.system_title}")

        df = None
        cols = []
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            df = self.data_accessor.control_data
            cols = df.columns
        elif (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                df = self.data_accessor.data
                cols = df.columns
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())

        if df is not None and "Control_Time" in cols:
            time = df["Control_Time"].values
        elif df is not None and "CONTROL_DT" in cols:
            dt_val = df["CONTROL_DT"].iloc[0]
            time = np.arange(len(df)) * float(dt_val)
        else:
            time = np.arange(len(df) if df is not None else self._get_len()) * float(self.dt)

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
            axes[0].set_title(f"Reaction Wheel Output - {self.system_title}")
            for ax in axes[1:]:
                ax.axis("off")
            PlotStyle.save_figure(fig, plot_dir / "reaction_wheel_output.png")
            return

        def get_series(name: str) -> np.ndarray:
            if df is not None and name in cols:
                return df[name].values
            return self._col(name)

        def normalize_series(values: np.ndarray) -> np.ndarray:
            if values is None or len(values) == 0:
                return np.zeros(base_len, dtype=float)
            try:
                arr = np.array(values, dtype=float)
            except (ValueError, TypeError):
                return np.zeros(base_len, dtype=float)
            if arr.size < base_len:
                padded = np.zeros(base_len, dtype=float)
                padded[: arr.size] = arr
                return padded
            return arr[:base_len]

        rw_x = normalize_series(get_series("RW_Torque_X"))
        rw_y = normalize_series(get_series("RW_Torque_Y"))
        rw_z = normalize_series(get_series("RW_Torque_Z"))

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

        PlotStyle.save_figure(fig, plot_dir / "reaction_wheel_output.png")

    def generate_actuator_limits_plot(self, plot_dir: Path) -> None:
        """Generate actuator outputs with limit overlays."""
        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Actuator Limits - {self.system_title}")

        df = None
        cols = []
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            df = self.data_accessor.control_data
            cols = df.columns
        elif (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                df = self.data_accessor.data
                cols = df.columns
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())

        if df is not None and "Control_Time" in cols:
            time = df["Control_Time"].values
        elif df is not None and "CONTROL_DT" in cols:
            dt_val = df["CONTROL_DT"].iloc[0]
            time = np.arange(len(df)) * float(dt_val)
        else:
            time = np.arange(len(df) if df is not None else self._get_len()) * float(self.dt)

        # Thruster command envelope
        command_vectors = []
        if df is not None and "Command_Vector" in cols:
            for cmd_str in df["Command_Vector"].values:
                command_vectors.append(self._parse_command_vector(cmd_str))
        else:
            for idx in range(self._get_len()):
                row = self._row(idx)
                command_vectors.append(self._parse_command_vector(row.get("Command_Vector")))
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
            axes[0].axhline(y=1.0, color="black", linestyle="--", alpha=0.6, label="Max Limit")
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

        # Reaction wheel torque limits
        base_len = len(time)

        def get_series(name: str) -> np.ndarray:
            if df is not None and name in cols:
                return df[name].values
            return self._col(name)

        def normalize_series(values: np.ndarray) -> np.ndarray:
            if values is None or len(values) == 0:
                return np.zeros(base_len, dtype=float)
            try:
                arr = np.array(values, dtype=float)
            except (ValueError, TypeError):
                return np.zeros(base_len, dtype=float)
            if arr.size < base_len:
                padded = np.zeros(base_len, dtype=float)
                padded[: arr.size] = arr
                return padded
            return arr[:base_len]

        rw_x = normalize_series(get_series("RW_Torque_X"))
        rw_y = normalize_series(get_series("RW_Torque_Y"))
        rw_z = normalize_series(get_series("RW_Torque_Z"))

        try:
            from src.satellite_control.config.reaction_wheel_config import get_reaction_wheel_config

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
                axes[1].axhline(y=-max_rw, color="black", linestyle="--", alpha=0.6, label="RW Limit")
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

        PlotStyle.save_figure(fig, plot_dir / "actuator_limits.png")

    def generate_constraint_violations_plot(self, plot_dir: Path) -> None:
        """Generate constraint violation plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Constraint Violations - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        try:
            if self.app_config and self.app_config.mpc:
                pos_bound = float(self.app_config.mpc.position_bounds)
                max_vel = float(self.app_config.mpc.max_velocity)
                max_ang_vel = float(self.app_config.mpc.max_angular_velocity)
            else:
                from src.satellite_control.config.simulation_config import SimulationConfig

                cfg = SimulationConfig.create_default().app_config.mpc
                pos_bound = float(cfg.position_bounds)
                max_vel = float(cfg.max_velocity)
                max_ang_vel = float(cfg.max_angular_velocity)
        except Exception:
            pos_bound = 0.0
            max_vel = 0.0
            max_ang_vel = 0.0

        x = self._col("Current_X")
        y = self._col("Current_Y")
        z = self._col("Current_Z")
        vx = self._col("Current_VX")
        vy = self._col("Current_VY")
        vz = self._col("Current_VZ")
        wx = self._col("Current_WX")
        wy = self._col("Current_WY")
        wz = self._col("Current_WZ")

        min_len = min(len(time), len(x), len(y), len(z), len(vx), len(vy), len(vz), len(wx), len(wy), len(wz))
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
            PlotStyle.save_figure(fig, plot_dir / "constraint_violations.png")
            return

        pos_violation = np.maximum(
            np.max(np.abs(np.vstack([x[:min_len], y[:min_len], z[:min_len]])), axis=0) - pos_bound,
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

        PlotStyle.save_figure(fig, plot_dir / "constraint_violations.png")

    def generate_z_tilt_coupling_plot(self, plot_dir: Path) -> None:
        """Generate Z-tilt coupling plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Z Tilt Coupling - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        err_z = self._col("Error_Z")
        if len(err_z) == 0:
            current_z = self._col("Current_Z")
            target_z = self._col("Target_Z")
            min_len = min(len(current_z), len(target_z))
            err_z = current_z[:min_len] - target_z[:min_len]

        roll = np.degrees(self._col("Current_Roll"))
        pitch = np.degrees(self._col("Current_Pitch"))
        vz = self._col("Current_VZ")

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
            PlotStyle.save_figure(fig, plot_dir / "z_tilt_coupling.png")
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

        PlotStyle.save_figure(fig, plot_dir / "z_tilt_coupling.png")

    def generate_thruster_impulse_proxy_plot(self, plot_dir: Path) -> None:
        """Generate thruster impulse proxy plot."""
        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Thruster Impulse Proxy - {self.system_title}")

        df = None
        cols = []
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            df = self.data_accessor.control_data
            cols = df.columns
        elif (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                df = self.data_accessor.data
                cols = df.columns
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())

        if df is not None and "Control_Time" in cols:
            time = df["Control_Time"].values
        elif df is not None and "CONTROL_DT" in cols:
            dt_val = df["CONTROL_DT"].iloc[0]
            time = np.arange(len(df)) * float(dt_val)
        else:
            time = np.arange(len(df) if df is not None else self._get_len()) * float(self.dt)

        thruster_forces = {}
        thruster_dirs = {}
        try:
            if self.app_config and self.app_config.physics:
                thruster_forces = self.app_config.physics.thruster_forces
                thruster_dirs = self.app_config.physics.thruster_directions
            else:
                from src.satellite_control.config.simulation_config import SimulationConfig

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
            PlotStyle.save_figure(fig, plot_dir / "thruster_impulse_proxy.png")
            return

        force_matrix = []
        for tid in thruster_ids:
            direction = np.array(thruster_dirs[tid], dtype=float)
            force_matrix.append(float(thruster_forces[tid]) * direction)
        force_matrix = np.array(force_matrix)

        command_vectors = []
        if df is not None and "Command_Vector" in cols:
            for cmd_str in df["Command_Vector"].values:
                command_vectors.append(self._parse_command_vector(cmd_str))
        else:
            for idx in range(self._get_len()):
                row = self._row(idx)
                command_vectors.append(self._parse_command_vector(row.get("Command_Vector")))
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
            PlotStyle.save_figure(fig, plot_dir / "thruster_impulse_proxy.png")
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
            fallback_dt = float(np.median(dt_steps[1:])) if np.any(dt_steps[1:]) else float(self.dt)
        else:
            fallback_dt = float(self.dt)
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

        PlotStyle.save_figure(fig, plot_dir / "thruster_impulse_proxy.png")

    def generate_phase_position_velocity_plot(self, plot_dir: Path) -> None:
        """Generate position vs velocity phase plots."""
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        fig.suptitle(f"Phase Plot: Position vs Velocity - {self.system_title}")

        x = self._col("Current_X")
        y = self._col("Current_Y")
        z = self._col("Current_Z")
        vx = self._col("Current_VX")
        vy = self._col("Current_VY")
        vz = self._col("Current_VZ")

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

        PlotStyle.save_figure(fig, plot_dir / "phase_position_velocity.png")

    def generate_phase_attitude_rate_plot(self, plot_dir: Path) -> None:
        """Generate attitude vs rate phase plots."""
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        fig.suptitle(f"Phase Plot: Attitude vs Rates - {self.system_title}")

        roll = np.degrees(self._col("Current_Roll"))
        pitch = np.degrees(self._col("Current_Pitch"))
        yaw = np.degrees(self._col("Current_Yaw"))
        wx = np.degrees(self._col("Current_WX"))
        wy = np.degrees(self._col("Current_WY"))
        wz = np.degrees(self._col("Current_WZ"))

        axes[0].plot(roll, wx, color=PlotStyle.COLOR_SIGNAL_ANG, linewidth=PlotStyle.LINEWIDTH)
        axes[0].set_xlabel("Roll (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].set_ylabel("WX (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)

        axes[1].plot(pitch, wy, color=PlotStyle.COLOR_SIGNAL_ANG, linewidth=PlotStyle.LINEWIDTH)
        axes[1].set_xlabel("Pitch (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_ylabel("WY (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)

        axes[2].plot(yaw, wz, color=PlotStyle.COLOR_SIGNAL_ANG, linewidth=PlotStyle.LINEWIDTH)
        axes[2].set_xlabel("Yaw (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("WZ (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)

        PlotStyle.save_figure(fig, plot_dir / "phase_attitude_rate.png")

    def generate_solver_health_plot(self, plot_dir: Path) -> None:
        """Generate solver health summary plot."""
        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Solver Health - {self.system_title}")

        df = None
        cols = []
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            df = self.data_accessor.control_data
            cols = df.columns
        elif (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                df = self.data_accessor.data
                cols = df.columns
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())

        if df is not None and "MPC_Status" in cols:
            status_vals = df["MPC_Status"].values
        else:
            status_vals = self._col("MPC_Status")

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
            solve_times = self._col("MPC_Solve_Time")
            if len(solve_times) == 0:
                solve_times = self._col("MPC_Computation_Time")

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

    def generate_waypoint_progress_plot(self, plot_dir: Path) -> None:
        """Generate waypoint/mission phase progress plot."""
        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Waypoint Progress - {self.system_title}")

        df = None
        cols = []
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            df = self.data_accessor.control_data
            cols = df.columns
        elif (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                df = self.data_accessor.data
                cols = df.columns
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())

        if df is not None and "Control_Time" in cols:
            time = df["Control_Time"].values
        elif df is not None and "CONTROL_DT" in cols:
            dt_val = df["CONTROL_DT"].iloc[0]
            time = np.arange(len(df)) * float(dt_val)
        else:
            time = np.arange(len(df) if df is not None else self._get_len()) * float(self.dt)

        if df is not None and "Waypoint_Number" in cols:
            waypoint_vals = df["Waypoint_Number"].values
        else:
            waypoint_vals = self._col("Waypoint_Number")

        if df is not None and "Mission_Phase" in cols:
            phase_vals = df["Mission_Phase"].values
        else:
            phase_vals = self._col("Mission_Phase")

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
    
    def generate_velocity_tracking_plot(self, plot_dir: Path) -> None:
        """Generate velocity tracking over time plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Velocity Tracking - {self.system_title}")
        
        time = np.arange(self._get_len()) * float(self.dt)
        
        def plot_velocity(ax, axis_label, current_col, target_col):
            current_vals = self._col(current_col)
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
            target_vals = self._col(target_col)
            if len(target_vals) > 0:
                tgt_len = min(len(time), len(target_vals))
                ax.plot(
                    time[:tgt_len],
                    target_vals[:tgt_len],
                    color=PlotStyle.COLOR_TARGET,
                    linestyle="--",
                    linewidth=PlotStyle.LINEWIDTH,
                    label=f"Target {axis_label}",
                )
            ax.set_ylabel(f"{axis_label} Velocity (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        
        plot_velocity(axes[0], "X", "Current_VX", "Target_VX")
        plot_velocity(axes[1], "Y", "Current_VY", "Target_VY")
        plot_velocity(axes[2], "Z", "Current_VZ", "Target_VZ")
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        
        PlotStyle.save_figure(fig, plot_dir / "velocity_tracking.png")
    
    def generate_velocity_magnitude_plot(self, plot_dir: Path) -> None:
        """Generate velocity magnitude over time plot (speed vs time)."""
        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        
        n = self._get_len()
        if n < 2:
            ax.text(
                0.5,
                0.5,
                "Insufficient data for velocity magnitude plot",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            PlotStyle.save_figure(fig, plot_dir / "velocity_magnitude.png")
            return
        
        time = np.arange(n) * float(self.dt)
        
        # Calculate velocity magnitude
        vx = self._col("Current_VX")
        vy = self._col("Current_VY")
        vz = self._col("Current_VZ")
        
        min_len = min(len(time), len(vx), len(vy), len(vz))
        if min_len == 0:
            return
        
        velocity_magnitude = np.sqrt(
            vx[:min_len] ** 2 + vy[:min_len] ** 2 + vz[:min_len] ** 2
        )
        
        ax.plot(
            time[:min_len],
            velocity_magnitude,
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Velocity Magnitude",
        )
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Speed (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title(f"Velocity Magnitude Over Time - {self.system_title}")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        
        PlotStyle.save_figure(fig, plot_dir / "velocity_magnitude.png")
    
    def generate_mpc_performance_plot(self, plot_dir: Path) -> None:
        """Generate MPC performance plot."""
        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        
        # Determine data source
        df = None
        cols = []
        
        # Check for sibling control data first (dual-log mode)
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            df = self.data_accessor.control_data
            cols = df.columns
        elif (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                df = self.data_accessor.data
                cols = df.columns
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())
        
        if "MPC_Computation_Time" in cols or "MPC_Solve_Time" in cols:
            # Determine time axis
            if df is not None and "Control_Time" in cols:
                time = df["Control_Time"].values
            elif df is not None and "CONTROL_DT" in cols:
                dt_val = df["CONTROL_DT"].iloc[0]
                time = np.arange(len(df)) * float(dt_val)
            else:
                time = np.arange(len(df) if df is not None else self._get_len()) * float(self.dt)
            
            # Get computation times
            if df is not None:
                raw_comp_times = df.get(
                    "MPC_Solve_Time", df.get("MPC_Computation_Time", [])
                ).values
            else:
                raw_comp_times = (
                    self._col("MPC_Solve_Time")
                    if "MPC_Solve_Time" in cols
                    else self._col("MPC_Computation_Time")
                )
            
            comp_times = []
            for val in raw_comp_times:
                try:
                    comp_times.append(float(val) * 1000)  # Convert to ms
                except (ValueError, TypeError):
                    comp_times.append(0.0)
            comp_times = np.array(comp_times)
            
            # Optional: solver time limit per step
            limit_ms = None
            if "MPC_Solver_Time_Limit" in cols:
                if df is not None:
                    raw_limits = df["MPC_Solver_Time_Limit"].values
                else:
                    raw_limits = self._col("MPC_Solver_Time_Limit")
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
                y=max_ms,
                color="g",
                linestyle=":",
                alpha=0.7,
                label=f"Max: {max_ms:.1f} ms",
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
                    limit_val = float(limit_ms[0] if isinstance(limit_ms, np.ndarray) else limit_ms)
                    if limit_val > 0:
                        ax.axhline(
                            y=limit_val,
                            color="black",
                            linestyle=":",
                            alpha=0.6,
                            label="Time Limit",
                        )
                # Highlight exceedances
                if "MPC_Time_Limit_Exceeded" in cols:
                    exceeded_vals = self._col("MPC_Time_Limit_Exceeded")
                    exceeded_idx = []
                    for i, val in enumerate(exceeded_vals):
                        try:
                            if bool(val) or (isinstance(val, str) and val.lower() == "true"):
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
            ax.set_title(f"MPC Computation Time - {self.system_title}")
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
            ax.set_title(f"MPC Computation Time - {self.system_title}")
        
        PlotStyle.save_figure(fig, plot_dir / "mpc_performance.png")
    
    def generate_timing_intervals_plot(self, plot_dir: Path) -> None:
        """Generate timing intervals plot."""
        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        
        # Check for sibling control data first (dual-log mode)
        df = None
        cols = []
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            df = self.data_accessor.control_data
            cols = df.columns
        elif (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                df = self.data_accessor.data
                cols = df.columns
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            cols = list(self.data_accessor._col_data.keys())
        
        if "Actual_Time_Interval" in cols:
            # Determine time axis
            if df is not None and "Control_Time" in cols:
                time = df["Control_Time"].values
            elif df is not None and "CONTROL_DT" in cols:
                dt_val = df["CONTROL_DT"].iloc[0]
                time = np.arange(len(df)) * float(dt_val)
            else:
                time = np.arange(len(df) if df is not None else self._get_len()) * float(self.dt)
            
            # Get intervals
            if df is not None:
                intervals = df["Actual_Time_Interval"].values
            else:
                intervals = self._col("Actual_Time_Interval")
            
            # Determine target dt
            target_dt = self.dt
            if df is not None and "CONTROL_DT" in cols:
                target_dt = float(df["CONTROL_DT"].iloc[0])
            
            ax.plot(
                time,
                intervals,
                color="orange",
                linewidth=PlotStyle.LINEWIDTH,
                label="Actual Intervals",
            )
            ax.axhline(
                y=target_dt,
                color="r",
                linestyle="--",
                alpha=0.7,
                label=f"Target: {target_dt:.3f}s",
            )
            ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ax.set_ylabel("Time Interval (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ax.set_title(f"Timing Intervals - {self.system_title}")
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
            ax.set_title(f"Timing Intervals - {self.system_title}")
        
        PlotStyle.save_figure(fig, plot_dir / "timing_intervals.png")

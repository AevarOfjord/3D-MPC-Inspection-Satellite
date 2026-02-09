"""
Plot Generator for Visualization

Coordinates static plot generation for simulation analysis.

Most plotting implementations live in helper modules; this class keeps the
public plotting API and call ordering.
"""

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from satellite_control.config.models import AppConfig
from satellite_control.visualization.actuator_plots import (
    generate_actuator_limits_plot,
    generate_control_effort_plot,
    generate_pwm_quantization_plot,
    generate_reaction_wheel_output_plot,
    generate_thruster_impulse_proxy_plot,
    generate_thruster_usage_plot,
    generate_thruster_valve_activity_plot,
)
from satellite_control.visualization.command_utils import (
    get_thruster_count as infer_thruster_count,
)
from satellite_control.visualization.command_utils import (
    parse_command_vector as parse_command_vector_with_context,
)
from satellite_control.visualization.diagnostics_plots import (
    generate_mpc_performance_plot,
    generate_solver_health_plot,
    generate_timing_intervals_plot,
    generate_waypoint_progress_plot,
)
from satellite_control.visualization.plot_style import PlotStyle
from satellite_control.visualization.state_plots import (
    generate_constraint_violations_plot,
    generate_phase_attitude_rate_plot,
    generate_phase_position_velocity_plot,
    generate_velocity_magnitude_plot,
    generate_velocity_tracking_plot,
    generate_z_tilt_coupling_plot,
)
from satellite_control.visualization.trajectory_plots import (
    generate_trajectory_3d_interactive_plot,
    generate_trajectory_3d_orientation_plot,
    generate_trajectory_plot,
)


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
        app_config: AppConfig | None = None,
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

    def _row(self, idx: int) -> dict[str, Any]:
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
            self._col("Reference_X"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference X",
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
            self._col("Reference_Y"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference Y",
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
            self._col("Reference_Z"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference Z",
        )
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Z Position (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Z Position Tracking")

        PlotStyle.save_figure(fig, plot_dir / "02_tracking_position.png")

    def generate_position_error_plot(self, plot_dir: Path) -> None:
        """Generate position error plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Position Error - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        # Calculate errors
        error_x = self._col("Current_X") - self._col("Reference_X")
        error_y = self._col("Current_Y") - self._col("Reference_Y")
        error_z = self._col("Current_Z") - self._col("Reference_Z")

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

        PlotStyle.save_figure(fig, plot_dir / "02_error_position.png")

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
            np.degrees(self._col("Reference_Roll")),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference Roll",
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
            np.degrees(self._col("Reference_Pitch")),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference Pitch",
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
            np.degrees(self._col("Reference_Yaw")),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference Yaw",
        )
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Yaw (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Yaw Tracking")

        PlotStyle.save_figure(fig, plot_dir / "02_tracking_attitude.png")

    def generate_angular_error_plot(self, plot_dir: Path) -> None:
        """Generate angular error plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Angular Error - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        # Calculate errors
        error_roll = self._col("Current_Roll") - self._col("Reference_Roll")
        error_pitch = self._col("Current_Pitch") - self._col("Reference_Pitch")
        error_yaw = self._col("Current_Yaw") - self._col("Reference_Yaw")

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

        PlotStyle.save_figure(fig, plot_dir / "02_error_attitude.png")

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
        axes[0, 1].set_ylabel(
            "Velocity Error (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE
        )
        axes[0, 1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0, 1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[1, 0].plot(
            time,
            ang_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Attitude Error Norm",
        )
        axes[1, 0].set_ylabel(
            "Attitude Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE
        )
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
        axes[1, 1].set_ylabel(
            "Angular Rate Error (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE
        )
        axes[1, 1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1, 1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1, 1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "02_error_norms.png")

    def generate_trajectory_plot(self, plot_dir: Path) -> None:
        """Generate trajectory plot."""
        generate_trajectory_plot(self, plot_dir)

    def generate_trajectory_3d_interactive_plot(self, plot_dir: Path) -> None:
        """Generate interactive 3D trajectory plot (HTML)."""
        generate_trajectory_3d_interactive_plot(self, plot_dir)

    def generate_trajectory_3d_orientation_plot(self, plot_dir: Path) -> None:
        """Generate 3D trajectory plot with orientation arrows."""
        generate_trajectory_3d_orientation_plot(self, plot_dir)

    def generate_thruster_usage_plot(self, plot_dir: Path) -> None:
        """Generate thruster usage plot using actual valve states."""
        generate_thruster_usage_plot(self, plot_dir)

    def generate_thruster_valve_activity_plot(self, plot_dir: Path) -> None:
        """Generate detailed valve activity plot for each thruster (0.0 to 1.0)."""
        generate_thruster_valve_activity_plot(self, plot_dir)

    def generate_pwm_quantization_plot(self, plot_dir: Path) -> None:
        """Generate PWM duty cycle plot showing MPC u-values vs time."""
        generate_pwm_quantization_plot(self, plot_dir)

    def generate_pwm_duty_cycles_from_physics(self, plot_dir: Path) -> None:
        """Fallback: Generate PWM plot from physics_data Thruster_X_Cmd columns."""
        # This shows binary valve states, not continuous duty cycles
        pass

    def _get_thruster_count(self) -> int:
        """Determine thruster count based on available data or config."""
        return infer_thruster_count(self.data_accessor, self.app_config)

    def _parse_command_vector(self, command_str: Any) -> np.ndarray:
        """Parse command vector string to numpy array.

        Args:
            command_str: String representation of command vector (or any type)

        Returns:
            numpy array of thruster commands
        """
        return parse_command_vector_with_context(
            command_str, self.data_accessor, self.app_config
        )

    def generate_control_effort_plot(self, plot_dir: Path) -> None:
        """Generate control effort plot."""
        generate_control_effort_plot(self, plot_dir)

    def generate_reaction_wheel_output_plot(self, plot_dir: Path) -> None:
        """Generate reaction wheel torque output plot."""
        generate_reaction_wheel_output_plot(self, plot_dir)

    def generate_actuator_limits_plot(self, plot_dir: Path) -> None:
        """Generate actuator outputs with limit overlays."""
        generate_actuator_limits_plot(self, plot_dir)

    def generate_constraint_violations_plot(self, plot_dir: Path) -> None:
        """Generate constraint violation plot."""
        generate_constraint_violations_plot(self, plot_dir)

    def generate_z_tilt_coupling_plot(self, plot_dir: Path) -> None:
        """Generate Z-tilt coupling plot."""
        generate_z_tilt_coupling_plot(self, plot_dir)

    def generate_thruster_impulse_proxy_plot(self, plot_dir: Path) -> None:
        """Generate thruster impulse proxy plot."""
        generate_thruster_impulse_proxy_plot(self, plot_dir)

    def generate_phase_position_velocity_plot(self, plot_dir: Path) -> None:
        """Generate position vs velocity phase plots."""
        generate_phase_position_velocity_plot(self, plot_dir)

    def generate_phase_attitude_rate_plot(self, plot_dir: Path) -> None:
        """Generate attitude vs rate phase plots."""
        generate_phase_attitude_rate_plot(self, plot_dir)

    def generate_solver_health_plot(self, plot_dir: Path) -> None:
        """Generate solver health summary plot."""
        generate_solver_health_plot(self, plot_dir)

    def generate_waypoint_progress_plot(self, plot_dir: Path) -> None:
        """Generate waypoint/mission phase progress plot."""
        generate_waypoint_progress_plot(self, plot_dir)

    def generate_velocity_tracking_plot(self, plot_dir: Path) -> None:
        """Generate velocity tracking over time plot."""
        generate_velocity_tracking_plot(self, plot_dir)

    def generate_velocity_magnitude_plot(self, plot_dir: Path) -> None:
        """Generate velocity magnitude over time plot (speed vs time)."""
        generate_velocity_magnitude_plot(self, plot_dir)

    def generate_mpc_performance_plot(self, plot_dir: Path) -> None:
        """Generate MPC performance plot."""
        generate_mpc_performance_plot(self, plot_dir)

    def generate_timing_intervals_plot(self, plot_dir: Path) -> None:
        """Generate timing intervals plot."""
        generate_timing_intervals_plot(self, plot_dir)

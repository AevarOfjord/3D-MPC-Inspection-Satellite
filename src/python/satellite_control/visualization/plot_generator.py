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
from scipy.spatial.transform import Rotation

from satellite_control.config.models import AppConfig
from satellite_control.utils.orientation_utils import quat_angle_error
from satellite_control.visualization.actuator_plots import (
    generate_actuator_limits_plot,
    generate_command_vs_valve_tracking_plot,
    generate_control_effort_plot,
    generate_cumulative_impulse_delta_v_proxy_plot,
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
    generate_error_vs_solve_time_scatter_plot,
    generate_mpc_performance_plot,
    generate_obstacle_clearance_over_time_plot,
    generate_solver_health_plot,
    generate_solver_iterations_and_status_timeline_plot,
    generate_timing_intervals_plot,
    generate_waypoint_progress_plot,
)
from satellite_control.visualization.plot_style import PlotStyle
from satellite_control.visualization.state_plots import (
    generate_constraint_violations_plot,
    generate_phase_attitude_rate_plot,
    generate_phase_position_velocity_plot,
    generate_translation_attitude_coupling_plot,
    generate_velocity_magnitude_plot,
    generate_velocity_tracking_plot,
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

    def _get_quaternion_series(self, prefix: str) -> np.ndarray:
        """
        Get continuous quaternion series [N,4] in wxyz order.

        Prefers logged quaternion columns and falls back to Euler->quaternion
        conversion for legacy data files.
        """
        n = self._get_len()
        if n <= 0:
            return np.zeros((0, 4), dtype=float)

        qw = self._col(f"{prefix}_QW")
        qx = self._col(f"{prefix}_QX")
        qy = self._col(f"{prefix}_QY")
        qz = self._col(f"{prefix}_QZ")
        has_quat_cols = (
            len(qw) == n and len(qx) == n and len(qy) == n and len(qz) == n
        )

        if has_quat_cols:
            q = np.column_stack((qw, qx, qy, qz)).astype(float, copy=False)
        else:
            r = self._col(f"{prefix}_Roll")
            p = self._col(f"{prefix}_Pitch")
            y = self._col(f"{prefix}_Yaw")
            if len(r) != n or len(p) != n or len(y) != n:
                return np.zeros((0, 4), dtype=float)
            q_xyzw = Rotation.from_euler(
                "xyz", np.column_stack((r, p, y)), degrees=False
            ).as_quat()
            q = np.column_stack(
                (q_xyzw[:, 3], q_xyzw[:, 0], q_xyzw[:, 1], q_xyzw[:, 2])
            )

        # Normalize and enforce sign continuity (q and -q represent same rotation).
        norms = np.linalg.norm(q, axis=1, keepdims=True)
        norms[norms <= 1e-12] = 1.0
        q = q / norms
        for i in range(1, len(q)):
            if float(np.dot(q[i], q[i - 1])) < 0.0:
                q[i] = -q[i]
        return q

    def _get_euler_series_unwrapped(self, prefix: str) -> np.ndarray:
        """
        Get continuous Euler xyz series [N,3] in radians for display plots.

        Uses quaternion columns when available, then unwraps each component to avoid
        artificial +/-180 or 360 display discontinuities.
        """
        n = self._get_len()
        if n <= 0:
            return np.zeros((0, 3), dtype=float)

        q = self._get_quaternion_series(prefix)
        if len(q) == n:
            q_xyzw = np.column_stack((q[:, 1], q[:, 2], q[:, 3], q[:, 0]))
            e = Rotation.from_quat(q_xyzw).as_euler("xyz", degrees=False)
        else:
            r = self._col(f"{prefix}_Roll")
            p = self._col(f"{prefix}_Pitch")
            y = self._col(f"{prefix}_Yaw")
            if len(r) != n or len(p) != n or len(y) != n:
                return np.zeros((0, 3), dtype=float)
            e = np.column_stack((r, p, y)).astype(float, copy=False)

        return np.unwrap(e, axis=0)

    def generate_all_plots(self, plot_dir: Path) -> None:
        """
        Generate all performance analysis plots.

        Args:
            plot_dir: Directory to save plots
        """
        print("Generating performance analysis plots...")
        plot_dir.mkdir(parents=True, exist_ok=True)
        print(f" Created Plots directory: {plot_dir}")

        grouped_dirs = {
            "dashboard": plot_dir / "dashboard",
            "trajectory": plot_dir / "trajectory",
            "tracking": plot_dir / "tracking",
            "error": plot_dir / "error",
            "actuators": plot_dir / "actuators",
            "dynamics": plot_dir / "dynamics",
            "diagnostics": plot_dir / "diagnostics",
        }
        for out_dir in grouped_dirs.values():
            out_dir.mkdir(parents=True, exist_ok=True)

        # Generate specific performance plots
        self.generate_position_tracking_plot(grouped_dirs["tracking"])
        self.generate_position_error_plot(grouped_dirs["error"])
        self.generate_angular_tracking_plot(grouped_dirs["tracking"])
        self.generate_angular_error_plot(grouped_dirs["error"])
        self.generate_error_norms_plot(grouped_dirs["error"])
        self.generate_trajectory_plot(grouped_dirs["trajectory"])
        self.generate_trajectory_3d_interactive_plot(grouped_dirs["trajectory"])
        self.generate_trajectory_3d_orientation_plot(grouped_dirs["trajectory"])
        self.generate_thruster_usage_plot(grouped_dirs["actuators"])
        self.generate_thruster_valve_activity_plot(grouped_dirs["actuators"])
        self.generate_command_vs_valve_tracking_plot(grouped_dirs["actuators"])
        self.generate_pwm_quantization_plot(grouped_dirs["actuators"])
        self.generate_control_effort_plot(grouped_dirs["actuators"])
        self.generate_actuator_limits_plot(grouped_dirs["actuators"])
        self.generate_constraint_violations_plot(grouped_dirs["diagnostics"])
        self.generate_reaction_wheel_output_plot(grouped_dirs["actuators"])
        self.generate_translation_attitude_coupling_plot(grouped_dirs["dynamics"])
        self.generate_thruster_impulse_proxy_plot(grouped_dirs["actuators"])
        self.generate_cumulative_impulse_delta_v_proxy_plot(grouped_dirs["actuators"])
        self.generate_phase_position_velocity_plot(grouped_dirs["dynamics"])
        self.generate_phase_attitude_rate_plot(grouped_dirs["dynamics"])
        self.generate_velocity_tracking_plot(grouped_dirs["tracking"])
        self.generate_velocity_magnitude_plot(grouped_dirs["dynamics"])
        self.generate_quaternion_attitude_error_plot(grouped_dirs["diagnostics"])
        self.generate_mpc_performance_plot(grouped_dirs["diagnostics"])
        self.generate_solver_health_plot(grouped_dirs["diagnostics"])
        self.generate_solver_iterations_and_status_timeline_plot(
            grouped_dirs["diagnostics"]
        )
        self.generate_waypoint_progress_plot(grouped_dirs["diagnostics"])
        self.generate_obstacle_clearance_over_time_plot(grouped_dirs["diagnostics"])
        self.generate_error_vs_solve_time_scatter_plot(grouped_dirs["diagnostics"])
        self.generate_timing_intervals_plot(grouped_dirs["diagnostics"])

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

        PlotStyle.save_figure(fig, plot_dir / "position_tracking.png")

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

        PlotStyle.save_figure(fig, plot_dir / "position_error.png")

    def generate_angular_tracking_plot(self, plot_dir: Path) -> None:
        """Generate quaternion-component tracking plot."""
        fig, axes = plt.subplots(4, 1, figsize=(10, 10))
        fig.suptitle(f"Quaternion Tracking - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)
        q_cur = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        min_len = min(len(time), len(q_cur), len(q_ref))

        if min_len == 0:
            for ax in axes:
                ax.text(
                    0.5,
                    0.5,
                    "Quaternion tracking data\nnot available",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                    fontsize=PlotStyle.ANNOTATION_SIZE,
                )
                ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            PlotStyle.save_figure(fig, plot_dir / "attitude_tracking.png")
            return

        comp_labels = ("w", "x", "y", "z")
        for i, comp in enumerate(comp_labels):
            axes[i].plot(
                time[:min_len],
                q_cur[:min_len, i],
                color=PlotStyle.COLOR_SIGNAL_ANG,
                linewidth=PlotStyle.LINEWIDTH,
                label=f"Current q{comp}",
            )
            axes[i].plot(
                time[:min_len],
                q_ref[:min_len, i],
                color=PlotStyle.COLOR_TARGET,
                linestyle="--",
                linewidth=PlotStyle.LINEWIDTH,
                label=f"Reference q{comp}",
            )
            axes[i].set_ylabel(f"q{comp}", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            axes[i].grid(True, alpha=PlotStyle.GRID_ALPHA)
            axes[i].legend(fontsize=PlotStyle.LEGEND_SIZE)
            axes[i].set_title(f"q{comp} Tracking")

        axes[3].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "attitude_tracking.png")

    def generate_angular_error_plot(self, plot_dir: Path) -> None:
        """Generate quaternion-component error plot."""
        fig, axes = plt.subplots(4, 1, figsize=(10, 10))
        fig.suptitle(f"Quaternion Component Error - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)
        q_cur = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        min_len = min(len(time), len(q_cur), len(q_ref))

        if min_len == 0:
            for ax in axes:
                ax.text(
                    0.5,
                    0.5,
                    "Quaternion error data\nnot available",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                    fontsize=PlotStyle.ANNOTATION_SIZE,
                )
                ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            PlotStyle.save_figure(fig, plot_dir / "attitude_error.png")
            return

        # Use shortest-sign representation per sample.
        dot = np.sum(q_cur[:min_len] * q_ref[:min_len], axis=1)
        sign = np.where(dot < 0.0, -1.0, 1.0).reshape(-1, 1)
        q_err = q_cur[:min_len] - sign * q_ref[:min_len]

        comp_labels = ("w", "x", "y", "z")
        for i, comp in enumerate(comp_labels):
            axes[i].plot(
                time[:min_len],
                q_err[:, i],
                color=PlotStyle.COLOR_ERROR,
                linewidth=PlotStyle.LINEWIDTH,
                label=f"q{comp} Error",
            )
            axes[i].axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
            axes[i].set_ylabel(f"q{comp} err", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            axes[i].grid(True, alpha=PlotStyle.GRID_ALPHA)
            axes[i].legend(fontsize=PlotStyle.LEGEND_SIZE)
            axes[i].set_title(f"q{comp} Error")
        axes[3].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "attitude_error.png")

    def generate_quaternion_attitude_error_plot(self, plot_dir: Path) -> None:
        """Generate quaternion-geodesic attitude error (rotation-invariant)."""
        fig, axes = plt.subplots(2, 1, figsize=(10, 7))
        fig.suptitle(f"Quaternion Attitude Error - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)
        q_curr = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        min_len = min(len(time), len(q_curr), len(q_ref))
        if min_len == 0:
            for ax in axes:
                ax.text(
                    0.5,
                    0.5,
                    "Quaternion attitude data\nnot available",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                    fontsize=PlotStyle.ANNOTATION_SIZE,
                )
            PlotStyle.save_figure(fig, plot_dir / "attitude_error_quaternion.png")
            return

        q_err_deg = np.degrees(
            np.array(
                [
                    quat_angle_error(q_ref[i], q_curr[i])
                    for i in range(min_len)
                ],
                dtype=float,
            )
        )
        q_err_deg = np.nan_to_num(q_err_deg, nan=0.0, posinf=0.0, neginf=0.0)
        q_err_rate = np.gradient(q_err_deg, max(float(self.dt), 1e-9))

        axes[0].plot(
            time[:min_len],
            q_err_deg,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="SO(3) Angle Error",
        )
        axes[0].set_ylabel("Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("Quaternion Geodesic Error")

        axes[1].plot(
            time[:min_len],
            q_err_rate,
            color=PlotStyle.COLOR_SECONDARY,
            linewidth=PlotStyle.LINEWIDTH,
            label="d(Error)/dt",
        )
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_ylabel("deg/s", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "attitude_error_quaternion.png")

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
        err_wx = get_series("Error_WX")
        err_wy = get_series("Error_WY")
        err_wz = get_series("Error_WZ")

        pos_err_norm = np.sqrt(err_x**2 + err_y**2 + err_z**2)
        vel_err_norm = np.sqrt(err_vx**2 + err_vy**2 + err_vz**2)
        q_cur = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        q_len = min(len(time), len(q_cur), len(q_ref))
        if q_len > 0:
            ang_err_norm = np.degrees(
                np.array(
                    [quat_angle_error(q_ref[i], q_cur[i]) for i in range(q_len)],
                    dtype=float,
                )
            )
            if q_len < len(time):
                # Pad tail for mixed/legacy data lengths.
                pad = np.full(len(time) - q_len, ang_err_norm[-1], dtype=float)
                ang_err_norm = np.concatenate([ang_err_norm, pad])
        else:
            err_angle_rad = get_series("Error_Angle_Rad")
            if len(err_angle_rad) == len(time):
                ang_err_norm = np.degrees(err_angle_rad)
            else:
                ang_err_norm = np.zeros_like(time)
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

        PlotStyle.save_figure(fig, plot_dir / "error_norms.png")

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

    def generate_command_vs_valve_tracking_plot(self, plot_dir: Path) -> None:
        """Generate commanded-vs-actual valve tracking summary."""
        generate_command_vs_valve_tracking_plot(self, plot_dir)

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

    def generate_translation_attitude_coupling_plot(self, plot_dir: Path) -> None:
        """Generate frame-agnostic translation-attitude coupling plot."""
        generate_translation_attitude_coupling_plot(self, plot_dir)

    def generate_z_tilt_coupling_plot(self, plot_dir: Path) -> None:
        """Backward-compatible wrapper for legacy callsites."""
        generate_translation_attitude_coupling_plot(self, plot_dir)

    def generate_thruster_impulse_proxy_plot(self, plot_dir: Path) -> None:
        """Generate thruster impulse proxy plot."""
        generate_thruster_impulse_proxy_plot(self, plot_dir)

    def generate_cumulative_impulse_delta_v_proxy_plot(self, plot_dir: Path) -> None:
        """Generate cumulative impulse and delta-v proxy plot."""
        generate_cumulative_impulse_delta_v_proxy_plot(self, plot_dir)

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

    def generate_obstacle_clearance_over_time_plot(self, plot_dir: Path) -> None:
        """Generate obstacle clearance over time plot."""
        generate_obstacle_clearance_over_time_plot(self, plot_dir)

    def generate_velocity_tracking_plot(self, plot_dir: Path) -> None:
        """Generate velocity tracking over time plot."""
        generate_velocity_tracking_plot(self, plot_dir)

    def generate_velocity_magnitude_plot(self, plot_dir: Path) -> None:
        """Generate velocity magnitude over time plot (speed vs time)."""
        generate_velocity_magnitude_plot(self, plot_dir)

    def generate_mpc_performance_plot(self, plot_dir: Path) -> None:
        """Generate MPC performance plot."""
        generate_mpc_performance_plot(self, plot_dir)

    def generate_solver_iterations_and_status_timeline_plot(
        self, plot_dir: Path
    ) -> None:
        """Generate solver iterations and status timeline plot."""
        generate_solver_iterations_and_status_timeline_plot(self, plot_dir)

    def generate_error_vs_solve_time_scatter_plot(self, plot_dir: Path) -> None:
        """Generate error-vs-solve-time scatter plot."""
        generate_error_vs_solve_time_scatter_plot(self, plot_dir)

    def generate_timing_intervals_plot(self, plot_dir: Path) -> None:
        """Generate timing intervals plot."""
        generate_timing_intervals_plot(self, plot_dir)

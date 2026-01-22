"""
Video Renderer for Visualization

Provides video rendering functionality for animation generation.
Extracted from unified_visualizer.py to improve modularity.

This module handles all video/animation generation (MP4 output).
"""

import multiprocessing
import os
import sys
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

import imageio
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Circle

# V4.0.0: SatelliteConfig removed - use AppConfig/MissionState only
from src.satellite_control.config.mission_state import MissionState
from src.satellite_control.config.models import AppConfig
from src.satellite_control.visualization.unified_visualizer import PlotStyle

# Worker process cache for parallel frame rendering
_worker_gen_cache: Optional[Any] = None


class VideoRenderer:
    """
    Generates MP4 animations from simulation data.

    This class handles all video/animation generation, separating rendering
    logic from data management and plotting.
    """

    def __init__(
        self,
        data_accessor: Any,
        dt: float,
        fps: float,
        output_dir: Path,
        system_title: str = "Satellite Control System",
        speedup_factor: float = 1.0,
        satellite_size: float = 0.2,
        satellite_color: str = "blue",
        reference_color: str = "red",
        trajectory_color: str = "green",
        thrusters: Optional[Dict[int, List[float]]] = None,
        frame_title_template: str = "Frame {frame}",
        app_config: Optional[AppConfig] = None,
        mission_state: Optional[MissionState] = None,
    ):
        """
        Initialize video renderer.

        Args:
            data_accessor: Object with data access methods (_col, _row, _get_len)
            dt: Simulation timestep in seconds
            fps: Frame rate for video
            output_dir: Directory to save video
            system_title: Title for video
            speedup_factor: Animation speedup factor
            satellite_size: Size of satellite visualization
            satellite_color: Color of satellite
            reference_color: Color of reference
            trajectory_color: Color of trajectory
            thrusters: Dictionary of thruster positions
            dxf_base_shape: DXF base shape for overlay
            dxf_offset_path: DXF offset path for overlay
            dxf_center: DXF center point
            overlay_dxf: Whether to overlay DXF shape
            frame_title_template: Template for frame titles
            app_config: Optional AppConfig for accessing configuration (v4.0.0: no fallback)
            mission_state: Optional MissionState for accessing mission-specific data (v4.0.0: no fallback)
        """
        self.data_accessor = data_accessor
        self.dt = dt
        self.fps = fps
        self.output_dir = output_dir
        self.system_title = system_title
        self.speedup_factor = speedup_factor
        self.satellite_size = satellite_size
        self.satellite_color = satellite_color
        self.reference_color = reference_color
        self.trajectory_color = trajectory_color
        self.thrusters = thrusters or {}

        self.frame_title_template = frame_title_template

        # Store config references (v3.0.0)
        self.app_config = app_config
        self.mission_state = mission_state

        self.fig: Optional[Figure] = None
        self.ax_xy: Optional[Axes] = None
        self.ax_xz: Optional[Axes] = None
        self.ax_info: Optional[Axes] = None

    def _col(self, name: str) -> np.ndarray:
        """Get column data from data accessor."""
        return self.data_accessor._col(name)

    def _row(self, idx: int) -> Dict[str, Any]:
        """Get row data from data accessor."""
        return self.data_accessor._row(idx)

    def _get_len(self) -> int:
        """Get data length from data accessor."""
        return self.data_accessor._get_len()

    def _get_thruster_count(self) -> int:
        """Determine thruster count from config or fallback."""
        if self.app_config and self.app_config.physics:
            return len(self.app_config.physics.thruster_positions)
        return 8

    def parse_command_vector(self, command_str: Any) -> np.ndarray:
        """Parse command vector string to numpy array."""
        try:
            if command_str is None or command_str == "":
                return np.zeros(self._get_thruster_count())
            command_str = str(command_str)
            command_str = command_str.strip("[]")
            values = [float(x.strip()) for x in command_str.split(",")]
            return np.array(values)
        except Exception:
            return np.zeros(self._get_thruster_count())

    def get_active_thrusters(self, command_vector: np.ndarray) -> list:
        """Get list of active thruster IDs from command vector."""
        active = []
        for i, val in enumerate(command_vector):
            if abs(val) > 1e-6:  # Threshold for "active"
                active.append(i + 1)
        return active

    def setup_plot(self) -> None:
        """Set up the XY/XZ plots for animation."""
        self.fig, axes = plt.subplots(1, 2, figsize=(12, 8))
        self.ax_xy, self.ax_xz = axes
        self.fig.subplots_adjust(left=0.32, right=0.98, wspace=0.3)
        self.ax_info = self.fig.add_axes([0.03, 0.08, 0.26, 0.84])

    def draw_satellite(
        self, x: float, y: float, z: float, yaw: float, active_thrusters: list
    ) -> None:
        """Draw satellite at given position and orientation (XY + XZ)."""
        assert self.ax_xy is not None, "ax_xy must be initialized"
        assert self.ax_xz is not None, "ax_xz must be initialized"

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

        radius = self.satellite_size / 2
        circle_xy = Circle(
            (x, y),
            radius,
            facecolor=self.satellite_color,
            edgecolor="black",
            alpha=0.5,
            linewidth=0.5,
        )
        self.ax_xy.add_patch(circle_xy)

        circle_xz = Circle(
            (x, z),
            radius,
            facecolor=self.satellite_color,
            edgecolor="black",
            alpha=0.5,
            linewidth=0.5,
        )
        self.ax_xz.add_patch(circle_xz)

        # Draw thrusters in both projections
        for thruster_id, pos in self.thrusters.items():
            tx, ty, tz = pos[0], pos[1], pos[2] if len(pos) > 2 else 0.0

            rotated_thruster = np.array([tx, ty]) @ rotation_matrix.T
            thruster_x = x + rotated_thruster[0]
            thruster_y = y + rotated_thruster[1]
            thruster_z = z + tz

            if thruster_id in active_thrusters:
                color = "red"
                size = 80
                marker = "o"
                alpha = 1.0
            else:
                color = "gray"
                size = 20
                marker = "o"
                alpha = 0.3

            self.ax_xy.scatter(
                thruster_x,
                thruster_y,
                c=color,
                s=size,
                marker=marker,
                alpha=alpha,
                edgecolors="black",
                linewidth=0.5,
            )
            self.ax_xz.scatter(
                thruster_x,
                thruster_z,
                c=color,
                s=size,
                marker=marker,
                alpha=alpha,
                edgecolors="black",
                linewidth=0.5,
            )

        # Draw orientation arrow (Yaw shown in XY, forward axis in XZ)
        arrow_length = self.satellite_size * 1.5
        arrow_end_x = x + arrow_length * cos_yaw
        arrow_end_y = y + arrow_length * sin_yaw

        self.ax_xy.plot([x, arrow_end_x], [y, arrow_end_y], color="green", linewidth=2)
        self.ax_xz.plot(
            [x, x + arrow_length], [z, z], color="green", linewidth=2, alpha=0.7
        )

    def draw_reference(
        self, reference_x: float, reference_y: float, reference_z: float, reference_yaw: float
    ) -> None:
        """Draw reference position and orientation (XY + XZ)."""
        assert self.ax_xy is not None, "ax_xy must be initialized"
        assert self.ax_xz is not None, "ax_xz must be initialized"

        self.ax_xy.scatter(
            reference_x,
            reference_y,
            c=self.reference_color,
            s=200,
            marker="x",
            linewidth=4,
            label="Reference",
        )
        self.ax_xz.scatter(
            reference_x,
            reference_z,
            c=self.reference_color,
            s=200,
            marker="x",
            linewidth=4,
        )

        reference_radius = 0.1
        reference_xy = Circle(
            (reference_x, reference_y),
            reference_radius,
            fill=False,
            color=self.reference_color,
            alpha=0.5,
            linestyle="--",
            linewidth=1.5,
        )
        reference_xz = Circle(
            (reference_x, reference_z),
            reference_radius,
            fill=False,
            color=self.reference_color,
            alpha=0.5,
            linestyle="--",
            linewidth=1.5,
        )
        self.ax_xy.add_patch(reference_xy)
        self.ax_xz.add_patch(reference_xz)

        arrow_length = self.satellite_size * 0.6
        arrow_end_x = reference_x + arrow_length * np.cos(reference_yaw)
        arrow_end_y = reference_y + arrow_length * np.sin(reference_yaw)
        self.ax_xy.plot(
            [reference_x, arrow_end_x],
            [reference_y, arrow_end_y],
            color=self.reference_color,
            alpha=0.8,
            linewidth=2,
        )

    def draw_trajectory(
        self, trajectory_x: list, trajectory_y: list, trajectory_z: list
    ) -> None:
        """Draw satellite trajectory (XY + XZ)."""
        assert self.ax_xy is not None, "ax_xy must be initialized"
        assert self.ax_xz is not None, "ax_xz must be initialized"

        min_len = min(len(trajectory_x), len(trajectory_y), len(trajectory_z))
        if min_len > 1:
            self.ax_xy.plot(
                trajectory_x[:min_len],
                trajectory_y[:min_len],
                color=self.trajectory_color,
                linewidth=2,
                alpha=0.8,
                linestyle="-",
                label="Trajectory",
            )
            self.ax_xz.plot(
                trajectory_x[:min_len],
                trajectory_z[:min_len],
                color=self.trajectory_color,
                linewidth=2,
                alpha=0.8,
                linestyle="-",
            )

    def draw_obstacles(self, mission_state: Optional[MissionState] = None) -> None:
        """Draw obstacles if they are configured.

        Supports new V3.0.0 Obstacle objects (Sphere, Cylinder, Box).
        """
        assert self.ax_xy is not None, "ax_xy must be initialized"
        assert self.ax_xz is not None, "ax_xz must be initialized"

        obstacles = []
        # Try retrieving obstacles from data_accessor if available (usually UnifiedVisualizer)
        if (
            hasattr(self.data_accessor, "mission_state")
            and self.data_accessor.mission_state
            and getattr(self.data_accessor.mission_state, "obstacles", None)
        ):
            obstacles = self.data_accessor.mission_state.obstacles

        # Also check local mission_state arg or member
        state_to_use = mission_state or getattr(self, "mission_state", None)
        if state_to_use and getattr(state_to_use, "obstacles", None):
            obstacles = state_to_use.obstacles

        from matplotlib.patches import Rectangle, Circle

        for i, obs in enumerate(obstacles, 1):
            # Handle different obstacle formats
            # 1. New V3.0.0 Object format
            if hasattr(obs, "type") and hasattr(obs, "position"):
                pos = obs.position
                obs_type = getattr(obs.type, "value", obs.type)
                name = getattr(obs, "name", f"O{i}")

                # XY Projection
                if obs_type == "sphere":
                    radius = getattr(obs, "radius", 0.5)
                    self.ax_xy.add_patch(
                        Circle(
                            (pos[0], pos[1]), radius, color="red", alpha=0.3, zorder=15
                        )
                    )
                    self.ax_xz.add_patch(
                        Circle(
                            (pos[0], pos[2]), radius, color="red", alpha=0.3, zorder=15
                        )
                    )

                elif obs_type == "cylinder":
                    # Assume Z-axis aligned for now
                    radius = getattr(obs, "radius", 0.5)
                    self.ax_xy.add_patch(
                        Circle(
                            (pos[0], pos[1]),
                            radius,
                            color="orange",
                            alpha=0.3,
                            zorder=15,
                        )
                    )
                    # In XZ it looks like a rectangle (infinite or finite?)
                    # If finite, we need height. Assuming infinite for visualization or huge
                    self.ax_xz.add_patch(
                        Rectangle(
                            (pos[0] - radius, -10),
                            2 * radius,
                            20,
                            color="orange",
                            alpha=0.3,
                            zorder=15,
                        )
                    )

                elif obs_type == "box":
                    size = getattr(obs, "size", [1, 1, 1])
                    # XY: Rectangle centered at pos[0], pos[1] with size[0]*2, size[1]*2
                    self.ax_xy.add_patch(
                        Rectangle(
                            (pos[0] - size[0], pos[1] - size[1]),
                            size[0] * 2,
                            size[1] * 2,
                            color="magenta",
                            alpha=0.3,
                            zorder=15,
                        )
                    )
                    # XZ: Rectangle centered at pos[0], pos[2] with size[0]*2, size[2]*2
                    self.ax_xz.add_patch(
                        Rectangle(
                            (pos[0] - size[0], pos[2] - size[2]),
                            size[0] * 2,
                            size[2] * 2,
                            color="magenta",
                            alpha=0.3,
                            zorder=15,
                        )
                    )

            # 2. Legacy tuple format (x, y, z, r)
            elif isinstance(obs, (tuple, list)) and len(obs) >= 4:
                obs_x, obs_y, obs_z, obs_radius = obs[:4]
                self.ax_xy.add_patch(
                    Circle(
                        (obs_x, obs_y), obs_radius, color="red", alpha=0.3, zorder=15
                    )
                )
                self.ax_xz.add_patch(
                    Circle(
                        (obs_x, obs_z), obs_radius, color="red", alpha=0.3, zorder=15
                    )
                )

            # Add labels if possible (on XY)
            label_x = (
                pos[0]
                if "pos" in locals()
                else (obs[0] if isinstance(obs, (tuple, list)) else 0)
            )
            label_y = (
                pos[1]
                if "pos" in locals()
                else (obs[1] if isinstance(obs, (tuple, list)) else 0)
            )
            self.ax_xy.text(
                label_x,
                label_y,
                f"O{i}",
                fontsize=8,
                color="black",
                ha="center",
                va="center",
                zorder=16,
            )

    def update_info_panel(self, step: int, current_data: Any) -> None:
        """Update information panel with current data using professional styling."""
        assert self.ax_info is not None, "ax_info must be initialized"

        self.ax_info.clear()
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")

        # Professional Title
        self.ax_info.text(
            0.05,
            0.95,
            "System Telemetry",
            fontsize=14,
            weight="bold",
            color=PlotStyle.COLOR_PRIMARY,
            fontfamily=getattr(PlotStyle, "FONT_FAMILY", "serif"),
        )

        time = step * float(self.dt)

        # Determine Mission Phase and Metrics from Control Data
        mission_phase = ""
        mpc_solve_time = 0.0
        accumulated_usage_s = 0.0
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            import pandas as pd

            # Find closest control timestamp
            if (
                "Control_Time" in self.data_accessor.control_data.columns
                and "Mission_Phase" in self.data_accessor.control_data.columns
            ):
                # Ensure sorted
                idx = (
                    self.data_accessor.control_data["Control_Time"].searchsorted(
                        time, side="right"
                    )
                    - 1
                )
                idx = max(0, min(idx, len(self.data_accessor.control_data) - 1))
                row = self.data_accessor.control_data.iloc[idx]

                # Phase
                phase_val = row["Mission_Phase"]
                if phase_val is not None and str(phase_val).lower() != "nan":
                    mission_phase = str(phase_val)

                # Solver Time
                if "MPC_Solve_Time" in row and pd.notna(row["MPC_Solve_Time"]):
                    mpc_solve_time = float(row["MPC_Solve_Time"]) * 1000.0
                elif "MPC_Computation_Time" in row:
                    mpc_solve_time = (
                        float(row["MPC_Computation_Time"]) * 1000.0
                    )  # to ms

                # Accumulated Thruster Usage
                if "Accumulated_Usage_S" in row:
                    accumulated_usage_s = float(row["Accumulated_Usage_S"])

        # Display Logic: Refined Content
        header_lines = [f"Time: {time:.3f} s"]
        if mission_phase:
            header_lines.append(f"Phase: {mission_phase}")

        state_lines = [
            "STATE",
            f"  X:   {current_data['Current_X']:>7.3f} m",
            f"  Y:   {current_data['Current_Y']:>7.3f} m",
            f"  Z:   {current_data.get('Current_Z', 0.0):>7.3f} m",
            f"  Roll:  {np.degrees(current_data.get('Current_Roll', 0.0)):>6.1f}°",
            f"  Pitch: {np.degrees(current_data.get('Current_Pitch', 0.0)):>6.1f}°",
            f"  Yaw:   {np.degrees(current_data.get('Current_Yaw', 0.0)):>6.1f}°",
        ]

        lin_speed = current_data.get("Linear_Speed", 0.0)
        ang_speed = abs(np.degrees(current_data.get("Current_Angular_Vel", 0.0)))
        dynamics_lines = [
            "DYNAMICS",
            f"  Speed: {lin_speed:.3f} m/s",
            f"  Ang V: {ang_speed:.1f} °/s",
        ]

        err_x = abs(current_data["Error_X"])
        err_y = abs(current_data["Error_Y"])
        err_z = abs(current_data.get("Error_Z", 0.0))
        err_roll = abs(np.degrees(current_data.get("Error_Roll", 0.0)))
        err_pitch = abs(np.degrees(current_data.get("Error_Pitch", 0.0)))
        err_yaw = abs(np.degrees(current_data.get("Error_Yaw", 0.0)))

        error_lines = [
            "ERRORS",
            f"  Err X:   {err_x:.3f} m",
            f"  Err Y:   {err_y:.3f} m",
            f"  Err Z:   {err_z:.3f} m",
            f"  Err Roll:  {err_roll:.1f}°",
            f"  Err Pitch: {err_pitch:.1f}°",
            f"  Err Yaw:   {err_yaw:.1f}°",
        ]

        system_lines = [
            "SYSTEM",
            f"  Total Thruster Usage: {accumulated_usage_s:.1f} s",
            f"  Solver: {mpc_solve_time:.1f} ms",
        ]

        groups = [
            (header_lines, PlotStyle.COLOR_PRIMARY, None),
            (state_lines, PlotStyle.COLOR_SIGNAL_POS, 0),
            (dynamics_lines, PlotStyle.COLOR_SIGNAL_POS, 0),
            (error_lines, PlotStyle.COLOR_TARGET, 0),
            (system_lines, PlotStyle.COLOR_PRIMARY, 0),
        ]
        y_pos = 0.85
        line_height = 0.035
        group_spacing = 0.02

        font_family = getattr(PlotStyle, "FONT_FAMILY", "serif")

        for lines, color, bold_idx in groups:
            for i, line in enumerate(lines):
                weight = (
                    "bold" if (bold_idx is not None and i == bold_idx) else "normal"
                )
                size = 11 if weight == "bold" else 10

                self.ax_info.text(
                    0.05,
                    y_pos,
                    line,
                    fontsize=size,
                    weight=weight,
                    color=color,
                    verticalalignment="top",
                    fontfamily=font_family,
                )
                y_pos -= line_height
            y_pos -= group_spacing

    def animate_frame(self, frame: int) -> List[Any]:
        """Animation update function for each frame (XY + XZ)."""
        assert self.ax_xy is not None, "ax_xy must be initialized"
        assert self.ax_xz is not None, "ax_xz must be initialized"

        self.ax_xy.clear()
        self.ax_xz.clear()

        self.ax_xy.set_xlim(-3, 3)
        self.ax_xy.set_ylim(-3, 3)
        self.ax_xy.set_xlabel("X (m)")
        self.ax_xy.set_ylabel("Y (m)")
        self.ax_xy.set_title("X-Y Plane")
        self.ax_xy.set_aspect("equal", adjustable="box")

        self.ax_xz.set_xlim(-3, 3)
        self.ax_xz.set_ylim(-3, 3)
        self.ax_xz.set_xlabel("X (m)")
        self.ax_xz.set_ylabel("Z (m)")
        self.ax_xz.set_title("X-Z Plane")
        self.ax_xz.set_aspect("equal", adjustable="box")

        if self.fig is not None:
            self.fig.suptitle(self.frame_title_template.format(frame=frame))

        # Get current data
        step = min(int(frame * self.speedup_factor), self._get_len() - 1)
        current_data = self._row(step)

        # Parse command vector and get active thrusters
        command_vector = self.parse_command_vector(current_data["Command_Vector"])
        active_thrusters = self.get_active_thrusters(command_vector)

        # Type-safe extraction
        reference_x = float(current_data.get("Reference_X", 0.0) or 0.0)
        reference_y = float(current_data.get("Reference_Y", 0.0) or 0.0)
        reference_z = float(current_data.get("Reference_Z", 0.0) or 0.0)
        reference_yaw = float(current_data.get("Reference_Yaw", 0.0) or 0.0)

        self.draw_reference(reference_x, reference_y, reference_z, reference_yaw)

        # Draw trajectory
        traj_x = self._col("Current_X")[: step + 1].tolist()
        traj_y = self._col("Current_Y")[: step + 1].tolist()
        if "Current_Z" in current_data:
            traj_z = self._col("Current_Z")[: step + 1].tolist()
        else:
            traj_z = [0.0] * len(traj_x)
        self.draw_trajectory(traj_x, traj_y, traj_z)

        # Draw satellite
        curr_x = float(current_data.get("Current_X", 0.0) or 0.0)
        curr_y = float(current_data.get("Current_Y", 0.0) or 0.0)
        curr_z = float(current_data.get("Current_Z", 0.0) or 0.0)
        curr_yaw = float(current_data.get("Current_Yaw", 0.0) or 0.0)

        self.draw_satellite(
            curr_x,
            curr_y,
            curr_z,
            curr_yaw,
            active_thrusters,
        )

        # Draw obstacles
        self.draw_obstacles()

        # Add legend
        self.ax_xy.legend(loc="upper right", fontsize=9)

        # Update info panel
        self.update_info_panel(step, current_data)

        return []

    def generate_animation(self, output_filename: Optional[str] = None) -> None:
        """Generate and save the MP4 animation using parallel rendering + imageio."""
        if output_filename is None:
            output_filename = "animation.mp4"

        print(f"\n{'=' * 60}")
        print(f"GENERATING {self.system_title.upper()} ANIMATION (PARALLEL)")
        print(f"{'=' * 60}")

        # Calculate number of frames
        total_frames = self._get_len() // int(self.speedup_factor)
        if total_frames == 0:
            total_frames = 1

        print("Animation parameters:")
        print(f"  - Total data points: {self._get_len()}")
        print(f"  - Animation frames: {total_frames}")
        print(f"  - Frame rate: {self.fps} FPS")
        print(f"  - Speedup factor: {self.speedup_factor}x")
        print(f"  - Animation duration: {total_frames / self.fps:.1f} seconds")

        # Prepare configuration for workers
        if (
            hasattr(self.data_accessor, "_data_backend")
            and self.data_accessor._data_backend == "pandas"
        ):
            if (
                hasattr(self.data_accessor, "data")
                and self.data_accessor.data is not None
            ):
                data_dict = {
                    col: self.data_accessor.data[col].values
                    for col in self.data_accessor.data.columns
                }
        elif (
            hasattr(self.data_accessor, "_col_data")
            and self.data_accessor._col_data is not None
        ):
            data_dict = self.data_accessor._col_data
        else:
            raise ValueError("No data available for animation")

        # Config dictionary to reconstruct state in workers
        config = {
            "dt": self.dt,
            "fps": self.fps,
            "speedup_factor": self.speedup_factor,
            "system_title": self.system_title,
            "frame_title_template": self.frame_title_template,
            "satellite_size": self.satellite_size,
            "satellite_color": self.satellite_color,
            "reference_color": self.reference_color,
            "trajectory_color": self.trajectory_color,
            "thrusters": self.thrusters,
            "obstacles_enabled": (
                self.mission_state.obstacles_enabled
                if self.mission_state
                else False  # V4.0.0: No fallback
            ),
            "obstacles_list": (
                list(self.mission_state.obstacles)
                if self.mission_state and self.mission_state.obstacles
                else []  # V4.0.0: No fallback
            ),
            "data_directory": str(
                getattr(self.data_accessor, "data_directory", Path("."))
            ),
        }

        # Pass sibling control data if available
        if (
            hasattr(self.data_accessor, "control_data")
            and self.data_accessor.control_data is not None
        ):
            config["control_data_dict"] = self.data_accessor.control_data.to_dict(
                orient="list"
            )
        else:
            config["control_data_dict"] = None

        # Use absolute path to ensure file is saved correctly
        output_path = Path(self.output_dir).resolve() / output_filename

        # Use temp directory in output folder
        frames_dir = Path(self.output_dir).resolve() / f"_frames_{uuid.uuid4().hex[:8]}"
        frames_dir.mkdir(parents=True, exist_ok=True)

        try:
            print(
                f"\nRendering {total_frames} frames using "
                f"{multiprocessing.cpu_count()} cores..."
            )

            frames = list(range(total_frames))

            # Parallel frame rendering
            temp_dir = str(frames_dir)
            from concurrent.futures import ProcessPoolExecutor

            with ProcessPoolExecutor() as executor:
                results_iterator = executor.map(
                    VideoRenderer._render_frame_task,
                    [(f, temp_dir, data_dict, config) for f in frames],
                )

                try:
                    from tqdm import tqdm

                    futures = list(
                        tqdm(
                            results_iterator,
                            total=total_frames,
                            unit="frames",
                            desc="Rendering Frames",
                            ncols=80,
                        )
                    )
                except ImportError:
                    futures = []
                    print("Rendering frames (tqdm not installed)...")
                    for i, res in enumerate(results_iterator):
                        futures.append(res)
                        if i % 10 == 0 or i == total_frames - 1:
                            sys.stdout.write(
                                f"\rProcessed {i + 1}/{total_frames} frames"
                            )
                            sys.stdout.flush()
                    print()

            # Assemble video with imageio
            print("Assembling video with imageio...")

            fps_for_video = max(1.0, self.fps)

            # Ensure imageio finds the system ffmpeg
            import shutil

            ffmpeg_path = shutil.which("ffmpeg")
            if ffmpeg_path:
                os.environ["IMAGEIO_FFMPEG_EXE"] = ffmpeg_path

            try:
                with imageio.get_writer(
                    str(output_path),
                    fps=fps_for_video,
                    macro_block_size=1,
                    quality=8,
                ) as writer:
                    for frame_idx in range(total_frames):
                        frame_path = os.path.join(
                            temp_dir, f"frame_{frame_idx:05d}.png"
                        )
                        if os.path.exists(frame_path):
                            frame = imageio.imread(frame_path)
                            writer.append_data(frame)  # type: ignore[attr-defined]

                print("\n Animation saved successfully!")
                print(f" File location: {output_path}")
            except Exception as e:
                print(f"Error assembling video: {e}")
                raise
        finally:
            # Clean up frames directory
            import shutil

            if frames_dir.exists():
                shutil.rmtree(frames_dir)

    @staticmethod
    def _render_frame_task(args):
        """Worker function to render a single frame (runs in separate process)."""
        frame_idx, output_dir, data_dict, config = args

        try:
            global _worker_gen_cache
            if _worker_gen_cache is None:
                # Create a simple data accessor for worker process
                class WorkerDataAccessor:
                    def __init__(self, data_dict):
                        self._col_data = data_dict
                        self._data_backend = "csv"
                        self.control_data = None

                    def _col(self, name: str) -> np.ndarray:
                        return self._col_data.get(name, np.array([]))

                    def _row(self, idx: int) -> Dict[str, Any]:
                        return {
                            k: (self._col_data[k][idx] if k in self._col_data else None)
                            for k in self._col_data.keys()
                        }

                    def _get_len(self) -> int:
                        if self._col_data:
                            first_key = next(iter(self._col_data))
                            return len(self._col_data[first_key])
                        return 0

                # Create data accessor
                data_accessor = WorkerDataAccessor(data_dict)

                # Restore control data if provided
                if config.get("control_data_dict"):
                    try:
                        import pandas as pd

                        data_accessor.control_data = pd.DataFrame(
                            config["control_data_dict"]
                        )
                    except ImportError:
                        pass

                # Create renderer
                renderer = VideoRenderer(
                    data_accessor=data_accessor,
                    dt=config["dt"],
                    fps=config["fps"],
                    output_dir=Path(config["data_directory"]),
                    system_title=config["system_title"],
                    speedup_factor=config["speedup_factor"],
                    satellite_size=config["satellite_size"],
                    satellite_color=config["satellite_color"],
                    reference_color=config["reference_color"],
                    trajectory_color=config["trajectory_color"],
                    thrusters=config["thrusters"],
                    frame_title_template=config["frame_title_template"],
                )

                # V4.0.0: Config state is passed via config dict only (no global state mutation)
                # Worker process uses config dict values directly

                renderer.setup_plot()
                _worker_gen_cache = renderer
            else:
                renderer = _worker_gen_cache

            if renderer is not None:
                renderer.animate_frame(frame_idx)
                filename = os.path.join(output_dir, f"frame_{frame_idx:05d}.png")
                if renderer.fig is not None:
                    renderer.fig.savefig(filename, dpi=100)
        except Exception:
            import traceback

            traceback.print_exc()
            return False

        return True

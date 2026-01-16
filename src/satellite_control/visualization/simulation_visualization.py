"""
Simulation Visualization Manager for Satellite Control System

Handles matplotlib-based visualization and animation for simulation results.
Provides real-time drawing and high-quality MP4 animation generation.

Visualization components:
- Satellite body with thruster indicators
- Target positions and waypoints
- Trajectory paths with color-coded segments
- Circular obstacles with safety margins
- Thruster activation indicators
- Real-time state information display

Animation capabilities:
- Frame-by-frame simulation playback
- High-quality MP4 video export
- Configurable frame rate and resolution
- Progress bar with time estimates
- Automatic legend and axis scaling

Post-processing visualization:
- Complete trajectory plots
- State history graphs
- Control effort analysis
- Integration with visualize.py for comprehensive reports

Key features:
- Modular design separated from simulation logic
- Efficient frame rendering with matplotlib
- Memory-optimized for long simulations
- Consistent styling with real hardware visualizations
"""

# Import logging
import logging
from pathlib import Path

import matplotlib.patches as patches
import numpy as np
from matplotlib.animation import writers

# Conditional import for visualization generator
try:
    from src.satellite_control.visualization.unified_visualizer import (
        UnifiedVisualizationGenerator,
    )
except ImportError:
    UnifiedVisualizationGenerator = None  # type: ignore[assignment,misc]
logger = logging.getLogger(__name__)


class ProgressSuppressor:
    """
    Output stream wrapper that suppresses verbose output but allows progress bars.

    Filters out most matplotlib output while preserving progress-related output
    (lines with Encoding, Progress, etc.)
    """

    def __init__(self, stream):
        self.stream = stream
        self.progress_keywords = ["Encoding:", "Progress:", "█", "░", "%"]
        self.last_was_progress = False

    def write(self, text: str) -> None:
        """Write output, filtering out verbose messages."""
        # Check if this is progress bar output
        is_progress = any(keyword in text for keyword in self.progress_keywords)

        if is_progress:
            # Always output progress bar text
            self.stream.write(text)
            self.stream.flush()
            self.last_was_progress = True
        elif text == "\r":
            # Always allow carriage returns (used by progress bar)
            self.stream.write(text)
            self.stream.flush()
        elif text == "\n" and self.last_was_progress:
            # Only allow newlines that come after progress bar output
            self.stream.write(text)
            self.stream.flush()
            self.last_was_progress = False
        else:
            # Suppress other output
            self.last_was_progress = False

    def flush(self) -> None:
        """Flush the underlying stream."""
        self.stream.flush()

    def isatty(self) -> bool:
        """Check if stream is a TTY."""
        return hasattr(self.stream, "isatty") and self.stream.isatty()


class SimulationVisualizationManager:
    """
    Manages all visualization and animation for simulation mode.

    Handles:
    - Real-time matplotlib drawing
    - Satellite, target, and trajectory rendering
    - Animation export to MP4
    - Post-test visualization generation
    """

    def __init__(self, controller):
        """
        Initialize visualization manager.

        Args:
            controller: Reference to parent SatelliteMPCLinearizedSimulation instance
        """
        self.controller = controller
        # Convenient aliases to controller attributes
        self.satellite = controller.satellite
        self.target_state = None  # Will be updated via controller
        self.position_tolerance = None
        self.simulation_time = None
        self.state_history = None
        self.is_running = None
        self.mpc_solve_times = None
        self.max_simulation_time = None
        self.data_save_path = None

        # Get config from controller's simulation_config (v3.0.0)
        # In V3.0.0, controller should always have simulation_config
        if hasattr(controller, "simulation_config") and controller.simulation_config:
            self.app_config = controller.simulation_config.app_config
            self.mission_state = controller.simulation_config.mission_state
        else:
            # V4.0.0: Create defaults if not provided (no SatelliteConfig fallback)
            from src.satellite_control.config.simulation_config import SimulationConfig
            from src.satellite_control.config.mission_state import create_mission_state

            default_config = SimulationConfig.create_default()
            self.app_config = default_config.app_config
            self.mission_state = create_mission_state()

    def sync_from_controller(self):
        """Sync attributes from controller before each draw operation."""
        c = self.controller
        self.target_state = c.target_state
        self.position_tolerance = c.position_tolerance
        self.simulation_time = c.simulation_time
        self.state_history = c.state_history
        self.is_running = c.is_running
        self.mpc_solve_times = c.mpc_solve_times
        self.max_simulation_time = c.max_simulation_time
        self.data_save_path = c.data_save_path
        self.control_history = c.control_history
        self.mpc_info_history = c.mpc_info_history
        self.angle_tolerance = c.angle_tolerance
        self.control_update_interval = c.control_update_interval
        self.target_reached_time = c.target_reached_time
        self.target_maintenance_time = c.target_maintenance_time
        self.maintenance_position_errors = c.maintenance_position_errors
        self.maintenance_angle_errors = c.maintenance_angle_errors
        self.times_lost_target = c.times_lost_target

    def angle_difference(self, target_angle, current_angle):
        """Delegate to controller's angle_difference method."""
        return self.controller.angle_difference(target_angle, current_angle)

    def get_current_state(self):
        """Delegate to controller's get_current_state method."""
        return self.controller.get_current_state()

    def check_target_reached(self):
        """Delegate to controller's check_target_reached method."""
        return self.controller.check_target_reached()

    def save_animation_mp4(self, fig, ani):
        """Save the animation as MP4 file."""
        if not self.data_save_path:
            return

        mp4_path = self.data_save_path / "simulation_animation.mp4"

        try:
            # Set up the writer
            Writer = writers["ffmpeg"]
            writer = Writer(
                fps=10,
                metadata=dict(artist="Linearized MPC Simulation"),
                bitrate=1800,
            )

            # Save animation
            ani.save(str(mp4_path), writer=writer)
            logger.info(f"Animation saved to: {mp4_path}")
        except Exception as e:
            logger.error(f"ERROR: Error saving animation: {e}")
            logger.error(
                "WARNING: Make sure ffmpeg is installed: "
                "brew install ffmpeg (Mac) or apt install ffmpeg (Linux)"
            )

    def draw_simulation(self):
        """Draw the simulation including satellite, target, and trajectory."""
        # Store current axis limits to restore them after clearing
        xlim = self.satellite.ax_main.get_xlim()
        ylim = self.satellite.ax_main.get_ylim()

        # Clear the axes
        self.satellite.ax_main.clear()

        # Immediately restore axis properties to prevent any reset issues
        self.satellite.ax_main.set_xlim(xlim)
        self.satellite.ax_main.set_ylim(ylim)
        self.satellite.ax_main.set_aspect("equal")
        self.satellite.ax_main.grid(True, alpha=0.3)
        self.satellite.ax_main.set_xlabel(
            "X Position (m)", fontsize=12, fontweight="bold"
        )
        self.satellite.ax_main.set_ylabel(
            "Y Position (m)", fontsize=12, fontweight="bold"
        )

        # Draw target elements FIRST with highest z-order
        if self.target_state is not None:
            # Extract 2D components from 3D target
            target_x = self.target_state[0]
            target_y = self.target_state[1]
            # Yaw from Quat [w, x, y, z] -> [3:7]
            q_t = self.target_state[3:7]
            target_angle = np.arctan2(
                2 * (q_t[0] * q_t[3] + q_t[1] * q_t[2]),
                1 - 2 * (q_t[2] ** 2 + q_t[3] ** 2),
            )
        else:
            target_x, target_y, target_angle = 0.0, 0.0, 0.0

        # Target tolerance circle
        target_circle = patches.Circle(
            (target_x, target_y),
            self.position_tolerance,  # type: ignore[arg-type]
            fill=False,
            color="green",
            linestyle="--",
            alpha=0.5,
            zorder=20,
        )
        target_circle.set_zorder(20)
        self.satellite.ax_main.add_patch(target_circle)

        # Target orientation arrow
        target_arrow_length = 0.2
        target_arrow_end = np.array(
            [target_x, target_y]
        ) + target_arrow_length * np.array([np.cos(target_angle), np.sin(target_angle)])
        arrow = self.satellite.ax_main.annotate(
            "",
            xy=target_arrow_end,
            xytext=[target_x, target_y],
            arrowprops=dict(arrowstyle="->", lw=3, color="darkgreen", alpha=0.8),
        )
        arrow.set_zorder(21)

        # Target position marker
        self.satellite.ax_main.plot(
            target_x,
            target_y,
            "ro",
            markersize=12,
            markerfacecolor="red",
            markeredgecolor="darkred",
            linewidth=2,
            zorder=22,
        )

        self._draw_satellite_elements()

        # Update title with current metrics
        current_state = self.get_current_state()

        # Extract Yaw
        q_c = current_state[3:7]
        curr_yaw = np.arctan2(
            2 * (q_c[0] * q_c[3] + q_c[1] * q_c[2]), 1 - 2 * (q_c[2] ** 2 + q_c[3] ** 2)
        )

        if self.target_state is not None:
            pos_error = float(np.linalg.norm(current_state[:3] - self.target_state[:3]))

            # Recalculate target angle for error
            q_t = self.target_state[3:7]
            targ_yaw = np.arctan2(
                2 * (q_t[0] * q_t[3] + q_t[1] * q_t[2]),
                1 - 2 * (q_t[2] ** 2 + q_t[3] ** 2),
            )

            ang_error = abs(self.angle_difference(targ_yaw, curr_yaw))
        else:
            pos_error = 0.0
            ang_error = 0.0

        title = "Linearized MPC Simulation\n"
        ang_deg = np.degrees(ang_error)
        title += (
            f"Time: {self.simulation_time:.1f}s | Pos Error: {pos_error:.3f}m "
            f"| Ang Error: {ang_deg:.1f}°"
        )

        self.satellite.ax_main.set_title(title, fontsize=14, fontweight="bold")

    def _draw_obstacles(self):
        """Draw configured obstacles on the visualization."""
        # Use mission_state for obstacles (v3.0.0)
        obstacles_enabled = False
        obstacles = []
        if self.mission_state:
            obstacles_enabled = self.mission_state.obstacles_enabled
            obstacles = (
                list(self.mission_state.obstacles)
                if self.mission_state.obstacles
                else []
            )
        else:
            # V4.0.0: No fallback - if no mission_state, obstacles are empty
            obstacles_enabled = False
            obstacles = []

        if obstacles_enabled and obstacles:
            for i, (obs_x, obs_y, obs_z, obs_radius) in enumerate(obstacles, 1):
                # Draw obstacle as red circle (XY projection)
                obstacle_circle = patches.Circle(
                    (obs_x, obs_y),
                    obs_radius,
                    fill=True,
                    color="red",
                    alpha=0.6,
                    zorder=8,
                )
                self.satellite.ax_main.add_patch(obstacle_circle)

                # Draw safety zone as red dashed circle
                # Safety margin: use default 0.1m (v3.0.0: could be added to AppConfig if needed)
                safety_margin = 0.1
                safety_radius = obs_radius + safety_margin
                safety_circle = patches.Circle(
                    (obs_x, obs_y),
                    safety_radius,
                    fill=False,
                    color="red",
                    linestyle="--",
                    alpha=0.4,
                    zorder=9,
                )
                self.satellite.ax_main.add_patch(safety_circle)

                # Add obstacle label
                self.satellite.ax_main.text(
                    obs_x,
                    obs_y,
                    f"O{i}",
                    fontsize=10,
                    color="white",
                    ha="center",
                    va="center",
                    fontweight="bold",
                    zorder=24,
                )

    def _draw_obstacle_avoidance_waypoints(self):
        """Draw obstacle avoidance waypoints."""
        waypoints = getattr(self, "obstacle_avoiding_waypoints", None)
        if waypoints is None or len(waypoints) == 0:
            return

        current_idx = getattr(self, "current_obstacle_waypoint_idx", 0)

        # Draw waypoint path
        for i in range(len(waypoints) - 1):
            start_pos = waypoints[i]
            end_pos = waypoints[i + 1]

            # Color based on completion status
            if i < current_idx:
                color = "green"  # Completed
                alpha = 0.8
            elif i == current_idx:
                color = "orange"  # Current
                alpha = 1.0
            else:
                color = "magenta"  # Future (obstacle avoidance path)
                alpha = 0.6

            # Draw line segment
            self.satellite.ax_main.plot(
                [start_pos[0], end_pos[0]],
                [start_pos[1], end_pos[1]],
                color=color,
                linewidth=4,
                alpha=alpha,
                zorder=13,
            )

        # Draw waypoint markers
        for i, waypoint in enumerate(waypoints):
            if i < current_idx:
                color = "green"
                marker = "s"  # Square for completed
                size = 10
                alpha = 0.8
            elif i == current_idx:
                color = "orange"
                marker = "D"  # Diamond for current
                size = 14
                alpha = 1.0
            else:
                color = "magenta"
                marker = "s"  # Square for future
                size = 10
                alpha = 0.6

            self.satellite.ax_main.plot(
                waypoint[0],
                waypoint[1],
                marker,
                color=color,
                markersize=size,
                alpha=alpha,
                zorder=19,
            )

    def _draw_satellite_elements(self):
        """Draw satellite elements manually to avoid conflicts."""
        cos_angle = np.cos(self.satellite.angle)
        sin_angle = np.sin(self.satellite.angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

        # Draw trajectory trail
        if len(self.satellite.trajectory) > 1:
            trajectory_array = np.array(self.satellite.trajectory)
            alphas = np.linspace(0.1, 0.6, len(self.satellite.trajectory))

            for i in range(len(self.satellite.trajectory) - 1):
                self.satellite.ax_main.plot(
                    [trajectory_array[i, 0], trajectory_array[i + 1, 0]],
                    [trajectory_array[i, 1], trajectory_array[i + 1, 1]],
                    "b-",
                    alpha=alphas[i],
                    linewidth=1,
                    zorder=1,
                )

        radius = self.satellite.satellite_size / 2
        satellite_patch = patches.Circle(
            (self.satellite.position[0], self.satellite.position[1]),
            radius=radius,
            linewidth=3,
            edgecolor="black",
            facecolor="lightgray",
            alpha=0.7,
            zorder=5,
        )
        self.satellite.ax_main.add_patch(satellite_patch)

        # Draw thrusters with color coding
        for thruster_id, local_pos in self.satellite.thrusters.items():
            global_pos = (
                rotation_matrix @ np.array(local_pos)[:2] + self.satellite.position[:2]
            )
            color = self.satellite.thruster_colors[thruster_id]
            is_active = thruster_id in self.satellite.active_thrusters

            if is_active:
                thruster_size = 40
                alpha = 1.0
            else:
                thruster_size = 20
                alpha = 0.3

            self.satellite.ax_main.scatter(
                global_pos[0],
                global_pos[1],
                s=thruster_size,
                c=color,
                alpha=alpha,
                zorder=10,
            )

    def update_mpc_info_panel(self):
        """Update the information panel to match Visualize_Simulation_Linearized copy.py format."""
        self.satellite.ax_info.clear()
        self.satellite.ax_info.set_xlim(0, 1)
        self.satellite.ax_info.set_ylim(0, 1)
        self.satellite.ax_info.axis("off")
        self.satellite.ax_info.set_title("Simulation Info", fontsize=12, weight="bold")

        # Get current state and calculate values
        current_state = self.get_current_state()
        net_force, net_torque = self.satellite.calculate_forces_and_torques()

        # Extract 2D/3D components
        curr_x, curr_y = current_state[0], current_state[1]
        q_c = current_state[3:7]
        curr_yaw = np.arctan2(
            2 * (q_c[0] * q_c[3] + q_c[1] * q_c[2]), 1 - 2 * (q_c[2] ** 2 + q_c[3] ** 2)
        )

        # Calculate errors
        if self.target_state is not None:
            targ_x, targ_y = self.target_state[0], self.target_state[1]
            q_t = self.target_state[3:7]
            targ_yaw = np.arctan2(
                2 * (q_t[0] * q_t[3] + q_t[1] * q_t[2]),
                1 - 2 * (q_t[2] ** 2 + q_t[3] ** 2),
            )

            pos_error = float(np.linalg.norm(current_state[:2] - self.target_state[:2]))
            angle_error = abs(np.degrees(self.angle_difference(targ_yaw, curr_yaw)))
            target_x_str = f"X: {targ_x:.3f} m"
            target_y_str = f"Y: {targ_y:.3f} m"
            target_yaw_str = f"Yaw: {np.degrees(targ_yaw):.1f}°"
        else:
            pos_error = 0.0
            angle_error = 0.0
            target_x_str = "X: N/A"
            target_y_str = "Y: N/A"
            target_yaw_str = "Yaw: N/A"

        # Get active thrusters
        active_thrusters = (
            list(sorted(self.satellite.active_thrusters))
            if self.satellite.active_thrusters
            else []
        )

        mpc_time = self.mpc_solve_times[-1] if self.mpc_solve_times else 0.0
        mpc_status = "Active" if self.is_running else "Stopped"

        # Current step estimation
        current_step = int(self.simulation_time / self.satellite.dt)
        max_steps = None
        if self.max_simulation_time and self.max_simulation_time > 0:
            max_steps = int(self.max_simulation_time / self.satellite.dt)

        # Information text matching the visualization format
        info_text = [
            "LINEARIZED MPC SIMULATION",
            f"{'=' * 25}",
            f"Step: {current_step}/{max_steps if max_steps is not None else 'unlimited'}",
            f"Time: {self.simulation_time:.1f}s",
            "",
            "CURRENT STATE:",
            f"X: {curr_x:.3f} m",
            f"Y: {curr_y:.3f} m",
            f"Yaw: {np.degrees(curr_yaw):.1f}°",
            "",
            "TARGET STATE:",
            target_x_str,
            target_y_str,
            target_yaw_str,
            "",
            "CONTROL ERRORS:",
            f"Position: {pos_error:.3f} m",
            f"Angle: {angle_error:.1f}°",
            "",
            "MPC CONTROLLER:",
            f"Status: {mpc_status}",
            f"Solve Time: {mpc_time:.3f}s",
            "",
            "THRUSTER CONTROL:",
            f"Active: {active_thrusters}",
            f"Count: {len(active_thrusters)}/8",
        ]

        # Display text with the same formatting as visualization
        y_pos = 0.95
        for line in info_text:
            if line.startswith("="):
                weight = "normal"
                size = 9
            elif line.isupper() and line.endswith(":"):
                weight = "bold"
                size = 10
            elif line.startswith(("LINEARIZED MPC", "Step:", "Time:")):
                weight = "bold"
                size = 11
            else:
                weight = "normal"
                size = 9

            self.satellite.ax_info.text(
                0.05,
                y_pos,
                line,
                fontsize=size,
                weight=weight,
                verticalalignment="top",
                fontfamily="monospace",
            )
            y_pos -= 0.035

    def print_performance_summary(self):
        """Print comprehensive simulation performance summary for Linearized MPC."""
        if not self.state_history:
            return

        # Print brief success message
        final_state = self.get_current_state()
        if self.target_state is not None:
            pos_error_final = float(
                np.linalg.norm(final_state[:2] - self.target_state[:2])
            )

            q_c = final_state[3:7]
            curr_yaw = np.arctan2(
                2 * (q_c[0] * q_c[3] + q_c[1] * q_c[2]),
                1 - 2 * (q_c[2] ** 2 + q_c[3] ** 2),
            )

            q_t = self.target_state[3:7]
            targ_yaw = np.arctan2(
                2 * (q_t[0] * q_t[3] + q_t[1] * q_t[2]),
                1 - 2 * (q_t[2] ** 2 + q_t[3] ** 2),
            )

            ang_error_final = abs(self.angle_difference(targ_yaw, curr_yaw))
        else:
            pos_error_final = 0.0
            ang_error_final = 0.0
        success = self.check_target_reached()

        print(f"\n{'=' * 60}")
        print(" MISSION COMPLETE!")
        print(f"{'=' * 60}")
        print(f"Status:            {' SUCCESS' if success else '  INCOMPLETE'}")
        print(
            f"Final Pos Error:   {pos_error_final:.4f} m (target: <{self.position_tolerance:.3f} m)"
        )
        ang_tol_deg = np.degrees(self.angle_tolerance)
        print(
            f"Final Ang Error:   {np.degrees(ang_error_final):.2f}° "
            f"(target: <{ang_tol_deg:.1f}°)"
        )
        print(f"Duration:          {self.simulation_time:.1f} s")
        print(f"{'=' * 60}\n")

    def reset_simulation(self):
        """Reset simulation to initial state."""
        # Reset satellite state to initial values
        initial_pos = getattr(self, "initial_start_pos", (0.0, 0.0, 0.0))
        initial_angle = getattr(self, "initial_start_angle", (0.0, 0.0, 0.0))
        self.satellite.position = np.array(initial_pos)
        self.satellite.velocity = np.array([0.0, 0.0, 0.0])
        self.satellite.angle = initial_angle
        self.satellite.angular_velocity = 0.0

        # Reset simulation variables
        self.simulation_time = 0.0
        self.is_running = True
        self.last_control_update = 0.0

        # Reset target tracking
        self.target_reached_time = None
        self.target_maintenance_time = 0.0
        self.times_lost_target = 0
        self.maintenance_position_errors.clear()
        self.maintenance_angle_errors.clear()

        # Reset data logging
        if self.state_history:
            self.state_history.clear()
        self.control_history.clear()
        target_history = getattr(self, "target_history", None)
        if target_history:
            target_history.clear()
        if self.mpc_solve_times:
            self.mpc_solve_times.clear()
        self.mpc_info_history.clear()

        # Reset control
        self.current_thrusters = np.zeros(8, dtype=np.float64)
        self.previous_thrusters = np.zeros(8, dtype=np.float64)
        self.satellite.active_thrusters.clear()

        # Reset trajectory
        self.satellite.trajectory = [self.satellite.position.copy()]

    def save_trajectory_animation(
        self, output_dir: Path, fps: int = 30, width: int = 1920, height: int = 1080
    ) -> None:
        """
        Generate a matplotlib-based trajectory animation with X-Y and X-Z panels.

        Args:
            output_dir: Directory to save the video file
            fps: Frames per second for the animation
            width: Video width in pixels
            height: Video height in pixels
        """
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        import pandas as pd

        csv_path = output_dir / "control_data.csv"
        if not csv_path.exists():
            logger.warning(f"WARNING: CSV log not found at {csv_path}; skip animation.")
            return

        print(f"Loading simulation data for animation from: {csv_path}")
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            logger.warning(f"WARNING: Could not read CSV data: {e}")
            return

        if len(df) == 0:
            logger.warning("WARNING: Empty simulation data; skipping animation.")
            return

        video_path = output_dir / "Simulation_3D_Render.mp4"

        # Extract trajectory data
        x = df["Current_X"].values
        y = df["Current_Y"].values
        z = df.get("Current_Z", pd.Series([0] * len(df))).values

        # Get target position from last row or defaults
        target_x = df.get("Target_X", pd.Series([0] * len(df))).values[-1]
        target_y = df.get("Target_Y", pd.Series([0] * len(df))).values[-1]
        target_z = df.get("Target_Z", pd.Series([0] * len(df))).values[-1]

        # Calculate bounds with padding
        all_x = np.concatenate([x, [target_x]])
        all_y = np.concatenate([y, [target_y]])
        all_z = np.concatenate([z, [target_z]])

        x_range = max(all_x) - min(all_x)
        y_range = max(all_y) - min(all_y)
        z_range = max(all_z) - min(all_z)
        padding = max(0.2, max(x_range, y_range, z_range) * 0.15)

        x_min, x_max = min(all_x) - padding, max(all_x) + padding
        y_min, y_max = min(all_y) - padding, max(all_y) + padding
        z_min, z_max = min(all_z) - padding, max(all_z) + padding

        # Create figure with two subplots side by side
        fig, (ax_xy, ax_xz) = plt.subplots(
            1, 2, figsize=(width / 100, height / 100), dpi=100
        )
        fig.patch.set_facecolor("#1a1a2e")

        # Style both axes
        for ax, xlabel, ylabel, title in [
            (ax_xy, "X (m)", "Y (m)", "X-Y Plane (Top View)"),
            (ax_xz, "X (m)", "Z (m)", "X-Z Plane (Side View)"),
        ]:
            ax.set_facecolor("#16213e")
            ax.set_xlabel(xlabel, fontsize=12, color="white")
            ax.set_ylabel(ylabel, fontsize=12, color="white")
            ax.set_title(title, fontsize=14, fontweight="bold", color="white")
            ax.grid(True, alpha=0.3, color="gray")
            ax.tick_params(colors="white")
            for spine in ax.spines.values():
                spine.set_color("gray")

        # Set axis limits
        ax_xy.set_xlim(x_min, x_max)
        ax_xy.set_ylim(y_min, y_max)
        ax_xz.set_xlim(x_min, x_max)
        ax_xz.set_ylim(z_min, z_max)
        ax_xy.set_aspect("equal", adjustable="box")
        ax_xz.set_aspect("equal", adjustable="box")

        # Plot targets
        ax_xy.plot(target_x, target_y, "g*", markersize=15, label="Target", zorder=5)
        ax_xz.plot(target_x, target_z, "g*", markersize=15, label="Target", zorder=5)

        # Initialize trajectory lines
        (line_xy,) = ax_xy.plot(
            [], [], "cyan", linewidth=1.5, alpha=0.8, label="Trajectory"
        )
        (line_xz,) = ax_xz.plot(
            [], [], "cyan", linewidth=1.5, alpha=0.8, label="Trajectory"
        )

        # Initialize satellite markers (simple cube representation)
        satellite_size = 0.05  # 5cm satellite representation
        satellite_xy = plt.Rectangle(
            (0, 0),
            satellite_size,
            satellite_size,
            fc="#00d4ff",
            ec="white",
            linewidth=2,
            zorder=10,
        )
        satellite_xz = plt.Rectangle(
            (0, 0),
            satellite_size,
            satellite_size,
            fc="#00d4ff",
            ec="white",
            linewidth=2,
            zorder=10,
        )
        ax_xy.add_patch(satellite_xy)
        ax_xz.add_patch(satellite_xz)

        # Time text
        time_text = fig.text(
            0.5,
            0.02,
            "",
            ha="center",
            fontsize=14,
            color="white",
            fontweight="bold",
            transform=fig.transFigure,
        )

        # Legends
        ax_xy.legend(loc="upper right", facecolor="#1a1a2e", labelcolor="white")
        ax_xz.legend(loc="upper right", facecolor="#1a1a2e", labelcolor="white")

        fig.tight_layout(rect=[0, 0.05, 1, 1])

        # Subsample for reasonable animation length
        total_frames = len(df)
        target_duration = min(30, total_frames / fps)  # Max 30 second video
        step = max(1, int(total_frames / (target_duration * fps)))
        frame_indices = list(range(0, total_frames, step))

        def init():
            line_xy.set_data([], [])
            line_xz.set_data([], [])
            satellite_xy.set_xy((x[0] - satellite_size / 2, y[0] - satellite_size / 2))
            satellite_xz.set_xy((x[0] - satellite_size / 2, z[0] - satellite_size / 2))
            time_text.set_text("")
            return line_xy, line_xz, satellite_xy, satellite_xz, time_text

        def update(frame_idx):
            i = frame_indices[frame_idx]
            # Update trajectory (show trail)
            line_xy.set_data(x[: i + 1], y[: i + 1])
            line_xz.set_data(x[: i + 1], z[: i + 1])
            # Update satellite position
            satellite_xy.set_xy((x[i] - satellite_size / 2, y[i] - satellite_size / 2))
            satellite_xz.set_xy((x[i] - satellite_size / 2, z[i] - satellite_size / 2))
            # Update time
            sim_time = df["Time"].iloc[i] if "Time" in df.columns else i * 0.05
            time_text.set_text(f"Time: {sim_time:.2f}s")
            return line_xy, line_xz, satellite_xy, satellite_xz, time_text

        try:
            # Check for ffmpeg writer
            if "ffmpeg" not in writers.list():
                logger.warning("WARNING: ffmpeg not available; using pillow.")
                writer_name = "pillow"
                video_path = output_dir / "Simulation_3D_Render.gif"
            else:
                writer_name = "ffmpeg"

            anim = FuncAnimation(
                fig,
                update,
                frames=len(frame_indices),
                init_func=init,
                blit=True,
                interval=1000 / fps,
            )

            print(f"Rendering {len(frame_indices)} frames...")
            anim.save(
                str(video_path),
                writer=writer_name,
                fps=fps,
                progress_callback=lambda i, n: print(f"\r  Frame {i + 1}/{n}", end=""),
            )
            print(f"\n Trajectory animation saved to: {video_path}")

        except Exception as e:
            logger.warning(f"WARNING: Could not write animation. Error: {e}")
        finally:
            plt.close(fig)

    def auto_generate_visualizations(self):
        """
        Automatically generate all visualizations after simulation completion.
        This replaces the need for a separate visualization script.
        """
        if UnifiedVisualizationGenerator is None:
            print(
                "  Visualization components not available. Skipping auto-visualization."
            )
            return

        if self.data_save_path is None:
            print("  No data path available. Skipping auto-visualization.")
            return

        try:
            print("\n Animation, Plots and Summary will now be generated!")

            # Check for MissionState (V4.0.0)
            mission_state = None
            if hasattr(self.controller, "mission_manager") and getattr(
                self.controller.mission_manager, "mission_state", None
            ):
                mission_state = self.controller.mission_manager.mission_state

            # Check for AppConfig (V4.0.0)
            app_config = None
            if hasattr(self.controller, "simulation_config") and getattr(
                self.controller.simulation_config, "app_config", None
            ):
                app_config = self.controller.simulation_config.app_config

            generator = UnifiedVisualizationGenerator(
                data_directory=str(self.data_save_path.parent),
                prefer_pandas=True,
                app_config=app_config,
                mission_state=mission_state,
            )

            # Override paths to use current simulation data
            generator.csv_path = self.data_save_path / "physics_data.csv"
            generator.output_dir = self.data_save_path

            # Load the data silently
            import io
            import sys

            old_stdout = sys.stdout
            sys.stdout = io.StringIO()  # Suppress output
            try:
                generator.load_csv_data()
            finally:
                sys.stdout = old_stdout

            # Generate performance plots
            try:
                print("\nCreating Plots...")
                plots_path = self.data_save_path / "Plots"
                print(f"Saving Plots to: {plots_path}")

                sys.stdout = io.StringIO()  # Suppress output
                try:
                    generator.generate_performance_plots()
                finally:
                    sys.stdout = old_stdout

                print(" Plots saved successfully!")
                print(f" File location: {plots_path}")
            except Exception as plots_err:
                print(f"  Performance plots generation failed: {plots_err}")

            # Generate matplotlib-based animation (no MuJoCo)
            try:
                print("\nCreating animation...")
                animation_path = self.data_save_path / "Simulation_3D_Render.mp4"
                print(f"Saving animation to: {animation_path}")

                old_stdout = sys.stdout
                sys.stdout = ProgressSuppressor(sys.stdout)
                try:
                    generator.generate_animation()
                finally:
                    sys.stdout = old_stdout

                if animation_path.exists():
                    print(" Animation saved successfully!")
                    print(f" File location: {animation_path}")
                else:
                    print(" Animation generation skipped (matplotlib-based).")
            except Exception as anim_err:
                print(f"  Animation generation failed: {anim_err}")

        except Exception as e:
            print(f" Error during auto-visualization: {e}")
            print(" You can manually run visualizations later using Visualize.py")


def create_simulation_visualizer(controller):
    """Factory function to create a SimulationVisualizationManager instance."""
    return SimulationVisualizationManager(controller)

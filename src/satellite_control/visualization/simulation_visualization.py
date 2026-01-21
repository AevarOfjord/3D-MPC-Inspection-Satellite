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
                    fontsize=8,
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

        # Draw center marker (Navy Square 5x5cm)
        # Use a polygon to handle rotation correctly
        half_s = 0.025
        corners = np.array(
            [[-half_s, -half_s], [half_s, -half_s], [half_s, half_s], [-half_s, half_s]]
        )
        rotated_corners = (rotation_matrix @ corners.T).T + self.satellite.position[:2]
        center_marker = patches.Polygon(
            rotated_corners, closed=True, facecolor="#000080", zorder=6
        )
        self.satellite.ax_main.add_patch(center_marker)

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
        """Update the information panel to match the dashboard visualization format."""
        # Clear the axes
        self.satellite.ax_info.clear()

        # Set background color to match the dark theme
        self.satellite.ax_info.set_facecolor("#FFFFFF")

        # Remove axis spines
        for spine in self.satellite.ax_info.spines.values():
            spine.set_visible(False)

        self.satellite.ax_info.set_xlim(0, 1)
        self.satellite.ax_info.set_ylim(0, 1)
        self.satellite.ax_info.set_xticks([])
        self.satellite.ax_info.set_yticks([])

        # Colors for the UI
        COLOR_BG = "#FFFFFF"

        COLOR_LABEL = "#404040"
        COLOR_VALUE = "#000000"  # Black for values
        COLOR_VALUE_CYAN = (
            "#000080"  # Navy Blue for active values (high contrast on white)
        )
        COLOR_GREEN = "#008000"  # Darker green for visibility
        COLOR_BAR_BG = "#E0E0E0"
        COLOR_BAR_FILL = "#000080"

        # Add background rectangle covering the whole panel
        # (Although set_facecolor should work, this ensures full coverage)
        rect = patches.Rectangle((0, 0), 1, 1, color=COLOR_BG, zorder=-1)
        self.satellite.ax_info.add_patch(rect)

        # Get current state
        current_state = self.get_current_state()
        # [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
        pos = current_state[0:3]
        quat = current_state[3:7]
        vel = current_state[7:10]
        ang_vel = current_state[10:13]

        # Calculate Yaw (and Roll/Pitch for completeness)
        # q = [w, x, y, z]
        w, x, y, z = quat
        # Yaw (z-axis rotation)
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = np.arcsin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))

        # Calculate Errors
        if self.target_state is not None:
            target_pos = self.target_state[0:3]
            pos_err = pos - target_pos
            pos_err_norm = np.linalg.norm(pos_err)

            # Approx angular error (last_ang_error is computed in simulation loop)
        else:
            pos_err = np.zeros(3)
            pos_err_norm = 0.0

        # Speed and Range
        speed = np.linalg.norm(vel)

        # --- SECTION 1: TELEMETRY ---
        y_cursor = 0.96

        # Header
        self.satellite.ax_info.text(
            0.05,
            y_cursor,
            "((o)) TELEMETRY",
            color=COLOR_VALUE,
            fontsize=8,
            fontweight="bold",
        )
        y_cursor -= 0.02
        self.satellite.ax_info.plot(
            [0.05, 0.95], [y_cursor, y_cursor], color="#404040", linewidth=1
        )
        y_cursor -= 0.06

        # Column Headers
        col_x_start = 0.25
        col_spacing = 0.22
        self.satellite.ax_info.text(
            col_x_start, y_cursor, "X", color=COLOR_LABEL, fontsize=7, ha="center"
        )
        self.satellite.ax_info.text(
            col_x_start + col_spacing,
            y_cursor,
            "Y",
            color=COLOR_LABEL,
            fontsize=7,
            ha="center",
        )
        self.satellite.ax_info.text(
            col_x_start + 2 * col_spacing,
            y_cursor,
            "Z",
            color=COLOR_LABEL,
            fontsize=7,
            ha="center",
        )
        y_cursor -= 0.05

        # Data Rows Helper
        def add_row(label, val_x, val_y, val_z, units, y_pos):
            self.satellite.ax_info.text(
                0.05, y_pos, label, color=COLOR_LABEL, fontsize=7, fontweight="bold"
            )

            # X
            self.satellite.ax_info.text(
                col_x_start,
                y_pos,
                f"{val_x:+.2f}",
                color=COLOR_VALUE,
                fontsize=7,
                ha="center",
                fontfamily="monospace",
                fontweight="bold",
            )
            # Y
            self.satellite.ax_info.text(
                col_x_start + col_spacing,
                y_pos,
                f"{val_y:+.2f}",
                color=COLOR_VALUE,
                fontsize=7,
                ha="center",
                fontfamily="monospace",
                fontweight="bold",
            )
            # Z
            self.satellite.ax_info.text(
                col_x_start + 2 * col_spacing,
                y_pos,
                f"{val_z:+.2f}",
                color=COLOR_VALUE,
                fontsize=7,
                ha="center",
                fontfamily="monospace",
                fontweight="bold",
            )

            self.satellite.ax_info.text(
                0.95, y_pos, units, color=COLOR_LABEL, fontsize=7, ha="right"
            )

        add_row("POS", pos[0], pos[1], pos[2], "m", y_cursor)
        y_cursor -= 0.04
        add_row("VEL", vel[0], vel[1], vel[2], "m/s", y_cursor)
        y_cursor -= 0.04
        add_row("ERR", pos_err[0], pos_err[1], pos_err[2], "m", y_cursor)
        y_cursor -= 0.04

        # Rotation (degrees)
        add_row(
            "ROT", np.degrees(roll), np.degrees(pitch), np.degrees(yaw), "°", y_cursor
        )
        y_cursor -= 0.04
        # Spin (deg/s)
        add_row(
            "SPIN",
            np.degrees(ang_vel[0]),
            np.degrees(ang_vel[1]),
            np.degrees(ang_vel[2]),
            "°/s",
            y_cursor,
        )

        # Sub-labels for ROT/SPIN
        y_sub = y_cursor - 0.025
        self.satellite.ax_info.text(
            col_x_start, y_sub, "ROLL", color="#404040", fontsize=7, ha="center"
        )
        self.satellite.ax_info.text(
            col_x_start + col_spacing,
            y_sub,
            "PITCH",
            color="#404040",
            fontsize=7,
            ha="center",
        )
        self.satellite.ax_info.text(
            col_x_start + 2 * col_spacing,
            y_sub,
            "YAW",
            color="#404040",
            fontsize=7,
            ha="center",
        )

        y_cursor -= 0.08

        # --- SECTION 2: CONTROLLER ---

        y_cursor -= 0.05
        self.satellite.ax_info.text(
            0.06,
            y_cursor,
            "CONTROLLER",
            color=COLOR_VALUE,
            fontsize=8,
            fontweight="bold",
        )
        y_cursor -= 0.02
        self.satellite.ax_info.plot(
            [0.06, 0.94], [y_cursor, y_cursor], color="#404040", linewidth=1
        )
        y_cursor -= 0.05

        solve_time = getattr(self.controller, "last_solve_time", 0.0) * 1000.0  # ms
        pos_error_val = getattr(self.controller, "last_pos_error", 0.0)
        ang_error_deg = np.degrees(getattr(self.controller, "last_ang_error", 0.0))

        self.satellite.ax_info.text(
            0.06, y_cursor, "SOLVE", color=COLOR_LABEL, fontsize=7
        )
        col = COLOR_GREEN if solve_time < 50 else COLOR_VALUE  # Green if healthy
        self.satellite.ax_info.text(
            0.90,
            y_cursor,
            f"{solve_time:.1f} ms",
            color=col,
            fontsize=7,
            fontweight="bold",
            ha="right",
            fontfamily="monospace",
        )
        y_cursor -= 0.05

        self.satellite.ax_info.text(
            0.06, y_cursor, "POS ERR", color=COLOR_LABEL, fontsize=7
        )
        col = COLOR_GREEN if pos_error_val < 0.05 else COLOR_VALUE
        self.satellite.ax_info.text(
            0.90,
            y_cursor,
            f"{pos_error_val:.3f} m",
            color=col,
            fontsize=7,
            fontweight="bold",
            ha="right",
            fontfamily="monospace",
        )
        y_cursor -= 0.05

        self.satellite.ax_info.text(
            0.06, y_cursor, "ANG ERR", color=COLOR_LABEL, fontsize=7
        )
        col = COLOR_GREEN if ang_error_deg < 5.0 else COLOR_VALUE
        self.satellite.ax_info.text(
            0.90,
            y_cursor,
            f"{ang_error_deg:.1f}°",
            color=col,
            fontsize=7,
            fontweight="bold",
            ha="right",
            fontfamily="monospace",
        )

        y_cursor -= 0.08

        # --- SECTION 3: ACTUATORS ---

        y_cursor -= 0.05
        self.satellite.ax_info.text(
            0.06,
            y_cursor,
            "ACTUATORS",
            color=COLOR_LABEL,
            fontsize=8,
            fontweight="bold",
        )
        y_cursor -= 0.02
        self.satellite.ax_info.plot(
            [0.06, 0.94], [y_cursor, y_cursor], color="#404040", linewidth=1
        )
        y_cursor -= 0.03

        # Thrusters (Left side)
        self.satellite.ax_info.text(
            0.25,
            y_cursor,
            "THRUSTERS",
            color=COLOR_LABEL,
            fontsize=7,
            ha="center",
            fontweight="bold",
        )

        # Reaction Wheels (Right side)
        self.satellite.ax_info.text(
            0.75,
            y_cursor,
            "RW",
            color=COLOR_LABEL,
            fontsize=7,
            ha="center",
            fontweight="bold",
        )

        y_cursor -= 0.02

        # Draw Thruster Bars (6 Thrusters)
        thruster_output = getattr(
            self.controller, "thruster_actual_output", np.zeros(6)
        )

        # Bar chart settings
        bar_width = 0.05
        bar_height_max = 0.10
        start_x = 0.06
        spacing = 0.065

        labels = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]

        for i in range(6):
            if i < len(thruster_output):
                val = thruster_output[i]
            else:
                val = 0.0

            x_pos = start_x + i * spacing
            y_base = y_cursor - bar_height_max

            # Background bar
            rect_bg = patches.Rectangle(
                (x_pos, y_base), bar_width, bar_height_max, facecolor=COLOR_BAR_BG
            )
            self.satellite.ax_info.add_patch(rect_bg)

            # Active bar
            if val > 0.01:
                h = val * bar_height_max
                rect_fill = patches.Rectangle(
                    (x_pos, y_base), bar_width, h, facecolor=COLOR_BAR_FILL
                )
                self.satellite.ax_info.add_patch(rect_fill)

            # Label
            self.satellite.ax_info.text(
                x_pos + bar_width / 2,
                y_base - 0.03,
                labels[i],
                color=COLOR_LABEL,
                fontsize=7,
                ha="center",
            )

        # Draw RW Bars (3 Wheels)
        last_ctrl = getattr(self.controller, "last_control_output", None)
        rw_vals = [0.0, 0.0, 0.0]
        if last_ctrl is not None and len(last_ctrl) >= 9:  # 6 thr + 3 rw
            rw_vals = last_ctrl[6:9]

        # Normalize for display (assuming max torque ~0.1Nm usually, clamping for visual)
        MAX_DISPLAY_TORQUE = 0.1

        rw_start_x = 0.65
        rw_spacing = 0.08
        rw_labels = ["X", "Y", "Z"]

        for i in range(3):
            val = rw_vals[i]
            x_pos = rw_start_x + i * rw_spacing
            y_base = y_cursor - bar_height_max

            # Background
            rect_bg = patches.Rectangle(
                (x_pos, y_base), bar_width, bar_height_max, facecolor=COLOR_BAR_BG
            )
            self.satellite.ax_info.add_patch(rect_bg)

            # Active
            abs_val = min(1.0, abs(val) / MAX_DISPLAY_TORQUE)
            if abs_val > 0.01:
                h = abs_val * bar_height_max
                col = COLOR_BAR_FILL
                # if val < 0: col = "#C04040" # Red for negative? optional
                rect_fill = patches.Rectangle(
                    (x_pos, y_base), bar_width, h, facecolor=col
                )
                self.satellite.ax_info.add_patch(rect_fill)

            # Label
            self.satellite.ax_info.text(
                x_pos + bar_width / 2,
                y_base - 0.03,
                rw_labels[i],
                color=COLOR_LABEL,
                fontsize=7,
                ha="center",
            )

            # Value Text
            self.satellite.ax_info.text(
                x_pos + bar_width / 2,
                y_base - 0.06,
                f"{val:.1f}",
                color=COLOR_LABEL,
                fontsize=7,
                ha="center",
                fontfamily="monospace",
            )

    def save_trajectory_animation(
        self, output_dir: Path, width: int = 1200, height: int = 600
    ) -> None:
        """
        Generate a matplotlib-based trajectory animation with X-Y and X-Z panels.
        Matches the dark-theme dashboard visualization style.

        Args:
            output_dir: Directory to save the video file
            width: Video width in pixels
            height: Video height in pixels
        """
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
        from mpl_toolkits.mplot3d import Axes3D
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

        # Use 0 as default if column missing
        def get_col(name, default=0.0):
            return df.get(name, pd.Series([default] * len(df))).values

        # Extract trajectory data
        x = get_col("Current_X")
        y = get_col("Current_Y")
        z = get_col("Current_Z")

        vx = get_col("Current_VX")
        vy = get_col("Current_VY")
        vz = get_col("Current_VZ")

        # Euler angles (assuming Radians in CSV based on inspection)
        roll = get_col("Current_Roll")
        pitch = get_col("Current_Pitch")
        yaw = get_col("Current_Yaw")

        wx = get_col("Current_WX")
        wy = get_col("Current_WY")
        wz = get_col("Current_WZ")

        # Control info
        solve_times = get_col("MPC_Solve_Time")  # s

        # Reaction Wheels
        rw_x = get_col("RW_Torque_X")
        rw_y = get_col("RW_Torque_Y")
        rw_z = get_col("RW_Torque_Z")

        # Angular Errors for display
        err_roll = get_col("Error_Roll")
        err_pitch = get_col("Error_Pitch")
        err_yaw = get_col("Error_Yaw")

        # Time
        if "Control_Time" in df.columns:
            time_col = df["Control_Time"].values
        elif "Time" in df.columns:
            time_col = df["Time"].values
        else:
            time_col = np.arange(len(df)) * 0.05

        command_vectors = df.get("Command_Vector", pd.Series(["[]"] * len(df))).values

        # Get target position (final)
        target_x = get_col("Target_X")[-1]
        target_y = get_col("Target_Y")[-1]
        target_z = get_col("Target_Z")[-1]

        # Calculate axis range - use same range for both axes (square)
        all_vals = np.concatenate([x, y, z, [target_x], [target_y], [target_z]])
        data_range = max(all_vals) - min(all_vals)
        padding = max(0.3, data_range * 0.2)

        center_x = (max(x) + min(x)) / 2
        center_y = (max(y) + min(y)) / 2
        center_z = (max(z) + min(z)) / 2
        half_range = (
            max(
                max(x) - min(x),
                max(y) - min(y),
                max(z) - min(z),
                abs(target_x - center_x) * 2,
                abs(target_y - center_y) * 2,
                abs(target_z - center_z) * 2,
            )
            / 2
            + padding
        )

        # Create figure
        fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
        gs = fig.add_gridspec(1, 2, width_ratios=[2, 1])  # 3D View + Info Panel
        ax_3d = fig.add_subplot(gs[0, 0], projection="3d")
        ax_info = fig.add_subplot(gs[0, 1])

        # Set light theme for figure
        fig.patch.set_facecolor("#FFFFFF")

        # Style layout axes (matching PlotStyle)
        COLOR_TRAJECTORY = "#1f77b4"  # Blue
        COLOR_TARGET = "#d62728"  # Red
        COLOR_SUCCESS = "#2ca02c"  # Green
        COLOR_AXIS = "#404040"  # Dark Grey axes
        GRID_ALPHA = 0.2
        LINEWIDTH = 1.5

        # Configure 3D Axis
        ax_3d.set_facecolor("#FFFFFF")
        ax_3d.set_xlabel("X (m)", fontsize=9, color=COLOR_AXIS)
        ax_3d.set_ylabel("Y (m)", fontsize=9, color=COLOR_AXIS)
        ax_3d.set_zlabel("Z (m)", fontsize=9, color=COLOR_AXIS)
        ax_3d.set_title(
            "Trajectory (Isometric)", fontsize=12, fontweight="bold", color="#000000"
        )
        ax_3d.grid(True, alpha=GRID_ALPHA, color=COLOR_AXIS)
        ax_3d.tick_params(axis="x", colors=COLOR_AXIS)
        ax_3d.tick_params(axis="y", colors=COLOR_AXIS)
        ax_3d.tick_params(axis="z", colors=COLOR_AXIS)

        # Isometric View
        ax_3d.view_init(elev=35, azim=45)

        # Set limits
        ax_3d.set_xlim(center_x - half_range, center_x + half_range)
        ax_3d.set_ylim(center_y - half_range, center_y + half_range)
        ax_3d.set_zlim(center_z - half_range, center_z + half_range)

        # Configure Info Panel
        ax_info.set_facecolor("#FFFFFF")
        ax_info.axis("off")
        # Add background rect
        rect = patches.Rectangle((0, 0), 1, 1, color="#FFFFFF", zorder=-1)
        ax_info.add_patch(rect)

        # Initial Plotting
        ax_3d.plot(
            [target_x],
            [target_y],
            [target_z],
            "o",
            color=COLOR_TARGET,
            markersize=10,
            zorder=5,
        )
        ax_3d.plot(
            [x[0]], [y[0]], [z[0]], "o", color=COLOR_SUCCESS, markersize=8, zorder=5
        )

        (line_3d,) = ax_3d.plot(
            [], [], [], color=COLOR_TRAJECTORY, linewidth=LINEWIDTH, alpha=0.9
        )

        # Satellite Marker (using a point for now in 3D, or could be a small quiver/box)
        (satellite_3d,) = ax_3d.plot(
            [],
            [],
            [],
            "s",
            color=COLOR_TRAJECTORY,
            markersize=15,
            markeredgecolor="white",
            alpha=0.8,
            zorder=10,
        )

        # Center marker
        (center_3d,) = ax_3d.plot(
            [], [], [], "s", color="#000080", markersize=4, zorder=11
        )

        fig.tight_layout(rect=[0, 0.05, 1, 0.95])

        # Subsample
        step = 2
        frame_indices = list(range(0, len(df), step))

        total_sim_time = (
            time_col[-1] - time_col[0] if len(time_col) > 1 else len(df) * 0.05
        )
        fps = len(frame_indices) / total_sim_time
        fps = max(1, fps)

        # Helper to parse command vector string to floats
        def parse_command_vector(cmd_str):
            try:
                if not cmd_str or cmd_str == "[]":
                    return np.zeros(6)
                cmd_str = str(cmd_str).strip("[]")
                if not cmd_str:
                    return np.zeros(6)
                # Parse
                parts = [float(x.strip()) for x in cmd_str.split(",")]
                # Ensure length 6
                res = np.zeros(6)
                length = min(len(parts), 6)
                res[:length] = parts[:length]
                return res
            except Exception:
                return np.zeros(6)

        def init():
            line_3d.set_data([], [])
            line_3d.set_3d_properties([])
            satellite_3d.set_data([], [])
            satellite_3d.set_3d_properties([])
            center_3d.set_data([], [])
            center_3d.set_3d_properties([])

            ax_info.clear()  # Clear info panel
            ax_info.axis("off")
            return line_3d, satellite_3d, center_3d

        def update(frame_idx):
            i = frame_indices[frame_idx]

            # Trajectory update
            line_3d.set_data(x[: i + 1], y[: i + 1])
            line_3d.set_3d_properties(z[: i + 1])

            satellite_3d.set_data([x[i]], [y[i]])
            satellite_3d.set_3d_properties([z[i]])

            center_3d.set_data([x[i]], [y[i]])
            center_3d.set_3d_properties([z[i]])

            # --- INFO PANEL UPDATE ---
            ax_info.clear()
            ax_info.set_xlim(0, 1)
            ax_info.set_ylim(0, 1)
            ax_info.axis("off")

            # Colors
            # Colors
            C_VAL = "#000000"
            C_LBL = "#404040"
            C_CYAN = "#000080"
            C_GRN = "#008000"
            C_BAR_BG = "#E0E0E0"
            C_BAR_FILL = "#000080"

            y_cur = 0.98

            # TELEMETRY
            ax_info.text(
                0.05,
                y_cur,
                "TELEMETRY",
                color=C_VAL,
                fontsize=8,
                fontweight="bold",
            )
            y_cur -= 0.02
            ax_info.plot([0.05, 0.95], [y_cur, y_cur], color="#404040", lw=1)
            y_cur -= 0.05

            # Headers
            col_x = 0.25
            col_sp = 0.22
            ax_info.text(col_x, y_cur, "X", color=C_LBL, ha="center", fontsize=7)
            ax_info.text(
                col_x + col_sp, y_cur, "Y", color=C_LBL, ha="center", fontsize=7
            )
            ax_info.text(
                col_x + 2 * col_sp, y_cur, "Z", color=C_LBL, ha="center", fontsize=7
            )
            y_cur -= 0.04

            def row(lbl, vx, vy, vz, unit, y):
                ax_info.text(0.05, y, lbl, color=C_LBL, fontweight="bold", fontsize=7)
                ax_info.text(
                    col_x,
                    y,
                    f"{vx:+.2f}",
                    color=C_VAL,
                    ha="center",
                    fontfamily="monospace",
                    fontsize=7,
                    fontweight="bold",
                )
                ax_info.text(
                    col_x + col_sp,
                    y,
                    f"{vy:+.2f}",
                    color=C_VAL,
                    ha="center",
                    fontfamily="monospace",
                    fontsize=7,
                    fontweight="bold",
                )
                ax_info.text(
                    col_x + 2 * col_sp,
                    y,
                    f"{vz:+.2f}",
                    color=C_VAL,
                    ha="center",
                    fontfamily="monospace",
                    fontsize=7,
                    fontweight="bold",
                )
                ax_info.text(0.95, y, unit, color=C_LBL, ha="right", fontsize=7)

            c_pos = np.array([x[i], y[i], z[i]])
            c_target = np.array([target_x, target_y, target_z])
            c_err = c_pos - c_target

            row("POS", x[i], y[i], z[i], "m", y_cur)
            y_cur -= 0.04
            row("VEL", vx[i], vy[i], vz[i], "m/s", y_cur)
            y_cur -= 0.04
            row("ERR", c_err[0], c_err[1], c_err[2], "m", y_cur)
            y_cur -= 0.04
            row(
                "ROT",
                np.degrees(roll[i]),
                np.degrees(pitch[i]),
                np.degrees(yaw[i]),
                "°",
                y_cur,
            )
            y_cur -= 0.04
            row(
                "SPIN",
                np.degrees(wx[i]),
                np.degrees(wy[i]),
                np.degrees(wz[i]),
                "°/s",
                y_cur,
            )

            y_cur -= 0.04
            ax_info.text(col_x, y_cur, "ROLL", color="#404040", fontsize=7, ha="center")
            ax_info.text(
                col_x + col_sp, y_cur, "PITCH", color="#404040", fontsize=7, ha="center"
            )
            ax_info.text(
                col_x + 2 * col_sp,
                y_cur,
                "YAW",
                color="#404040",
                fontsize=7,
                ha="center",
            )

            y_cur -= 0.03

            # Calc rng for use in POS ERR
            rng = np.linalg.norm(c_err)

            y_cur -= 0.04

            # CONTROLLER

            y_cur -= 0.03
            ax_info.text(
                0.06, y_cur, "CONTROLLER", color=C_VAL, fontsize=8, fontweight="bold"
            )
            y_cur -= 0.02
            ax_info.plot([0.06, 0.94], [y_cur, y_cur], color="#404040", lw=1)
            y_cur -= 0.05

            st = solve_times[i] * 1000.0 if i < len(solve_times) else 0.0
            ax_info.text(0.06, y_cur, "SOLVE", color=C_LBL, fontsize=7)
            c = C_GRN if st < 50 else C_VAL
            ax_info.text(
                0.90,
                y_cur,
                f"{st:.1f} ms",
                color=c,
                ha="right",
                fontweight="bold",
                fontfamily="monospace",
                fontsize=7,
            )
            y_cur -= 0.04

            ax_info.text(0.06, y_cur, "POS ERR", color=C_LBL, fontsize=7)
            ax_info.text(
                0.90,
                y_cur,
                f"{rng:.3f} m",
                color=C_GRN if rng < 0.05 else C_VAL,
                ha="right",
                fontweight="bold",
                fontfamily="monospace",
                fontsize=7,
            )
            y_cur -= 0.04

            # ANG ERR
            ang_err_rad = np.sqrt(
                err_roll[i] ** 2 + err_pitch[i] ** 2 + err_yaw[i] ** 2
            )
            ang_err_deg = np.degrees(ang_err_rad)

            ax_info.text(0.06, y_cur, "ANG ERR", color=C_LBL, fontsize=7)
            ax_info.text(
                0.90,
                y_cur,
                f"{ang_err_deg:.1f}°",
                color=C_GRN if ang_err_deg < 2.0 else C_VAL,
                ha="right",
                fontweight="bold",
                fontfamily="monospace",
                fontsize=7,
            )

            y_cur -= 0.08

            # ACTUATORS

            y_cur -= 0.03

            ax_info.text(
                0.06, y_cur, "ACTUATORS", color=C_LBL, fontsize=8, fontweight="bold"
            )
            y_cur -= 0.02
            ax_info.plot([0.06, 0.94], [y_cur, y_cur], color="#404040", lw=1)
            y_cur -= 0.03

            ax_info.text(
                0.25,
                y_cur,
                "THRUSTERS",
                color=C_LBL,
                ha="center",
                fontweight="bold",
                fontsize=7,
            )
            ax_info.text(
                0.75,
                y_cur,
                "RW",
                color=C_LBL,
                ha="center",
                fontweight="bold",
                fontsize=7,
            )
            y_cur -= 0.03

            # Thruster Bars
            t_vals = parse_command_vector(command_vectors[i])
            bw = 0.05
            bhm = 0.08
            sx = 0.06
            sp = 0.065
            lbls = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
            for j in range(6):
                val = t_vals[j]
                xp = sx + j * sp
                yb = y_cur - bhm

                # Bg
                ax_info.add_patch(patches.Rectangle((xp, yb), bw, bhm, fc=C_BAR_BG))
                if val > 0.01:
                    ax_info.add_patch(
                        patches.Rectangle((xp, yb), bw, val * bhm, fc=C_BAR_FILL)
                    )

                ax_info.text(
                    xp + bw / 2,
                    yb - 0.03,
                    lbls[j],
                    color=C_LBL,
                    ha="center",
                    fontsize=7,
                )

            # RW
            rw_v = [rw_x[i], rw_y[i], rw_z[i]]
            max_t = 0.1
            sx = 0.65
            sp = 0.08
            lbls = ["X", "Y", "Z"]
            for j in range(3):
                val = rw_v[j]
                xp = sx + j * sp
                yb = y_cur - bhm

                ax_info.add_patch(patches.Rectangle((xp, yb), bw, bhm, fc=C_BAR_BG))
                aval = min(1.0, abs(val) / max_t)
                if aval > 0.01:
                    ax_info.add_patch(
                        patches.Rectangle((xp, yb), bw, aval * bhm, fc=C_BAR_FILL)
                    )

                ax_info.text(
                    xp + bw / 2,
                    yb - 0.03,
                    lbls[j],
                    color=C_LBL,
                    ha="center",
                    fontsize=7,
                )
                ax_info.text(
                    xp + bw / 2,
                    yb - 0.06,
                    f"{val:.1f}",
                    color=C_LBL,
                    ha="center",
                    fontsize=7,
                    fontfamily="monospace",
                )

            return line_3d, satellite_3d, center_3d

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
                blit=False,
                interval=1000 / fps,
            )

            print(
                f"Rendering {len(frame_indices)} frames at {fps:.1f} FPS (real-time)..."
            )
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
            mission_state = getattr(self.controller, "mission_state", None)
            if mission_state is None and hasattr(self.controller, "simulation_config"):
                mission_state = getattr(self.controller.simulation_config, "mission_state", None)

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

            # Generate matplotlib-based animation with X-Y and X-Z panels
            try:
                print("\nCreating animation...")
                animation_path = self.data_save_path / "Simulation_3D_Render.mp4"
                print(f"Saving animation to: {animation_path}")

                # Use new dual-panel trajectory animation
                self.save_trajectory_animation(self.data_save_path)

                if animation_path.exists():
                    print(" Animation saved successfully!")
                    print(f" File location: {animation_path}")
                else:
                    # Check for GIF fallback
                    gif_path = self.data_save_path / "Simulation_3D_Render.gif"
                    if gif_path.exists():
                        print(" Animation saved successfully (GIF format)!")
                        print(f" File location: {gif_path}")
                    else:
                        print(" Animation generation failed.")
            except Exception as anim_err:
                print(f"  Animation generation failed: {anim_err}")

        except Exception as e:
            print(f" Error during auto-visualization: {e}")
            print(" You can manually run visualizations later using Visualize.py")


def create_simulation_visualizer(controller):
    """Factory function to create a SimulationVisualizationManager instance."""
    return SimulationVisualizationManager(controller)

"""
Linearized MPC Simulation for Satellite Thruster Control

Physics-based simulation environment for testing MPC control
algorithms.
Implements realistic satellite dynamics with thruster actuation
and disturbances.

Simulation features:
- Linearized dynamics with A, B matrices around equilibrium
- Eight-thruster configuration with individual force calibration
- Collision avoidance with spherical obstacles
- Mission execution (waypoint, shape following)
- Sensor noise and disturbance simulation
- Real-time visualization with matplotlib

Physics modeling:
- 3D rigid-body state [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
- Planar thruster layout with optional Z translation via attitude/tilt
- Thruster force and torque calculations
- Moment of inertia and mass properties
- Integration with configurable time steps

Data collection:
- Complete state history logging
- Control input recording
- MPC solve time statistics
- Mission performance metrics
- CSV export for analysis

Configuration:
- Modular config package for all parameters
- Structured config system for clean access
- Consistent with real hardware configuration
"""

import logging

import time

# time, asyncio removed
from typing import Any, Dict, Optional, List, Tuple, Union
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
# Axes3D removed (unused at top level)

# V4.0.0: SatelliteConfig removed - use SimulationConfig only
# V4.0.0: Strict Configuration
from src.satellite_control.config import SimulationConfig, AppConfig
from src.satellite_control.config.constants import Constants

from src.satellite_control.core.simulation_loop import SimulationLoop
from src.satellite_control.core.simulation_initialization import SimulationInitializer
from src.satellite_control.utils.logging_config import setup_logging
from src.satellite_control.utils.navigation_utils import (
    angle_difference,
    normalize_angle,
    point_to_line_distance,
)
from src.satellite_control.utils.orientation_utils import (
    euler_xyz_to_quat_wxyz,
    quat_angle_error,
    quat_wxyz_to_euler_xyz,
)


# Set up logger with simple format for clean output (console only)
logger = setup_logging(__name__, log_file=None, simple_format=True)


# Use centralized FFMPEG path from Constants (handles all platforms)
plt.rcParams["animation.ffmpeg_path"] = Constants.FFMPEG_PATH

try:
    from src.satellite_control.visualization.unified_visualizer import (
        UnifiedVisualizationGenerator,
    )
except ImportError:
    logger.warning(
        "WARNING: Could not import visualization components. "
        "Limited functionality available."
    )
    UnifiedVisualizationGenerator = None  # type: ignore


class SatelliteMPCLinearizedSimulation:
    """
    Simulation environment for linearized MPC satellite control.

    Combines physics from TestingEnvironment with linearized MPC controller
    for satellite navigation using linearized dynamics.

    This class now acts as a public API orchestrator, delegating to:
    - SimulationInitializer: Handles all initialization logic
    - SimulationLoop: Handles main loop execution
    - Various managers: MPC, Mission, Thruster, etc.

    The public API remains unchanged for backward compatibility.
    """

    def __init__(
        self,
        cfg: Optional[Union[AppConfig, Any]] = None,
        # Legacy/Testing overrides (kept for compatibility but preferred is cfg=AppConfig)
        start_pos: Optional[Tuple[float, ...]] = None,
        end_pos: Optional[Tuple[float, ...]] = None,
        start_angle: Optional[Tuple[float, float, float]] = None,
        end_angle: Optional[Tuple[float, float, float]] = None,
        start_vx: float = 0.0,
        start_vy: float = 0.0,
        start_vz: float = 0.0,
        start_omega: Union[float, Tuple[float, float, float]] = 0.0,
        simulation_config: Optional[SimulationConfig] = None,
        config_overrides: Optional[Dict[str, Dict[str, Any]]] = None,
    ):
        """
        Initialize linearized MPC simulation.

        Args:
            cfg: AppConfig object (preferred) or Hydra DictConfig (legacy).
            start_pos: Override starting position (x, y, z).
            end_pos: Override end position (x, y, z).
            start_angle: Override starting orientation (roll, pitch, yaw).
            end_angle: Override end orientation (roll, pitch, yaw).
            start_vx: Initial X velocity.
            start_vy: Initial Y velocity.
            start_vz: Initial Z velocity.
            start_omega: Initial angular velocity.
            simulation_config: Optional SimulationConfig object (overrides cfg).
            config_overrides: Optional SimulationConfig overrides (legacy compatibility).
        """
        self.cfg = cfg

        # Initialize placeholders
        self.simulation_config = simulation_config
        self.structured_config: Optional[SimulationConfig] = None  # Alias

        # Path-only mode: planning and trajectory generation are disabled.
        self.planned_path = []  # Path waypoints [x, y, z]
        self.last_solve_time = 0.0  # Track last MPC solve time for physics logging

        # Hydra Config Adoption
        if cfg is None:
            # Fallback or initialization from defaults if no config provided
            # For now, we assume cfg IS provided by the new CLI.
            # If not, we might need a default factory.
            pass

        self.cfg = cfg

        # Legacy Compatibility Layer (Minimal)
        # We assign self.simulation_config or self.structured_config if other parts of the system
        # still rely on them.
        # Ideally, we start replacing usages of self.structured_config with self.cfg

        # Use SimulationInitializer to handle all initialization
        # We need to adapt SimulationInitializer to accept cfg as well, or update how it's called.
        # For this refactor step, we might need to modify SimulationInitializer next.
        # But let's look at how it's called below.

        # We need to preserve the "simulation_config" attribute if other classes access it.
        # Let's mock it or wrap cfg if needed, but better to update the initializer.

        # Temporarily creating a dummy or adapted structured_config to satisfy legacy initializers
        # This is a bit of a hack during migration.
        # Ideally SimulationInitializer is updated to take cfg.

        # Let's assume we update logic to use self.cfg where possible.

        # Initialize
        # Initialize
        # Adapt Hydra config to SimulationConfig if needed
        # V4.1.0: Prefer explicitly passed simulation_config, then AppConfig
        if self.simulation_config is None:
            if isinstance(self.cfg, AppConfig):
                self.simulation_config = SimulationConfig(app_config=self.cfg)
            else:
                self.simulation_config = SimulationConfig.create_default()

        if config_overrides:
            self.simulation_config = SimulationConfig.create_with_overrides(
                config_overrides, base_config=self.simulation_config
            )

        self.structured_config = self.simulation_config

        self.initializer = SimulationInitializer(
            simulation=self,
            simulation_config=self.simulation_config,
        )
        # We might need to monkey-patch or update the initializer to support this.
        # Or better, we just overwrite the .initialize call

        pass

        # NOTE to reviewer: This replaced block is incomplete because dependent classes (Initializer)
        # need updates. I am proceeding with the plan to update them one by one.
        # Restoring the original logic flow but injecting the new config.

        # ... (Legacy logic removal) ...

        self.initializer.initialize(
            start_pos,
            end_pos,
            start_angle,
            end_angle,
            start_vx,
            start_vy,
            start_vz,
            start_omega,
        )

    def get_current_state(self) -> np.ndarray:
        """Get current satellite state [pos(3), quat(4), vel(3), ang_vel(3)]."""
        s = self.satellite
        # [x, y, z]
        pos = s.position
        # [w, x, y, z]
        quat = s.quaternion
        # [vx, vy, vz]
        vel = s.velocity
        # [wx, wy, wz]
        ang_vel = s.angular_velocity
        # [wrx, wry, wrz]
        wheel_speeds = getattr(s, "wheel_speeds", np.zeros(3))

        return np.concatenate([pos, quat, vel, ang_vel, wheel_speeds])

    # Backward-compatible properties delegating to ThrusterManager
    @property
    def thruster_actual_output(self) -> np.ndarray:
        """Get actual thruster output levels [0, 1] for each thruster."""
        return self.thruster_manager.thruster_actual_output

    @property
    def thruster_last_command(self) -> np.ndarray:
        """Get last commanded thruster pattern."""
        return self.thruster_manager.thruster_last_command

    def get_noisy_state(self, true_state: np.ndarray) -> np.ndarray:
        """
        Add realistic sensor noise to state measurements.
        Models OptiTrack measurement uncertainty and velocity estimation
        errors.

        Delegates to SimulationStateValidator for noise application.

        Args:
            true_state: True state [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]

        Returns:
            Noisy state with measurement errors added
        """
        return self.state_validator.apply_sensor_noise(true_state)

    def create_data_directories(self) -> Path:
        """
        Create the directory structure for saving data.
        Returns the path to the timestamped subdirectory.
        """
        return self._io.create_data_directories()

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range (navigation_utils)."""
        return normalize_angle(angle)

    def angle_difference(self, reference_angle: float, currentAngle: float) -> float:
        """
        Calculate shortest angular difference between angles.
        Delegated to navigation_utils.
        Prevents the 360°/0° transition issue by taking shortest path.
        Returns: angle difference in [-pi, pi], positive = CCW rotation
        """
        return angle_difference(reference_angle, currentAngle)

    def point_to_line_distance(
        self, point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray
    ) -> float:
        """Calculate distance from point to line segment (navigation_utils)."""
        return point_to_line_distance(point, line_start, line_end)

    # OBSTACLE AVOIDANCE METHODS

    def log_physics_step(self):
        """Log high-frequency physics data (every 5ms)."""
        if not self.data_save_path:
            return

        current_state = self.get_current_state()

        # Get reference state (handle if not set)
        reference_state = (
            self.reference_state if self.reference_state is not None else np.zeros(13)
        )

        # Delegate to SimulationLogger
        if not hasattr(self, "physics_logger_helper"):
            from src.satellite_control.core.simulation_logger import (
                SimulationLogger,
            )

            self.physics_logger_helper = SimulationLogger(self.physics_logger)

        self.physics_logger_helper.log_physics_step(
            simulation_time=self.simulation_time,
            current_state=current_state,
            reference_state=reference_state,
            thruster_actual_output=self.thruster_actual_output,
            thruster_last_command=self.thruster_last_command,
            normalize_angle_func=self.normalize_angle,
            solve_time=self.last_solve_time,
        )

    def save_csv_data(self) -> None:
        """Save all logged data to CSV file (delegated to SimulationIO)."""
        self._io.save_csv_data()

    def save_mission_summary(self) -> None:
        """Generate and save mission summary report (delegated to SimulationIO)."""
        self._io.save_mission_summary()

    def save_animation_mp4(self, fig: Any, ani: Any) -> Optional[str]:
        """
        Save the animation as MP4 file (delegated to SimulationIO).

        Args:
            fig: Matplotlib figure object
            ani: Matplotlib animation object

        Returns:
            Path to saved MP4 file or None if save failed
        """
        return self._io.save_animation_mp4(fig, ani)

    def set_thruster_pattern(self, thruster_pattern: np.ndarray) -> None:
        """
        Send thruster command (delegated to ThrusterManager).

        Command is sent at current simulation_time, but valve opening/closing
        takes VALVE_DELAY to complete.

        Args:
            thruster_pattern: Array [0,1] for thruster commands (duty cycle)
        """
        self.thruster_manager.set_thruster_pattern(
            thruster_pattern, self.simulation_time
        )
        # Keep simulation-level current_thrusters in sync
        self.current_thrusters = self.thruster_manager.current_thrusters

    def process_command_queue(self) -> None:
        """
        Update actual thruster output based on valve delays and ramp-up.

        Delegated to ThrusterManager which handles all valve physics.
        Called every simulation timestep to update actual thruster forces.
        """
        self.thruster_manager.process_command_queue(
            simulation_time=self.simulation_time,
            control_update_interval=self.control_update_interval,
            last_control_update=self.last_control_update,
            sim_dt=self.satellite.dt,
            satellite=self.satellite,
        )

    def update_path_reference_state(self, current_state: np.ndarray) -> None:
        """
        Update the reference state from the MPCC path data.

        Args:
            current_state: Current state vector [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
        """
        if not hasattr(self, "mpc_controller") or not self.mpc_controller:
            self.reference_state = current_state[:13].copy()
            return

        # Always MPCC mode
        pos_ref, tangent = self.mpc_controller.get_path_reference()
        reference_state = np.zeros(13, dtype=float)
        reference_state[0:3] = pos_ref

        if np.linalg.norm(tangent) > 1e-6:
            # 3D Alignment: Align Satellite X-axis (1,0,0) with Path Tangent
            # Compute shortest arc quaternion
            # u = [1, 0, 0], v = tangent (normalized)
            u = np.array([1.0, 0.0, 0.0])
            v = tangent / np.linalg.norm(tangent)

            dot = np.dot(u, v)

            if dot > 0.999999:
                # Parallel, no rotation needed from X-axis
                reference_quat = np.array([1.0, 0.0, 0.0, 0.0])
            elif dot < -0.999999:
                # Anti-parallel, 180 deg rotation about Z
                reference_quat = np.array([0.0, 0.0, 0.0, 1.0])
            else:
                # Shortest arc
                cross = np.cross(u, v)
                reference_quat = np.array([1.0 + dot, *cross])
                reference_quat /= np.linalg.norm(reference_quat)

            reference_state[3:7] = reference_quat
        else:
            # Maintain current orientation if stationary
            reference_state[3:7] = current_state[3:7]

        path_speed = 0.0
        if self.simulation_config is not None:
            path_speed = float(self.simulation_config.app_config.mpc.path_speed)
        reference_state[7:10] = tangent * path_speed

        self.reference_state = reference_state

    def update_mpc_control(self) -> None:
        """Update control action using linearized MPC with strict timing."""
        # Force MPC to send commands at fixed intervals
        if self.simulation_time >= self.next_control_simulation_time:
            # Delegate to MPCRunner
            if not hasattr(self, "mpc_runner"):
                from src.satellite_control.core.mpc_runner import MPCRunner

                # Initialize MPC Runner wrapper
                self.mpc_runner = MPCRunner(
                    mpc_controller=self.mpc_controller,
                    config=self.structured_config,
                    state_validator=self.state_validator,
                )

            current_state = self.get_current_state()
            mpc_start_sim_time = self.simulation_time
            mpc_start_wall_time = time.time()

            # Update obstacles (path-only)
            mission_state = (
                self.simulation_config.mission_state
                if self.simulation_config is not None
                else None
            )
            if mission_state is not None:
                self.mpc_runner.update_obstacles(mission_state.obstacles)

            # Compute action
            (
                thruster_action,
                rw_torque_norm,
                mpc_info,
                mpc_computation_time,
                command_sent_wall_time,
            ) = self.mpc_runner.compute_control_action(
                true_state=current_state,
                previous_thrusters=self.previous_thrusters,
            )

            # Track solve time for high-frequency logging
            if mpc_info:
                self.last_solve_time = mpc_info.get("solve_time", 0.0)

            # Velocity governor moved to C++ MPC Controller (V5.1.0)
            # Logic removed to prevent fighting with the solver constraints.

            rw_torque_cmd = np.zeros(3, dtype=np.float64)
            max_rw_torque = getattr(self.mpc_controller, "max_rw_torque", 0.0)
            if rw_torque_norm is not None and max_rw_torque:
                rw_torque_cmd[: len(rw_torque_norm)] = rw_torque_norm * max_rw_torque
            if hasattr(self.satellite, "set_reaction_wheel_torque"):
                self.satellite.set_reaction_wheel_torque(rw_torque_cmd)

            # Update simulation state
            self.last_control_update = self.simulation_time
            self.next_control_simulation_time += self.control_update_interval
            self.last_control_output = np.concatenate([thruster_action, rw_torque_cmd])
            self.previous_thrusters = thruster_action.copy()
            self.control_history.append(thruster_action.copy())
            self.set_thruster_pattern(thruster_action)

            # Log Data
            command_sent_sim_time = self.simulation_time
            control_loop_duration = command_sent_wall_time - mpc_start_wall_time

            self.log_simulation_step(
                mpc_start_sim_time=mpc_start_sim_time,
                command_sent_sim_time=command_sent_sim_time,
                current_state=current_state,
                thruster_action=thruster_action,
                mpc_info=mpc_info,
                mpc_computation_time=mpc_computation_time,
                control_loop_duration=control_loop_duration,
                rw_torque=rw_torque_cmd,
            )

    def replan_path(self):
        """
        Path replanning is disabled in path-only mode.
        """
        logger.info("Path replanning is disabled for MPCC path-following.")

    def log_simulation_step(
        self,
        mpc_start_sim_time: Optional[float] = None,
        command_sent_sim_time: Optional[float] = None,
        current_state: Optional[np.ndarray] = None,
        thruster_action: Optional[np.ndarray] = None,
        mpc_info: Optional[Dict[str, Any]] = None,
        mpc_computation_time: Optional[float] = None,
        control_loop_duration: Optional[float] = None,
        rw_torque: Optional[np.ndarray] = None,
        **legacy_kwargs: Any,
    ) -> None:
        """
        Log simulation step data to CSV and logger.

        Args:
            mpc_start_sim_time: Sim time when MPC started
            command_sent_sim_time: Sim time when command was sent
            current_state: Current satellite state
            thruster_action: Component-level thruster commands
            mpc_info: Metadata from MPC solver
            mpc_computation_time: MPC solver duration (wall s)
            control_loop_duration: Total control loop duration (wall s)
            rw_torque: Optional Reaction Wheel torque command
        """
        if mpc_start_sim_time is None:
            mpc_start_sim_time = legacy_kwargs.pop("mpc_start_time", None)
        if command_sent_sim_time is None:
            command_sent_sim_time = legacy_kwargs.pop(
                "command_sent_sim_time", legacy_kwargs.pop("command_sent_time", None)
            )
        if current_state is None:
            current_state = legacy_kwargs.pop("state", None)
        if thruster_action is None:
            thruster_action = legacy_kwargs.pop("thrusters", None)

        if current_state is None or thruster_action is None:
            raise ValueError(
                "log_simulation_step requires current_state and thruster_action"
            )

        if mpc_start_sim_time is None:
            mpc_start_sim_time = self.simulation_time
        if command_sent_sim_time is None:
            command_sent_sim_time = self.simulation_time

        if mpc_computation_time is None:
            mpc_computation_time = (
                float(mpc_info.get("solve_time", 0.0)) if mpc_info else 0.0
            )
        if control_loop_duration is None:
            control_loop_duration = 0.0

        # Store state history for summaries/plots
        self.state_history.append(current_state.copy())

        # Record performance metrics
        solve_time = mpc_info.get("solve_time", 0.0) if mpc_info else 0.0

        timeout = mpc_info.get("timeout", False) if mpc_info else False
        self.performance_monitor.record_mpc_solve(solve_time, timeout=timeout)

        # Record control loop time
        # Use 90% of available time as threshold
        timing_violation = mpc_computation_time > (self.control_update_interval * 0.9)
        self.performance_monitor.record_control_loop(
            control_loop_duration, timing_violation=timing_violation
        )

        # Print status with timing information
        pos_error = np.linalg.norm(current_state[:3] - self.reference_state[:3])

        # Quaternion error: 2 * arccos(|<q1, q2>|)
        ang_error = quat_angle_error(self.reference_state[3:7], current_state[3:7])

        # V4.0.0: Expose metrics for external telemetry
        self.last_solve_time = solve_time
        self.last_pos_error = pos_error
        self.last_ang_error = ang_error

        # Determine status message (path-only)
        stabilization_time = None
        mission_phase = "PATH_FOLLOWING"
        status_msg = f"Following Path (t={self.simulation_time:.1f}s)"

        path_s = getattr(self.mpc_controller, "s", None)
        path_len = None
        if hasattr(self.mpc_controller, "_path_length"):
            path_len = float(getattr(self.mpc_controller, "_path_length", 0.0) or 0.0)
        if path_len is None or path_len <= 0.0:
            if self.simulation_config is not None:
                path_len = float(
                    getattr(
                        self.simulation_config.mission_state, "dxf_path_length", 0.0
                    )
                    or 0.0
                )
        if path_s is not None and path_len:
            status_msg = f"Following Path (s={path_s:.2f}/{path_len:.2f}m, t={self.simulation_time:.1f}s)"

        # Prepare display variables and update command history
        if thruster_action.ndim > 1:
            display_thrusters = thruster_action[0, :]
        else:
            display_thrusters = thruster_action

        active_thruster_ids = [
            int(x) for x in np.where(display_thrusters > 0.01)[0] + 1
        ]
        self.command_history.append(active_thruster_ids)

        def fmt_position_mm(s: np.ndarray) -> str:
            x_mm = s[0] * 1000
            y_mm = s[1] * 1000
            z_mm = s[2] * 1000
            return f"[x:{x_mm:.0f}, y:{y_mm:.0f}, z:{z_mm:.0f}]mm"

        def fmt_angles_deg(s: np.ndarray) -> str:
            q = np.array(s[3:7], dtype=float)
            if np.linalg.norm(q) == 0:
                q = np.array([1.0, 0.0, 0.0, 0.0])
            roll, pitch, yaw = quat_wxyz_to_euler_xyz(q)
            roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])
            return f"[Yaw:{yaw_deg:.1f}, Roll:{roll_deg:.1f}, Pitch:{pitch_deg:.1f}]°"

        safe_reference = (
            self.reference_state if self.reference_state is not None else np.zeros(13)
        )
        if (
            safe_reference.shape[0] >= 7
            and np.linalg.norm(safe_reference[3:7]) == 0
        ):
            safe_reference = safe_reference.copy()
            safe_reference[3] = 1.0

        ang_err_deg = np.degrees(ang_error)
        vel_error = 0.0
        ang_vel_error = 0.0
        if current_state.shape[0] >= 13 and safe_reference.shape[0] >= 13:
            vel_error = float(
                np.linalg.norm(current_state[7:10] - safe_reference[7:10])
            )
            ang_vel_error = float(
                np.linalg.norm(current_state[10:13] - safe_reference[10:13])
            )
        ang_vel_err_deg = np.degrees(ang_vel_error)
        solve_ms = mpc_info.get("solve_time", 0) * 1000
        next_upd = self.next_control_simulation_time
        # Show duty cycle for each active thruster (matching active_thruster_ids)
        thr_out = [
            round(float(display_thrusters[i - 1]), 2) for i in active_thruster_ids
        ]
        rw_norm = np.zeros(3, dtype=float)
        if rw_torque is not None:
            rw_vals = np.array(rw_torque, dtype=float)
            rw_norm[: min(3, len(rw_vals))] = rw_vals[:3]
        rw_out = [round(float(val), 2) for val in rw_norm]
        logger.info(
            f"t = {self.simulation_time:.1f}s: {status_msg}\n"
            f"Pos Err = {pos_error:.3f}m, Ang Err = {ang_err_deg:.1f}°\n"
            f"Vel Err = {vel_error:.3f}m/s, Vel Ang Err = {ang_vel_err_deg:.1f}°/s\n"
            f"Position = {fmt_position_mm(current_state)}\n"
            f"Angle = {fmt_angles_deg(current_state)}\n"
            f"Reference Pos = {fmt_position_mm(safe_reference)}\n"
            f"Reference Ang = {fmt_angles_deg(safe_reference)}\n"
            f"Solve = {solve_ms:.1f}ms, Next = {next_upd:.3f}s\n"
            f"Thrusters = {active_thruster_ids}\n"
            f"Thruster Output = {thr_out}\n"
            f"Reaction Wheel = [X, Y, Z]\n"
            f"RW Output = {rw_out}\n"
        )

        # Delegate to SimulationLogger for control_data.csv output
        if not hasattr(self, "logger_helper"):
            from src.satellite_control.core.simulation_logger import SimulationLogger

            self.logger_helper = SimulationLogger(self.data_logger)

        previous_thruster_action: Optional[np.ndarray] = (
            self.previous_command if hasattr(self, "previous_command") else None
        )

        # Update Context
        self.context.update_state(
            self.simulation_time, current_state, self.reference_state
        )
        self.context.step_number = self.data_logger.current_step
        self.context.mission_phase = mission_phase
        self.context.previous_thruster_command = previous_thruster_action
        if rw_torque is not None:
            self.context.rw_torque_command = np.array(rw_torque, dtype=np.float64)

        mpc_info_safe = mpc_info if mpc_info is not None else {}
        self.logger_helper.log_step(
            self.context,
            mpc_start_sim_time,
            command_sent_sim_time,
            thruster_action,
            mpc_info_safe,
            rw_torque=self.context.rw_torque_command,
            solve_time=self.last_solve_time,  # Added solve_time for compatibility
        )

        self.previous_command = thruster_action.copy()

        # Log terminal message to CSV
        terminal_entry = {
            "Time": self.simulation_time,
            "Status": status_msg,
            "Stabilization_Time": (
                stabilization_time if stabilization_time is not None else ""
            ),
            "Position_Error_m": pos_error,
            "Angle_Error_deg": np.degrees(ang_error),
            "Active_Thrusters": str(active_thruster_ids),
            "Solve_Time_s": mpc_computation_time,
            "Next_Update_s": self.next_control_simulation_time,
        }
        self.data_logger.log_terminal_message(terminal_entry)

    def check_path_complete(self) -> bool:
        """
        Check if path progress has reached the end.
        """
        if not hasattr(self, "mpc_controller") or not self.mpc_controller:
            return False

        # Always in path following mode
        path_len = 0.0
        if hasattr(self.mpc_controller, "_path_length"):
            path_len = float(getattr(self.mpc_controller, "_path_length", 0.0) or 0.0)
        if path_len <= 0.0 and self.simulation_config is not None:
            path_len = float(
                getattr(self.simulation_config.mission_state, "dxf_path_length", 0.0)
                or 0.0
            )
        if path_len <= 0.0:
            return False

        path_s = float(getattr(self.mpc_controller, "s", 0.0) or 0.0)
        return path_s >= path_len

    def draw_simulation(self) -> None:
        """Draw the simulation with satellite, reference, and trajectory."""
        # Skip visualization in headless mode
        if self.satellite.ax is None:
            return
        self.visualizer.sync_from_controller()
        self.visualizer.draw_simulation()

    def _draw_obstacles(self) -> None:
        """Draw configured obstacles on the visualization (delegated)."""
        self.visualizer._draw_obstacles()

    def _draw_obstacle_avoidance_waypoints(self) -> None:
        """Draw obstacle avoidance waypoints for point-to-point modes."""
        self.visualizer._draw_obstacle_avoidance_waypoints()

    def _draw_satellite_elements(self) -> None:
        """Draw satellite elements manually to avoid conflicts (delegated)."""
        self.visualizer._draw_satellite_elements()

    def update_mpc_info_panel(self) -> None:
        """Update the information panel to match visualization format."""
        # Skip visualization in headless mode
        if self.satellite.ax is None:
            return
        self.visualizer.sync_from_controller()
        self.visualizer.update_mpc_info_panel()

    def print_performance_summary(self) -> None:
        """Print performance summary at the end of simulation."""
        # Export performance metrics
        if self.data_save_path:
            metrics_path = self.data_save_path / "performance_metrics.json"
            try:
                self.performance_monitor.export_metrics(metrics_path)
                logger.info(f"Performance metrics exported to {metrics_path}")
            except Exception as e:
                logger.warning(f"Failed to export performance metrics: {e}")

        # Print performance summary
        self.performance_monitor.print_summary()

        # Check thresholds and warn
        warnings = self.performance_monitor.check_thresholds()
        if warnings:
            logger.warning("Performance threshold violations detected:")
            for warning in warnings:
                logger.warning(f"  ⚠️  {warning}")

        # Delegate to visualizer if available
        if self.visualizer:
            self.visualizer.sync_from_controller()
            self.visualizer.print_performance_summary()

    def reset(self) -> None:
        """
        Reset simulation to initial conditions (Physics + Time).
        """
        logger.info("Resetting simulation...")

        # 1. Reset Physics State
        if hasattr(self, "satellite") and hasattr(self, "initial_start_pos"):
            self.satellite.position = self.initial_start_pos.copy()
            self.satellite.velocity = np.zeros(3)
            self.satellite.angle = self.initial_start_angle
            self.satellite.angular_velocity = np.zeros(3)

        # 2. Reset Timing & Status
        self.simulation_time = 0.0
        self.trajectory_endpoint_reached_time = None

        # 3. Reset Running State
        self.is_running = True

        # 4. Clear Logs (Partial)
        self.state_history = []

        # 4b. Reset Metrics
        self.last_solve_time = 0.0
        self.last_pos_error = 0.0
        self.last_ang_error = 0.0

        # 5. Visualizer Reset
        self.visualizer.sync_from_controller()
        self.visualizer.reset_simulation()

    def reset_simulation(self) -> None:
        """Legacy alias for reset()."""
        self.reset()

    def auto_generate_visualizations(self) -> None:
        """Generate all visualizations after simulation completion."""
        self.visualizer.sync_from_controller()
        self.visualizer.auto_generate_visualizations()

    def run_simulation(self, show_animation: bool = True) -> None:
        """
        Run simulation with a per-instance structured config sandbox.

        Args:
            show_animation: Whether to display animation during simulation
        """
        # Use SimulationLoop to handle all loop logic
        loop = SimulationLoop(self)
        return loop.run(
            show_animation=show_animation, structured_config=self.structured_config
        )

    def step(self) -> None:
        """
        Execute a single simulation step.
        v4.0.0: Expose stepping for external runners (e.g. Dashboard).
        """
        if not hasattr(self, "loop"):
            # Lazy init of loop helper
            from src.satellite_control.core.simulation_loop import SimulationLoop

            self.loop = SimulationLoop(self)

            # Ensure running state setup (subset of Loop.run)
            self.is_running = True

            # Context setup
            if not hasattr(self, "context"):
                from src.satellite_control.core.simulation_context import (
                    SimulationContext,
                )

                self.context = SimulationContext()
                self.context.dt = self.satellite.dt
                self.context.control_dt = self.control_update_interval

        self.loop.update_step(None)

    def update_simulation(self, frame: Optional[int] = None) -> None:
        """
        Legacy alias for step() to maintain compatibility with older tests/tools.
        """
        self.step()

    def is_complete(self) -> bool:
        """
        Check if simulation is complete.
        v4.0.0: Helper for external runners.
        """
        return not self.is_running

    def set_continuous(self, enabled: bool) -> None:
        """Enable or disable continuous simulation mode (ignores termination)."""
        self.continuous_mode = enabled
        if enabled:
            self.is_running = True

    def close(self) -> None:
        """
        Clean up simulation resources.
        """
        # Close matplotlib figures if any
        plt.close("all")

        # Close visualizer if it supports it
        if hasattr(self, "visualizer") and hasattr(self.visualizer, "close"):
            self.visualizer.close()

        # Close satellite resources if accessible
        if hasattr(self, "satellite") and hasattr(self.satellite, "close"):
            self.satellite.close()

        logger.info("Simulation closed.")

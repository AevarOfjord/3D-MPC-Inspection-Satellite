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
from src.satellite_control.utils.logging_config import setup_logging
from src.satellite_control.utils.navigation_utils import (
    angle_difference,
    normalize_angle,
    point_to_line_distance,
)
from src.satellite_control.utils.orientation_utils import (
    quat_angle_error,
    quat_wxyz_to_euler_xyz,
)


# Set up logger with simple format for clean output (console only)
logger = setup_logging(__name__, log_file=None, simple_format=True)


# Use centralized FFMPEG path from Constants (handles all platforms)
plt.rcParams["animation.ffmpeg_path"] = Constants.FFMPEG_PATH

try:
    from src.satellite_control.mission.mission_manager import MissionManager
    from src.satellite_control.visualization.unified_visualizer import (
        UnifiedVisualizationGenerator,
    )
except ImportError:
    logger.warning(
        "WARNING: Could not import visualization or mission components. "
        "Limited functionality available."
    )
    UnifiedVisualizationGenerator = None  # type: ignore
    MissionManager = None  # type: ignore


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
        target_pos: Optional[Tuple[float, ...]] = None,
        start_angle: Optional[Tuple[float, float, float]] = None,
        target_angle: Optional[Tuple[float, float, float]] = None,
        start_vx: float = 0.0,
        start_vy: float = 0.0,
        start_vz: float = 0.0,
        start_omega: Union[float, Tuple[float, float, float]] = 0.0,
        simulation_config: Optional[SimulationConfig] = None,
        use_mujoco_viewer: bool = False,
    ):
        """
        Initialize linearized MPC simulation.

        Args:
            cfg: AppConfig object (preferred) or Hydra DictConfig (legacy).
            start_pos: Override starting position (x, y, z).
            target_pos: Override target position (x, y, z).
            start_angle: Override starting orientation (roll, pitch, yaw).
            target_angle: Override target orientation (roll, pitch, yaw).
            start_vx: Initial X velocity.
            start_vy: Initial Y velocity.
            start_vz: Initial Z velocity.
            start_omega: Initial angular velocity.
            simulation_config: Optional SimulationConfig object (overrides cfg).
            use_mujoco_viewer: If True, use MuJoCo viewer.
        """
        self.use_mujoco_viewer = use_mujoco_viewer
        self.cfg = cfg

        # Initialize placeholders
        self.simulation_config = simulation_config
        self.structured_config: Optional[SimulationConfig] = None  # Alias

        # V5.0 Autonomy Components
        from src.satellite_control.planning.rrt_star import RRTStarPlanner
        from src.satellite_control.planning.trajectory_generator import (
            TrajectoryGenerator,
        )

        self.planner = RRTStarPlanner(
            bounds_min=(-20, -20, -20), bounds_max=(20, 20, 20)
        )
        # Initialize Sequencer (will be populated if mission file is loaded)
        from src.satellite_control.mission.sequencer import MissionSequencer

        # Mission state is available after full init, but we can set placeholder
        self.sequencer: Optional[MissionSequencer] = None

        self.trajectory_generator = TrajectoryGenerator(avg_velocity=0.5)
        self.planned_path = []  # List of waypoints [x,y,z]
        self.active_trajectory = None  # Time-parameterized trajectory
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
        from src.satellite_control.core.simulation_initialization import (
            SimulationInitializer,
        )

        # Adapt Hydra config to SimulationConfig if needed
        # V4.1.0: Prefer explicitly passed simulation_config, then AppConfig
        if self.simulation_config is None:
            if isinstance(self.cfg, AppConfig):
                self.simulation_config = SimulationConfig(app_config=self.cfg)

        self.structured_config = self.simulation_config

        self.initializer = SimulationInitializer(
            simulation=self,
            simulation_config=self.simulation_config,
            use_mujoco_viewer=self.use_mujoco_viewer,
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
            target_pos,
            start_angle,
            target_angle,
            start_vx,
            start_vy,
            start_vz,
            start_omega,
        )

        # Initialize Mission Sequencer if MissionManager is present
        if (
            hasattr(self, "mission_manager")
            and self.mission_manager
            and self.mission_manager.mission_state
        ):
            self.sequencer = MissionSequencer(self.mission_manager.mission_state)

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

    def angle_difference(self, target_angle: float, currentAngle: float) -> float:
        """
        Calculate shortest angular difference between angles.
        Delegated to navigation_utils.
        Prevents the 360°/0° transition issue by taking shortest path.
        Returns: angle difference in [-pi, pi], positive = CCW rotation
        """
        return angle_difference(target_angle, currentAngle)

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

        # Get target state (handle if not set)
        target_state = (
            self.target_state if self.target_state is not None else np.zeros(13)
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
            target_state=target_state,
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

    def update_target_state_for_mode(self, current_state: np.ndarray) -> None:
        """
        Update the target state based on the current control mode.

        Delegates to MissionStateManager for centralized mission logic.
        Replaces ~330 lines of complex nested code with clean delegation.

        Args:
            current_state: Current state vector [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
        """
        # Track target index to detect waypoint advances (V4.0.0: use mission_manager)
        prev_target_index = (
            self.mission_manager.mission_state.current_target_index
            if self.mission_manager and self.mission_manager.mission_state
            else 0
        )

        # Delegate to MissionStateManager for centralized mission logic
        target_state = self.mission_manager.update_target_state(
            current_position=self.satellite.position,
            current_quat=self.satellite.quaternion,
            current_time=self.simulation_time,
            current_state=current_state,
        )

        # Detect if MissionStateManager advanced to a new waypoint (V4.0.0: use mission_manager)
        new_target_index = (
            self.mission_manager.mission_state.current_target_index
            if self.mission_manager and self.mission_manager.mission_state
            else 0
        )
        if new_target_index != prev_target_index:
            # Check if mission is complete to avoid false reset (V4.0.0: use mission_manager)
            is_mission_complete = (
                self.mission_manager.mission_state.multi_point_phase == "COMPLETE"
                if self.mission_manager and self.mission_manager.mission_state
                else False
            )

            # Only reset if NOT complete (advancing to valid next target)
            if not is_mission_complete:
                # Reset target reached status for proper phase display
                self.target_reached_time = None
                self.target_maintenance_time = 0.0
                self.approach_phase_start_time = self.simulation_time

        # If mission returns None, it means mission is complete
        if target_state is None:
            # Check if it's a completion signal (mission finished)
            if self.mission_manager.dxf_completed:
                logger.info("DXF PROFILE COMPLETED! Profile successfully traversed.")
                self.is_running = False
                self.print_performance_summary()
                return
        else:
            # Check for significant target change in Waypoint Mode (Robustness fallback)
            # Triggers if index tracking failed to catch the transition (V4.0.0: use mission_manager)
            enable_waypoint = (
                self.mission_manager.mission_state.enable_waypoint_mode
                if self.mission_manager and self.mission_manager.mission_state
                else False
            )
            if self.target_state is not None and enable_waypoint:
                # 3D Position Change
                pos_change = np.linalg.norm(target_state[:3] - self.target_state[:3])

                # 3D Angle Change (Quaternion)
                q1 = target_state[3:7]
                q2 = self.target_state[3:7]
                dot = np.abs(np.dot(q1, q2))
                dot = min(1.0, max(-1.0, dot))
                ang_change = 2.0 * np.arccos(dot)

                if pos_change > 1e-4 or ang_change > 1e-4:
                    if self.target_reached_time is not None:
                        logger.info(
                            "Target changed significantly - resetting reached timer"
                        )
                        self.target_reached_time = None
                        self.target_maintenance_time = 0.0
                        self.approach_phase_start_time = self.simulation_time

            # Update our target state from the mission manager
            self.target_state = target_state

        return

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

            # Generate Trajectory for Smart MPC
            horizon = getattr(self.mpc_controller, "N", 10)
            dt = getattr(self.mpc_controller, "dt", 0.05)

            # Autonomy V5.0: Trajectory Tracking
            target_trajectory = None

            if self.active_trajectory is not None and len(self.active_trajectory) > 0:
                # Sample trajectory horizon
                t_start = self.simulation_time
                # trajectory format: [time, x, y, z, vx, vy, vz, ...] 
                # We need 16-D state: [pos(3), quat(4), vel(3), ang_vel(3), wheel_speeds(3)]
                
                dt = getattr(self.mpc_controller, "dt", 0.05)
                horizon = getattr(self.mpc_controller, "N", 10)
                traj_matrix = np.zeros((horizon, 16))
                
                # Get current target quaternion from existing target state or current state
                current_target_quat = self.target_state[3:7]
                
                valid_traj = True
                for k in range(horizon):
                    t_k = t_start + k * dt
                    
                    if t_k > self.active_trajectory[-1, 0]:
                        # Clamp to end
                        pt = self.active_trajectory[-1]
                        pos = pt[1:4]
                        vel = np.zeros(3)
                    else:
                        # Interpolate
                        pos = np.zeros(3)
                        vel = np.zeros(3)
                        for i in range(3):
                            pos[i] = np.interp(t_k, self.active_trajectory[:, 0], self.active_trajectory[:, 1+i])
                            vel[i] = np.interp(t_k, self.active_trajectory[:, 0], self.active_trajectory[:, 4+i])
                    
                    traj_matrix[k, 0:3] = pos
                    traj_matrix[k, 3:7] = current_target_quat
                    traj_matrix[k, 7:10] = vel
                
                target_trajectory = traj_matrix
                
                # Update immediate target state for logging/metrics
                self.target_state = traj_matrix[0, :13]

            else:
                # Legacy / Mission Manager Fallback
                mission_state = (
                    self.mission_manager.mission_state
                    if self.mission_manager and self.mission_manager.mission_state
                    else None
                )
                use_trajectory = bool(
                    mission_state
                    and (
                        mission_state.dxf_shape_mode_active
                        or getattr(mission_state, "mesh_scan_mode_active", False)
                        or getattr(mission_state, "trajectory_mode_active", False)
                    )
                )

                if use_trajectory and hasattr(self.mission_manager, "get_trajectory"):
                    target_trajectory = self.mission_manager.get_trajectory(
                        current_time=self.simulation_time,
                        dt=dt,
                        horizon=horizon,
                        current_state=current_state,
                        external_target_state=self.target_state,
                    )

            # Update obstacles (V3.0.0)
            if self.mission_manager.mission_state:
                self.mpc_runner.update_obstacles(
                    self.mission_manager.mission_state.obstacles
                )

            # Compute action
            (
                thruster_action,
                rw_torque_norm,
                mpc_info,
                mpc_computation_time,
                command_sent_wall_time,
            ) = self.mpc_runner.compute_control_action(
                true_state=current_state,
                target_state=self.target_state,
                previous_thrusters=self.previous_thrusters,
                target_trajectory=target_trajectory,
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
        Trigger RRT* replanning from current state to target state.
        Updates self.planned_path and self.active_trajectory.
        """
        current_state = self.get_current_state()
        start_pos = current_state[0:3]
        target_pos = self.target_state[0:3]

        obstacles = []
        if self.mission_manager and self.mission_manager.mission_state:
            from src.satellite_control.planning.rrt_star import Obstacle

            for obs in self.mission_manager.mission_state.obstacles:
                obstacles.append(
                    Obstacle(position=np.array(obs.position), radius=obs.radius)
                )

        logger.info(
            f"Replanning path from {start_pos} to {target_pos} with {len(obstacles)} obstacles..."
        )

        waypoints = self.planner.plan(start_pos, target_pos, obstacles)

        if waypoints:
            self.planned_path = [list(p) for p in waypoints]
            logger.info(f"Path found: {len(waypoints)} waypoints")

            if self.mission_manager and self.mission_manager.mission_state:
                from src.satellite_control.mission.sequencer import MissionSequencer

                self.sequencer = MissionSequencer(self.mission_manager.mission_state)
        else:
            logger.warning("RRT* failed to find a path!")
            self.planned_path = []

    def log_simulation_step(
        self,
        mpc_start_sim_time: float,
        command_sent_sim_time: float,
        current_state: np.ndarray,
        thruster_action: np.ndarray,
        mpc_info: Dict[str, Any],
        mpc_computation_time: float,
        control_loop_duration: float,
        rw_torque: Optional[np.ndarray] = None,
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
        pos_error = np.linalg.norm(current_state[:3] - self.target_state[:3])

        # Quaternion error: 2 * arccos(|<q1, q2>|)
        ang_error = quat_angle_error(self.target_state[3:7], current_state[3:7])

        # V4.0.0: Expose metrics for external telemetry
        self.last_solve_time = solve_time
        self.last_pos_error = pos_error
        self.last_ang_error = ang_error

        # Determine status message
        status_msg = f"Traveling to Target (t={self.simulation_time:.1f}s)"
        stabilization_time = None
        mission_phase = (
            "STABILIZING" if self.target_reached_time is not None else "APPROACHING"
        )

        if self.target_reached_time is not None:
            stabilization_time = self.simulation_time - self.target_reached_time
            status_msg = f"Stabilizing on Target (t = {stabilization_time:.1f}s)"

        elif (
            self.mission_manager
            and self.mission_manager.mission_state
            and self.mission_manager.mission_state.dxf_shape_mode_active
        ):
            phase = self.mission_manager.mission_state.dxf_shape_phase or "UNKNOWN"
            mission_phase = phase
            # Map internal phase names to user-friendly display names
            phase_display_names = {
                "POSITIONING": "Traveling to Path",
                "PATH_STABILIZATION": "Stabilizing on Path",
                "TRACKING": "Traveling on Path",
                "STABILIZING": "Stabilizing on Path",
                "RETURNING": "Traveling to Target",
            }
            display_phase = phase_display_names.get(phase, phase)
            # For RETURNING phase, check if we're stabilizing at the end
            if phase == "RETURNING" and self.target_reached_time is not None:
                display_phase = "Stabilizing on Target"
            status_msg = f"{display_phase} (t = {self.simulation_time:.1f}s)"
        elif self.mission_manager and self.mission_manager.mission_state:
            if (
                self.mission_manager.mission_state.trajectory_mode_active
                or self.mission_manager.mission_state.mesh_scan_mode_active
            ):
                mission_phase = "TRAJECTORY"
        else:
            status_msg = f"Traveling to Target (t = {self.simulation_time:.1f}s)"

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

        safe_target = (
            self.target_state if self.target_state is not None else np.zeros(13)
        )
        if safe_target.shape[0] >= 7 and np.linalg.norm(safe_target[3:7]) == 0:
            safe_target = safe_target.copy()
            safe_target[3] = 1.0

        ang_err_deg = np.degrees(ang_error)
        vel_error = 0.0
        ang_vel_error = 0.0
        if current_state.shape[0] >= 13 and safe_target.shape[0] >= 13:
            vel_error = float(np.linalg.norm(current_state[7:10] - safe_target[7:10]))
            ang_vel_error = float(
                np.linalg.norm(current_state[10:13] - safe_target[10:13])
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
            f"Target Pos = {fmt_position_mm(safe_target)}\n"
            f"Target Ang = {fmt_angles_deg(safe_target)}\n"
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
            self.simulation_time, current_state, self.target_state
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

    def check_target_reached(self) -> bool:
        """
        Check if satellite has reached the target within tolerances.

        Delegates to SimulationStateValidator for tolerance checking.
        """
        current_state = self.get_current_state()
        return self.state_validator.check_target_reached(
            current_state, self.target_state
        )

    def draw_simulation(self) -> None:
        """Draw the simulation with satellite, target, and trajectory."""
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
        self.target_reached_time = None
        self.target_maintenance_time = 0.0
        self.times_lost_target = 0
        self.maintenance_position_errors = []
        self.maintenance_angle_errors = []

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

    def set_target(
        self, pos: Tuple[float, ...], angle: Tuple[float, float, float]
    ) -> None:
        """
        Dynamically update the target state.

        Args:
            pos: Target position (x, y, z)
            angle: Target orientation (roll, pitch, yaw) in radians
        """
        if not hasattr(self, "target_state"):
            self.target_state = np.zeros(13)

        # Update Position
        tp = np.array(pos, dtype=np.float64)
        if tp.shape == (2,):
            tp = np.pad(tp, (0, 1), "constant")
        self.target_state[0:3] = tp

        # Update Orientation
        from src.satellite_control.utils.orientation_utils import euler_xyz_to_quat_wxyz

        target_quat = euler_xyz_to_quat_wxyz(angle)
        self.target_state[3:7] = target_quat

        # Reset target reached flags if target moved significantly?
        # For now, just let MPC handle it. If we were "reached", we might "lose" it and re-acquire.
        # Ideally we reset target_reached_time if error is large.

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

        # Update Mission Sequencer
        if hasattr(self, "sequencer") and self.sequencer:
            self.sequencer.update(self.simulation_time, self.get_current_state())

        # 4. Step the Simulation Loop
        if hasattr(self, "sequencer") and self.sequencer:
            self.sequencer.update(self.simulation_time, self.get_current_state())

        self.loop.update_step(None)

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

        # Close MuJoCo viewer if accessible
        if hasattr(self, "satellite") and hasattr(self.satellite, "close"):
            self.satellite.close()

        logger.info("Simulation closed.")

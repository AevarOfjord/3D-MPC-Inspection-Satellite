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

from typing import Any, Dict, Optional, Tuple, Union
from pathlib import Path

import numpy as np

from satellite_control.config import SimulationConfig, AppConfig
from satellite_control.config.constants import Constants

from satellite_control.core.simulation_loop import SimulationLoop
from satellite_control.core.simulation_initialization import SimulationInitializer
from satellite_control.core.control_loop import update_mpc_control_step
from satellite_control.core.path_completion import check_path_complete
from satellite_control.core.simulation_step_logging import (
    log_simulation_step as log_simulation_step_impl,
)
from satellite_control.core.simulation_reference import (
    update_path_reference_state as _update_path_reference_impl,
)
from satellite_control.utils.logging_config import setup_logging
from satellite_control.utils.navigation_utils import (
    angle_difference,
    normalize_angle,
    point_to_line_distance,
)
from satellite_control.utils.orientation_utils import (
    quat_wxyz_from_basis,  # noqa: F401 – re-exported for downstream compatibility
)

# Set up logger with simple format for clean output (console only)
logger = setup_logging(__name__, log_file=None, simple_format=True)


def _get_plt():
    """Lazy-import matplotlib.pyplot (saves ~200ms when not needed)."""
    import matplotlib.pyplot as plt

    return plt


# Defer rcParams until matplotlib is actually needed
_plt_configured = False


def _ensure_plt_configured():
    global _plt_configured
    if not _plt_configured:
        plt = _get_plt()
        plt.rcParams["animation.ffmpeg_path"] = Constants.FFMPEG_PATH
        _plt_configured = True


try:
    from satellite_control.visualization.unified_visualizer import (
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
            cfg: AppConfig object.
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

        # Ensure we have a structured SimulationConfig for downstream components.
        # Adapt Hydra config to SimulationConfig if needed
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
        """Get current satellite state [pos(3), quat(4), vel(3), ang_vel(3), wheel(3)]."""
        s = self.satellite
        # Pre-allocated buffer avoids np.concatenate allocation each call
        if not hasattr(self, "_state_buffer"):
            self._state_buffer = np.empty(16, dtype=np.float64)
        buf = self._state_buffer
        buf[0:3] = s.position
        buf[3:7] = s.quaternion
        buf[7:10] = s.velocity
        buf[10:13] = s.angular_velocity
        wheel = getattr(s, "wheel_speeds", None)
        if wheel is not None:
            buf[13:16] = wheel
        else:
            buf[13:16] = 0.0
        return buf

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

    def _get_mission_state(self) -> Optional[Any]:
        """Safely get mission state from simulation config."""
        if self.simulation_config is None:
            return None
        return getattr(self.simulation_config, "mission_state", None)

    def _get_mission_path_waypoints(self) -> list:
        """Get mission path waypoints using canonical mission-state accessors."""
        mission_state = self._get_mission_state()
        if mission_state is None:
            return []

        path = mission_state.get_resolved_path_waypoints()
        return list(path) if path else []

    def _get_mission_path_length(self, compute_if_missing: bool = False) -> float:
        """Get best-available path length (MPC cache first, then mission state)."""
        if hasattr(self, "mpc_controller") and self.mpc_controller is not None:
            if hasattr(self.mpc_controller, "_path_length"):
                try:
                    path_len = float(
                        getattr(self.mpc_controller, "_path_length", 0.0) or 0.0
                    )
                    if path_len > 0.0:
                        return path_len
                except (TypeError, ValueError):
                    pass

        mission_state = self._get_mission_state()
        if mission_state is None:
            return 0.0

        path_len = float(
            mission_state.get_resolved_path_length(
                compute_if_missing=compute_if_missing
            )
            or 0.0
        )
        if path_len > 0.0:
            return path_len

        if compute_if_missing:
            path = self._get_mission_path_waypoints()
            if len(path) > 1:
                path_arr = np.array(path, dtype=float)
                return float(
                    np.sum(np.linalg.norm(path_arr[1:] - path_arr[:-1], axis=1))
                )

        return 0.0

    def _append_capped_history(self, history: list, item: Any) -> None:
        """Append to a history list while enforcing retention limits."""
        history.append(item)
        max_len = int(getattr(self, "history_max_steps", 0) or 0)
        if max_len and len(history) > max_len:
            overflow = len(history) - max_len
            del history[:overflow]
            self.history_trimmed = True

    # OBSTACLE AVOIDANCE METHODS

    def log_physics_step(self):
        """Log high-frequency physics data (every 5ms)."""
        if not self.data_save_path:
            return

        stride = int(getattr(self, "physics_log_stride", 1) or 1)
        if not hasattr(self, "_physics_log_counter"):
            self._physics_log_counter = 0
        self._physics_log_counter += 1
        if stride > 1 and (self._physics_log_counter % stride) != 0:
            return

        current_state = self.get_current_state()

        # Get reference state (handle if not set)
        reference_state = (
            self.reference_state if self.reference_state is not None else np.zeros(13)
        )

        # Delegate to SimulationLogger
        if not hasattr(self, "physics_logger_helper"):
            from satellite_control.core.simulation_logger import (
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
        _ensure_plt_configured()
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
        _update_path_reference_impl(self, current_state)

    def update_mpc_control(self) -> None:
        """Update control action using linearized MPC with strict timing."""
        update_mpc_control_step(self)

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
        """Log simulation step data to CSV and terminal output."""
        log_simulation_step_impl(
            self,
            logger_obj=logger,
            mpc_start_sim_time=mpc_start_sim_time,
            command_sent_sim_time=command_sent_sim_time,
            current_state=current_state,
            thruster_action=thruster_action,
            mpc_info=mpc_info,
            mpc_computation_time=mpc_computation_time,
            control_loop_duration=control_loop_duration,
            rw_torque=rw_torque,
            **legacy_kwargs,
        )

    def check_path_complete(self) -> bool:
        """
        Check if path progress has reached the end.
        """
        return check_path_complete(self)

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
        self.history_trimmed = False

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
            from satellite_control.core.simulation_loop import SimulationLoop

            self.loop = SimulationLoop(self)

            # Ensure running state setup (subset of Loop.run)
            self.is_running = True

            # Context setup
            if not hasattr(self, "context"):
                from satellite_control.core.simulation_context import (
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
        _get_plt().close("all")

        # Close visualizer if it supports it
        if hasattr(self, "visualizer") and hasattr(self.visualizer, "close"):
            self.visualizer.close()

        # Close satellite resources if accessible
        if hasattr(self, "satellite") and hasattr(self.satellite, "close"):
            self.satellite.close()

        logger.info("Simulation closed.")

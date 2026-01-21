"""
Simulation Initialization Module

Handles all initialization logic for the simulation.
Extracted from simulation.py to improve modularity.

This module handles:
- Satellite physics initialization
- MPC controller setup
- Mission state setup
- Data logging setup
- Performance monitoring setup
- State validation setup
"""

import logging
from typing import Any, List, Optional, Tuple, Union

import numpy as np

# V4.0.0: SatelliteConfig removed - using SimulationConfig and Constants
from src.satellite_control.config.constants import Constants
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.control.mpc_controller import MPCController
from src.satellite_control.core.simulation_io import SimulationIO
from src.satellite_control.core.thruster_manager import ThrusterManager
from src.satellite_control.mission.mission_report_generator import (
    create_mission_report_generator,
)
from src.satellite_control.utils.data_logger import create_data_logger
from src.satellite_control.utils.orientation_utils import euler_xyz_to_quat_wxyz
from src.satellite_control.utils.simulation_state_validator import (
    create_state_validator_from_config,
)

logger = logging.getLogger(__name__)


class SimulationInitializer:
    """
    Handles initialization of simulation components.

    This class encapsulates all the setup logic that was previously
    in SatelliteMPCLinearizedSimulation._initialize_from_active_config.
    """

    def __init__(
        self,
        simulation: Any,  # SatelliteMPCLinearizedSimulation instance
        simulation_config: Optional[SimulationConfig] = None,
    ):
        """
        Initialize the simulation initializer.

        Args:
            simulation: The simulation instance to initialize
            simulation_config: Optional SimulationConfig (preferred)
        """
        self.simulation = simulation
        self.simulation_config = simulation_config

    def initialize(
        self,
        start_pos: Optional[Tuple[float, ...]],
        target_pos: Optional[Tuple[float, ...]],
        start_angle: Optional[Tuple[float, float, float]],
        target_angle: Optional[Tuple[float, float, float]],
        start_vx: float = 0.0,
        start_vy: float = 0.0,
        start_vz: float = 0.0,
        start_omega: Union[float, Tuple[float, float, float]] = 0.0,
    ) -> None:
        """
        Initialize all simulation components.

        Args:
            start_pos: Starting position (x, y, z)
            target_pos: Target position (x, y, z)
            start_angle: Starting orientation
            target_angle: Target orientation
            start_vx: Initial X velocity
            start_vy: Initial Y velocity
            start_vz: Initial Z velocity
            start_omega: Initial angular velocity (scalar yaw or (wx, wy, wz))
        """
        # V3.0.0: Always require simulation_config
        if self.simulation_config is None:
            raise ValueError(
                "simulation_config is required (V3.0.0: no SatelliteConfig fallback)"
            )
        app_config = self.simulation_config.app_config

        # Path-only mode: enforce MPCC path-following
        app_config.mpc.mode_path_following = True

        # Use Constants for default positions (these are not mutable)
        if start_pos is None:
            start_pos = Constants.DEFAULT_START_POS
        if target_pos is None:
            target_pos = Constants.DEFAULT_TARGET_POS
        if start_angle is None:
            start_angle = Constants.DEFAULT_START_ANGLE
        if target_angle is None:
            target_angle = Constants.DEFAULT_TARGET_ANGLE

        path_target_pos = target_pos
        if app_config.mpc.mode_path_following:
            target_pos = start_pos
            target_angle = start_angle

        # Initialize satellite physics
        self._initialize_satellite_physics(
            start_pos, start_angle, start_vx, start_vy, start_vz, start_omega
        )

        # Initialize target state
        self._initialize_target_state(target_pos, target_angle)

        # Initialize simulation timing
        self._initialize_simulation_timing(app_config)

        # Initialize thruster manager
        self._initialize_thruster_manager(app_config)

        # Initialize tracking variables
        self._initialize_tracking_variables()

        # Initialize data logging
        self._initialize_data_logging()

        # Initialize performance monitoring
        self._initialize_performance_monitoring()

        # Initialize tolerances
        self._initialize_tolerances()

        # Initialize MPC controller
        self._initialize_mpc_controller(app_config)

        # Initialize mission state (path-only)
        self.simulation.mission_state = self.simulation_config.mission_state
        mission_state = self.simulation.mission_state

        # Configure Obstacles on Controller (V3.0.0)
        if (
            hasattr(self.simulation.mpc_controller, "set_obstacles")
            and mission_state.obstacles_enabled
        ):
            logger.info("Configuring MPC Controller with obstacles...")
            self.simulation.mpc_controller.set_obstacles(mission_state.obstacles)
        
        # Configure Path for Path-Following Mode (V4.0.1)
        if (
            app_config.mpc.mode_path_following
            and not getattr(mission_state, "mpcc_path_waypoints", None)
            and start_pos is not None
            and path_target_pos is not None
        ):
            from src.satellite_control.mission.path_following import (
                build_point_to_point_path,
            )

            path = build_point_to_point_path(
                waypoints=[tuple(start_pos), tuple(path_target_pos)],
                obstacles=None,
                step_size=0.1,
            )
            path_length = float(
                np.sum(
                    np.linalg.norm(
                        np.array(path[1:], dtype=float)
                        - np.array(path[:-1], dtype=float),
                        axis=1,
                    )
                )
            )
            mission_state.mpcc_path_waypoints = path
            mission_state.dxf_shape_path = path
            mission_state.dxf_path_length = path_length
            mission_state.dxf_target_speed = float(app_config.mpc.v_target)

        if app_config.mpc.mode_path_following:
            mission_state.enable_waypoint_mode = False
            mission_state.enable_multi_point_mode = False
            mission_state.trajectory_mode_active = False
            mission_state.dxf_shape_mode_active = False

        if (
            app_config.mpc.mode_path_following
            and hasattr(self.simulation.mpc_controller, "set_path")
            and hasattr(mission_state, "mpcc_path_waypoints")
            and mission_state.mpcc_path_waypoints
        ):
            logger.info("Configuring MPC Controller with path data for path-following...")
            self.simulation.mpc_controller.set_path(mission_state.mpcc_path_waypoints)
            self.simulation.planned_path = list(mission_state.mpcc_path_waypoints)

        # Initialize state validator
        self._initialize_state_validator()

        # Initialize IO helper
        self._initialize_io_helper()

        # Initialize simulation context
        self._initialize_simulation_context()

        # Log initialization summary
        # Log initialization summary
        self._log_initialization_summary(
            start_pos, target_pos, start_angle, target_angle, app_config
        )

        # Initialize visualization attributes (colors, etc)
        self._initialize_visualization_attributes(app_config)

    def _initialize_visualization_attributes(self, app_config: Any) -> None:
        """Initialize attributes needed for visualization (e.g. thruster colors)."""
        # Define a palette of colors
        colors = [
            "#FF0000",
            "#00FF00",
            "#0000FF",
            "#FFFF00",
            "#00FFFF",
            "#FF00FF",
            "#800000",
            "#008000",
            "#000080",
            "#808000",
            "#008080",
            "#800080",
        ]

        self.simulation.satellite.thruster_colors = {}
        thruster_positions = app_config.physics.thruster_positions

        # Assign colors
        for i, tid in enumerate(sorted(thruster_positions.keys())):
            color_idx = i % len(colors)
            self.simulation.satellite.thruster_colors[tid] = colors[color_idx]

    def _initialize_satellite_physics(
        self,
        start_pos: Tuple[float, ...],
        start_angle: Tuple[float, float, float],
        start_vx: float,
        start_vy: float,
        start_vz: float,
        start_omega: Union[float, Tuple[float, float, float]],
    ) -> None:
        """Initialize satellite physics object and set initial state (V4.0.0: app_config required)."""
        if self.simulation_config is None:
            raise ValueError(
                "simulation_config is required (V4.0.0: no SatelliteConfig fallback)"
            )

        logger.info("Initializing C++ Physics Engine...")
        try:
            from src.satellite_control.core.cpp_satellite import CppSatelliteSimulator

            self.simulation.satellite = CppSatelliteSimulator(
                app_config=self.simulation_config.app_config,
            )
            logger.info("C++ Physics Engine initialized successfully.")
        except ImportError as e:
            raise RuntimeError(
                f"C++ Physics Engine required but not found: {e}. "
                "Please run 'pip install -e .' to compile."
            ) from e

        self.simulation.satellite.external_simulation_mode = True

        # Set initial state (including velocities)
        # Ensure start_pos is 3D
        sp = np.array(start_pos, dtype=np.float64)
        if sp.shape == (2,):
            sp = np.pad(sp, (0, 1), "constant")
        self.simulation.satellite.position = sp

        self.simulation.satellite.velocity = np.array(
            [start_vx, start_vy, start_vz], dtype=np.float64
        )
        self.simulation.satellite.angle = start_angle
        # Type ignore: Property setter accepts float, getter returns ndarray
        self.simulation.satellite.angular_velocity = start_omega  # type: ignore

        # Store initial starting position and angle for reset functionality
        self.simulation.initial_start_pos = sp.copy()
        self.simulation.initial_start_angle = start_angle

    def _initialize_target_state(
        self,
        target_pos: Tuple[float, ...],
        target_angle: Tuple[float, float, float],
    ) -> None:
        """Initialize target state vector."""
        # Point-to-point mode (3D State: [p(3), q(4), v(3), w(3)])
        # Target State
        self.simulation.target_state = np.zeros(13)

        # Robust 3D target assignment
        tp = np.array(target_pos, dtype=np.float64)
        if tp.shape == (2,):
            tp = np.pad(tp, (0, 1), "constant")
        self.simulation.target_state[0:3] = tp

        # Target Orientation (3D Euler -> Quaternion)
        target_quat = euler_xyz_to_quat_wxyz(target_angle)
        self.simulation.target_state[3:7] = target_quat
        # Velocities = 0

        logger.info(
            f"INFO: Initial reference state set to ({tp[0]:.2f}, {tp[1]:.2f}, {tp[2]:.2f})"
        )

    def _initialize_simulation_timing(self, app_config: Any) -> None:
        """Initialize simulation timing parameters."""
        self.simulation.is_running = False
        self.simulation.simulation_time = 0.0
        self.simulation.max_simulation_time = app_config.simulation.max_duration
        self.simulation.control_update_interval = app_config.mpc.dt
        self.simulation.last_control_update = 0.0
        self.simulation.next_control_simulation_time = (
            0.0  # Track next scheduled control update
        )

    def _initialize_thruster_manager(self, app_config: Any) -> None:
        """Initialize thruster manager with delays and physics settings."""
        # ===== HARDWARE COMMAND DELAY SIMULATION =====
        # Simulates the delay between sending a command and
        # thrusters actually firing
        # Uses Config parameters for realistic physics when enabled
        # V3.0.0: Use defaults until these are added to SatellitePhysicalParams
        # TODO: Add thruster_valve_delay and thruster_rampup_time to SatellitePhysicalParams
        if app_config.physics.use_realistic_physics:
            self.simulation.VALVE_DELAY = app_config.physics.thruster_valve_delay
            self.simulation.THRUST_RAMPUP_TIME = app_config.physics.thruster_rampup_time
        else:
            self.simulation.VALVE_DELAY = 0.0  # Instant response for idealized physics
            self.simulation.THRUST_RAMPUP_TIME = 0.0

        # Thruster management (valve delays, ramp-up, PWM) - delegated
        self.simulation.num_thrusters = len(app_config.physics.thruster_positions)
        self.simulation.thruster_manager = ThrusterManager(
            num_thrusters=self.simulation.num_thrusters,
            valve_delay=self.simulation.VALVE_DELAY,
            thrust_rampup_time=self.simulation.THRUST_RAMPUP_TIME,
            use_realistic_physics=app_config.physics.use_realistic_physics,
            thruster_type=app_config.mpc.thruster_type,
        )

    def _initialize_tracking_variables(self) -> None:
        """Initialize target maintenance and tracking variables."""
        # Target maintenance tracking
        self.simulation.target_reached_time: Optional[float] = None
        self.simulation.approach_phase_start_time = 0.0
        self.simulation.target_maintenance_time = 0.0
        self.simulation.times_lost_target = 0
        self.simulation.maintenance_position_errors: List[float] = []
        self.simulation.maintenance_angle_errors: List[float] = []
        self.simulation.trajectory_endpoint_reached_time: Optional[float] = None

        # Data logging
        self.simulation.state_history: List[np.ndarray] = []
        self.simulation.command_history: List[List[int]] = []  # For visual replay
        self.simulation.control_history: List[np.ndarray] = []
        self.simulation.target_history: List[np.ndarray] = []
        self.simulation.mpc_solve_times: List[float] = []
        self.simulation.mpc_info_history: List[dict] = []

        # Previous command for rate limiting
        self.simulation.previous_command: Optional[np.ndarray] = None

        # Current control
        self.simulation.current_thrusters = np.zeros(
            self.simulation.num_thrusters, dtype=np.float64
        )
        self.simulation.previous_thrusters = np.zeros(
            self.simulation.num_thrusters, dtype=np.float64
        )

    def _initialize_data_logging(self) -> None:
        """Initialize data loggers."""
        self.simulation.data_logger = create_data_logger(
            mode="simulation",
            filename="control_data.csv",
        )
        self.simulation.physics_logger = create_data_logger(
            mode="physics",
            filename="physics_data.csv",
        )

        # V4.0.0: Pass simulation_config to report generator
        self.simulation.report_generator = create_mission_report_generator(
            self.simulation_config
        )
        self.simulation.data_save_path = None

    def _initialize_performance_monitoring(self) -> None:
        """Initialize performance monitoring."""
        from src.satellite_control.core.performance_monitor import PerformanceMonitor

        self.simulation.performance_monitor = PerformanceMonitor()

    def _initialize_tolerances(self) -> None:
        """Initialize tolerance values."""
        # V4.0.0: Use Constants class attributes
        from src.satellite_control.config.constants import Constants

        self.simulation.position_tolerance = Constants.POSITION_TOLERANCE
        self.simulation.angle_tolerance = Constants.ANGLE_TOLERANCE
        self.simulation.velocity_tolerance = Constants.VELOCITY_TOLERANCE
        self.simulation.angular_velocity_tolerance = (
            Constants.ANGULAR_VELOCITY_TOLERANCE
        )

    def _initialize_mpc_controller(self, app_config: Any) -> None:
        """Initialize MPC controller."""
        logger.info("Initializing MPC Controller (Mode: PWM/OSQP)...")

        # V4.1.0: Refactored to use AppConfig directly
        # If we have an override cfg (legacy/manual), use it
        if hasattr(self.simulation, "cfg") and self.simulation.cfg is not None:
            self.simulation.mpc_controller = MPCController(cfg=self.simulation.cfg)
        else:
            # Preferred: Pass AppConfig directly
            # This avoids the complex round-trip to Hydra OmegaConf
            self.simulation.mpc_controller = MPCController(cfg=app_config)

    def _initialize_state_validator(self) -> None:
        """Initialize state validator."""
        self.simulation.state_validator = create_state_validator_from_config(
            {
                "position_tolerance": self.simulation.position_tolerance,
                "angle_tolerance": self.simulation.angle_tolerance,
                "velocity_tolerance": self.simulation.velocity_tolerance,
                "angular_velocity_tolerance": self.simulation.angular_velocity_tolerance,
            },
            app_config=self.simulation_config.app_config if self.simulation_config else None,
        )

    def _initialize_io_helper(self) -> None:
        """Initialize IO helper for data export operations."""
        self.simulation._io = SimulationIO(self.simulation)

    def _initialize_simulation_context(self) -> None:
        """Initialize simulation context for logging."""
        from src.satellite_control.core.simulation_context import SimulationContext

        self.simulation.context = SimulationContext()
        self.simulation.context.dt = self.simulation.satellite.dt
        self.simulation.context.control_dt = self.simulation.control_update_interval

    def _log_initialization_summary(
        self,
        start_pos: Tuple[float, ...],
        target_pos: Tuple[float, ...],
        start_angle: Tuple[float, float, float],
        target_angle: Tuple[float, float, float],
        app_config: Any,
    ) -> None:
        """Log initialization summary."""
        logger.info("Linearized MPC Simulation initialized:")
        logger.info("INFO: Formulation: A*x[k] + B*u[k] (Linearized Dynamics)")

        def _format_euler_deg(euler: Tuple[float, float, float]) -> str:
            roll, pitch, yaw = np.degrees(euler)
            return f"roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°"

        s_ang_str = _format_euler_deg(start_angle)
        t_ang_str = _format_euler_deg(target_angle)

        logger.info(f"INFO: Start: {start_pos} m, {s_ang_str}")
        logger.info(f"INFO: Target: {target_pos} m, {t_ang_str}")
        logger.info(
            f"INFO: Control update rate: "
            f"{1 / self.simulation.control_update_interval:.1f} Hz"
        )
        logger.info(f"INFO: Prediction horizon: {app_config.mpc.prediction_horizon}")
        logger.info(f"INFO: Control horizon: {app_config.mpc.control_horizon}")

        if app_config.physics.use_realistic_physics:
            logger.info("WARNING: REALISTIC PHYSICS ENABLED:")
            logger.info(
                f"WARNING: - Valve delay: {self.simulation.VALVE_DELAY * 1000:.0f} ms"
            )
            logger.info(
                f"WARNING: - Ramp-up time: "
                f"{self.simulation.THRUST_RAMPUP_TIME * 1000:.0f} ms"
            )
            logger.info(
                f"WARNING: - Linear damping: "
                f"{app_config.physics.damping_linear:.3f} N/(m/s)"
            )
            logger.info(
                f"WARNING: - Rotational damping: "
                f"{app_config.physics.damping_angular:.4f} N*m/(rad/s)"
            )

            position_noise_std = app_config.physics.position_noise_std
            angle_noise_std = app_config.physics.angle_noise_std

            logger.info(
                f"WARNING: - Position noise: {position_noise_std * 1000:.2f} mm std"
            )
            angle_noise_deg = np.degrees(angle_noise_std)
            logger.info(f"WARNING: - Angle noise: {angle_noise_deg:.2f}° std")
        else:
            logger.info("INFO: Idealized physics (no delays, noise, or damping)")

        # Apply obstacle avoidance based on mode
        # V3.0.0: Get obstacles from mission_state
        mission_state = self.simulation_config.mission_state
        if mission_state.obstacles_enabled and mission_state.obstacles:
            logger.info("Obstacle avoidance enabled.")

        # Initialize visualization manager
        from src.satellite_control.visualization.simulation_visualization import (
            create_simulation_visualizer,
        )

        self.simulation.visualizer = create_simulation_visualizer(self.simulation)

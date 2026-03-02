"""
Simulation Initialization Module

Handles all initialization logic for the simulation.
Extracted from controller.shared.python.simulation.py to improve modularity.

This module handles:
- Satellite physics initialization
- MPC controller setup
- Mission state setup
- Data logging setup
- Performance monitoring setup
- State validation setup
"""

import logging
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from controller.configs.constants import Constants
from controller.configs.models import AppConfig
from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller
from controller.shared.python.mission.mission_report_generator import (
    create_mission_report_generator,
)
from controller.shared.python.mission.state import DEFAULT_PATH_HOLD_END_S
from controller.shared.python.runtime.policy import (
    ActuatorPolicy,
    ControllerModeManager,
    PointingGuardrail,
    ReferenceScheduler,
    SolverHealth,
    TerminalSupervisor,
)
from controller.shared.python.runtime.thruster_manager import ThrusterManager
from controller.shared.python.simulation.artifact_paths import artifact_relative_path
from controller.shared.python.simulation.data_logger import create_data_logger
from controller.shared.python.simulation.io import SimulationIO
from controller.shared.python.simulation.state_validator import (
    create_state_validator_from_config,
)
from controller.shared.python.utils.orientation_utils import (
    euler_xyz_to_quat_wxyz,
    quat_wxyz_to_euler_xyz,
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
        simulation_config: SimulationConfig | None = None,
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
        start_pos: tuple[float, ...] | None,
        end_pos: tuple[float, ...] | None,
        start_angle: tuple[float, float, float] | None,
        end_angle: tuple[float, float, float] | None,
        start_vx: float = 0.0,
        start_vy: float = 0.0,
        start_vz: float = 0.0,
        start_omega: float | tuple[float, float, float] = 0.0,
    ) -> None:
        """
        Initialize all simulation components.

        Args:
            start_pos: Starting position (x, y, z)
            end_pos: End position (x, y, z)
            start_angle: Starting orientation
            end_angle: End orientation
            start_vx: Initial X velocity
            start_vy: Initial Y velocity
            start_vz: Initial Z velocity
            start_omega: Initial angular velocity (scalar yaw or (wx, wy, wz))
        """
        if self.simulation_config is None:
            raise ValueError("simulation_config is required.")
        app_config = self.simulation_config.app_config

        # Path-only mode logic is now handled by defaults or explicit config
        # hardcoded override removed to respect app_config settings

        # Use Constants for default positions (these are not mutable)
        if start_pos is None:
            start_pos = Constants.DEFAULT_START_POS
        if end_pos is None:
            end_pos = Constants.DEFAULT_END_POS
        if start_angle is None:
            start_angle = Constants.DEFAULT_START_ANGLE
        if end_angle is None:
            end_angle = Constants.DEFAULT_END_ANGLE

        path_end_pos = end_pos
        # MPCC Mode always active: set initial reference to start_pos
        reference_pos = start_pos
        reference_angle = start_angle

        # Initialize satellite physics
        self._initialize_satellite_physics(
            start_pos, start_angle, start_vx, start_vy, start_vz, start_omega
        )

        # Initialize reference state
        self._initialize_reference_state(reference_pos, reference_angle)

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
        contracts_cfg = getattr(app_config, "controller_contracts", None)
        if contracts_cfg is not None:
            configured_hold = float(getattr(contracts_cfg, "hold_duration_s", 10.0))
            current_hold = getattr(mission_state, "path_hold_end", None)
            if (
                current_hold is None
                or abs(float(current_hold) - DEFAULT_PATH_HOLD_END_S) <= 1e-9
            ):
                mission_state.path_hold_end = configured_hold

        # Configure Path for Path-Following Mode (Always Active)
        if (
            not mission_state.path_waypoints
            and start_pos is not None
            and path_end_pos is not None
        ):
            from controller.shared.python.mission.path_following import (
                build_point_to_point_path,
            )

            path = build_point_to_point_path(
                waypoints=[tuple(start_pos), tuple(path_end_pos)],
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
            mission_state.path_waypoints = path
            mission_state.path_length = path_length
            mission_state.path_speed = float(app_config.mpc.path_speed)

        if mission_state.path_waypoints:
            logger.info(
                "Configuring MPC Controller with path data for path-following..."
            )
            self.simulation.mpc_controller.set_path(mission_state.path_waypoints)
            self.simulation.planned_path = list(mission_state.path_waypoints)

        # Configure scan attitude context from controller.shared.python.mission.runtime metadata.
        self.simulation.mpc_controller.set_scan_attitude_context(
            mission_state.scan_attitude_center,
            mission_state.scan_attitude_axis,
            mission_state.scan_attitude_direction,
        )

        # Align initial attitude so body +X starts along the first path segment.
        if mission_state.path_waypoints:
            aligned_start_angle = self._align_start_angle_to_path(
                start_angle=start_angle,
                path_waypoints=list(mission_state.path_waypoints),
                start_pos=np.array(start_pos, dtype=float),
                frame_origin=np.array(mission_state.frame_origin, dtype=float),
                scan_center=np.array(
                    mission_state.scan_attitude_center,
                    dtype=float,
                )
                if mission_state.scan_attitude_center is not None
                else None,
                scan_axis=np.array(
                    mission_state.scan_attitude_axis,
                    dtype=float,
                )
                if mission_state.scan_attitude_axis is not None
                else None,
                scan_direction=str(mission_state.scan_attitude_direction),
            )
            if aligned_start_angle != start_angle:
                logger.info(
                    "Aligning initial attitude to path frame: "
                    "body +X -> path, body +Z -> scan axis (when configured)."
                )
                start_angle = aligned_start_angle
                self.simulation.satellite.angle = start_angle
                self.simulation.initial_start_angle = start_angle
                reference_angle = start_angle
                self.simulation.reference_state[3:7] = euler_xyz_to_quat_wxyz(
                    reference_angle
                )

        # Initialize state validator
        self._initialize_state_validator()
        self._initialize_runtime_components(app_config)

        # Initialize IO helper
        self._initialize_io_helper()

        # Initialize simulation context
        self._initialize_simulation_context()

        # Log initialization summary
        # Log initialization summary
        self._log_initialization_summary(
            start_pos, end_pos, start_angle, end_angle, app_config
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

    def _align_start_angle_to_path(
        self,
        start_angle: tuple[float, float, float],
        path_waypoints: list[tuple[float, float, float]],
        start_pos: np.ndarray,
        frame_origin: np.ndarray,
        scan_center: np.ndarray | None = None,
        scan_axis: np.ndarray | None = None,
        scan_direction: str = "CW",
    ) -> tuple[float, float, float]:
        """
        Build initial Euler orientation so:
        - body +X follows path travel direction
        - in scan mode: body +Z aligns to scan axis and +Y faces scan center
        - otherwise: body +Z follows orbital/world up as much as possible
        """
        if len(path_waypoints) < 2:
            return start_angle

        p0 = np.array(path_waypoints[0], dtype=float).reshape(-1)
        if p0.size < 3:
            return start_angle
        p0 = p0[:3]

        direction: np.ndarray | None = None
        for point in path_waypoints[1:]:
            p1 = np.array(point, dtype=float).reshape(-1)
            if p1.size < 3:
                continue
            d = p1[:3] - p0
            if float(np.linalg.norm(d)) > 1e-9:
                direction = d
                break

        if direction is None:
            return start_angle

        x_axis = np.array(direction, dtype=float)
        x_norm = float(np.linalg.norm(x_axis))
        if x_norm <= 1e-9:
            return start_angle
        x_axis = x_axis / x_norm

        # Prefer C++ reference frame so startup orientation matches MPC attitude
        # target exactly (single source of truth).
        try:
            q_start = euler_xyz_to_quat_wxyz(start_angle)
            _, _, q_ref = self.simulation.mpc_controller.get_path_reference_state(
                s_query=0.0, q_current=q_start
            )
            q_ref_arr = np.array(q_ref, dtype=float).reshape(-1)
            if q_ref_arr.size >= 4:
                q_ref_arr = q_ref_arr[:4]
                q_norm = float(np.linalg.norm(q_ref_arr))
                if q_norm > 1e-9:
                    q_ref_arr = q_ref_arr / q_norm
                    euler_xyz = quat_wxyz_to_euler_xyz(q_ref_arr)
                    return (
                        float(euler_xyz[0]),
                        float(euler_xyz[1]),
                        float(euler_xyz[2]),
                    )
        except Exception:
            logger.debug(
                "C++ startup reference query failed, using Python frame fallback",
                exc_info=True,
            )

        z_axis: np.ndarray
        y_axis: np.ndarray

        scan_axis_vec = None
        if scan_axis is not None:
            axis_arr = np.array(scan_axis, dtype=float).reshape(-1)
            if axis_arr.size >= 3:
                axis3 = axis_arr[:3]
                axis_norm = float(np.linalg.norm(axis3))
                if axis_norm > 1e-9:
                    scan_axis_vec = axis3 / axis_norm

        if scan_axis_vec is not None:
            # Match scan-attitude convention at t=0:
            # +X along path, +Y object-facing, +Z along scan axis.
            z_line = scan_axis_vec
            if scan_center is not None:
                center_arr = np.array(scan_center, dtype=float).reshape(-1)
                center = (
                    center_arr[:3] if center_arr.size >= 3 else np.zeros(3, dtype=float)
                )
            else:
                center = np.zeros(3, dtype=float)
            pos0 = start_pos[:3] if start_pos.size >= 3 else np.zeros(3, dtype=float)
            radial_in = center - pos0
            radial_in = radial_in - float(np.dot(radial_in, z_line)) * z_line
            radial_norm = float(np.linalg.norm(radial_in))
            if radial_norm > 1e-9:
                radial_dir = radial_in / radial_norm
            else:
                ref = (
                    np.array([0.0, 0.0, 1.0], dtype=float)
                    if abs(float(z_line[2])) < 0.9
                    else np.array([1.0, 0.0, 0.0], dtype=float)
                )
                radial_dir = np.cross(z_line, ref)
                r_norm = float(np.linalg.norm(radial_dir))
                if r_norm <= 1e-9:
                    radial_dir = np.array([0.0, 1.0, 0.0], dtype=float)
                else:
                    radial_dir = radial_dir / r_norm

            t_plane = x_axis - float(np.dot(x_axis, z_line)) * z_line
            t_plane_norm = float(np.linalg.norm(t_plane))
            if t_plane_norm > 1e-9:
                x_axis = t_plane / t_plane_norm
            else:
                scan_direction_cw = str(scan_direction).strip().upper() != "CCW"
                x_axis = (
                    np.cross(z_line, radial_dir)
                    if scan_direction_cw
                    else np.cross(radial_dir, z_line)
                )
                x_norm = float(np.linalg.norm(x_axis))
                if x_norm <= 1e-9:
                    return start_angle
                x_axis = x_axis / x_norm

            z_axis = z_line
            y_axis = np.cross(z_axis, x_axis)
            y_norm = float(np.linalg.norm(y_axis))
            if y_norm <= 1e-9:
                return start_angle
            y_axis = y_axis / y_norm
            x_axis = np.cross(y_axis, z_axis)
            x_norm = float(np.linalg.norm(x_axis))
            if x_norm <= 1e-9:
                return start_angle
            x_axis = x_axis / x_norm
            # Keep +X forward along initial path travel direction.
            if float(np.dot(x_axis, direction)) < 0.0:
                x_axis = -x_axis
                y_axis = -y_axis
        else:
            # Choose "up" from orbital radial direction when available.
            # world_pos ~= frame_origin + local_start_pos in mixed-frame runs.
            world_pos = np.zeros(3, dtype=float)
            if start_pos.size >= 3:
                world_pos += start_pos[:3]
            if frame_origin.size >= 3:
                world_pos += frame_origin[:3]
            world_pos_norm = float(np.linalg.norm(world_pos))
            if world_pos_norm > 1e-6:
                world_up = world_pos / world_pos_norm
            else:
                world_up = np.array([0.0, 0.0, 1.0], dtype=float)
            z_axis = world_up - float(np.dot(world_up, x_axis)) * x_axis
            z_norm = float(np.linalg.norm(z_axis))
            if z_norm <= 1e-9:
                # If moving almost perfectly vertical, choose a deterministic fallback
                # and still keep a right-handed frame.
                fallback_up = np.array([0.0, 1.0, 0.0], dtype=float)
                z_axis = fallback_up - float(np.dot(fallback_up, x_axis)) * x_axis
                z_norm = float(np.linalg.norm(z_axis))
                if z_norm <= 1e-9:
                    return start_angle
            z_axis = z_axis / z_norm

            # Right-handed body frame: x × y = z  => y = z × x
            y_axis = np.cross(z_axis, x_axis)
            y_norm = float(np.linalg.norm(y_axis))
            if y_norm <= 1e-9:
                return start_angle
            y_axis = y_axis / y_norm

        # Re-orthonormalize z from x,y to reduce drift.
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / max(float(np.linalg.norm(z_axis)), 1e-12)

        rot = np.column_stack((x_axis, y_axis, z_axis))
        euler_xyz = Rotation.from_matrix(rot).as_euler("xyz", degrees=False)
        return (float(euler_xyz[0]), float(euler_xyz[1]), float(euler_xyz[2]))

    def _initialize_satellite_physics(
        self,
        start_pos: tuple[float, ...],
        start_angle: tuple[float, float, float],
        start_vx: float,
        start_vy: float,
        start_vz: float,
        start_omega: float | tuple[float, float, float],
    ) -> None:
        """Initialize satellite physics object and set initial state.."""
        if self.simulation_config is None:
            raise ValueError("simulation_config is required.")

        logger.info("Initializing C++ Physics Engine...")
        try:
            from controller.shared.python.simulation.cpp_backend import (
                CppSatelliteSimulator,
            )

            self.simulation.satellite = CppSatelliteSimulator(
                app_config=self.simulation_config.app_config,
            )
            logger.info("C++ Physics Engine initialized successfully.")
        except ImportError as e:
            err_text = str(e)
            guidance = ""
            if "Python version mismatch" in err_text:
                # cpp_satellite already includes targeted ABI mismatch guidance.
                guidance = ""
            else:
                guidance = " Build the extension with 'pip install -e .'."
            raise RuntimeError(f"C++ Physics Engine unavailable: {e}.{guidance}") from e

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

    def _initialize_reference_state(
        self,
        reference_pos: tuple[float, ...],
        reference_angle: tuple[float, float, float],
    ) -> None:
        """Initialize reference state vector."""
        # Reference state (3D State: [p(3), q(4), v(3), w(3)])
        self.simulation.reference_state = np.zeros(13)

        # Robust 3D reference assignment
        ref_pos = np.array(reference_pos, dtype=np.float64)
        if ref_pos.shape == (2,):
            ref_pos = np.pad(ref_pos, (0, 1), "constant")
        self.simulation.reference_state[0:3] = ref_pos

        # Reference Orientation (3D Euler -> Quaternion)
        reference_quat = euler_xyz_to_quat_wxyz(reference_angle)
        self.simulation.reference_state[3:7] = reference_quat
        # Velocities = 0

        logger.info(
            "INFO: Initial reference state set to "
            f"({ref_pos[0]:.2f}, {ref_pos[1]:.2f}, {ref_pos[2]:.2f})"
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
        self.simulation.physics_log_stride = int(
            getattr(app_config.simulation, "physics_log_stride", 1)
        )
        self.simulation.control_log_stride = int(
            getattr(app_config.simulation, "control_log_stride", 1)
        )

    def _initialize_thruster_manager(self, app_config: Any) -> None:
        """Initialize thruster manager with delays and physics settings."""
        # ===== HARDWARE COMMAND DELAY SIMULATION =====
        # Simulates the delay between sending a command and
        # thrusters actually firing
        # Uses Config parameters for realistic physics when enabled
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
        """Initialize reference tracking variables."""
        # Path/reference tracking
        self.simulation.approach_phase_start_time = 0.0
        self.simulation.trajectory_endpoint_reached_time: float | None = None

        # Data logging
        self.simulation.state_history: list[np.ndarray] = []
        self.simulation.command_history: list[list[int]] = []  # For visual replay
        self.simulation.control_history: list[np.ndarray] = []
        self.simulation.reference_history: list[np.ndarray] = []
        self.simulation.mpc_solve_times: list[float] = []
        self.simulation.mpc_info_history: list[dict] = []

        history_max_steps = int(
            getattr(
                self.simulation_config.app_config.simulation, "history_max_steps", 0
            )
            or 0
        )
        history_downsample_stride = int(
            getattr(
                self.simulation_config.app_config.simulation,
                "history_downsample_stride",
                1,
            )
            or 1
        )
        self.simulation.history_max_steps = history_max_steps
        self.simulation.history_downsample_stride = max(1, history_downsample_stride)
        self.simulation.history_trimmed = False

        # Logging stride counters
        self.simulation._physics_log_counter = 0
        self.simulation._control_log_counter = 0
        self.simulation._history_downsample_counter = 0
        self.simulation._control_history_downsample_counter = 0

        # Previous command for rate limiting
        self.simulation.previous_command: np.ndarray | None = None

        # Current control
        self.simulation.current_thrusters = np.zeros(
            self.simulation.num_thrusters, dtype=np.float64
        )
        self.simulation.previous_thrusters = np.zeros(
            self.simulation.num_thrusters, dtype=np.float64
        )
        self.simulation.mode_timeline: list[dict[str, Any]] = []
        self.simulation.completion_gate_trace: list[dict[str, Any]] = []
        self.simulation.controller_health: dict[str, Any] = {}

    def _initialize_data_logging(self) -> None:
        """Initialize data loggers."""
        history_max_steps = int(getattr(self.simulation, "history_max_steps", 0) or 0)
        self.simulation.data_logger = create_data_logger(
            mode="simulation",
            filename=str(artifact_relative_path("control_data.csv")),
            max_terminal_entries=history_max_steps,
        )
        self.simulation.physics_logger = create_data_logger(
            mode="physics",
            filename=str(artifact_relative_path("physics_data.csv")),
            max_terminal_entries=history_max_steps,
        )

        self.simulation.report_generator = create_mission_report_generator(
            self.simulation_config
        )
        self.simulation.data_save_path = None

    def _initialize_performance_monitoring(self) -> None:
        """Initialize performance monitoring."""
        from controller.shared.python.runtime.performance_monitor import (
            PerformanceMonitor,
        )

        self.simulation.performance_monitor = PerformanceMonitor()
        sim_cfg = self.simulation_config.app_config.simulation
        self.simulation.performance_monitor.set_mpc_timing_contract(
            target_mean_ms=float(sim_cfg.mpc_target_mean_solve_time_ms),
            hard_max_ms=float(sim_cfg.mpc_hard_max_solve_time_ms),
            enforce=bool(sim_cfg.enforce_mpc_timing_contract),
        )

    def _initialize_tolerances(self) -> None:
        """Initialize tolerance values."""
        import math

        from controller.configs.constants import Constants

        contracts_cfg = getattr(
            getattr(self.simulation_config, "app_config", None),
            "controller_contracts",
            None,
        )

        self.simulation.position_tolerance = float(
            getattr(contracts_cfg, "position_error_m_max", Constants.POSITION_TOLERANCE)
        )
        self.simulation.angle_tolerance = float(
            math.radians(
                float(
                    getattr(
                        contracts_cfg,
                        "angle_error_deg_max",
                        math.degrees(Constants.ANGLE_TOLERANCE),
                    )
                )
            )
        )
        self.simulation.velocity_tolerance = float(
            getattr(
                contracts_cfg, "velocity_error_mps_max", Constants.VELOCITY_TOLERANCE
            )
        )
        self.simulation.angular_velocity_tolerance = float(
            math.radians(
                float(
                    getattr(
                        contracts_cfg,
                        "angular_velocity_error_degps_max",
                        math.degrees(Constants.ANGULAR_VELOCITY_TOLERANCE),
                    )
                )
            )
        )
        self.simulation.position_hold_exit_tolerance = float(
            getattr(
                contracts_cfg,
                "position_error_exit_m_max",
                Constants.TERMINAL_POSITION_EXIT_TOLERANCE_M,
            )
        )
        self.simulation.angle_hold_exit_tolerance = float(
            math.radians(
                float(
                    getattr(
                        contracts_cfg,
                        "angle_error_exit_deg_max",
                        Constants.TERMINAL_ANGLE_EXIT_TOLERANCE_DEG,
                    )
                )
            )
        )
        self.simulation.velocity_hold_exit_tolerance = float(
            getattr(
                contracts_cfg,
                "velocity_error_exit_mps_max",
                Constants.TERMINAL_VELOCITY_EXIT_TOLERANCE_MPS,
            )
        )
        self.simulation.angular_velocity_hold_exit_tolerance = float(
            math.radians(
                float(
                    getattr(
                        contracts_cfg,
                        "angular_velocity_error_exit_degps_max",
                        Constants.TERMINAL_ANGULAR_VELOCITY_EXIT_TOLERANCE_DEGPS,
                    )
                )
            )
        )

    def _initialize_mpc_controller(self, app_config: Any) -> None:
        """Initialize MPC controller."""
        logger.info("Initializing MPC Controller...")

        # If the simulation object carries an explicit AppConfig override, prefer it.
        cfg_override = getattr(self.simulation, "cfg", None)
        if isinstance(cfg_override, AppConfig):
            self.simulation.mpc_controller = create_controller(cfg=cfg_override)
            self.simulation.controller_core_mode = str(
                getattr(self.simulation.mpc_controller, "controller_core", "sqp")
            )
            self.simulation.controller_profile_mode = str(
                getattr(self.simulation.mpc_controller, "controller_profile", "hybrid")
            )
            self.simulation.linearization_mode = str(
                getattr(
                    self.simulation.mpc_controller,
                    "linearization_mode",
                    "hybrid_tolerant_stage",
                )
            )
            return

        # Preferred: Pass AppConfig directly
        self.simulation.mpc_controller = create_controller(cfg=app_config)
        self.simulation.controller_core_mode = str(
            getattr(self.simulation.mpc_controller, "controller_core", "sqp")
        )
        self.simulation.controller_profile_mode = str(
            getattr(self.simulation.mpc_controller, "controller_profile", "hybrid")
        )
        self.simulation.linearization_mode = str(
            getattr(
                self.simulation.mpc_controller,
                "linearization_mode",
                "hybrid_tolerant_stage",
            )
        )

    def _initialize_state_validator(self) -> None:
        """Initialize state validator."""
        self.simulation.state_validator = create_state_validator_from_config(
            {
                "position_tolerance": self.simulation.position_tolerance,
                "angle_tolerance": self.simulation.angle_tolerance,
                "velocity_tolerance": self.simulation.velocity_tolerance,
                "angular_velocity_tolerance": self.simulation.angular_velocity_tolerance,
                "position_hold_exit_tolerance": (
                    self.simulation.position_hold_exit_tolerance
                ),
                "angle_hold_exit_tolerance": self.simulation.angle_hold_exit_tolerance,
                "velocity_hold_exit_tolerance": (
                    self.simulation.velocity_hold_exit_tolerance
                ),
                "angular_velocity_hold_exit_tolerance": (
                    self.simulation.angular_velocity_hold_exit_tolerance
                ),
            },
            app_config=(
                self.simulation_config.app_config if self.simulation_config else None
            ),
        )

    def _initialize_runtime_components(self, app_config: Any) -> None:
        """Initialize mode/gate/scheduler/policy runtime helpers."""
        contracts_cfg = getattr(app_config, "controller_contracts", None)
        mpc_core_cfg = getattr(app_config, "mpc_core", None)
        actuator_cfg = getattr(app_config, "actuator_policy", None)

        self.simulation.mode_manager = ControllerModeManager(
            recover_enter_error_m=float(
                getattr(contracts_cfg, "recover_enter_error_m", 0.20)
            ),
            recover_enter_hold_s=float(
                getattr(contracts_cfg, "recover_enter_hold_s", 0.5)
            ),
            recover_exit_error_m=float(
                getattr(contracts_cfg, "recover_exit_error_m", 0.10)
            ),
            recover_exit_hold_s=float(
                getattr(contracts_cfg, "recover_exit_hold_s", 1.0)
            ),
            recover_contour_scale=float(
                getattr(mpc_core_cfg, "recover_contour_scale", 2.0)
            ),
            recover_lag_scale=float(getattr(mpc_core_cfg, "recover_lag_scale", 2.0)),
            recover_progress_scale=float(
                getattr(mpc_core_cfg, "recover_progress_scale", 0.6)
            ),
            recover_attitude_scale=float(
                getattr(mpc_core_cfg, "recover_attitude_scale", 2.0)
            ),
            settle_progress_scale=float(
                getattr(mpc_core_cfg, "settle_progress_scale", 0.0)
            ),
            settle_terminal_pos_scale=float(
                getattr(mpc_core_cfg, "settle_terminal_pos_scale", 2.0)
            ),
            settle_terminal_attitude_scale=float(
                getattr(mpc_core_cfg, "settle_terminal_attitude_scale", 1.5)
            ),
            settle_velocity_align_scale=float(
                getattr(mpc_core_cfg, "settle_velocity_align_scale", 1.5)
            ),
            settle_angular_velocity_scale=float(
                getattr(mpc_core_cfg, "settle_angular_velocity_scale", 2.0)
            ),
            hold_smoothness_scale=float(
                getattr(mpc_core_cfg, "hold_smoothness_scale", 1.5)
            ),
            hold_thruster_pair_scale=float(
                getattr(mpc_core_cfg, "hold_thruster_pair_scale", 1.2)
            ),
        )
        self.simulation.mode_manager.reset(sim_time_s=0.0)
        self.simulation.mode_state = self.simulation.mode_manager.state

        hold_required = float(getattr(contracts_cfg, "hold_duration_s", 10.0))
        self.simulation.terminal_supervisor = TerminalSupervisor(
            hold_required_s=hold_required
        )
        self.simulation.completion_gate = None
        self.simulation.completion_reached = False

        self.simulation.actuator_policy = ActuatorPolicy(
            enable_thruster_hysteresis=bool(
                getattr(actuator_cfg, "enable_thruster_hysteresis", True)
            ),
            thruster_hysteresis_on=float(
                getattr(actuator_cfg, "thruster_hysteresis_on", 0.015)
            ),
            thruster_hysteresis_off=float(
                getattr(actuator_cfg, "thruster_hysteresis_off", 0.007)
            ),
            terminal_bypass_band_m=float(
                getattr(actuator_cfg, "terminal_bypass_band_m", 0.20)
            ),
        )
        self.simulation.reference_scheduler = ReferenceScheduler()
        self.simulation.reference_slice = None
        self.simulation.solver_health = SolverHealth()
        self.simulation.pointing_guardrail = PointingGuardrail(
            enabled=bool(getattr(contracts_cfg, "pointing_guardrails_enabled", True)),
            z_error_deg_max=float(
                getattr(contracts_cfg, "pointing_z_error_deg_max", 4.0)
            ),
            x_error_deg_max=float(
                getattr(contracts_cfg, "pointing_x_error_deg_max", 6.0)
            ),
            breach_hold_s=float(getattr(contracts_cfg, "pointing_breach_hold_s", 0.30)),
            clear_hold_s=float(getattr(contracts_cfg, "pointing_clear_hold_s", 0.80)),
        )
        self.simulation.pointing_status = {
            "pointing_context_source": "none",
            "pointing_policy": "transit_free",
            "pointing_axis_world": [0.0, 0.0, 1.0],
            "z_axis_error_deg": 0.0,
            "x_axis_error_deg": 0.0,
            "pointing_guardrail_breached": False,
            "object_visible_side": None,
            "pointing_guardrail_reason": None,
        }

    def _initialize_io_helper(self) -> None:
        """Initialize IO helper for data export operations."""
        self.simulation._io = SimulationIO(self.simulation)

    def _initialize_simulation_context(self) -> None:
        """Initialize simulation context for logging."""
        from controller.shared.python.simulation.context import SimulationContext

        self.simulation.context = SimulationContext()
        self.simulation.context.dt = self.simulation.satellite.dt
        self.simulation.context.control_dt = self.simulation.control_update_interval

    def _log_initialization_summary(
        self,
        start_pos: tuple[float, ...],
        end_pos: tuple[float, ...],
        start_angle: tuple[float, float, float],
        end_angle: tuple[float, float, float],
        app_config: Any,
    ) -> None:
        """Log initialization summary."""
        logger.info("Linearized MPC Simulation initialized:")
        logger.info("INFO: Formulation: A*x[k] + B*u[k] (Linearized Dynamics)")

        def _format_euler_deg(euler: tuple[float, float, float]) -> str:
            roll, pitch, yaw = np.degrees(euler)
            return f"roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°"

        s_ang_str = _format_euler_deg(start_angle)
        end_ang_str = _format_euler_deg(end_angle)

        logger.info(f"INFO: Start: {start_pos} m, {s_ang_str}")
        logger.info(f"INFO: End: {end_pos} m, {end_ang_str}")
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

        # Initialize visualization manager
        from controller.shared.python.visualization.simulation_visualization import (
            create_simulation_visualizer,
        )

        self.simulation.visualizer = create_simulation_visualizer(self.simulation)

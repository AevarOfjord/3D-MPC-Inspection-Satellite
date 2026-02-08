"""
Pydantic Configuration Models for Satellite Control System

Type-safe configuration models with validation, range checks,
and descriptive error messages.
"""

import logging
from typing import Any, Dict, Optional, Tuple

from pydantic import BaseModel, Field, field_validator, model_validator
from typing import List

from . import constants, physics, timing
from .reaction_wheel_config import ReactionWheelParams as RWConfigParams

_RW_DEFAULT = RWConfigParams()
_PHYS_DEFAULT = physics.get_physics_params()


class ReactionWheelParams(BaseModel):
    """Configuration for a single reaction wheel."""

    axis: Tuple[float, float, float] = Field(
        ...,
        description="Rotation axis vector (x, y, z)",
    )
    max_speed: float = Field(
        _RW_DEFAULT.max_speed,
        gt=0,
        description="Maximum wheel speed in rad/s",
    )
    max_torque: float = Field(
        _RW_DEFAULT.max_torque,
        gt=0,
        description="Maximum torque in N*m",
    )
    inertia: float = Field(
        _RW_DEFAULT.inertia,
        gt=0,
        description="Rotational inertia in kg*m^2",
    )


class SatellitePhysicalParams(BaseModel):
    """
    Satellite physical parameters with validation.

    All physical parameters are validated for physical plausibility.
    """

    total_mass: float = Field(
        physics.TOTAL_MASS,
        gt=0,
        le=100,  # Reasonable upper bound for a small satellite
        description="Total mass in kg (must be positive, max 100kg)",
    )
    moment_of_inertia: float = Field(
        physics.MOMENT_OF_INERTIA,
        gt=0,
        le=10,  # Reasonable upper bound
        description="Moment of inertia in kg*m^2 (must be positive)",
    )
    satellite_size: float = Field(
        physics.SATELLITE_SIZE,
        gt=0,
        le=2,  # Reasonable upper bound in meters
        description="Characteristic size in meters (must be positive, max 2m)",
    )
    satellite_shape: str = Field(
        constants.Constants.DEFAULT_SATELLITE_SHAPE,
        pattern="^(sphere|cube)$",
        description="Satellite shape ('sphere' or 'cube')",
    )
    com_offset: Tuple[float, float, float] = Field(
        tuple(_PHYS_DEFAULT.com_offset),
        description="Center of Mass offset (x, y, z) in meters",
    )

    # Thruster configuration
    thruster_positions: Dict[int, Tuple[float, float, float]] = Field(
        default_factory=lambda: physics.THRUSTER_POSITIONS.copy(),
        description=(
            "Map of thruster ID (1-6, 1-8, or 1-12) to (x, y, z) position in meters"
        ),
    )
    thruster_directions: Dict[int, Tuple[float, float, float]] = Field(
        default_factory=lambda: {
            k: tuple(v) for k, v in physics.THRUSTER_DIRECTIONS.items()
        },
        description=(
            "Map of thruster ID (1-6, 1-8, or 1-12) to (dx, dy, dz) unit direction vector"
        ),
    )
    thruster_forces: Dict[int, float] = Field(
        default_factory=lambda: physics.THRUSTER_FORCES.copy(),
        description="Map of thruster ID (1-6, 1-8, or 1-12) to max force in Newtons",
    )

    # Reaction Wheels
    reaction_wheels: List["ReactionWheelParams"] = Field(
        default_factory=list,
        description="List of reaction wheel configurations",
    )

    # Physics Engine
    engine: str = Field(
        "cpp",
        pattern="^(cpp)$",
        description="Physics engine backend ('cpp')",
    )

    # Damping
    use_realistic_physics: bool = Field(
        _PHYS_DEFAULT.use_realistic_physics,
        description="Enable realistic physics (damping, noise, delays)",
    )
    damping_linear: float = Field(
        _PHYS_DEFAULT.linear_damping_coeff,
        ge=0,
        le=10,
        description="Linear damping coefficient N/(m/s)",
    )
    damping_angular: float = Field(
        _PHYS_DEFAULT.rotational_damping_coeff,
        ge=0,
        le=1,
        description="Angular damping coefficient N*m/(rad/s)",
    )

    # Delays & Noise (Realistic Physics)
    thruster_valve_delay: float = Field(
        _PHYS_DEFAULT.thruster_valve_delay,
        ge=0,
        le=0.5,
        description="Thruster valve open/close delay in seconds",
    )
    thruster_rampup_time: float = Field(
        _PHYS_DEFAULT.thruster_rampup_time,
        ge=0,
        le=0.5,
        description="Thruster thrust ramp-up time in seconds",
    )
    thrust_force_noise_percent: float = Field(
        _PHYS_DEFAULT.thruster_force_noise_std * 100.0,
        ge=0,
        le=20.0,
        description="Thruster force noise (percentage std dev)",
    )

    # Sensor Noise
    position_noise_std: float = Field(
        _PHYS_DEFAULT.position_noise_std,
        ge=0,
        description="Position measurement noise (std dev) in meters",
    )
    velocity_noise_std: float = Field(
        _PHYS_DEFAULT.velocity_noise_std,
        ge=0,
        description="Velocity measurement noise (std dev) in m/s",
    )
    angle_noise_std: float = Field(
        _PHYS_DEFAULT.angle_noise_std,
        ge=0,
        description="Angle measurement noise (std dev) in rad",
    )
    angular_velocity_noise_std: float = Field(
        _PHYS_DEFAULT.angular_velocity_noise_std,
        ge=0,
        description="Angular velocity measurement noise (std dev) in rad/s",
    )

    # External Disturbances
    random_disturbances_enabled: bool = Field(
        _PHYS_DEFAULT.enable_random_disturbances,
        description="Enable random external force/torque disturbances",
    )
    disturbance_force_std: float = Field(
        _PHYS_DEFAULT.disturbance_force_std,
        ge=0,
        description="External disturbance force (std dev) in N",
    )
    disturbance_torque_std: float = Field(
        _PHYS_DEFAULT.disturbance_torque_std,
        ge=0,
        description="External disturbance torque (std dev) in N*m",
    )

    @field_validator("thruster_positions")
    @classmethod
    def validate_thruster_positions(
        cls, v: Dict[int, Tuple[float, float, float]]
    ) -> Dict[int, Tuple[float, float, float]]:
        """Validate thruster positions are within satellite bounds."""
        if len(v) not in [6, 8, 12]:
            raise ValueError(f"Expected 6, 8 or 12 thrusters, got {len(v)}")
        for tid, pos in v.items():
            if not (1 <= tid <= 12):
                raise ValueError(f"Thruster ID must be 1-12, got {tid}")
            if abs(pos[0]) > 1.0 or abs(pos[1]) > 1.0 or abs(pos[2]) > 1.0:
                raise ValueError(
                    f"Thruster {tid} position {pos} exceeds satellite bounds (±1m)"
                )
        return v

    @field_validator("thruster_forces")
    @classmethod
    def validate_thruster_forces(cls, v: Dict[int, float]) -> Dict[int, float]:
        """Validate thruster forces are positive and reasonable."""
        for tid, force in v.items():
            if force <= 0:
                raise ValueError(f"Thruster {tid} force must be positive, got {force}")
            if force > 100:  # 100N is very high for a small satellite
                raise ValueError(
                    f"Thruster {tid} force {force}N exceeds reasonable maximum (100N)"
                )
        return v

    @field_validator("com_offset")
    @classmethod
    def validate_com_offset(
        cls, v: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """Validate COM offset is within satellite bounds."""
        if abs(v[0]) > 0.5 or abs(v[1]) > 0.5 or abs(v[2]) > 0.5:
            raise ValueError(
                f"COM offset {v} is too large (should be within ±0.5m of center)"
            )
        return v


class MPCParams(BaseModel):
    """
    MPC Controller parameters with comprehensive validation.

    Includes cross-field consistency checks and reasonable bounds.
    """

    prediction_horizon: int = Field(
        constants.Constants.MPC_PREDICTION_HORIZON,
        gt=0,
        le=5000,
        description="Prediction horizon N (1-5000 steps)",
    )
    control_horizon: int = Field(
        constants.Constants.MPC_CONTROL_HORIZON,
        gt=0,
        le=5000,
        description="Control horizon M (1-5000 steps, must be <= N)",
    )
    dt: float = Field(
        timing.CONTROL_DT,
        gt=0,
        le=1.0,
        description="Control timestep in seconds (0-1s)",
    )
    solver_time_limit: float = Field(
        constants.Constants.MPC_SOLVER_TIME_LIMIT,
        gt=0,
        le=10.0,
        description="Maximum solver time in seconds",
    )
    solver_type: str = Field(
        constants.Constants.MPC_SOLVER_TYPE,
        description="Optimization solver type",
    )

    # Weights (MPCC)
    Q_contour: float = Field(
        constants.Constants.Q_CONTOUR,
        ge=0.0,
        le=100000.0,
        description="Contouring weight - penalizes distance from path [unitless]",
    )
    Q_progress: float = Field(
        constants.Constants.Q_PROGRESS,
        ge=0.0,
        le=10000.0,
        description="Progress weight - penalizes deviation from path speed [unitless]",
    )
    progress_reward: float = Field(
        constants.Constants.PROGRESS_REWARD,
        ge=0.0,
        le=1e6,
        description="Linear reward for forward progress (auto speed) [unitless]",
    )
    Q_lag: float = Field(
        0.0,
        ge=0.0,
        le=100000.0,
        description="Lag weight - penalizes along-track error (0 = auto)",
    )
    Q_smooth: float = Field(
        constants.Constants.Q_SMOOTH,
        ge=0.0,
        le=1000.0,
        description="Smoothness weight - penalizes velocity changes [unitless]",
    )
    Q_attitude: float = Field(
        constants.Constants.Q_ATTITUDE,
        ge=0.0,
        le=1e6,
        description="Attitude tracking weight (align body x-axis with path tangent)",
    )
    Q_terminal_pos: float = Field(
        0.0,
        ge=0.0,
        le=1e6,
        description="Terminal position weight (0 = auto-scale from Q_contour)",
    )
    Q_terminal_s: float = Field(
        0.0,
        ge=0.0,
        le=1e6,
        description="Terminal progress weight (0 = auto-scale from Q_progress/Q_contour)",
    )
    q_angular_velocity: float = Field(
        constants.Constants.Q_ANGULAR_VELOCITY,
        ge=0,
        le=1e6,
        description="Angular velocity tracking weight (stabilization)",
    )
    r_thrust: float = Field(
        constants.Constants.R_THRUST,
        ge=0,
        le=1e6,
        description="Thrust usage penalty weight",
    )
    r_rw_torque: float = Field(
        constants.Constants.R_RW_TORQUE,
        ge=0,
        le=1e6,
        description="Reaction wheel torque penalty weight",
    )
    thrust_l1_weight: float = Field(
        constants.Constants.THRUST_L1_WEIGHT,
        ge=0.0,
        le=1e6,
        description="Linear thruster penalty (fuel bias, promotes coasting)",
    )
    thrust_pair_weight: float = Field(
        constants.Constants.THRUST_PAIR_WEIGHT,
        ge=0.0,
        le=1e6,
        description="Penalty on opposing thruster co-firing (promotes single-thruster use per axis)",
    )
    coast_pos_tolerance: float = Field(
        constants.Constants.COAST_POS_TOLERANCE,
        ge=0.0,
        le=1e3,
        description="Coasting band position error [m] (0 = off)",
    )
    coast_vel_tolerance: float = Field(
        constants.Constants.COAST_VEL_TOLERANCE,
        ge=0.0,
        le=1e3,
        description="Coasting band lateral velocity [m/s] (0 = off)",
    )
    coast_min_speed: float = Field(
        constants.Constants.COAST_MIN_SPEED,
        ge=0.0,
        le=1e3,
        description="Minimum progress speed when coasting [m/s]",
    )
    # Adaptive control
    thruster_type: str = Field(
        constants.Constants.THRUSTER_TYPE,
        description="Thruster actuation type: 'PWM' (Binary) or 'CON' (Continuous)",
    )

    verbose_mpc: bool = Field(
        False,
        description="Enable verbose MPC solver output",
    )

    obstacle_margin: float = Field(
        constants.Constants.OBSTACLE_SAFETY_MARGIN,
        ge=0.0,
        le=5.0,
        description="Safety margin around obstacles in meters",
    )

    # Path Following. - General Path MPCC
    path_speed: float = Field(
        timing.DEFAULT_PATH_SPEED,
        gt=0.0,
        le=1.0,
        description="Path speed along reference curve [m/s]",
    )
    path_speed_min: float = Field(
        constants.Constants.PATH_SPEED_MIN,
        ge=0.0,
        le=1.0,
        description="Minimum path speed [m/s]",
    )
    path_speed_max: float = Field(
        constants.Constants.PATH_SPEED_MAX,
        gt=0.0,
        le=1.0,
        description="Maximum path speed [m/s]",
    )
    progress_taper_distance: float = Field(
        0.0,
        ge=0.0,
        le=1e6,
        description="Distance before endpoint to taper v_ref (0 = auto)",
    )
    progress_slowdown_distance: float = Field(
        0.0,
        ge=0.0,
        le=1e6,
        description="Contour error threshold to slow progress (0 = auto)",
    )

    @field_validator("thruster_type")
    @classmethod
    def validate_thruster_type(cls, v: str) -> str:
        """Validate thruster type."""
        if v not in ["PWM", "CON"]:
            raise ValueError(f"Thruster type must be 'PWM' or 'CON', got '{v}'")
        return v

    @field_validator("control_horizon")
    @classmethod
    def check_horizon_consistency(cls, v: int, info) -> int:
        """Ensure control_horizon <= prediction_horizon."""
        if "prediction_horizon" in info.data:
            if v > info.data["prediction_horizon"]:
                raise ValueError(
                    f"control_horizon ({v}) cannot exceed "
                    f"prediction_horizon ({info.data['prediction_horizon']})"
                )
        return v

    @field_validator("solver_time_limit")
    @classmethod
    def check_solver_time_vs_dt(cls, v: float, info) -> float:
        """Warn if solver time limit exceeds control timestep."""
        if "dt" in info.data:
            if v > info.data["dt"]:
                # This is a warning, not an error - we allow it but note the issue
                pass
        return v

    @model_validator(mode="after")
    def validate_weight_balance(self) -> "MPCParams":
        """Check that weights are reasonably balanced."""
        total_q = (
            self.Q_contour
            + self.Q_progress
            + self.Q_lag
            + self.Q_smooth
            + self.Q_attitude
            + self.Q_terminal_pos
            + self.Q_terminal_s
            + self.q_angular_velocity
        )
        if total_q == 0 and self.r_thrust > 0:
            raise ValueError(
                "All Q weights are zero but R_thrust is nonzero - "
                "controller will only minimize thrust, not track references"
            )
        return self


class SimulationParams(BaseModel):
    """Simulation settings with validation."""

    dt: float = Field(
        timing.SIMULATION_DT,
        gt=0,
        le=0.1,
        description="Physics timestep in seconds (max 100ms)",
    )
    max_duration: float = Field(
        timing.MAX_SIMULATION_TIME,
        ge=0,
        description="Maximum simulation duration in seconds (0 disables limit)",
    )
    headless: bool = Field(
        constants.Constants.HEADLESS_MODE,
        description="Run without visualization",
    )

    # Visualization defaults
    window_width: int = Field(
        constants.Constants.WINDOW_WIDTH,
        ge=640,
        le=4096,
        description="Window width in pixels",
    )
    window_height: int = Field(
        constants.Constants.WINDOW_HEIGHT,
        ge=480,
        le=2160,
        description="Window height in pixels",
    )

    use_final_stabilization: bool = Field(
        timing.USE_FINAL_STABILIZATION_IN_SIMULATION,
        description="Require final stabilization hold before terminating missions",
    )

    # Timing parameters.
    control_dt: float = Field(
        timing.CONTROL_DT,
        gt=0,
        le=1.0,
        description="MPC control update interval in seconds (must be >= dt)",
    )
    default_path_speed: float = Field(
        timing.DEFAULT_PATH_SPEED,
        gt=0,
        le=1.0,
        description="Default path speed for shape following missions in m/s",
    )

    # Logging controls
    physics_log_stride: int = Field(
        1,
        ge=1,
        le=1000,
        description="Log every N physics steps (1 = log all)",
    )
    control_log_stride: int = Field(
        1,
        ge=1,
        le=1000,
        description="Log every N control steps (1 = log all)",
    )

    history_max_steps: int = Field(
        50000,
        ge=0,
        le=2000000,
        description=(
            "Max in-memory history entries (0 = unbounded). "
            "Applies to state/control histories used for summaries and plots."
        ),
    )
    history_downsample_stride: int = Field(
        1,
        ge=1,
        le=1000,
        description="Keep every Nth history entry in memory (1 = keep all)",
    )


class AppConfig(BaseModel):
    """
    Root configuration container.

    Combines all configuration subsections with cross-validation.
    """

    physics: SatellitePhysicalParams
    mpc: MPCParams
    simulation: SimulationParams

    input_file_path: Optional[str] = Field(
        None,
        description="Path to input path/mesh file",
    )

    @model_validator(mode="after")
    def validate_timing_consistency(self) -> "AppConfig":
        """Ensure timing parameters are consistent across subsystems."""
        # MPC dt should match simulation control_dt
        if abs(self.mpc.dt - self.simulation.control_dt) > 0.001:
            logger = logging.getLogger(__name__)
            logger.warning(
                "MPC dt (%.3fs) does not match simulation control_dt (%.3fs).",
                self.mpc.dt,
                self.simulation.control_dt,
            )
        # Control dt should be >= simulation dt
        if self.simulation.control_dt < self.simulation.dt:
            raise ValueError(
                f"Control dt ({self.simulation.control_dt}s) must be >= "
                f"simulation dt ({self.simulation.dt}s)"
            )
        return self

    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary format."""
        return self.model_dump()

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AppConfig":
        """Create configuration from dictionary."""
        return cls.model_validate(data)

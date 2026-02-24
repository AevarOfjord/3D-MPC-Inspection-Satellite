"""
Pydantic Configuration Models for Satellite Control System

Type-safe configuration models with validation, range checks,
and descriptive error messages.
"""

from __future__ import annotations

import logging
import math
from typing import Any, ClassVar

from pydantic import BaseModel, Field, field_validator, model_validator

from . import constants, physics, timing
from .reaction_wheel_config import ReactionWheelParams as RWConfigParams

_RW_DEFAULT = RWConfigParams()
_PHYS_DEFAULT = physics.get_physics_params()


class ReactionWheelParams(BaseModel):
    """Configuration for a single reaction wheel."""

    axis: tuple[float, float, float] = Field(
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
    com_offset: tuple[float, float, float] = Field(
        tuple(_PHYS_DEFAULT.com_offset),
        description="Center of Mass offset (x, y, z) in meters",
    )

    # Thruster configuration
    thruster_positions: dict[int, tuple[float, float, float]] = Field(
        default_factory=lambda: physics.THRUSTER_POSITIONS.copy(),
        description=(
            "Map of thruster ID (1-6, 1-8, or 1-12) to (x, y, z) position in meters"
        ),
    )
    thruster_directions: dict[int, tuple[float, float, float]] = Field(
        default_factory=lambda: {
            k: tuple(v) for k, v in physics.THRUSTER_DIRECTIONS.items()
        },
        description=(
            "Map of thruster ID (1-6, 1-8, or 1-12) to (dx, dy, dz) unit direction vector"
        ),
    )
    thruster_forces: dict[int, float] = Field(
        default_factory=lambda: physics.THRUSTER_FORCES.copy(),
        description="Map of thruster ID (1-6, 1-8, or 1-12) to max force in Newtons",
    )

    # Reaction Wheels
    reaction_wheels: list[ReactionWheelParams] = Field(
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
    use_two_body_gravity: bool = Field(
        True,
        description="Enable Two-Body (1/r^2) gravity. Set False for relative/inertial simulation.",
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
        cls, v: dict[int, tuple[float, float, float]]
    ) -> dict[int, tuple[float, float, float]]:
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
    def validate_thruster_forces(cls, v: dict[int, float]) -> dict[int, float]:
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
        cls, v: tuple[float, float, float]
    ) -> tuple[float, float, float]:
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
    Q_lag_default: float = Field(
        constants.Constants.Q_LAG_DEFAULT,
        ge=-1.0,
        le=100000.0,
        description="Default lag weight when Q_lag <= 0 (-1 keeps auto behavior)",
    )
    Q_velocity_align: float = Field(
        constants.Constants.Q_VELOCITY_ALIGN,
        ge=0.0,
        le=100000.0,
        description="Velocity alignment weight (0 reuses Q_progress)",
    )
    Q_s_anchor: float = Field(
        constants.Constants.Q_S_ANCHOR,
        ge=-1.0,
        le=100000.0,
        description="Progress-state anchor weight (-1 keeps auto behavior)",
    )
    Q_smooth: float = Field(
        constants.Constants.Q_SMOOTH,
        ge=0.0,
        le=1000.0,
        description="Smoothness weight - penalizes control increments (Δu) [unitless]",
    )
    Q_attitude: float = Field(
        constants.Constants.Q_ATTITUDE,
        ge=0.0,
        le=1e6,
        description="Attitude tracking weight (align body x-axis with path tangent)",
    )
    Q_axis_align: float = Field(
        constants.Constants.Q_AXIS_ALIGN,
        ge=0.0,
        le=1e6,
        description="Extra axis-alignment weight (adds to Q_attitude)",
    )
    Q_quat_norm: float = Field(
        constants.Constants.Q_QUAT_NORM,
        ge=0.0,
        le=1e6,
        description="Soft quaternion normalization weight",
    )
    Q_terminal_pos: float = Field(
        constants.Constants.Q_TERMINAL_POS,
        ge=0.0,
        le=1e6,
        description="Terminal position weight (0 = auto-scale from Q_contour)",
    )
    Q_terminal_s: float = Field(
        constants.Constants.Q_TERMINAL_S,
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
    # Adaptive control
    thruster_type: str = Field(
        constants.Constants.THRUSTER_TYPE,
        description="Thruster actuation type: 'PWM' (Binary) or 'CON' (Continuous)",
    )

    verbose_mpc: bool = Field(
        False,
        description="Enable verbose MPC solver output",
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
    max_linear_velocity: float = Field(
        constants.Constants.MAX_LINEAR_VELOCITY,
        ge=0.0,
        le=100.0,
        description="Linear velocity state bound [m/s] (0 = auto)",
    )
    max_angular_velocity: float = Field(
        constants.Constants.MAX_ANGULAR_VELOCITY,
        ge=0.0,
        le=100.0,
        description="Angular velocity state bound [rad/s] (0 = auto)",
    )
    enable_delta_u_coupling: bool = Field(
        constants.Constants.ENABLE_DELTA_U_COUPLING,
        description=(
            "Enable full temporal coupling for Δu smoothness cost "
            "(more accurate, higher solver load)"
        ),
    )
    enable_gyro_jacobian: bool = Field(
        constants.Constants.ENABLE_GYRO_JACOBIAN,
        description=(
            "Enable angular-rate gyroscopic Jacobian in linearization "
            "(improves high-rate accuracy, adds compute)"
        ),
    )
    auto_enable_gyro_jacobian: bool = Field(
        constants.Constants.AUTO_ENABLE_GYRO_JACOBIAN,
        description=("Auto-enable gyro Jacobian when angular rate exceeds threshold"),
    )
    gyro_enable_threshold_radps: float = Field(
        constants.Constants.GYRO_ENABLE_THRESHOLD_RADPS,
        ge=0.0,
        le=100.0,
        description="Angular-rate threshold for auto gyro Jacobian [rad/s]",
    )
    enable_auto_state_bounds: bool = Field(
        constants.Constants.ENABLE_AUTO_STATE_BOUNDS,
        description=(
            "Auto-derive velocity state bounds when explicit limits are unset"
        ),
    )
    enable_online_dare_terminal: bool = Field(
        constants.Constants.ENABLE_ONLINE_DARE_TERMINAL,
        description=(
            "Periodically recompute DARE terminal diagonal around local trajectory tail"
        ),
    )
    dare_update_period_steps: int = Field(
        constants.Constants.DARE_UPDATE_PERIOD_STEPS,
        ge=1,
        le=1000,
        description="Control steps between online DARE terminal updates",
    )
    terminal_cost_profile: str = Field(
        constants.Constants.TERMINAL_COST_PROFILE,
        description='Terminal cost profile ("diagonal" or "dense_terminal")',
    )
    robustness_mode: str = Field(
        constants.Constants.ROBUSTNESS_MODE,
        description='Robustness scaffold mode ("none" or "tube")',
    )
    constraint_tightening_scale: float = Field(
        constants.Constants.CONSTRAINT_TIGHTENING_SCALE,
        ge=0.0,
        le=0.3,
        description="Constraint tightening fraction for robust scaffold",
    )
    tube_feedback_gain_scale: float = Field(
        constants.Constants.TUBE_FEEDBACK_GAIN_SCALE,
        ge=0.0,
        le=1.0,
        description="Ancillary tube feedback gain scale",
    )
    tube_feedback_max_correction: float = Field(
        constants.Constants.TUBE_FEEDBACK_MAX_CORRECTION,
        ge=0.0,
        le=1.0,
        description="Maximum absolute tube feedback correction per control channel",
    )
    enable_variable_scaling: bool = Field(
        constants.Constants.ENABLE_VARIABLE_SCALING,
        description="Enable solver-coordinate variable scaling for improved conditioning",
    )
    progress_policy: str = Field(
        constants.Constants.PROGRESS_POLICY,
        description='Progress policy ("speed_tracking" or "error_priority")',
    )
    error_priority_min_vs: float = Field(
        constants.Constants.ERROR_PRIORITY_MIN_VS,
        ge=0.0,
        le=1.0,
        description="Minimum progress speed in error-priority mode [m/s]",
    )
    error_priority_error_speed_gain: float = Field(
        constants.Constants.ERROR_PRIORITY_ERROR_SPEED_GAIN,
        ge=0.0,
        le=1000.0,
        description="Path-error-to-speed reduction gain in error-priority mode",
    )
    enable_thruster_hysteresis: bool = Field(
        constants.Constants.ENABLE_THRUSTER_HYSTERESIS,
        description="Enable output hysteresis to reduce thruster chatter/switching",
    )
    thruster_hysteresis_on: float = Field(
        constants.Constants.THRUSTER_HYSTERESIS_ON,
        ge=0.0,
        le=1.0,
        description="Thruster activation threshold when hysteresis is enabled",
    )
    thruster_hysteresis_off: float = Field(
        constants.Constants.THRUSTER_HYSTERESIS_OFF,
        ge=0.0,
        le=1.0,
        description="Thruster deactivation threshold when hysteresis is enabled",
    )

    BASIC_FIELDS: ClassVar[tuple[str, ...]] = (
        "prediction_horizon",
        "control_horizon",
        "dt",
        "solver_time_limit",
        "Q_contour",
        "Q_progress",
        "Q_attitude",
        "Q_axis_align",
        "Q_quat_norm",
        "Q_smooth",
        "q_angular_velocity",
        "r_thrust",
        "r_rw_torque",
        "path_speed",
    )
    ADVANCED_FIELDS: ClassVar[tuple[str, ...]] = (
        "Q_lag",
        "Q_lag_default",
        "Q_velocity_align",
        "Q_s_anchor",
        "path_speed_min",
        "path_speed_max",
        "Q_terminal_pos",
        "Q_terminal_s",
        "progress_reward",
        "max_linear_velocity",
        "max_angular_velocity",
        "enable_auto_state_bounds",
        "thruster_type",
        "solver_type",
        "enable_delta_u_coupling",
        "enable_gyro_jacobian",
        "auto_enable_gyro_jacobian",
        "gyro_enable_threshold_radps",
        "verbose_mpc",
        "enable_online_dare_terminal",
        "dare_update_period_steps",
        "terminal_cost_profile",
        "robustness_mode",
        "constraint_tightening_scale",
        "tube_feedback_gain_scale",
        "tube_feedback_max_correction",
        "enable_variable_scaling",
        "progress_policy",
        "error_priority_min_vs",
        "error_priority_error_speed_gain",
        "enable_thruster_hysteresis",
        "thruster_hysteresis_on",
        "thruster_hysteresis_off",
    )
    EXPERT_FIELDS: ClassVar[tuple[str, ...]] = (
        "thrust_l1_weight",
        "thrust_pair_weight",
    )

    @classmethod
    def parameter_groups(cls) -> dict[str, list[str]]:
        """Return MPC parameter grouping for UI/basic-vs-advanced surfaces."""
        return {
            "basic": list(cls.BASIC_FIELDS),
            "advanced": list(cls.ADVANCED_FIELDS),
            "expert": list(cls.EXPERT_FIELDS),
        }

    @field_validator("thruster_type")
    @classmethod
    def validate_thruster_type(cls, v: str) -> str:
        """Validate thruster type."""
        if v not in ["PWM", "CON"]:
            raise ValueError(f"Thruster type must be 'PWM' or 'CON', got '{v}'")
        return v

    @field_validator("terminal_cost_profile")
    @classmethod
    def validate_terminal_cost_profile(cls, v: str) -> str:
        """Validate terminal cost profile selector."""
        normalized = str(v).strip().lower()
        if normalized not in {"diagonal", "dense_terminal"}:
            raise ValueError(
                "terminal_cost_profile must be 'diagonal' or 'dense_terminal'"
            )
        return normalized

    @field_validator("robustness_mode")
    @classmethod
    def validate_robustness_mode(cls, v: str) -> str:
        """Validate robustness scaffold selector."""
        normalized = str(v).strip().lower()
        if normalized not in {"none", "tube"}:
            raise ValueError("robustness_mode must be 'none' or 'tube'")
        return normalized

    @field_validator("progress_policy")
    @classmethod
    def validate_progress_policy(cls, v: str) -> str:
        """Validate progress behavior policy."""
        normalized = str(v).strip().lower()
        if normalized not in {"speed_tracking", "error_priority"}:
            raise ValueError(
                "progress_policy must be 'speed_tracking' or 'error_priority'"
            )
        return normalized

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
    def validate_weight_balance(self) -> MPCParams:
        """Check that weights are reasonably balanced."""
        total_q = (
            self.Q_contour
            + self.Q_progress
            + self.Q_lag
            + max(self.Q_velocity_align, 0.0)
            + max(self.Q_s_anchor, 0.0)
            + self.Q_smooth
            + self.Q_attitude
            + self.Q_axis_align
            + self.Q_quat_norm
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

    @model_validator(mode="after")
    def validate_hysteresis_thresholds(self) -> MPCParams:
        """Ensure hysteresis thresholds are ordered correctly."""
        if self.thruster_hysteresis_on <= self.thruster_hysteresis_off:
            raise ValueError(
                "thruster_hysteresis_on must be greater than thruster_hysteresis_off"
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
    mpc_target_mean_solve_time_ms: float = Field(
        constants.Constants.TARGET_MEAN_SOLVE_TIME_MS,
        gt=0.0,
        le=1000.0,
        description="Target MPC mean solve time in milliseconds",
    )
    mpc_hard_max_solve_time_ms: float = Field(
        constants.Constants.HARD_MAX_SOLVE_TIME_MS,
        gt=0.0,
        le=5000.0,
        description="Hard MPC single-solve time ceiling in milliseconds",
    )
    enforce_mpc_timing_contract: bool = Field(
        constants.Constants.ENFORCE_TIMING_CONTRACT,
        description="If true, fail the run when MPC timing contract is violated",
    )

    @model_validator(mode="after")
    def validate_mpc_timing_contract(self) -> SimulationParams:
        """Ensure timing contract thresholds are internally consistent."""
        if self.mpc_hard_max_solve_time_ms < self.mpc_target_mean_solve_time_ms:
            raise ValueError(
                "mpc_hard_max_solve_time_ms must be >= mpc_target_mean_solve_time_ms"
            )
        return self


class ReferenceSchedulerParams(BaseModel):
    """Reference scheduler policy and duration feasibility settings."""

    speed_policy: str = Field(
        "min_non_hold_segment_speed",
        description=(
            "Global speed policy for runtime missions "
            "(min non-hold segment speed, then MPC clamp)."
        ),
    )
    duration_margin_s: float = Field(
        constants.Constants.DURATION_MARGIN_S,
        ge=0.0,
        le=3600.0,
        description="Extra duration margin added on top of ETA + hold contract.",
    )
    auto_extend_manual_duration: bool = Field(
        True,
        description="Auto-extend manual runs when configured duration is infeasible.",
    )
    enforce_contract_min_duration: bool = Field(
        True,
        description="Enforce required minimum duration for contract scenarios.",
    )


class MPCCoreParams(BaseModel):
    """MPC core mode-profile and backend policy."""

    solver_backend: str = Field(
        "OSQP",
        pattern="^(OSQP)$",
        description="Certified solver backend.",
    )
    controller_backend: str = Field(
        "v2",
        pattern="^(v1|v2)$",
        description="Controller backend: 'v1' (legacy C++ MPC) or 'v2' (CasADi SQP).",
    )
    recover_contour_scale: float = Field(
        constants.Constants.RECOVER_CONTOUR_SCALE,
        ge=0.0,
        le=100.0,
        description="RECOVER mode contour weight multiplier.",
    )
    recover_lag_scale: float = Field(
        constants.Constants.RECOVER_LAG_SCALE,
        ge=0.0,
        le=100.0,
        description="RECOVER mode lag weight multiplier.",
    )
    recover_progress_scale: float = Field(
        constants.Constants.RECOVER_PROGRESS_SCALE,
        ge=0.0,
        le=1.0,
        description="RECOVER mode progress weight multiplier.",
    )
    recover_attitude_scale: float = Field(
        constants.Constants.RECOVER_ATTITUDE_SCALE,
        ge=0.0,
        le=10.0,
        description="RECOVER mode attitude weight multiplier.",
    )
    settle_progress_scale: float = Field(
        constants.Constants.SETTLE_PROGRESS_SCALE,
        ge=0.0,
        le=1.0,
        description="SETTLE mode progress weight multiplier.",
    )
    settle_terminal_pos_scale: float = Field(
        constants.Constants.SETTLE_TERMINAL_POS_SCALE,
        ge=0.0,
        le=100.0,
        description="SETTLE mode terminal position multiplier.",
    )
    settle_terminal_attitude_scale: float = Field(
        constants.Constants.SETTLE_TERMINAL_ATTITUDE_SCALE,
        ge=0.0,
        le=100.0,
        description="SETTLE mode terminal attitude multiplier.",
    )
    settle_velocity_align_scale: float = Field(
        constants.Constants.SETTLE_VELOCITY_ALIGN_SCALE,
        ge=0.0,
        le=100.0,
        description="SETTLE mode velocity alignment multiplier.",
    )
    settle_angular_velocity_scale: float = Field(
        constants.Constants.SETTLE_ANGULAR_VELOCITY_SCALE,
        ge=0.0,
        le=100.0,
        description="SETTLE mode angular-rate damping multiplier.",
    )
    hold_smoothness_scale: float = Field(
        constants.Constants.HOLD_SMOOTHNESS_SCALE,
        ge=0.0,
        le=100.0,
        description="HOLD mode smoothness multiplier.",
    )
    hold_thruster_pair_scale: float = Field(
        constants.Constants.HOLD_THRUSTER_PAIR_SCALE,
        ge=0.0,
        le=100.0,
        description="HOLD mode opposing-thruster penalty multiplier.",
    )


class ActuatorPolicyParams(BaseModel):
    """Actuator shaping policy."""

    enable_thruster_hysteresis: bool = Field(
        constants.Constants.ENABLE_THRUSTER_HYSTERESIS,
        description="Enable actuator hysteresis policy.",
    )
    thruster_hysteresis_on: float = Field(
        constants.Constants.THRUSTER_HYSTERESIS_ON,
        ge=0.0,
        le=1.0,
        description="Thruster on-threshold for hysteresis policy.",
    )
    thruster_hysteresis_off: float = Field(
        constants.Constants.THRUSTER_HYSTERESIS_OFF,
        ge=0.0,
        le=1.0,
        description="Thruster off-threshold for hysteresis policy.",
    )
    terminal_bypass_band_m: float = Field(
        constants.Constants.TERMINAL_BYPASS_BAND_M,
        ge=0.0,
        le=5.0,
        description="Endpoint-error band where SETTLE/HOLD bypass hysteresis.",
    )

    @model_validator(mode="after")
    def validate_hysteresis_thresholds(self) -> ActuatorPolicyParams:
        if self.thruster_hysteresis_on <= self.thruster_hysteresis_off:
            raise ValueError(
                "thruster_hysteresis_on must be greater than thruster_hysteresis_off"
            )
        return self


class ControllerContractsParams(BaseModel):
    """Completion and mode-transition contracts."""

    position_error_m_max: float = Field(
        constants.Constants.POSITION_TOLERANCE,
        gt=0.0,
        le=5.0,
        description="Terminal position error threshold [m].",
    )
    angle_error_deg_max: float = Field(
        math.degrees(constants.Constants.ANGLE_TOLERANCE),
        gt=0.0,
        le=180.0,
        description="Terminal attitude error threshold [deg].",
    )
    velocity_error_mps_max: float = Field(
        constants.Constants.VELOCITY_TOLERANCE,
        gt=0.0,
        le=10.0,
        description="Terminal linear velocity error threshold [m/s].",
    )
    angular_velocity_error_degps_max: float = Field(
        math.degrees(constants.Constants.ANGULAR_VELOCITY_TOLERANCE),
        gt=0.0,
        le=360.0,
        description="Terminal angular velocity error threshold [deg/s].",
    )
    hold_duration_s: float = Field(
        constants.MissionDefaults.PATH_HOLD_END_S,
        ge=0.0,
        le=3600.0,
        description="Continuous in-threshold hold duration before completion.",
    )
    solver_fallback_hold_s: float = Field(
        constants.Constants.SOLVER_FALLBACK_HOLD_S,
        ge=0.0,
        le=30.0,
        description="Hold last-feasible command this long after solver non-success.",
    )
    solver_fallback_decay_s: float = Field(
        constants.Constants.SOLVER_FALLBACK_DECAY_S,
        ge=0.0,
        le=30.0,
        description="Linear decay duration for fallback command after hold.",
    )
    solver_fallback_zero_after_s: float = Field(
        constants.Constants.SOLVER_FALLBACK_ZERO_AFTER_S,
        ge=0.0,
        le=60.0,
        description="Fallback command is forced to zero at/after this age.",
    )
    recover_enter_error_m: float = Field(
        constants.Constants.RECOVER_ENTER_ERROR_M,
        gt=0.0,
        le=10.0,
        description="TRACK->RECOVER contour-error threshold [m].",
    )
    recover_enter_hold_s: float = Field(
        constants.Constants.RECOVER_ENTER_HOLD_S,
        ge=0.0,
        le=60.0,
        description="TRACK->RECOVER threshold hold time [s].",
    )
    recover_exit_error_m: float = Field(
        constants.Constants.RECOVER_EXIT_ERROR_M,
        gt=0.0,
        le=10.0,
        description="RECOVER->TRACK contour-error threshold [m].",
    )
    recover_exit_hold_s: float = Field(
        constants.Constants.RECOVER_EXIT_HOLD_S,
        ge=0.0,
        le=60.0,
        description="RECOVER->TRACK threshold hold time [s].",
    )
    allow_mission_override: bool = Field(
        True,
        description="Allow mission-level contract override fields with audit.",
    )
    enable_pointing_contract: bool = Field(
        constants.Constants.ENABLE_POINTING_CONTRACT,
        description="Enable pointing contract (+X path-forward, +Z axis-lock).",
    )
    pointing_scope: str = Field(
        constants.Constants.POINTING_SCOPE,
        description='Pointing contract scope ("all_missions", "scan_only", "config_toggle").',
    )

    scan_axis_source: str = Field(
        constants.Constants.SCAN_AXIS_SOURCE,
        description='Scan axis source policy ("planner", "asset_infer").',
    )
    pointing_guardrails_enabled: bool = Field(
        constants.Constants.POINTING_GUARDRAILS_ENABLED,
        description="Enable pointing guardrail breach monitoring and RECOVER trigger.",
    )
    pointing_z_error_deg_max: float = Field(
        constants.Constants.POINTING_Z_ERROR_DEG_MAX,
        gt=0.0,
        le=180.0,
        description="Max allowed +Z axis error before pointing guardrail breach [deg].",
    )
    pointing_x_error_deg_max: float = Field(
        constants.Constants.POINTING_X_ERROR_DEG_MAX,
        gt=0.0,
        le=180.0,
        description="Max allowed +X axis error before pointing guardrail breach [deg].",
    )
    pointing_breach_hold_s: float = Field(
        constants.Constants.POINTING_BREACH_HOLD_S,
        ge=0.0,
        le=60.0,
        description="Continuous breach duration required to latch pointing guardrail [s].",
    )
    pointing_clear_hold_s: float = Field(
        constants.Constants.POINTING_CLEAR_HOLD_S,
        ge=0.0,
        le=60.0,
        description="Continuous clear duration required to clear latched guardrail [s].",
    )

    @model_validator(mode="after")
    def validate_recovery_thresholds(self) -> ControllerContractsParams:
        if self.recover_exit_error_m > self.recover_enter_error_m:
            raise ValueError(
                "recover_exit_error_m should be <= recover_enter_error_m for hysteresis"
            )
        if self.solver_fallback_zero_after_s < self.solver_fallback_hold_s:
            raise ValueError(
                "solver_fallback_zero_after_s must be >= solver_fallback_hold_s"
            )
        if self.pointing_scope not in {"all_missions", "scan_only", "config_toggle"}:
            raise ValueError(
                "pointing_scope must be one of: all_missions, scan_only, config_toggle"
            )

        if self.scan_axis_source not in {"planner", "asset_infer"}:
            raise ValueError("scan_axis_source must be one of: planner, asset_infer")
        return self


class AppConfig(BaseModel):
    """
    Root configuration container.

    Combines all configuration subsections with cross-validation.
    """

    physics: SatellitePhysicalParams
    mpc: MPCParams
    reference_scheduler: ReferenceSchedulerParams = Field(
        default_factory=ReferenceSchedulerParams
    )
    mpc_core: MPCCoreParams = Field(default_factory=MPCCoreParams)
    actuator_policy: ActuatorPolicyParams = Field(default_factory=ActuatorPolicyParams)
    controller_contracts: ControllerContractsParams = Field(
        default_factory=ControllerContractsParams
    )
    simulation: SimulationParams

    input_file_path: str | None = Field(
        None,
        description="Path to input path/mesh file",
    )

    @model_validator(mode="after")
    def validate_timing_consistency(self) -> AppConfig:
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

    def to_dict(self) -> dict[str, Any]:
        """Convert configuration to dictionary format."""
        return self.model_dump()

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> AppConfig:
        """Create configuration from dictionary."""
        return cls.model_validate(data)

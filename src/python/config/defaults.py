"""
Default Configuration Factory

Provides factory functions to create default configuration objects.
This centralizes all default values for the Pydantic config models.
"""

from config import constants, physics, timing
from config.models import (
    ActuatorPolicyParams,
    AppConfig,
    ControllerContractsParams,
    MPCCoreParams,
    MPCParams,
    ReactionWheelParams,
    ReferenceSchedulerParams,
    SatellitePhysicalParams,
    SimulationParams,
)
from config.reaction_wheel_config import get_reaction_wheel_config


def create_default_app_config() -> AppConfig:
    """
    Create default application configuration.

    Constructs a complete AppConfig using default values from
    physics, timing, and constants modules.

    Returns:
        AppConfig with default parameters
    """
    # Physics
    rw_cfg = get_reaction_wheel_config()
    phys_defaults = physics.get_physics_params()
    phys = SatellitePhysicalParams(
        total_mass=physics.TOTAL_MASS,
        moment_of_inertia=physics.MOMENT_OF_INERTIA,
        satellite_size=physics.SATELLITE_SIZE,
        satellite_shape=constants.Constants.DEFAULT_SATELLITE_SHAPE,
        com_offset=tuple(physics.COM_OFFSET),
        thruster_positions=physics.THRUSTER_POSITIONS,
        thruster_directions={
            k: tuple(v) for k, v in physics.THRUSTER_DIRECTIONS.items()
        },
        thruster_forces=physics.THRUSTER_FORCES,
        reaction_wheels=[
            ReactionWheelParams(
                axis=rw_cfg.wheel_x.axis,
                max_torque=rw_cfg.wheel_x.max_torque,
                inertia=rw_cfg.wheel_x.inertia,
                max_speed=rw_cfg.wheel_x.max_speed,
            ),  # X-axis
            ReactionWheelParams(
                axis=rw_cfg.wheel_y.axis,
                max_torque=rw_cfg.wheel_y.max_torque,
                inertia=rw_cfg.wheel_y.inertia,
                max_speed=rw_cfg.wheel_y.max_speed,
            ),  # Y-axis
            ReactionWheelParams(
                axis=rw_cfg.wheel_z.axis,
                max_torque=rw_cfg.wheel_z.max_torque,
                inertia=rw_cfg.wheel_z.inertia,
                max_speed=rw_cfg.wheel_z.max_speed,
            ),  # Z-axis
        ],
        use_realistic_physics=phys_defaults.use_realistic_physics,
        damping_linear=phys_defaults.linear_damping_coeff,
        damping_angular=phys_defaults.rotational_damping_coeff,
    )

    # MPC
    mpc = MPCParams(
        prediction_horizon=constants.Constants.MPC_PREDICTION_HORIZON,
        control_horizon=constants.Constants.MPC_CONTROL_HORIZON,
        dt=timing.CONTROL_DT,
        solver_time_limit=constants.Constants.MPC_SOLVER_TIME_LIMIT,
        solver_type=constants.Constants.MPC_SOLVER_TYPE,
        # MPCC Weights
        Q_contour=constants.Constants.Q_CONTOUR,
        Q_progress=constants.Constants.Q_PROGRESS,
        progress_reward=constants.Constants.PROGRESS_REWARD,
        Q_lag=0.0,
        Q_lag_default=constants.Constants.Q_LAG_DEFAULT,
        Q_velocity_align=constants.Constants.Q_VELOCITY_ALIGN,
        Q_s_anchor=constants.Constants.Q_S_ANCHOR,
        Q_smooth=constants.Constants.Q_SMOOTH,
        Q_attitude=constants.Constants.Q_ATTITUDE,
        Q_axis_align=constants.Constants.Q_AXIS_ALIGN,
        Q_terminal_pos=constants.Constants.Q_TERMINAL_POS,
        Q_terminal_s=constants.Constants.Q_TERMINAL_S,
        q_angular_velocity=constants.Constants.Q_ANGULAR_VELOCITY,
        r_thrust=constants.Constants.R_THRUST,
        r_rw_torque=constants.Constants.R_RW_TORQUE,
        thrust_l1_weight=constants.Constants.THRUST_L1_WEIGHT,
        thrust_pair_weight=constants.Constants.THRUST_PAIR_WEIGHT,
        thruster_type=constants.Constants.THRUSTER_TYPE,
        verbose_mpc=False,
        max_linear_velocity=constants.Constants.MAX_LINEAR_VELOCITY,
        max_angular_velocity=constants.Constants.MAX_ANGULAR_VELOCITY,
        enable_delta_u_coupling=constants.Constants.ENABLE_DELTA_U_COUPLING,
        enable_gyro_jacobian=constants.Constants.ENABLE_GYRO_JACOBIAN,
        enable_auto_state_bounds=constants.Constants.ENABLE_AUTO_STATE_BOUNDS,
        # Path Following
        path_speed=constants.Constants.PATH_SPEED_MAX,
        path_speed_min=constants.Constants.PATH_SPEED_MIN,
        path_speed_max=constants.Constants.PATH_SPEED_MAX,
        enable_thruster_hysteresis=constants.Constants.ENABLE_THRUSTER_HYSTERESIS,
        thruster_hysteresis_on=constants.Constants.THRUSTER_HYSTERESIS_ON,
        thruster_hysteresis_off=constants.Constants.THRUSTER_HYSTERESIS_OFF,
    )

    # Simulation
    sim = SimulationParams(
        dt=timing.SIMULATION_DT,  # Single source of truth from timing.py
        max_duration=timing.MAX_SIMULATION_TIME,
        headless=constants.Constants.HEADLESS_MODE,
        window_width=constants.Constants.WINDOW_WIDTH,
        window_height=constants.Constants.WINDOW_HEIGHT,
        use_final_stabilization=timing.USE_FINAL_STABILIZATION_IN_SIMULATION,
        control_dt=timing.CONTROL_DT,
        default_path_speed=timing.DEFAULT_PATH_SPEED,
        physics_log_stride=1,
        control_log_stride=1,
        history_max_steps=50000,
        history_downsample_stride=1,
        mpc_target_mean_solve_time_ms=constants.Constants.TARGET_MEAN_SOLVE_TIME_MS,
        mpc_hard_max_solve_time_ms=constants.Constants.HARD_MAX_SOLVE_TIME_MS,
        enforce_mpc_timing_contract=constants.Constants.ENFORCE_TIMING_CONTRACT,
    )

    reference_scheduler = ReferenceSchedulerParams(
        speed_policy="min_non_hold_segment_speed",
        duration_margin_s=constants.Constants.V6_DURATION_MARGIN_S,
        auto_extend_manual_duration=True,
        enforce_contract_min_duration=True,
    )

    mpc_core = MPCCoreParams(
        solver_backend="OSQP",
        recover_contour_scale=constants.Constants.V6_RECOVER_CONTOUR_SCALE,
        recover_lag_scale=constants.Constants.V6_RECOVER_LAG_SCALE,
        recover_progress_scale=constants.Constants.V6_RECOVER_PROGRESS_SCALE,
        recover_attitude_scale=constants.Constants.V6_RECOVER_ATTITUDE_SCALE,
        settle_progress_scale=constants.Constants.V6_SETTLE_PROGRESS_SCALE,
        settle_terminal_pos_scale=constants.Constants.V6_SETTLE_TERMINAL_POS_SCALE,
        settle_terminal_attitude_scale=constants.Constants.V6_SETTLE_TERMINAL_ATTITUDE_SCALE,
        settle_velocity_align_scale=constants.Constants.V6_SETTLE_VELOCITY_ALIGN_SCALE,
        settle_angular_velocity_scale=constants.Constants.V6_SETTLE_ANGULAR_VELOCITY_SCALE,
        hold_smoothness_scale=constants.Constants.V6_HOLD_SMOOTHNESS_SCALE,
        hold_thruster_pair_scale=constants.Constants.V6_HOLD_THRUSTER_PAIR_SCALE,
    )

    actuator_policy = ActuatorPolicyParams(
        enable_thruster_hysteresis=constants.Constants.ENABLE_THRUSTER_HYSTERESIS,
        thruster_hysteresis_on=constants.Constants.THRUSTER_HYSTERESIS_ON,
        thruster_hysteresis_off=constants.Constants.THRUSTER_HYSTERESIS_OFF,
        terminal_bypass_band_m=constants.Constants.V6_TERMINAL_BYPASS_BAND_M,
    )

    controller_contracts = ControllerContractsParams(
        position_error_m_max=constants.Constants.POSITION_TOLERANCE,
        angle_error_deg_max=float(
            constants.Constants.ANGLE_TOLERANCE * 180.0 / 3.141592653589793
        ),
        velocity_error_mps_max=constants.Constants.VELOCITY_TOLERANCE,
        angular_velocity_error_degps_max=float(
            constants.Constants.ANGULAR_VELOCITY_TOLERANCE * 180.0 / 3.141592653589793
        ),
        hold_duration_s=10.0,
        solver_fallback_hold_s=constants.Constants.V6_SOLVER_FALLBACK_HOLD_S,
        solver_fallback_decay_s=constants.Constants.V6_SOLVER_FALLBACK_DECAY_S,
        solver_fallback_zero_after_s=constants.Constants.V6_SOLVER_FALLBACK_ZERO_AFTER_S,
        recover_enter_error_m=constants.Constants.V6_RECOVER_ENTER_ERROR_M,
        recover_enter_hold_s=constants.Constants.V6_RECOVER_ENTER_HOLD_S,
        recover_exit_error_m=constants.Constants.V6_RECOVER_EXIT_ERROR_M,
        recover_exit_hold_s=constants.Constants.V6_RECOVER_EXIT_HOLD_S,
        allow_mission_override=True,
        enable_pointing_contract=constants.Constants.V6_ENABLE_POINTING_CONTRACT,
        pointing_scope=constants.Constants.V6_POINTING_SCOPE,
        pointing_axis_priority=constants.Constants.V6_POINTING_AXIS_PRIORITY,
        pointing_sensor_policy=constants.Constants.V6_POINTING_SENSOR_POLICY,
        pointing_transfer_axis_policy=constants.Constants.V6_POINTING_TRANSFER_AXIS_POLICY,
        scan_axis_source=constants.Constants.V6_SCAN_AXIS_SOURCE,
        pointing_guardrails_enabled=constants.Constants.V6_POINTING_GUARDRAILS_ENABLED,
        pointing_z_error_deg_max=constants.Constants.V6_POINTING_Z_ERROR_DEG_MAX,
        pointing_x_error_deg_max=constants.Constants.V6_POINTING_X_ERROR_DEG_MAX,
        pointing_breach_hold_s=constants.Constants.V6_POINTING_BREACH_HOLD_S,
        pointing_clear_hold_s=constants.Constants.V6_POINTING_CLEAR_HOLD_S,
    )

    return AppConfig(
        physics=phys,
        mpc=mpc,
        reference_scheduler=reference_scheduler,
        mpc_core=mpc_core,
        actuator_policy=actuator_policy,
        controller_contracts=controller_contracts,
        simulation=sim,
        input_file_path=None,
    )

"""
Default Configuration Factory

Provides factory functions to create default configuration objects.
This replaces the implicit defaults in the legacy SatelliteConfig.
"""

from src.satellite_control.config import constants, physics, timing
from src.satellite_control.config.reaction_wheel_config import get_reaction_wheel_config
from src.satellite_control.config.models import (
    AppConfig,
    MPCParams,
    SatellitePhysicalParams,
    ReactionWheelParams,
    SimulationParams,
)


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
        progress_reward=constants.Constants.Q_PROGRESS * timing.DEFAULT_PATH_SPEED,
        Q_lag=0.0,
        Q_smooth=constants.Constants.Q_SMOOTH,
        Q_attitude=0.0,
        Q_terminal_pos=0.0,
        Q_terminal_s=0.0,
        q_angular_velocity=constants.Constants.Q_ANGULAR_VELOCITY,
        r_thrust=constants.Constants.R_THRUST,
        r_rw_torque=constants.Constants.R_RW_TORQUE,
        thrust_l1_weight=constants.Constants.THRUST_L1_WEIGHT,
        thrust_pair_weight=constants.Constants.THRUST_PAIR_WEIGHT,
        coast_pos_tolerance=constants.Constants.COAST_POS_TOLERANCE,
        coast_vel_tolerance=constants.Constants.COAST_VEL_TOLERANCE,
        coast_min_speed=constants.Constants.COAST_MIN_SPEED,
        thruster_type=constants.Constants.THRUSTER_TYPE,
        verbose_mpc=False,
        # Path Following
        path_speed=constants.Constants.PATH_SPEED_MAX,
        path_speed_min=constants.Constants.PATH_SPEED_MIN,
        path_speed_max=constants.Constants.PATH_SPEED_MAX,
        progress_taper_distance=0.0,
        progress_slowdown_distance=0.0,
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
    )

    return AppConfig(physics=phys, mpc=mpc, simulation=sim, input_file_path=None)

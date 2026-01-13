"""
Default Configuration Factory

Provides factory functions to create default configuration objects.
This replaces the implicit defaults in the legacy SatelliteConfig.
"""

from src.satellite_control.config import constants, physics, timing
from src.satellite_control.config.models import (
    AppConfig,
    MPCParams,
    SatellitePhysicalParams,
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
    phys = SatellitePhysicalParams(
        total_mass=physics.TOTAL_MASS,
        moment_of_inertia=physics.MOMENT_OF_INERTIA,
        satellite_size=physics.SATELLITE_SIZE,
        satellite_shape="cube",
        com_offset=tuple(physics.COM_OFFSET),
        thruster_positions=physics.THRUSTER_POSITIONS,
        thruster_directions={
            k: tuple(v) for k, v in physics.THRUSTER_DIRECTIONS.items()
        },
        thruster_forces=physics.THRUSTER_FORCES,
        use_realistic_physics=False,
        damping_linear=0.0,
        damping_angular=0.0,
    )

    # MPC
    mpc = MPCParams(
        prediction_horizon=constants.Constants.MPC_PREDICTION_HORIZON,
        control_horizon=constants.Constants.MPC_CONTROL_HORIZON,
        dt=timing.CONTROL_DT,
        solver_time_limit=constants.Constants.MPC_SOLVER_TIME_LIMIT,
        solver_type=constants.Constants.MPC_SOLVER_TYPE,
        q_position=constants.Constants.Q_POSITION,
        q_velocity=constants.Constants.Q_VELOCITY,
        q_angle=constants.Constants.Q_ANGLE,
        q_angular_velocity=constants.Constants.Q_ANGULAR_VELOCITY,
        r_thrust=constants.Constants.R_THRUST,
        r_rw_torque=constants.Constants.R_RW_TORQUE,
        max_velocity=constants.Constants.MAX_VELOCITY,
        max_angular_velocity=constants.Constants.MAX_ANGULAR_VELOCITY,
        position_bounds=constants.Constants.POSITION_BOUNDS,
        damping_zone=constants.Constants.DAMPING_ZONE,
        velocity_threshold=constants.Constants.VELOCITY_THRESHOLD,
        max_velocity_weight=constants.Constants.MAX_VELOCITY_WEIGHT,
        thruster_type=constants.Constants.THRUSTER_TYPE,
        enable_rw_yaw=True,
        enable_z_tilt=True,
        z_tilt_gain=0.35,
        z_tilt_max_deg=20.0,
        verbose_mpc=False,
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
        target_hold_time=timing.TARGET_HOLD_TIME,
        waypoint_final_stabilization_time=timing.WAYPOINT_FINAL_STABILIZATION_TIME,
        shape_final_stabilization_time=timing.SHAPE_FINAL_STABILIZATION_TIME,
        shape_positioning_stabilization_time=timing.SHAPE_POSITIONING_STABILIZATION_TIME,
        default_target_speed=timing.DEFAULT_TARGET_SPEED,
    )

    return AppConfig(physics=phys, mpc=mpc, simulation=sim, input_file_path=None)

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
        reaction_wheels=[
            ReactionWheelParams(
                axis=(1.0, 0.0, 0.0), max_torque=0.1, inertia=1e-4
            ),  # X-axis
            ReactionWheelParams(
                axis=(0.0, 1.0, 0.0), max_torque=0.1, inertia=1e-4
            ),  # Y-axis
            ReactionWheelParams(
                axis=(0.0, 0.0, 1.0), max_torque=0.1, inertia=1e-4
            ),  # Z-axis
        ],
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
        # MPCC Weights
        Q_contour=constants.Constants.Q_CONTOUR,
        Q_progress=constants.Constants.Q_PROGRESS,
        Q_smooth=constants.Constants.Q_SMOOTH,
        q_angular_velocity=constants.Constants.Q_ANGULAR_VELOCITY,
        r_thrust=constants.Constants.R_THRUST,
        r_rw_torque=constants.Constants.R_RW_TORQUE,
        thruster_type=constants.Constants.THRUSTER_TYPE,
        verbose_mpc=False,
        # Path Following
        path_speed=timing.DEFAULT_PATH_SPEED,
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
    )

    return AppConfig(physics=phys, mpc=mpc, simulation=sim, input_file_path=None)

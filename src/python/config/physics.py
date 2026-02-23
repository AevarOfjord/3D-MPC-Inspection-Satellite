"""
Physical Parameters for Satellite Control System

    Complete physical model parameters for satellite dynamics and thruster
    configuration.
    Includes mass properties, thruster geometry, and realistic physics
    effects.

Configuration sections:
- Mass Properties: Total mass, moment of inertia, center of mass offset
- Thruster Configuration: Six-thruster layout with positions and directions
- Thruster Forces: Individual force calibration per thruster
- Realistic Physics: Damping, friction, sensor noise
- Air Bearing System: Three-point support configuration

Thruster layout:
- Six thrusters at the center of each cube face
- Individual position and direction vectors
- Configurable force magnitude per thruster
- Support for force calibration and testing

Key features:
- Individual thruster force calibration
- Realistic damping and friction modeling
- Sensor noise simulation for testing
- Center of mass calculation from air bearing
- Integration with testing_environment physics
"""

from dataclasses import dataclass

import numpy as np

from .constants import Constants


@dataclass
class PhysicsConfig:
    """
    Physical properties and optional realism toggles.

    Attributes:
        total_mass: Total satellite mass in kg
        moment_of_inertia: Rotational inertia in kg·m²
        satellite_size: Characteristic dimension in meters
        com_offset: Center of mass offset [x, y, z] in meters
        thruster_positions: Dict mapping thruster ID (1-6)
            to (x, y, z) position in meters
        thruster_directions: Dict mapping thruster ID to unit direction vector
        thruster_forces: Dict mapping thruster ID to force magnitude in Newtons
        use_realistic_physics: Enable realistic physics modeling
        linear_damping_coeff: Linear drag coefficient in N/(m/s)
        rotational_damping_coeff: Rotational drag coefficient in N*m/(rad/s)
        position_noise_std: Position measurement noise std dev in meters
        velocity_noise_std: Velocity estimation noise std dev in m/s
        angle_noise_std: Orientation noise std dev in radians
        angular_velocity_noise_std: Angular velocity noise std dev in rad/s
        thruster_valve_delay: Solenoid valve opening delay in seconds
        thruster_rampup_time: Time for thrust to reach full force in seconds
        thruster_force_noise_std: Fractional thrust force variation (std dev)
        enable_random_disturbances: Enable random environmental disturbances
        disturbance_force_std: Random disturbance force std dev in Newtons
        disturbance_torque_std: Random disturbance torque std dev in N*m
    """

    # Core physical properties
    total_mass: float
    moment_of_inertia: float
    satellite_size: float
    com_offset: np.ndarray

    # Thruster configuration
    thruster_positions: dict[int, tuple[float, float, float]]
    thruster_directions: dict[int, np.ndarray]
    thruster_forces: dict[int, float]

    # Realistic physics modeling
    use_realistic_physics: bool = True
    linear_damping_coeff: float = 1.8
    rotational_damping_coeff: float = 0.3

    # Sensor noise
    position_noise_std: float = 0.000
    velocity_noise_std: float = 0.000
    angle_noise_std: float = 0.0
    angular_velocity_noise_std: float = 0.0

    # Actuator dynamics
    thruster_valve_delay: float = 0.04
    thruster_rampup_time: float = 0.01
    thruster_force_noise_std: float = 0.00

    # Environmental disturbances
    enable_random_disturbances: bool = True
    disturbance_force_std: float = 0.4
    disturbance_torque_std: float = 0.1


# DEFAULT PHYSICAL PARAMETERS
# ============================================================================

# Mass properties
TOTAL_MASS = Constants.TOTAL_MASS
# Mass properties

SATELLITE_SIZE = Constants.SATELLITE_SIZE

# Moment of Inertia for a solid cube: I = (1/6) * m * s^2
MOMENT_OF_INERTIA = (1 / 6) * TOTAL_MASS * SATELLITE_SIZE**2

# Thruster configuration (6 thrusters, one per face)
HALF_SIZE = SATELLITE_SIZE * 0.5
THRUSTER_POSITIONS = {
    1: (HALF_SIZE, 0.0, 0.0),  # +X face
    2: (-HALF_SIZE, 0.0, 0.0),  # -X face
    3: (0.0, HALF_SIZE, 0.0),  # +Y face
    4: (0.0, -HALF_SIZE, 0.0),  # -Y face
    5: (0.0, 0.0, HALF_SIZE),  # +Z face
    6: (0.0, 0.0, -HALF_SIZE),  # -Z face
}

# Thrust direction is the force direction on the satellite
# (points toward center).
THRUSTER_DIRECTIONS = {
    1: np.array([-1.0, 0.0, 0.0]),
    2: np.array([1.0, 0.0, 0.0]),
    3: np.array([0.0, -1.0, 0.0]),
    4: np.array([0.0, 1.0, 0.0]),
    5: np.array([0.0, 0.0, -1.0]),
    6: np.array([0.0, 0.0, 1.0]),
}

THRUSTER_FORCES = {
    1: 0.45,
    2: 0.45,
    3: 0.45,
    4: 0.45,
    5: 0.45,
    6: 0.45,
}

THRUSTER_IDS = tuple(sorted(THRUSTER_POSITIONS.keys()))
THRUSTER_COUNT = len(THRUSTER_IDS)


def calculate_com_offset() -> np.ndarray:
    """
    Calculate center of mass offset.
    Hardcoded to (0,0,0) as per configuration request.
    """
    # Force CoM to be at geometric center
    return np.zeros(3)


COM_OFFSET = calculate_com_offset()


def get_physics_params() -> PhysicsConfig:
    """
    Get default physics configuration.

    Returns:
        PhysicsConfig with default physical parameters
    """
    return PhysicsConfig(
        total_mass=TOTAL_MASS,
        moment_of_inertia=MOMENT_OF_INERTIA,
        satellite_size=SATELLITE_SIZE,
        com_offset=COM_OFFSET.copy(),
        thruster_positions=THRUSTER_POSITIONS.copy(),
        thruster_directions={k: v.copy() for k, v in THRUSTER_DIRECTIONS.items()},
        thruster_forces=THRUSTER_FORCES.copy(),
        use_realistic_physics=False,
        linear_damping_coeff=0.0,
        rotational_damping_coeff=0.0,
        position_noise_std=0.0,
        velocity_noise_std=0.0,
        angle_noise_std=np.deg2rad(0.0),
        angular_velocity_noise_std=np.deg2rad(0.0),
        thruster_valve_delay=0.0,
        thruster_rampup_time=0.0,
        thruster_force_noise_std=0.0,
        enable_random_disturbances=False,
        disturbance_force_std=0.0,
        disturbance_torque_std=0.0,
    )

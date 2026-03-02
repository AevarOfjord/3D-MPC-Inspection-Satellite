"""
Orbital Configuration Module

Defines orbital parameters for LEO inspection mission scenarios.
Uses Hill-Clohessy-Wiltshire (CW) relative motion model.
"""

from dataclasses import dataclass

import numpy as np

# Physical constants
MU_EARTH = 3.986004418e14  # Earth gravitational parameter [m³/s²]
EARTH_RADIUS = 6.371e6  # Earth mean radius [m]


@dataclass(frozen=True)
class OrbitalConfig:
    """
    Orbital parameters for the target satellite's reference orbit.

    The inspector satellite's position is defined relative to this
    target satellite using Hill's frame
    (LVLH - Local Vertical Local Horizontal):
    - X: Radial (away from Earth)
    - Y: Along-track (velocity direction)
    - Z: Cross-track (normal to orbital plane)

    Attributes:
        altitude: Orbital altitude above Earth surface [m]
        mu: Gravitational parameter [m³/s²]
        earth_radius: Earth radius [m]
        inclination: Orbital inclination [rad] (not used in CW, for reference)
    """

    altitude: float = 400_000  # 400 km LEO (ISS altitude)
    mu: float = MU_EARTH
    earth_radius: float = EARTH_RADIUS
    inclination: float = np.deg2rad(51.6)  # ISS inclination

    @property
    def orbital_radius(self) -> float:
        """Semi-major axis (circular orbit radius) [m]."""
        return self.earth_radius + self.altitude

    @property
    def mean_motion(self) -> float:
        """Orbital mean motion n = √(μ/a³) [rad/s]."""
        return np.sqrt(self.mu / self.orbital_radius**3)

    @property
    def orbital_period(self) -> float:
        """Orbital period T = 2π/n [s]."""
        return 2 * np.pi / self.mean_motion

    @property
    def orbital_velocity(self) -> float:
        """Circular orbital velocity [m/s]."""
        return np.sqrt(self.mu / self.orbital_radius)

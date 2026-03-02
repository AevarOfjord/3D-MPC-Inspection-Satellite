"""
Reaction Wheel Configuration

Physical parameters and constraints for cubesat reaction wheel actuators.
Typical values based on commercial cubesat reaction wheels
(e.g., Blue Canyon XACT).
"""

from dataclasses import dataclass

import numpy as np

# ----------------------------------------------------------------------------
# Primary design inputs (edit these three to retune wheel defaults globally)
# ----------------------------------------------------------------------------

WHEEL_MASS_KG: float = 0.8
WHEEL_RADIUS_M: float = 0.075
WHEEL_THICKNESS_M: float = 0.01


def _derive_wheel_defaults(
    mass_kg: float, radius_m: float, thickness_m: float
) -> tuple[float, float, float, float]:
    """
    Derive physical wheel defaults from mass/geometry.

    Returns:
        (inertia, max_torque, max_speed, density)

    Assumptions:
        - Wheel is modeled as a solid disk: I = 0.5 * m * r^2
        - Keep historical max speed baseline (6000 RPM -> 628 rad/s)
        - Keep historical angular acceleration baseline (~100 rad/s^2),
          so max_torque scales with inertia.
    """
    mass = float(max(mass_kg, 1e-9))
    radius = float(max(radius_m, 1e-9))
    thickness = float(max(thickness_m, 1e-9))

    volume = np.pi * radius * radius * thickness
    density = mass / volume
    inertia = 0.5 * mass * radius * radius
    max_speed = 1000
    max_torque = inertia * 100.0
    return inertia, max_torque, max_speed, density


_INERTIA_DEFAULT, _MAX_TORQUE_DEFAULT, _MAX_SPEED_DEFAULT, _ = _derive_wheel_defaults(
    WHEEL_MASS_KG,
    WHEEL_RADIUS_M,
    WHEEL_THICKNESS_M,
)


@dataclass(frozen=True)
class ReactionWheelParams:
    """
    Physical parameters for a single reaction wheel.

    Attributes:
        inertia: Wheel moment of inertia [kg·m²]
        max_torque: Maximum motor torque [N·m]
        max_speed: Maximum wheel speed [rad/s]
        friction: Viscous friction coefficient [N·m·s/rad]
        axis: Rotation axis in body frame (unit vector)
    """

    inertia: float = _INERTIA_DEFAULT  # kg·m², derived from mass/radius
    max_torque: float = _MAX_TORQUE_DEFAULT  # N·m, derived from inertia
    max_speed: float = _MAX_SPEED_DEFAULT  # rad/s (6000 RPM baseline)
    friction: float = 0.0001  # N·m·s/rad
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)


@dataclass
class ReactionWheelArrayConfig:
    """
    Configuration for 3-axis reaction wheel assembly.

    Standard orthogonal configuration with one wheel per axis.
    """

    wheel_x: ReactionWheelParams
    wheel_y: ReactionWheelParams
    wheel_z: ReactionWheelParams

    # Saturation warning thresholds (fraction of max speed)
    saturation_warning_threshold: float = 0.8
    saturation_critical_threshold: float = 0.95

    @classmethod
    def create_default(cls) -> "ReactionWheelArrayConfig":
        """Create default 3-axis reaction wheel configuration."""
        return cls(
            wheel_x=ReactionWheelParams(axis=(1.0, 0.0, 0.0)),
            wheel_y=ReactionWheelParams(axis=(0.0, 1.0, 0.0)),
            wheel_z=ReactionWheelParams(axis=(0.0, 0.0, 1.0)),
        )

    @property
    def max_torque_vector(self) -> np.ndarray:
        """Maximum torque vector [3]."""
        return np.array(
            [
                self.wheel_x.max_torque,
                self.wheel_y.max_torque,
                self.wheel_z.max_torque,
            ]
        )

    @property
    def max_speed_vector(self) -> np.ndarray:
        """Maximum speed vector [3]."""
        return np.array(
            [
                self.wheel_x.max_speed,
                self.wheel_y.max_speed,
                self.wheel_z.max_speed,
            ]
        )


def get_reaction_wheel_config() -> ReactionWheelArrayConfig:
    """Get default reaction wheel configuration."""
    return ReactionWheelArrayConfig.create_default()

"""
Unified Actuator Configuration

Supports multiple actuator modes:
- reaction_wheels: 3 reaction wheels + 8 thrusters (new default)
- legacy_thrusters: 8 thrusters only (backward compatibility)

This module provides a unified interface for actuator configuration
regardless of the underlying actuation mode.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Literal, Optional, Tuple

import numpy as np

from .physics import THRUSTER_DIRECTIONS, THRUSTER_POSITIONS
from .reaction_wheel_config import ReactionWheelArrayConfig, get_reaction_wheel_config


class ActuatorMode(Enum):
    """Actuator configuration modes."""

    REACTION_WHEELS = "reaction_wheels"  # RW + thrusters (torque-only)
    LEGACY_THRUSTERS = "legacy_thrusters"  # Thrusters only (8 thrusters)


@dataclass
class ThrusterSetConfig:
    """
    Configuration for six thrusters (IDs 1-6) around the body.

    Used in reaction wheel mode for translation + yaw torque.
    """

    # Thruster positions (IDs 1-6 from PhysicsConfig)
    positions: Dict[int, Tuple[float, float, float]] = field(
        default_factory=lambda: {k: tuple(v) for k, v in THRUSTER_POSITIONS.items()}
    )

    # Thruster directions (thrust vector)
    directions: Dict[int, Tuple[float, float, float]] = field(
        default_factory=lambda: {k: tuple(v) for k, v in THRUSTER_DIRECTIONS.items()}
    )

    # Force magnitude per thruster [N]
    force: float = 0.45

    # Thruster dynamics
    valve_delay: float = 0.04  # seconds
    rampup_time: float = 0.01  # seconds

    def get_force_vectors(self) -> Dict[int, np.ndarray]:
        """Get force vectors (direction * magnitude) for each thruster."""
        return {
            name: self.force * np.array(direction)
            for name, direction in self.directions.items()
        }


@dataclass
class ActuatorConfig:
    """
    Unified actuator configuration supporting multiple modes.

    Attributes:
        mode: ActuatorMode (reaction_wheels or legacy_thrusters)
        reaction_wheels: Reaction wheel config (if mode is reaction_wheels)
        thrusters: Thruster set config (if mode is reaction_wheels)
        legacy_thruster_positions: 6-thruster positions (if mode is legacy)
        legacy_thruster_directions: 6-thruster directions (if mode is legacy)
        legacy_thruster_forces: 6-thruster forces (if mode is legacy)
    """

    mode: ActuatorMode = ActuatorMode.REACTION_WHEELS

    # Reaction wheel mode config
    reaction_wheels: Optional[ReactionWheelArrayConfig] = None
    thrusters: Optional[ThrusterSetConfig] = None

    # Legacy mode config (6 thrusters)
    legacy_thruster_positions: Optional[Dict[int, Tuple[float, float, float]]] = None
    legacy_thruster_directions: Optional[Dict[int, Tuple[float, float, float]]] = None
    legacy_thruster_forces: Optional[Dict[int, float]] = None

    def __post_init__(self):
        """Initialize mode-specific configurations."""
        if self.mode == ActuatorMode.REACTION_WHEELS:
            if self.reaction_wheels is None:
                self.reaction_wheels = get_reaction_wheel_config()
            if self.thrusters is None:
                self.thrusters = ThrusterSetConfig()
        elif self.mode == ActuatorMode.LEGACY_THRUSTERS:
            if self.legacy_thruster_positions is None:
                self._init_legacy_thrusters()

    def _init_legacy_thrusters(self):
        """Initialize legacy 6-thruster configuration."""
        thrusters = ThrusterSetConfig()
        self.legacy_thruster_positions = thrusters.positions.copy()
        self.legacy_thruster_directions = thrusters.directions.copy()
        self.legacy_thruster_forces = {
            i: thrusters.force for i in thrusters.positions.keys()
        }

    @property
    def control_dimension(self) -> int:
        """Get control vector dimension based on mode."""
        if self.mode == ActuatorMode.REACTION_WHEELS:
            return 9  # 3 RW torques + 6 thruster forces
        else:
            return 6  # 6 thruster duty cycles

    @property
    def state_dimension(self) -> int:
        """Get state vector dimension based on mode."""
        if self.mode == ActuatorMode.REACTION_WHEELS:
            return 13  # 13 base (torque-only, no wheel speeds)
        else:
            return 13  # Standard 6-DOF state

    @property
    def model_path(self) -> str:
        """Get MuJoCo model path for this actuator mode."""
        return "models/satellite_3d.xml"

    def get_control_labels(self) -> list:
        """Get human-readable labels for control vector elements."""
        if self.mode == ActuatorMode.REACTION_WHEELS:
            return [
                "τ_rw_x",
                "τ_rw_y",
                "τ_rw_z",  # Reaction wheel torques
                "u_1",
                "u_2",
                "u_3",
                "u_4",
                "u_5",
                "u_6",  # Thruster forces
            ]
        else:
            return [f"u_{i}" for i in range(1, 7)]

    def get_state_labels(self) -> list:
        """Get human-readable labels for state vector elements."""
        base_labels = [
            "x",
            "y",
            "z",  # Position
            "qw",
            "qx",
            "qy",
            "qz",  # Quaternion
            "vx",
            "vy",
            "vz",  # Linear velocity
            "ωx",
            "ωy",
            "ωz",  # Angular velocity
        ]
        return base_labels


def create_actuator_config(
    mode: Literal["reaction_wheels", "legacy_thrusters"] = "reaction_wheels",
) -> ActuatorConfig:
    """
    Create actuator configuration.

    Args:
        mode: "reaction_wheels" (new) or "legacy_thrusters" (thrusters-only)

    Returns:
        ActuatorConfig instance
    """
    actuator_mode = ActuatorMode(mode)
    return ActuatorConfig(mode=actuator_mode)


# Default configuration
DEFAULT_ACTUATOR_MODE: ActuatorMode = ActuatorMode.REACTION_WHEELS

"""
Compatibility model module for core thruster geometry.

This keeps older integrations/tests working by exposing the
thruster layout from the current configuration defaults.
"""

from src.satellite_control.config.physics import (
    THRUSTER_DIRECTIONS,
    THRUSTER_FORCES,
    THRUSTER_POSITIONS,
)

thruster_positions = THRUSTER_POSITIONS
thruster_directions = THRUSTER_DIRECTIONS
thruster_forces = THRUSTER_FORCES

__all__ = [
    "thruster_positions",
    "thruster_directions",
    "thruster_forces",
]

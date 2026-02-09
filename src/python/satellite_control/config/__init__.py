"""
Configuration Package for Satellite Control System

Pydantic-based configuration system..

Primary API:
    from satellite_control.config import SimulationConfig
    config = SimulationConfig.create_default()
"""

from .constants import Constants
from .mission_state import MissionState
from .models import AppConfig, MPCParams, SatellitePhysicalParams, SimulationParams
from .physics import PhysicsConfig, get_physics_params
from .simulation_config import SimulationConfig
from .timing import TimingConfig, get_timing_params
from .validator import ConfigValidator, validate_config_at_startup

# Legacy exports (for backward compatibility during transition)

__all__ = [
    # Primary API.
    "SimulationConfig",
    "AppConfig",
    "MPCParams",
    "SatellitePhysicalParams",
    "SimulationParams",
    # Supporting modules
    "PhysicsConfig",
    "TimingConfig",
    "MissionState",
    "Constants",
    "get_physics_params",
    "get_timing_params",
    # Validation
    "ConfigValidator",
    "validate_config_at_startup",
]

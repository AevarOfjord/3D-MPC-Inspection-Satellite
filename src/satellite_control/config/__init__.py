"""
Configuration Package for Satellite Control System

Hydra-based configuration system (V4.0.0+).

Primary API:
    from src.satellite_control.config import SimulationConfig*   config = SimulationConfig.create_default()

Configuration is loaded from YAML files in the `config/` directory.
"""

from .constants import Constants
from .mission_state import MissionState
from .models import AppConfig, MPCParams, SatellitePhysicalParams, SimulationParams
from .obstacles import ObstacleManager
from .physics import PhysicsConfig, get_physics_params
from .presets import ConfigPreset, get_preset_description, list_presets, load_preset
from .simulation_config import SimulationConfig
from .timing import TimingConfig, get_timing_params
from .validator import ConfigValidator, validate_config_at_startup

# Legacy exports (for backward compatibility during transition)
# TODO: Remove in V5.0.0

__all__ = [
    # Primary API (V4.0.0+)
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
    "ObstacleManager",
    "get_physics_params",
    "get_timing_params",
    # Presets
    "ConfigPreset",
    "get_preset_description",
    "list_presets",
    "load_preset",
    # Validation
    "ConfigValidator",
    "validate_config_at_startup",
]

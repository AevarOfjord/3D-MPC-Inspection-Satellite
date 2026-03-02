"""
Configuration package for Satellite Control System.

Pydantic-based configuration system.

Primary API:
    from controller.configs import SimulationConfig
    config = SimulationConfig.create_default()
"""

from .constants import Constants
from .models import (
    ActuatorPolicyParams,
    AppConfig,
    ControllerContractsParams,
    MPCCoreParams,
    MPCParams,
    ReferenceSchedulerParams,
    SatellitePhysicalParams,
    SimulationParams,
)
from .physics import PhysicsConfig, get_physics_params
from .simulation_config import SimulationConfig
from .validator import ConfigValidator, validate_config_at_startup

__all__ = [
    # Primary API.
    "SimulationConfig",
    "AppConfig",
    "MPCParams",
    "SatellitePhysicalParams",
    "SimulationParams",
    "ReferenceSchedulerParams",
    "MPCCoreParams",
    "ActuatorPolicyParams",
    "ControllerContractsParams",
    # Supporting modules
    "PhysicsConfig",
    "Constants",
    "get_physics_params",
    # Validation
    "ConfigValidator",
    "validate_config_at_startup",
]

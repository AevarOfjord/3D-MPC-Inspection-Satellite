"""
Backward-compatibility shim — exceptions now live in error_handling.py.
"""

from satellite_control.core.error_handling import (  # noqa: F401
    ConfigurationError,
    ControllerError,
    DashboardError,
    MissionError,
    OptimizationError,
    SatelliteControlError,
    SimulationError,
    VisualizationError,
)

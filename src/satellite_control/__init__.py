"""
satellite_control – 3-D satellite simulation with MPC path-following.

Public API re-exports for convenient top-level access.
"""

from importlib.metadata import version as _pkg_version, PackageNotFoundError

try:
    __version__: str = _pkg_version("satellite-control")
except PackageNotFoundError:  # pragma: no cover – editable / dev installs
    __version__ = "1.0.0"

# ── Core public classes ──────────────────────────────────────────────
from satellite_control.config.models import AppConfig  # noqa: F401
from satellite_control.core.exceptions import (  # noqa: F401
    SatelliteControlError,
    ConfigurationError,
    SimulationError,
    ControllerError,
    OptimizationError,
    MissionError,
    DashboardError,
    VisualizationError,
)

__all__ = [
    "__version__",
    # Config
    "AppConfig",
    # Exceptions
    "SatelliteControlError",
    "ConfigurationError",
    "SimulationError",
    "ControllerError",
    "OptimizationError",
    "MissionError",
    "DashboardError",
    "VisualizationError",
]

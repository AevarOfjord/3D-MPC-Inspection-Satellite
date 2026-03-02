"""
satellite_control – 3-D satellite simulation with MPC path-following.

Top-level package now rooted at ``controller``.
"""

from importlib.metadata import PackageNotFoundError
from importlib.metadata import version as _pkg_version

try:
    __version__: str = _pkg_version("satellite-control")
except PackageNotFoundError:  # pragma: no cover – editable / dev installs
    __version__ = "1.0.0"

from controller.configs.models import AppConfig  # noqa: F401
from controller.exceptions import (  # noqa: F401
    ConfigurationError,
    ControllerError,
    DashboardError,
    MissionError,
    OptimizationError,
    SatelliteControlError,
    SimulationError,
    VisualizationError,
)
from controller.factory import (  # noqa: F401
    create_controller,
    resolve_controller_profile,
)

__all__ = [
    "__version__",
    "AppConfig",
    "create_controller",
    "resolve_controller_profile",
    "SatelliteControlError",
    "ConfigurationError",
    "SimulationError",
    "ControllerError",
    "OptimizationError",
    "MissionError",
    "DashboardError",
    "VisualizationError",
]

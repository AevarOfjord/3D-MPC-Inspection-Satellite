"""
Shared error-handling helpers.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from functools import wraps
from typing import Any, TypeVar

logger = logging.getLogger(__name__)

F = TypeVar("F", bound=Callable[..., Any])


# --------------------
# Exception Classes
# --------------------


class SatelliteControlError(Exception):
    """Base exception for all satellite control errors."""

    pass


class ConfigurationError(SatelliteControlError):
    """Raised when configuration is invalid."""

    pass


class SimulationError(SatelliteControlError):
    """Raised when simulation fails."""

    pass


class ControllerError(SatelliteControlError):
    """Raised when controller fails."""

    pass


class OptimizationError(SatelliteControlError):
    """Raised when optimization fails."""

    pass


class MissionError(SatelliteControlError):
    """Raised when mission loading/execution fails."""

    pass


class DashboardError(SatelliteControlError):
    """Raised when dashboard operations fail."""

    pass


class VisualizationError(SatelliteControlError):
    """Raised when visualization fails."""

    pass


# --------------------
# Error Handling Decorator
# --------------------


def with_error_context(context: str, reraise: bool = True) -> Callable[[F], F]:
    """
    Decorate a callable to add contextual logging around exceptions.

    Args:
        context: Human-readable operation label.
        reraise: If True, re-raise the original exception after logging.
    """

    def decorator(func: F) -> F:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            try:
                return func(*args, **kwargs)
            except Exception as exc:
                logger.exception("%s failed: %s", context, exc)
                if reraise:
                    raise
                return None

        return wrapper  # type: ignore[return-value]

    return decorator

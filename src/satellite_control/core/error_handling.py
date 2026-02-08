"""
Shared error-handling helpers and core exception types.

Exception Hierarchy
-------------------
SatelliteControlError
├── ConfigurationError     – invalid/missing configuration
├── SimulationError        – physics or loop runtime failures
├── ControllerError        – MPC / control-pipeline failures
│   └── OptimizationError  – solver-specific failures
├── MissionError           – mission definition or compilation issues
├── DashboardError         – dashboard / API runtime failures
└── VisualizationError     – plotting or rendering failures
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from dataclasses import dataclass
from functools import wraps
from typing import Any, TypeVar

logger = logging.getLogger(__name__)

F = TypeVar("F", bound=Callable[..., Any])


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


# ── Base ─────────────────────────────────────────────────────────────


class SatelliteControlError(Exception):
    """Base exception for project-specific runtime errors."""


# ── Configuration ────────────────────────────────────────────────────


class ConfigurationError(SatelliteControlError):
    """Raised when configuration is invalid or incomplete."""


# ── Simulation ───────────────────────────────────────────────────────


class SimulationError(SatelliteControlError):
    """Raised when the simulation loop encounters a runtime failure."""


# ── Control ──────────────────────────────────────────────────────────


class ControllerError(SatelliteControlError):
    """Raised when the MPC or control pipeline fails."""


@dataclass
class OptimizationError(ControllerError):
    """Raised when an optimizer/solver step fails."""

    code: str
    details: str

    def __str__(self) -> str:
        return f"[{self.code}] {self.details}"


# ── Mission ──────────────────────────────────────────────────────────


class MissionError(SatelliteControlError):
    """Raised when mission definition or compilation fails."""


# ── Dashboard ────────────────────────────────────────────────────────


class DashboardError(SatelliteControlError):
    """Raised when the dashboard / API encounters a runtime failure."""


# ── Visualization ────────────────────────────────────────────────────


class VisualizationError(SatelliteControlError):
    """Raised when plotting or rendering fails."""

"""
Core exception types used by control and simulation modules.

Hierarchy
---------
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

from dataclasses import dataclass

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

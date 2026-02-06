"""
Core exception types used by control and simulation modules.
"""

from __future__ import annotations

from dataclasses import dataclass


class SatelliteControlError(Exception):
    """Base exception for project-specific runtime errors."""


@dataclass
class OptimizationError(SatelliteControlError):
    """Raised when an optimizer/solver step fails."""

    code: str
    details: str

    def __str__(self) -> str:
        return f"[{self.code}] {self.details}"

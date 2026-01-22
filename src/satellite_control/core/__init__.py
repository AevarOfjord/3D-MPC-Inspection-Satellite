"""
Core simulation and control modules.

This package contains the core simulation engine, physics interface,
and control algorithms.
"""

# V4.0.0: Simplified - removed unused exception and error handling exports

from . import model

__all__ = [
    "model",
]

"""
Simulation Backend Protocol

Defines the structural interface expected of any physics backend.
The sole concrete implementation is CppSatelliteSimulator in cpp_satellite.py.

Uses typing.Protocol instead of ABC so CppSatelliteSimulator satisfies
the contract via structural subtyping (duck-typing) without explicit
inheritance.
"""

from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class SimulationBackend(Protocol):
    """
    Structural interface for satellite physics backends.

    Any object that exposes these properties and methods satisfies
    this protocol — no inheritance required.
    """

    @property
    def position(self) -> np.ndarray: ...

    @property
    def velocity(self) -> np.ndarray: ...

    @property
    def quaternion(self) -> np.ndarray: ...

    @property
    def angular_velocity(self) -> np.ndarray: ...

    def update_physics(self, dt: float) -> None: ...

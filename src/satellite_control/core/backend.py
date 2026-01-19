"""
Simulation Backend Interface
"""

from abc import ABC, abstractmethod
from typing import Optional, Tuple
import numpy as np


class SimulationBackend(ABC):
    """
    Abstract base class for satellite physics simulation backends.

    Defines the interface required by the main simulation loop.
    """

    @property
    @abstractmethod
    def dt(self) -> float:
        """Physics timestep in seconds."""
        pass

    @property
    @abstractmethod
    def simulation_time(self) -> float:
        """Current simulation time in seconds."""
        pass

    @simulation_time.setter
    @abstractmethod
    def simulation_time(self, value: float) -> None:
        """Set simulation time."""
        pass

    @property
    @abstractmethod
    def position(self) -> np.ndarray:
        """Get position [x, y, z]."""
        pass

    @property
    @abstractmethod
    def velocity(self) -> np.ndarray:
        """Get velocity [vx, vy, vz]."""
        pass

    @property
    @abstractmethod
    def quaternion(self) -> np.ndarray:
        """Get quaternion [w, x, y, z]."""
        pass

    @property
    @abstractmethod
    def angular_velocity(self) -> np.ndarray:
        """Get angular velocity [wx, wy, wz]."""
        pass

    @abstractmethod
    def get_state(self) -> np.ndarray:
        """Get full state vector (13 elements)."""
        pass

    @abstractmethod
    def set_state(
        self,
        state: Optional[np.ndarray] = None,
        position: Optional[np.ndarray] = None,
        velocity: Optional[np.ndarray] = None,
        quaternion: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
        angle: Optional[Tuple[float, float, float]] = None,
    ):
        """Set satellite state."""
        pass

    @abstractmethod
    def update_physics(self, dt: Optional[float] = None):
        """Update physics for one timestep."""
        pass

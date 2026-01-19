"""
Orbital Dynamics Module

Implements Hill-Clohessy-Wiltshire (CW) equations for relative motion
around a target satellite in circular orbit.

CW Equations (relative motion in Hill/LVLH frame):
    ẍ = 2nẏ + 3n²x + ax    (Radial - away from Earth)
    ÿ = -2nẋ + ay          (Along-track - velocity direction)
    z̈ = -n²z + az          (Cross-track - out of plane)

where n = √(μ/a³) is the orbital mean motion and a is thrust acceleration.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from src.satellite_control.config.orbital_config import OrbitalConfig


@dataclass
class CWDynamics:
    """
    Hill-Clohessy-Wiltshire relative motion dynamics.

    Computes accelerations due to gravity gradient effects
    for a chaser satellite relative to a target in circular orbit.

    V2.0.0: Uses C++ backend (`_cpp_physics`) for performance if available.
    """

    orbital_config: OrbitalConfig

    def __post_init__(self):
        """Initialize C++ backend or fallback to Python."""
        self.n = self.orbital_config.mean_motion
        self.n_sq = self.n**2
        self._cpp_backend = None

        try:
            from satellite_control.cpp._cpp_physics import CWDynamics as CppCWDynamics

            self._cpp_backend = CppCWDynamics(self.n)
        except ImportError:
            pass  # Fallback to pure Python (below)

    @staticmethod
    def calculate_acceleration(
        position: np.ndarray, velocity: np.ndarray, n: float
    ) -> np.ndarray:
        """
        Compute CW gravitational acceleration (stateless).

        Args:
            position: Relative position [x, y, z] [m]
            velocity: Relative velocity [vx, vy, vz] [m/s]
            n: Mean motion [rad/s]

        Returns:
            Acceleration [ax, ay, az] [m/s²]
        """
        # Unpack components
        x, _, z = position[0], position[1], position[2]
        vx, vy, _ = velocity[0], velocity[1], velocity[2]

        n_sq = n * n

        # CW Equations
        # ẍ = 2nẏ + 3n²x
        # ÿ = -2nẋ
        # z̈ = -n²z
        ax = 3 * n_sq * x + 2 * n * vy
        ay = -2 * n * vx
        az = -n_sq * z

        return np.array([ax, ay, az])

    def compute_acceleration(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
    ) -> np.ndarray:
        """
        Compute CW gravitational acceleration.

        Args:
            position: Relative position [x, y, z] in Hill frame [m]
            velocity: Relative velocity [vx, vy, vz] in Hill frame [m/s]

        Returns:
            Gravitational acceleration [ax, ay, az] [m/s²]
        """
        if self._cpp_backend:
            return self._cpp_backend.compute_acceleration(position, velocity)

        return self.calculate_acceleration(position, velocity, self.n)

    def get_state_matrices(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get discrete-time state transition matrices for CW dynamics.

        State: [x, y, z, vx, vy, vz]

        Args:
            dt: Time step [s]

        Returns:
            (A, B) discrete-time matrices where x_{k+1} = A @ x_k + B @ u_k
        """
        if self._cpp_backend:
            return self._cpp_backend.get_state_matrices(dt)

        n = self.n
        n_sq = self.n_sq

        # Continuous-time A matrix
        # State: [x, y, z, vx, vy, vz]
        # Rows 0-2: velocity integration
        # Rows 3-5: acceleration equations
        A_cont = np.array(
            [
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
                [3 * n_sq, 0, 0, 0, 2 * n, 0],
                [0, 0, 0, -2 * n, 0, 0],
                [0, 0, -n_sq, 0, 0, 0],
            ]
        )

        # B matrix (input is acceleration)
        B_cont = np.zeros((6, 3))
        B_cont[3:, :] = np.eye(3)

        # Discretize (Forward Euler)
        # Note: For higher precision, use expm(A*dt), but Euler is sufficient for small dt
        A_disc = np.eye(6) + A_cont * dt
        B_disc = B_cont * dt

        return A_disc, B_disc

    def get_mpc_dynamics_matrices(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get 16-state dynamics matrices including CW terms for MPC.

        Integrates CW accelerations into the full satellite state.

        Args:
            dt: Time step [s]

        Returns:
            (A_cw, B_cw) matrices to add to existing MPC dynamics
        """
        if self._cpp_backend:
            return self._cpp_backend.get_mpc_dynamics_matrices(dt)

        n = self.n
        n_sq = self.n_sq

        # Additional A terms for velocity due to CW (affects positions 7-9)
        # State indices: 0-2(pos), 3-6(quat), 7-9(vel), ...
        A_cw = np.zeros((16, 16))

        # Position -> Velocity coupling
        # dvx/dx = 3n²
        A_cw[7, 0] = 3 * n_sq * dt
        # dvx/dvy = 2n (coriolis)
        A_cw[7, 8] = 2 * n * dt
        # dvy/dvx = -2n (coriolis)
        A_cw[8, 7] = -2 * n * dt
        # dvz/dz = -n²
        A_cw[9, 2] = -n_sq * dt

        return A_cw, np.zeros((16, 9))


def compute_cw_acceleration(
    position: np.ndarray,
    velocity: np.ndarray,
    mean_motion: float,
) -> np.ndarray:
    """
    Compute CW gravitational acceleration (standalone function).

    Delegates to CWDynamics logic.
    """
    return CWDynamics.calculate_acceleration(position, velocity, mean_motion)


def compute_cw_force(
    position: np.ndarray,
    velocity: np.ndarray,
    mass: float,
    orbital_config: Optional[OrbitalConfig] = None,
) -> np.ndarray:
    """
    Compute CW gravitational force for MuJoCo xfrc_applied.

    Args:
        position: Relative position [x, y, z] [m]
        velocity: Relative velocity [vx, vy, vz] [m/s]
        mass: Satellite mass [kg]
        orbital_config: Orbital configuration (default: LEO 400km)

    Returns:
        Force [Fx, Fy, Fz] [N]
    """
    n = orbital_config.mean_motion if orbital_config else OrbitalConfig().mean_motion
    accel = compute_cw_acceleration(position, velocity, n)
    return mass * accel

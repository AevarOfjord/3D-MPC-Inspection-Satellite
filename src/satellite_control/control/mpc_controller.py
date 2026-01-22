"""
MPC Controller - C++ Backend Wrapper.

This module provides a Python interface to the C++ MPC controller.
The entire control loop runs in C++ for maximum performance.
"""

import logging
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np

# Configuration
from src.satellite_control.config.models import AppConfig

# C++ Backend (required)
from src.satellite_control.cpp._cpp_mpc import (
    SatelliteParams,
    MPCParams as CppMPCParams,
    MPCControllerCpp,
    Obstacle,
    ObstacleSet,
    ObstacleType,
)

from .base import Controller

logger = logging.getLogger(__name__)


class MPCController(Controller):
    """
    Satellite Model Predictive Controller (C++ Backend).

    State: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, wrx, wry, wrz, s] (17 elements).
    Control: [τ_rw_x, τ_rw_y, τ_rw_z, u1, ..., uN] (RW torques + thrusters).
    """

    def __init__(self, cfg: Union[AppConfig, Any]):
        """
        Initialize MPC controller with C++ backend.

        Args:
            cfg: Configuration object. Can be AppConfig (preferred) or OmegaConf/Dict (legacy).
        """
        # Determine config type and extract parameters
        if isinstance(cfg, AppConfig):
            self._extract_params_from_app_config(cfg)
        else:
            # Try to see if it's a wrapped AppConfig or compatible object
            try:
                # Last ditch effort: assume it behaves like AppConfig
                if hasattr(cfg, "physics") and hasattr(cfg, "mpc"):
                    self._extract_params_from_app_config(cfg)  # type: ignore
                else:
                    raise ValueError("Invalid config structure")
            except Exception as e:
                raise ValueError(
                    f"MPCController requires AppConfig. Got {type(cfg)}. Error: {e}"
                )

        # Build C++ SatelliteParams
        sat_params = SatelliteParams()
        sat_params.dt = self._dt
        sat_params.mass = self.total_mass
        sat_params.inertia = self.moment_of_inertia
        sat_params.num_thrusters = self.num_thrusters
        sat_params.num_rw = self.num_rw_axes
        sat_params.thruster_positions = [np.array(p) for p in self.thruster_positions]
        sat_params.thruster_directions = [np.array(d) for d in self.thruster_directions]
        sat_params.thruster_forces = self.thruster_forces
        sat_params.rw_torque_limits = self.rw_torque_limits
        sat_params.rw_inertia = self.rw_inertia
        sat_params.com_offset = self.com_offset

        # Build C++ MPCParams
        mpc_params = CppMPCParams()
        mpc_params.prediction_horizon = self.N
        mpc_params.dt = self._dt
        mpc_params.solver_time_limit = self.solver_time_limit

        mpc_params.Q_contour = self.Q_contour
        mpc_params.Q_progress = self.Q_progress
        mpc_params.Q_smooth = self.Q_smooth
        mpc_params.Q_angvel = self.Q_angvel
        mpc_params.R_thrust = self.R_thrust
        mpc_params.R_rw_torque = self.R_rw_torque
        if hasattr(mpc_params, "path_speed"):
            mpc_params.path_speed = self.path_speed

        # Collision Avoidance
        mpc_params.enable_collision_avoidance = cfg.mpc.enable_collision_avoidance
        mpc_params.obstacle_margin = cfg.mpc.obstacle_margin

        self._cpp_controller = MPCControllerCpp(sat_params, mpc_params)

        # Performance tracking
        self.solve_times: list[float] = []

        # Path following state
        self.s = 0.0
        self._path_data: list[list[float]] = []  # [(s, x, y, z), ...]
        self._path_set = False

        # Dimensions (Fixed for MPCC)
        self.nx = 17
        self.nu = self.num_rw_axes + self.num_thrusters

        logger.info(
            f"MPC Controller initialized (C++ backend). Thrusters: {self.num_thrusters}, RW: {self.num_rw_axes}"
        )
        logger.info("MPC Path Following Mode (MPCC) Enabled.")

    def set_path(self, path_points: list[tuple[float, float, float]]) -> None:
        """
        Set the path for path-following mode.

        Args:
            path_points: List of (x, y, z) waypoints. Arc-length is computed automatically.
        """
        if not path_points or len(path_points) < 2:
            logger.warning("Path must have at least 2 points")
            return

        # Build arc-length parameterized path
        self._path_data = []
        s = 0.0
        prev_pt = None

        for pt in path_points:
            if prev_pt is not None:
                # Compute segment length
                dx = pt[0] - prev_pt[0]
                dy = pt[1] - prev_pt[1]
                dz = pt[2] - prev_pt[2]
                s += (dx**2 + dy**2 + dz**2) ** 0.5

            self._path_data.append([s, pt[0], pt[1], pt[2]])
            prev_pt = pt

        # Store total length for clamping
        self._path_length = s

        # Send to C++ controller
        self._cpp_controller.set_path_data(self._path_data)
        self._path_set = True
        self.s = 0.0  # Reset path parameter

        logger.info(f"Path set with {len(path_points)} points, total length: {s:.3f}m")

    def get_path_reference(
        self, s_query: Optional[float] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the path reference position and tangent for a given arc-length.

        Returns:
            Tuple of (position, unit_tangent). Falls back to zeros if path not set.
        """
        if not self._path_data or len(self._path_data) < 2:
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        s_val = float(self.s if s_query is None else s_query)
        if hasattr(self, "_path_length"):
            s_val = max(0.0, min(s_val, float(self._path_length)))

        # Find the segment that contains s_val
        idx = 0
        while idx + 1 < len(self._path_data) and self._path_data[idx + 1][0] < s_val:
            idx += 1

        s0, x0, y0, z0 = self._path_data[idx]
        s1, x1, y1, z1 = self._path_data[min(idx + 1, len(self._path_data) - 1)]

        seg_len = s1 - s0
        if seg_len <= 1e-9:
            pos = np.array([x0, y0, z0], dtype=float)
            tangent = np.array([0.0, 0.0, 0.0], dtype=float)
            return pos, tangent

        alpha = (s_val - s0) / seg_len
        pos = np.array(
            [x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0), z0 + alpha * (z1 - z0)],
            dtype=float,
        )
        tangent = np.array([x1 - x0, y1 - y0, z1 - z0], dtype=float)
        tan_norm = np.linalg.norm(tangent)
        if tan_norm > 1e-9:
            tangent /= tan_norm
        else:
            tangent[:] = 0.0

        return pos, tangent

    def _extract_params_from_app_config(self, cfg: AppConfig) -> None:
        """Extract parameters from AppConfig."""
        physics = cfg.physics
        mpc = cfg.mpc

        # Physics
        self.total_mass = physics.total_mass

        # Inertia: AppConfig has float (simulating uniform cube) or we might need to expand it
        # For now, treat scalar as diagonal
        I_val = physics.moment_of_inertia
        self.moment_of_inertia = np.array([I_val, I_val, I_val], dtype=float)

        self.com_offset = np.array(physics.com_offset)

        # Thrusters
        # physics.thruster_positions is Dict[int, Tuple]
        # We need to sort by ID to ensure consistent ordering
        sorted_ids = sorted(physics.thruster_positions.keys())
        self.num_thrusters = len(sorted_ids)
        self.thruster_positions = [physics.thruster_positions[i] for i in sorted_ids]
        self.thruster_directions = [physics.thruster_directions[i] for i in sorted_ids]
        self.thruster_forces = [physics.thruster_forces[i] for i in sorted_ids]

        # Reaction Wheels (Now in AppConfig.physics)
        if hasattr(physics, "reaction_wheels") and physics.reaction_wheels:
            self.reaction_wheels = physics.reaction_wheels
            self.num_rw_axes = len(self.reaction_wheels)
            self.rw_torque_limits = [
                float(rw.max_torque) for rw in self.reaction_wheels
            ]
            self.rw_inertia = [float(rw.inertia) for rw in self.reaction_wheels]
        else:
            self.reaction_wheels = []
            self.num_rw_axes = 0
            self.rw_torque_limits = []
            self.rw_inertia = []

        self.max_rw_torque = (
            max(self.rw_torque_limits) if self.rw_torque_limits else 0.0
        )

        # MPC
        self.N = mpc.prediction_horizon
        self._dt = mpc.dt
        self.solver_time_limit = mpc.solver_time_limit

        self.Q_angvel = mpc.q_angular_velocity
        self.R_thrust = mpc.r_thrust
        self.R_rw_torque = mpc.r_rw_torque if hasattr(mpc, "r_rw_torque") else 0.1

        # Path Following (V4.0.1) - General Path MPCC
        self.mode_path_following = True  # Always True now
        self.Q_contour = mpc.Q_contour
        self.Q_progress = mpc.Q_progress
        self.Q_smooth = mpc.Q_smooth
        self.path_speed = mpc.path_speed

    @property
    def dt(self) -> float:
        """Control update interval."""
        return self._dt

    @property
    def prediction_horizon(self) -> int:
        """Prediction horizon."""
        return self.N

    @property
    def body_frame_forces(self) -> list[np.ndarray]:
        """Compute body frame force vector for each thruster."""
        forces = []
        for i in range(self.num_thrusters):
            f_mag = self.thruster_forces[i]
            f_dir = np.array(self.thruster_directions[i])
            forces.append(f_mag * f_dir)
        return forces

    @property
    def body_frame_torques(self) -> list[np.ndarray]:
        """Compute body frame torque vector for each thruster."""
        torques = []
        forces = self.body_frame_forces
        for i in range(self.num_thrusters):
            pos = np.array(self.thruster_positions[i])
            r = pos - self.com_offset
            torques.append(np.cross(r, forces[i]))
        return torques

    @property
    def max_thrust(self) -> float:
        """Maximum thrust force (assumes uniform thrusters)."""
        if not self.thruster_forces:
            return 0.0
        return max(self.thruster_forces)

    def split_control(self, control: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Split control vector into RW torques and thruster commands."""
        rw_torques = control[: self.num_rw_axes]
        thruster_cmds = control[self.num_rw_axes :]
        return rw_torques, thruster_cmds

    def get_solver_stats(self) -> Dict[str, Any]:
        """Get solver performance statistics."""
        if not self.solve_times:
            return {"solve_count": 0, "average_solve_time": 0.0, "max_solve_time": 0.0}
        return {
            "solve_times": self.solve_times.copy(),
            "solve_count": len(self.solve_times),
            "average_solve_time": sum(self.solve_times) / len(self.solve_times),
            "max_solve_time": max(self.solve_times),
        }

    def get_control_action(
        self,
        x_current: np.ndarray,
        previous_thrusters: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Compute optimal control action via C++ backend."""
        # Handle Path Following State Augmentation
        # Append current path parameter s to state
        # Ensure x_current is basic 16-dim state before appending
        if len(x_current) == 16:
            x_input = np.append(x_current, self.s)
        else:
            x_input = x_current  # Assuming already augmented or custom

        try:
            result = self._cpp_controller.get_control_action(x_input)
        except TypeError:
            # Older/newer C++ bindings expect (x_current, x_target).
            x_target = np.array(x_input, dtype=float)
            try:
                pos_ref, _ = self.get_path_reference()
                if x_target.shape[0] >= 3:
                    x_target[0:3] = pos_ref
            except Exception:
                pass
            result = self._cpp_controller.get_control_action(x_input, x_target)

        self.solve_times.append(result.solve_time)

        # Process output controls
        u_out = np.array(result.u)

        # Extract virtual control v_s (last element)
        v_s = u_out[-1]

        # Update internal path state s only if solved successfully
        if result.status == 1:
            self.s += v_s * self.dt
            # Clamp s to valid range [0, L]
            if self._path_set and hasattr(self, "_path_length"):
                self.s = max(0.0, min(self.s, self._path_length))

        # Strip virtual control from output so simulation gets only physical actuators
        u_phys = u_out[:-1]

        extras = {
            "status": result.status,
            "solve_time": result.solve_time,
            "path_s": self.s,
            "path_v_s": v_s,
        }
        return u_phys, extras

    def set_obstacles(self, mission_obstacles: list) -> None:
        """
        Set collision avoidance obstacles.

        Args:
            mission_obstacles: List of mission_types.Obstacle objects
        """
        cpp_obstacle_set = ObstacleSet()

        for obs in mission_obstacles:
            cpp_obs = Obstacle()
            # Map parameters
            cpp_obs.position = np.array(obs.position)
            cpp_obs.radius = float(obs.radius)
            cpp_obs.size = np.array(obs.size)
            cpp_obs.name = str(obs.name)

            # Map type (string value from enum to C++ enum)
            type_val = obs.type.value

            if type_val == "sphere":
                cpp_obs.type = ObstacleType.SPHERE
            elif type_val == "cylinder":
                cpp_obs.type = ObstacleType.CYLINDER
                # Default Z-axis for cylinder if not specified
                cpp_obs.axis = np.array([0.0, 0.0, 1.0])
            elif type_val == "box":
                cpp_obs.type = ObstacleType.BOX

            cpp_obstacle_set.add(cpp_obs)

        self._cpp_controller.set_obstacles(cpp_obstacle_set)

    def clear_obstacles(self) -> None:
        """Clear all collision avoidance obstacles."""
        self._cpp_controller.clear_obstacles()

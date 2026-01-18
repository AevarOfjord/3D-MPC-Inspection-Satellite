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

    State: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz] (13 elements).
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
        elif hasattr(cfg, "control") and hasattr(cfg, "vehicle"):
            self._extract_params_from_hydra(cfg)
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
                    f"MPCController requires AppConfig or valid Hydra config. Got {type(cfg)}. Error: {e}"
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
        sat_params.com_offset = self.com_offset

        # Build C++ MPCParams
        mpc_params = CppMPCParams()
        mpc_params.prediction_horizon = self.N
        mpc_params.dt = self._dt
        mpc_params.solver_time_limit = self.solver_time_limit
        mpc_params.Q_pos = self.Q_pos
        mpc_params.Q_vel = self.Q_vel
        mpc_params.Q_ang = self.Q_ang
        mpc_params.Q_angvel = self.Q_angvel
        mpc_params.R_thrust = self.R_thrust
        mpc_params.R_rw_torque = self.R_rw_torque
        mpc_params.enable_z_tilt = self.enable_z_tilt
        mpc_params.z_tilt_gain = self.z_tilt_gain
        mpc_params.z_tilt_max_rad = self.z_tilt_max_rad

        # Create C++ controller
        self._cpp_controller = MPCControllerCpp(sat_params, mpc_params)

        # Performance tracking
        self.solve_times: list[float] = []

        # Dimensions for external use
        self.nx = 13
        self.nu = self.num_rw_axes + self.num_thrusters

        logger.info("MPC Controller initialized (C++ backend)")

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

        # Reaction Wheels (Currently NOT in AppConfig explicitly, logic handled in valid_config construction)
        # TODO: Add ReactionWheelParams to AppConfig
        # For now, default to empty or look for non-standard attribute if injected
        if hasattr(cfg, "reaction_wheels") and cfg.reaction_wheels:
            # Legacy injection support if needed, or if we extend AppConfig later
            self.reaction_wheels = cfg.reaction_wheels
            self.num_rw_axes = len(self.reaction_wheels)
            # Assume object has max_torque
            self.rw_torque_limits = [
                float(rw.max_torque) for rw in self.reaction_wheels
            ]
        else:
            self.reaction_wheels = []
            self.num_rw_axes = 0
            self.rw_torque_limits = []

        self.max_rw_torque = (
            max(self.rw_torque_limits) if self.rw_torque_limits else 0.0
        )

        # MPC
        self.N = mpc.prediction_horizon
        self._dt = mpc.dt
        self.solver_time_limit = mpc.solver_time_limit

        self.Q_pos = mpc.q_position
        self.Q_vel = mpc.q_velocity
        self.Q_ang = mpc.q_angle
        self.Q_angvel = mpc.q_angular_velocity
        self.R_thrust = mpc.r_thrust
        self.R_rw_torque = mpc.r_rw_torque if hasattr(mpc, "r_rw_torque") else 0.1

        # Z-tilt (legacy support, might not be in MPCParams yet or defaults)
        self.enable_z_tilt = True  # Default
        self.z_tilt_gain = 0.35
        self.z_tilt_max_rad = np.deg2rad(20.0)

    def _extract_params_from_hydra(self, cfg: Any) -> None:
        """Extract parameters from legacy Hydra/Omegaconf object."""
        self.mpc_cfg = cfg.control.mpc
        self.vehicle_cfg = cfg.vehicle

        # Extract parameters
        self.total_mass = self.vehicle_cfg.mass
        inertia = self.vehicle_cfg.inertia
        if hasattr(inertia, "__len__") and len(inertia) == 3:
            self.moment_of_inertia = np.array(inertia, dtype=float)
        else:
            val = float(inertia)
            self.moment_of_inertia = np.array([val, val, val], dtype=float)

        self.com_offset = np.array(self.vehicle_cfg.center_of_mass)

        # Thruster config
        self.thrusters = self.vehicle_cfg.thrusters
        self.num_thrusters = len(self.thrusters)
        self.thruster_positions = [t.position for t in self.thrusters]
        self.thruster_directions = [t.direction for t in self.thrusters]
        self.thruster_forces = [float(t.max_thrust) for t in self.thrusters]

        # Reaction wheel config
        self.reaction_wheels = self.vehicle_cfg.reaction_wheels
        self.num_rw_axes = len(self.reaction_wheels)
        self.rw_torque_limits = [float(rw.max_torque) for rw in self.reaction_wheels]
        self.max_rw_torque = (
            max(self.rw_torque_limits) if self.rw_torque_limits else 0.0
        )

        # MPC parameters
        self.N = int(getattr(self.mpc_cfg, "prediction_horizon", 50))
        self._dt = float(getattr(self.mpc_cfg, "dt", 0.05))
        self.solver_time_limit = float(getattr(self.mpc_cfg, "solver_time_limit", 0.05))

        # Weights
        weights = self.mpc_cfg.weights
        self.Q_pos = float(getattr(weights, "position", 10.0))
        self.Q_vel = float(getattr(weights, "velocity", 1.0))
        self.Q_ang = float(getattr(weights, "angle", 10.0))
        self.Q_angvel = float(getattr(weights, "angular_velocity", 1.0))
        self.R_thrust = float(getattr(weights, "thrust", 0.1))
        self.R_rw_torque = float(getattr(weights, "rw_torque", 0.1))

        # Z-tilt settings
        settings = self.mpc_cfg.settings
        self.enable_z_tilt = bool(getattr(settings, "enable_z_tilt", True))
        self.z_tilt_gain = float(getattr(settings, "z_tilt_gain", 0.35))
        self.z_tilt_max_rad = (
            float(getattr(settings, "z_tilt_max_deg", 20.0)) * np.pi / 180.0
        )

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
        x_target: np.ndarray,
        previous_thrusters: Optional[np.ndarray] = None,
        x_target_trajectory: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Compute optimal control action via C++ backend."""
        use_traj = x_target_trajectory is not None
        if use_traj:
            traj = np.asarray(x_target_trajectory, dtype=float)
            if traj.ndim == 2 and traj.shape[1] == self.nx and traj.shape[0] > 0:
                result = self._cpp_controller.get_control_action_trajectory(
                    x_current, traj
                )
            else:
                result = self._cpp_controller.get_control_action(x_current, x_target)
        else:
            result = self._cpp_controller.get_control_action(x_current, x_target)
        self.solve_times.append(result.solve_time)
        return np.array(result.u), {
            "status": result.status,
            "solve_time": result.solve_time,
        }

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

"""
MPC Controller - C++ Backend Wrapper.

This module provides a Python interface to the C++ MPC controller.
The entire control loop runs in C++ for maximum performance.
"""

import logging
import sys
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np

# Configuration
from src.satellite_control.config.models import AppConfig

# C++ Backend (required)
_CPP_IMPORT_ERROR: Optional[ImportError] = None
try:
    from src.satellite_control.cpp._cpp_mpc import (
        SatelliteParams,
        MPCParams as CppMPCParams,
        MPCControllerCpp,
        Obstacle,
        ObstacleSet,
        ObstacleType,
    )
except ImportError as exc:  # pragma: no cover - depends on local runtime env
    _CPP_IMPORT_ERROR = exc
    SatelliteParams = None  # type: ignore[assignment]
    CppMPCParams = None  # type: ignore[assignment]
    MPCControllerCpp = None  # type: ignore[assignment]
    Obstacle = None  # type: ignore[assignment]
    ObstacleSet = None  # type: ignore[assignment]
    ObstacleType = None  # type: ignore[assignment]
from src.satellite_control.mission.mission_types import (
    Obstacle as MissionObstacle,
    ObstacleType as MissionObstacleType,
)

from .base import Controller

logger = logging.getLogger(__name__)


def _raise_cpp_binding_import_error() -> None:
    """Raise a detailed error when C++ MPC bindings cannot be imported."""
    assert _CPP_IMPORT_ERROR is not None

    py_ver = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    message = (
        "Failed to import C++ MPC bindings (`src.satellite_control.cpp._cpp_mpc`). "
        f"Running interpreter: Python {py_ver}. Original error: {_CPP_IMPORT_ERROR}"
    )

    if "Python version mismatch" in str(_CPP_IMPORT_ERROR):
        message += (
            " Detected ABI mismatch between the active Python interpreter and the compiled "
            "extension. Rebuild the extension with the same interpreter used to run the app/tests "
            "(for this repo, Python 3.11 is the supported development target)."
        )

    raise RuntimeError(message) from _CPP_IMPORT_ERROR


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
        if _CPP_IMPORT_ERROR is not None:
            _raise_cpp_binding_import_error()

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
        sat_params.rw_speed_limits = self.rw_speed_limits
        sat_params.com_offset = self.com_offset
        sat_params.orbital_mean_motion = self.orbital_mean_motion
        sat_params.orbital_mu = self.orbital_mu
        sat_params.orbital_radius = self.orbital_radius
        sat_params.use_two_body = self.use_two_body

        # Build C++ MPCParams
        mpc_params = CppMPCParams()
        mpc_params.prediction_horizon = self.N
        mpc_params.control_horizon = int(self.control_horizon)
        mpc_params.dt = self._dt
        mpc_params.solver_time_limit = self.solver_time_limit
        mpc_params.solver_type = str(self.solver_type)
        mpc_params.verbose_mpc = bool(self.verbose_mpc)

        mpc_params.Q_contour = self.Q_contour
        mpc_params.Q_progress = self.Q_progress
        if hasattr(mpc_params, "progress_reward"):
            mpc_params.progress_reward = float(self.progress_reward)
        if hasattr(mpc_params, "Q_lag"):
            mpc_params.Q_lag = self.Q_lag
        mpc_params.Q_smooth = self.Q_smooth
        if hasattr(mpc_params, "Q_terminal_pos"):
            mpc_params.Q_terminal_pos = self.Q_terminal_pos
        if hasattr(mpc_params, "Q_terminal_s"):
            mpc_params.Q_terminal_s = self.Q_terminal_s
        mpc_params.Q_angvel = self.Q_angvel
        if hasattr(mpc_params, "Q_attitude"):
            mpc_params.Q_attitude = self.Q_attitude
        mpc_params.R_thrust = self.R_thrust
        mpc_params.R_rw_torque = self.R_rw_torque
        if hasattr(mpc_params, "thrust_l1_weight"):
            mpc_params.thrust_l1_weight = float(self.thrust_l1_weight)
        if hasattr(mpc_params, "thrust_pair_weight"):
            mpc_params.thrust_pair_weight = float(self.thrust_pair_weight)
        if hasattr(mpc_params, "coast_pos_tolerance"):
            mpc_params.coast_pos_tolerance = float(self.coast_pos_tolerance)
        if hasattr(mpc_params, "coast_vel_tolerance"):
            mpc_params.coast_vel_tolerance = float(self.coast_vel_tolerance)
        if hasattr(mpc_params, "coast_min_speed"):
            mpc_params.coast_min_speed = float(self.coast_min_speed)
        if hasattr(mpc_params, "path_speed"):
            mpc_params.path_speed = self.path_speed
        if hasattr(mpc_params, "path_speed_min"):
            mpc_params.path_speed_min = float(self.path_speed_min)
        if hasattr(mpc_params, "path_speed_max"):
            mpc_params.path_speed_max = float(self.path_speed_max)
        if hasattr(mpc_params, "progress_taper_distance"):
            mpc_params.progress_taper_distance = self.progress_taper_distance
        if hasattr(mpc_params, "progress_slowdown_distance"):
            mpc_params.progress_slowdown_distance = self.progress_slowdown_distance

        # Collision Avoidance
        # Path-focused MPCC: obstacles are ignored by design.
        mpc_params.enable_collision_avoidance = False
        mpc_params.obstacle_margin = cfg.mpc.obstacle_margin

        self._cpp_controller = MPCControllerCpp(sat_params, mpc_params)

        # Performance tracking
        self.solve_times: list[float] = []

        # Path following state
        self.s = 0.0
        self._path_data: list[list[float]] = []  # [(s, x, y, z), ...]
        self._path_set = False
        self._path_length = 0.0
        self._last_path_projection: Dict[str, Any] = {}

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
        self._last_path_projection = {}

        logger.info(f"Path set with {len(path_points)} points, total length: {s:.3f}m")

    def set_scan_attitude_context(
        self,
        center: Optional[tuple[float, float, float]],
        axis: Optional[tuple[float, float, float]],
        direction: str = "CW",
    ) -> None:
        """Configure scan attitude context for stable object-facing camera alignment."""
        if center is None or axis is None:
            if hasattr(self._cpp_controller, "clear_scan_attitude_context"):
                self._cpp_controller.clear_scan_attitude_context()
            return

        if hasattr(self._cpp_controller, "set_scan_attitude_context"):
            c = np.array(center, dtype=float)
            a = np.array(axis, dtype=float)
            self._cpp_controller.set_scan_attitude_context(c, a, str(direction))

    def _project_onto_path(
        self, position: np.ndarray
    ) -> Tuple[float, np.ndarray, float]:
        """
        Project a position onto the path and return (s, closest_point, distance).

        Uses linear segments between path samples.
        """
        if not self._path_data or len(self._path_data) < 2:
            return 0.0, np.zeros(3, dtype=float), float("inf")

        pos_arr = np.array(position, dtype=float).reshape(-1)
        if pos_arr.size < 3:
            return 0.0, np.zeros(3, dtype=float), float("inf")
        pos = pos_arr[:3]

        if hasattr(self, "_cpp_controller") and hasattr(
            self._cpp_controller, "project_onto_path"
        ):
            try:
                s_val, proj, dist, _ = self._cpp_controller.project_onto_path(pos)
                return float(s_val), np.array(proj, dtype=float), float(dist)
            except Exception:
                pass

        min_dist = float("inf")
        best_s = 0.0
        best_point = np.array(self._path_data[0][1:4], dtype=float)

        for idx in range(len(self._path_data) - 1):
            s0, x0, y0, z0 = self._path_data[idx]
            s1, x1, y1, z1 = self._path_data[idx + 1]
            p0 = np.array([x0, y0, z0], dtype=float)
            p1 = np.array([x1, y1, z1], dtype=float)
            seg = p1 - p0
            seg_len2 = float(np.dot(seg, seg))
            if seg_len2 < 1e-12:
                t = 0.0
                proj = p0
            else:
                t = float(np.dot(pos - p0, seg) / seg_len2)
                t = float(np.clip(t, 0.0, 1.0))
                proj = p0 + t * seg
            dist = float(np.linalg.norm(pos - proj))
            if dist < min_dist:
                min_dist = dist
                best_point = proj
                best_s = float(s0 + t * (s1 - s0))

        if self._path_length > 0.0:
            best_s = float(np.clip(best_s, 0.0, self._path_length))

        return best_s, best_point, float(min_dist)

    def get_path_progress(
        self, position: Optional[np.ndarray] = None
    ) -> Dict[str, float]:
        """
        Compute progress metrics for a given position.

        Returns dict with s, progress (0-1), remaining, path_error, endpoint_error.
        """
        if not self._path_data or len(self._path_data) < 2:
            return {
                "s": 0.0,
                "progress": 0.0,
                "remaining": 0.0,
                "path_error": float("inf"),
                "endpoint_error": float("inf"),
            }

        if position is None:
            if self._last_path_projection:
                s_val = float(self._last_path_projection.get("s", self.s))
                path_error = float(self._last_path_projection.get("path_error", 0.0))
                endpoint_error = float(
                    self._last_path_projection.get("endpoint_error", 0.0)
                )
            else:
                s_val = float(self.s)
                path_error = float("inf")
                endpoint_error = float("inf")
        else:
            pos_arr = np.array(position, dtype=float).reshape(-1)
            pos = pos_arr[:3] if pos_arr.size >= 3 else pos_arr

            if hasattr(self, "_cpp_controller") and hasattr(
                self._cpp_controller, "project_onto_path"
            ):
                try:
                    s_val, _, path_error, endpoint_error = (
                        self._cpp_controller.project_onto_path(pos)
                    )
                    s_val = float(s_val)
                    path_error = float(path_error)
                    endpoint_error = float(endpoint_error)
                except Exception:
                    s_val, _, path_error = self._project_onto_path(position)
                    endpoint = np.array(self._path_data[-1][1:4], dtype=float)
                    endpoint_error = float(np.linalg.norm(pos - endpoint))
            else:
                s_val, _, path_error = self._project_onto_path(position)
                endpoint = np.array(self._path_data[-1][1:4], dtype=float)
                endpoint_error = float(np.linalg.norm(pos - endpoint))

            # Prevent global projection from jumping backwards on looping paths.
            try:
                backtrack_tol = max(0.1, 0.5 * float(self.path_speed) * float(self.dt))
                if self._path_length > 0.0:
                    backtrack_tol = min(backtrack_tol, float(self._path_length))
                if s_val < float(self.s) - backtrack_tol:
                    s_val = float(self.s)
            except Exception:
                pass

        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        progress = float(s_val / path_len) if path_len > 1e-9 else 0.0
        remaining = float(path_len - s_val) if path_len > 0 else 0.0

        return {
            "s": s_val,
            "progress": progress,
            "remaining": remaining,
            "path_error": float(path_error),
            "endpoint_error": float(endpoint_error),
        }

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
            self.rw_speed_limits = [
                float(getattr(rw, "max_speed", 0.0)) for rw in self.reaction_wheels
            ]
        else:
            self.reaction_wheels = []
            self.num_rw_axes = 0
            self.rw_torque_limits = []
            self.rw_inertia = []
            self.rw_speed_limits = []

        self.max_rw_torque = (
            max(self.rw_torque_limits) if self.rw_torque_limits else 0.0
        )

        # MPC
        self.N = mpc.prediction_horizon
        self._dt = mpc.dt
        self.solver_time_limit = mpc.solver_time_limit
        self.control_horizon = max(
            1,
            min(int(getattr(mpc, "control_horizon", self.N)), int(self.N)),
        )
        self.solver_type = str(getattr(mpc, "solver_type", "OSQP"))
        if self.solver_type.upper() != "OSQP":
            logger.warning("MPC solver_type '%s' not supported, using OSQP.", self.solver_type)
            self.solver_type = "OSQP"
        self.verbose_mpc = getattr(mpc, "verbose_mpc", False)

        self.Q_angvel = mpc.q_angular_velocity
        self.Q_attitude = getattr(mpc, "Q_attitude", 0.0)
        self.R_thrust = mpc.r_thrust
        self.R_rw_torque = mpc.r_rw_torque if hasattr(mpc, "r_rw_torque") else 0.1
        self.thrust_l1_weight = getattr(mpc, "thrust_l1_weight", 0.0)
        self.thrust_pair_weight = getattr(mpc, "thrust_pair_weight", 0.0)
        self.coast_pos_tolerance = getattr(mpc, "coast_pos_tolerance", 0.0)
        self.coast_vel_tolerance = getattr(mpc, "coast_vel_tolerance", 0.0)
        self.coast_min_speed = getattr(mpc, "coast_min_speed", 0.0)

        # Path Following (V4.0.1) - General Path MPCC
        self.mode_path_following = True  # Always True now
        self.Q_contour = mpc.Q_contour
        self.Q_progress = mpc.Q_progress
        self.progress_reward = getattr(mpc, "progress_reward", 0.0)
        self.Q_smooth = mpc.Q_smooth
        self.Q_lag = getattr(mpc, "Q_lag", 0.0)
        self.Q_terminal_pos = getattr(mpc, "Q_terminal_pos", 0.0)
        self.Q_terminal_s = getattr(mpc, "Q_terminal_s", 0.0)
        self.path_speed = mpc.path_speed
        self.path_speed_min = getattr(mpc, "path_speed_min", 0.0)
        self.path_speed_max = getattr(mpc, "path_speed_max", 0.0)
        self.progress_taper_distance = getattr(mpc, "progress_taper_distance", 0.0)
        self.progress_slowdown_distance = getattr(
            mpc, "progress_slowdown_distance", 0.0
        )

        # Orbital parameters (for MPC linearization)
        try:
            from src.satellite_control.config.orbital_config import OrbitalConfig
            orbital_cfg = getattr(physics, "orbital", None)
            if orbital_cfg is not None:
                self.orbital_mu = float(getattr(orbital_cfg, "mu", OrbitalConfig().mu))
                self.orbital_radius = float(
                    getattr(orbital_cfg, "orbital_radius", OrbitalConfig().orbital_radius)
                )
                self.orbital_mean_motion = float(
                    getattr(orbital_cfg, "mean_motion", OrbitalConfig().mean_motion)
                )
                self.use_two_body = bool(getattr(orbital_cfg, "use_two_body", True))
            else:
                orbital_default = OrbitalConfig()
                self.orbital_mu = float(orbital_default.mu)
                self.orbital_radius = float(orbital_default.orbital_radius)
                self.orbital_mean_motion = float(orbital_default.mean_motion)
                self.use_two_body = True
        except Exception:
            self.orbital_mu = 3.986004418e14
            self.orbital_radius = 6.778e6
            self.orbital_mean_motion = 0.0
            self.use_two_body = True

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
        s_for_state = float(self.s)
        path_error = None
        endpoint_error = None
        s_proj = None
        if self._path_set and len(x_current) >= 3:
            if hasattr(self, "_cpp_controller") and hasattr(
                self._cpp_controller, "project_onto_path"
            ):
                try:
                    s_proj, closest_point, proj_err, endpoint_error = (
                        self._cpp_controller.project_onto_path(x_current[:3])
                    )
                    s_proj = float(s_proj)
                    closest_point = np.array(closest_point, dtype=float)
                    path_error = float(proj_err)
                    endpoint_error = float(endpoint_error)
                except Exception:
                    s_proj, closest_point, proj_err = self._project_onto_path(
                        x_current[:3]
                    )
                    path_error = float(proj_err)
                    endpoint = np.array(self._path_data[-1][1:4], dtype=float)
                    endpoint_error = float(np.linalg.norm(x_current[:3] - endpoint))
            else:
                s_proj, closest_point, proj_err = self._project_onto_path(x_current[:3])
                path_error = float(proj_err)
                endpoint = np.array(self._path_data[-1][1:4], dtype=float)
                endpoint_error = float(np.linalg.norm(x_current[:3] - endpoint))

            # Keep s monotonic but bound lead to avoid drift. Ignore large backward jumps
            # from global projection (e.g., looping paths near the start/end).
            lead_max = 0.5 * float(self.path_speed) * float(self.dt) * float(self.N)
            lead_max = float(max(0.2, min(1.0, lead_max)))
            if self._path_length > 0.0:
                lead_max = min(lead_max, float(self._path_length))

            backtrack_tol = max(0.1, 0.5 * float(self.path_speed) * float(self.dt))
            if self._path_length > 0.0:
                backtrack_tol = min(backtrack_tol, float(self._path_length))

            s_proj_filtered = float(s_proj)
            if s_proj_filtered < float(self.s) - backtrack_tol:
                s_proj_filtered = float(self.s)

            if self.s < s_proj_filtered:
                self.s = float(s_proj_filtered)
            if self.s > s_proj_filtered + lead_max:
                self.s = float(s_proj_filtered) + lead_max

            s_for_state = float(self.s)

            self._last_path_projection = {
                "s": s_for_state,
                "s_proj": float(s_proj_filtered),
                "s_proj_raw": float(s_proj),
                "closest_point": closest_point,
                "path_error": path_error,
                "endpoint_error": endpoint_error,
                "lead_max": lead_max,
                "backtrack_tol": backtrack_tol,
            }

        if len(x_current) == 16:
            x_input = np.append(x_current, s_for_state)
        elif len(x_current) == 17:
            x_input = np.array(x_current, dtype=float).copy()
            x_input[16] = s_for_state
        else:
            x_input = x_current  # Assuming already augmented or custom

        if previous_thrusters is not None and hasattr(
            self._cpp_controller, "set_warm_start_control"
        ):
            try:
                self._cpp_controller.set_warm_start_control(
                    np.array(previous_thrusters, dtype=float)
                )
            except Exception:
                pass

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
            s_pred = s_for_state + v_s * self.dt
            if self._path_set and hasattr(self, "_path_length"):
                s_pred = max(0.0, min(float(s_pred), float(self._path_length)))
            self._last_path_projection["s_pred"] = float(s_pred)
            self.s = float(s_pred)

        # Strip virtual control from output so simulation gets only physical actuators
        u_phys = u_out[:-1]

        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        progress = float(s_for_state / path_len) if path_len > 1e-9 else 0.0
        remaining = float(path_len - s_for_state) if path_len > 0 else 0.0

        extras = {
            "status": result.status,
            "solve_time": result.solve_time,
            "solver_type": self.solver_type,
            "path_s": float(s_for_state),
            "path_s_proj": float(s_proj) if s_proj is not None else None,
            "path_v_s": v_s,
            "path_progress": progress,
            "path_remaining": remaining,
            "path_error": path_error,
            "path_endpoint_error": endpoint_error,
            "path_s_pred": self._last_path_projection.get("s_pred"),
        }
        return u_phys, extras

    def set_obstacles(self, mission_obstacles: list) -> None:
        """
        Set collision avoidance obstacles.

        Args:
            mission_obstacles: List of mission_types.Obstacle objects
        """
        logger.info("MPC is path-focused; obstacles are ignored in this controller.")
        normalized: list[MissionObstacle] = []

        for obs in mission_obstacles:
            if isinstance(obs, MissionObstacle):
                normalized.append(obs)
                continue

            if isinstance(obs, dict):
                try:
                    normalized.append(MissionObstacle.from_dict(obs))
                    continue
                except Exception:
                    pass

            if isinstance(obs, (list, tuple)):
                if len(obs) >= 4:
                    pos = np.array(obs[:3], dtype=float)
                    radius = float(obs[3])
                elif len(obs) == 3:
                    pos = np.array(obs[:3], dtype=float)
                    radius = 0.5
                else:
                    continue
                normalized.append(
                    MissionObstacle(position=pos, radius=radius)
                )
                continue

            if hasattr(obs, "position") and hasattr(obs, "radius"):
                try:
                    pos = np.array(getattr(obs, "position"), dtype=float)
                    radius = float(getattr(obs, "radius"))
                    size = np.array(getattr(obs, "size", [1.0, 1.0, 1.0]), dtype=float)
                    name = str(getattr(obs, "name", "obstacle"))
                    type_val = getattr(obs, "type", MissionObstacleType.SPHERE)
                    if isinstance(type_val, str):
                        type_val = MissionObstacleType(type_val)
                    normalized.append(
                        MissionObstacle(
                            type=type_val,
                            position=pos,
                            radius=radius,
                            size=size,
                            name=name,
                        )
                    )
                except Exception:
                    continue

        if not normalized:
            self._cpp_controller.clear_obstacles()
            return

        cpp_obstacle_set = ObstacleSet()

        for obs in normalized:
            cpp_obs = Obstacle()
            # Map parameters
            cpp_obs.position = np.array(obs.position)
            cpp_obs.radius = float(obs.radius)
            cpp_obs.size = np.array(obs.size)
            cpp_obs.name = str(obs.name)

            # Map type (string value from enum to C++ enum)
            type_val = obs.type.value if hasattr(obs.type, "value") else str(obs.type)

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

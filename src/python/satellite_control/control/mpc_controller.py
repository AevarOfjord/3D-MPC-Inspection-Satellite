"""
MPC Controller - C++ Backend Wrapper.

This module provides a Python interface to the C++ MPC controller.
The entire control loop runs in C++ for maximum performance.
"""

import logging
import sys
from typing import Any

import numpy as np

# Configuration
from satellite_control.config.models import AppConfig

# C++ Backend (required)
_CPP_IMPORT_ERROR: ImportError | None = None
try:
    from satellite_control.cpp import _cpp_mpc as _cpp_mpc_mod

    MPCControllerCpp = _cpp_mpc_mod.MPCControllerCpp
    SatelliteParams = _cpp_mpc_mod.SatelliteParams
    CppMPCParams = _cpp_mpc_mod.MPCParams
    Obstacle = getattr(_cpp_mpc_mod, "Obstacle", None)
    ObstacleSet = getattr(_cpp_mpc_mod, "ObstacleSet", None)
    ObstacleType = getattr(_cpp_mpc_mod, "ObstacleType", None)
except ImportError as exc:  # pragma: no cover - depends on local runtime env
    _CPP_IMPORT_ERROR = exc
    Obstacle = None  # type: ignore[assignment]
    ObstacleSet = None  # type: ignore[assignment]
    ObstacleType = None  # type: ignore[assignment]
    SatelliteParams = None  # type: ignore[assignment]
    CppMPCParams = None  # type: ignore[assignment]
    MPCControllerCpp = None  # type: ignore[assignment]

from .base import Controller

logger = logging.getLogger(__name__)


def _raise_cpp_binding_import_error() -> None:
    """Raise a detailed error when C++ MPC bindings cannot be imported."""
    assert _CPP_IMPORT_ERROR is not None

    py_ver = (
        f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    )
    message = (
        "Failed to import C++ MPC bindings (`satellite_control.cpp._cpp_mpc`). "
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

    def __init__(self, cfg: AppConfig):
        """
        Initialize MPC controller with C++ backend.

        Args:
            cfg: Application configuration.
        """
        if _CPP_IMPORT_ERROR is not None:
            _raise_cpp_binding_import_error()

        if not isinstance(cfg, AppConfig):
            if hasattr(cfg, "physics") and hasattr(cfg, "mpc"):
                # Duck-typed compatible object (e.g. test doubles)
                pass
            else:
                raise TypeError(
                    f"MPCController requires AppConfig, got {type(cfg).__name__}"
                )

        self._extract_params_from_app_config(cfg)

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
        if hasattr(mpc_params, "Q_lag_default"):
            mpc_params.Q_lag_default = float(self.Q_lag_default)
        if hasattr(mpc_params, "Q_velocity_align"):
            mpc_params.Q_velocity_align = float(self.Q_velocity_align)
        if hasattr(mpc_params, "Q_s_anchor"):
            mpc_params.Q_s_anchor = float(self.Q_s_anchor)
        mpc_params.Q_smooth = self.Q_smooth
        if hasattr(mpc_params, "Q_terminal_pos"):
            mpc_params.Q_terminal_pos = self.Q_terminal_pos
        if hasattr(mpc_params, "Q_terminal_s"):
            mpc_params.Q_terminal_s = self.Q_terminal_s
        mpc_params.Q_angvel = self.Q_angvel
        if hasattr(mpc_params, "Q_attitude"):
            mpc_params.Q_attitude = self.Q_attitude
        if hasattr(mpc_params, "Q_axis_align"):
            mpc_params.Q_axis_align = float(self.Q_axis_align)
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
        if hasattr(mpc_params, "recover_contour_scale"):
            mpc_params.recover_contour_scale = float(self.recover_contour_scale)
        if hasattr(mpc_params, "recover_lag_scale"):
            mpc_params.recover_lag_scale = float(self.recover_lag_scale)
        if hasattr(mpc_params, "recover_progress_scale"):
            mpc_params.recover_progress_scale = float(self.recover_progress_scale)
        if hasattr(mpc_params, "recover_attitude_scale"):
            mpc_params.recover_attitude_scale = float(self.recover_attitude_scale)
        if hasattr(mpc_params, "settle_progress_scale"):
            mpc_params.settle_progress_scale = float(self.settle_progress_scale)
        if hasattr(mpc_params, "settle_terminal_pos_scale"):
            mpc_params.settle_terminal_pos_scale = float(self.settle_terminal_pos_scale)
        if hasattr(mpc_params, "settle_terminal_attitude_scale"):
            mpc_params.settle_terminal_attitude_scale = float(
                self.settle_terminal_attitude_scale
            )
        if hasattr(mpc_params, "settle_velocity_align_scale"):
            mpc_params.settle_velocity_align_scale = float(
                self.settle_velocity_align_scale
            )
        if hasattr(mpc_params, "settle_angular_velocity_scale"):
            mpc_params.settle_angular_velocity_scale = float(
                self.settle_angular_velocity_scale
            )
        if hasattr(mpc_params, "hold_smoothness_scale"):
            mpc_params.hold_smoothness_scale = float(self.hold_smoothness_scale)
        if hasattr(mpc_params, "hold_thruster_pair_scale"):
            mpc_params.hold_thruster_pair_scale = float(self.hold_thruster_pair_scale)
        if hasattr(mpc_params, "solver_fallback_hold_s"):
            mpc_params.solver_fallback_hold_s = float(self.solver_fallback_hold_s)
        if hasattr(mpc_params, "solver_fallback_decay_s"):
            mpc_params.solver_fallback_decay_s = float(self.solver_fallback_decay_s)
        if hasattr(mpc_params, "solver_fallback_zero_after_s"):
            mpc_params.solver_fallback_zero_after_s = float(
                self.solver_fallback_zero_after_s
            )
        if hasattr(mpc_params, "max_linear_velocity"):
            mpc_params.max_linear_velocity = float(self.max_linear_velocity)
        if hasattr(mpc_params, "max_angular_velocity"):
            mpc_params.max_angular_velocity = float(self.max_angular_velocity)
        if hasattr(mpc_params, "enable_delta_u_coupling"):
            mpc_params.enable_delta_u_coupling = bool(self.enable_delta_u_coupling)
        if hasattr(mpc_params, "enable_gyro_jacobian"):
            mpc_params.enable_gyro_jacobian = bool(self.enable_gyro_jacobian)
        if hasattr(mpc_params, "enable_auto_state_bounds"):
            mpc_params.enable_auto_state_bounds = bool(self.enable_auto_state_bounds)

        # Collision Avoidance
        if hasattr(mpc_params, "enable_collision_avoidance"):
            mpc_params.enable_collision_avoidance = bool(
                self.enable_collision_avoidance
            )
        if hasattr(mpc_params, "obstacle_margin"):
            mpc_params.obstacle_margin = cfg.mpc.obstacle_margin

        self._cpp_controller = MPCControllerCpp(sat_params, mpc_params)

        # Performance tracking
        self.solve_times: list[float] = []

        # Path following state
        self.s = 0.0
        self._path_data: list[list[float]] = []  # [(s, x, y, z), ...]
        self._path_set = False
        self._path_length = 0.0
        self._last_path_projection: dict[str, Any] = {}
        self._scan_attitude_enabled = False
        self._runtime_mode = "TRACK"

        # Dimensions (Fixed for MPCC)
        self.nx = 17
        self.nu = self.num_rw_axes + self.num_thrusters

        logger.info(
            "MPC Controller initialized (C++ backend, core=%s, solver=%s). Thrusters: %d, RW: %d",
            self.controller_core,
            self.solver_backend,
            self.num_thrusters,
            self.num_rw_axes,
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

    def set_obstacles(self, obstacles: list[Any] | None) -> None:
        """
        Set runtime obstacle constraints for MPC.

        Supported formats:
        - tuple/list: (x, y, z, radius) for spherical obstacles
        - dict/object with fields: type, position, radius, size, axis, name
        """
        if not hasattr(self, "_cpp_controller") or not hasattr(
            self._cpp_controller, "set_obstacles"
        ):
            return

        if not obstacles:
            self.clear_obstacles()
            return

        if ObstacleSet is None or Obstacle is None or ObstacleType is None:
            logger.warning(
                "C++ obstacle bindings unavailable; skipping obstacle update"
            )
            return

        obstacle_set = ObstacleSet()
        added = 0

        def _to_enum(raw_type: Any) -> Any:
            if raw_type is None:
                return ObstacleType.SPHERE
            if hasattr(raw_type, "name"):
                raw_type = raw_type.name
            t = str(raw_type).strip().upper()
            if t.endswith(".SPHERE") or t == "SPHERE":
                return ObstacleType.SPHERE
            if t.endswith(".CYLINDER") or t == "CYLINDER":
                return ObstacleType.CYLINDER
            if t.endswith(".BOX") or t == "BOX":
                return ObstacleType.BOX
            return ObstacleType.SPHERE

        for item in obstacles:
            obs = Obstacle()
            try:
                if isinstance(item, list | tuple | np.ndarray) and len(item) >= 4:
                    arr = np.array(item, dtype=float).reshape(-1)
                    obs.type = ObstacleType.SPHERE
                    obs.position = np.array(arr[:3], dtype=float)
                    obs.radius = max(0.0, float(arr[3]))
                    obs.name = "obstacle"
                else:
                    if isinstance(item, dict):
                        position = item.get("position", None)
                    else:
                        position = getattr(item, "position", None)
                    if position is None:
                        continue

                    if isinstance(item, dict):
                        raw_type = item.get("type", "sphere")
                        radius = item.get("radius", obs.radius)
                        size = item.get("size", obs.size)
                        axis = item.get("axis", obs.axis)
                        name = item.get("name", "obstacle")
                    else:
                        raw_type = getattr(item, "type", "sphere")
                        radius = getattr(item, "radius", obs.radius)
                        size = getattr(item, "size", obs.size)
                        axis = getattr(item, "axis", obs.axis)
                        name = getattr(item, "name", "obstacle")

                    obs.type = _to_enum(raw_type)
                    obs.position = np.array(position, dtype=float)
                    obs.radius = max(0.0, float(radius))
                    obs.size = np.array(size, dtype=float)
                    obs.axis = np.array(axis, dtype=float)
                    obs.name = str(name)

                obstacle_set.add(obs)
                added += 1
            except Exception:
                logger.debug("Skipping invalid obstacle entry", exc_info=True)

        if added == 0:
            self.clear_obstacles()
            return

        self._cpp_controller.set_obstacles(obstacle_set)
        self.enable_collision_avoidance = True
        logger.info("Applied %d MPC obstacle constraints", added)

    def clear_obstacles(self) -> None:
        """Clear all runtime MPC obstacle constraints."""
        if hasattr(self, "_cpp_controller") and hasattr(
            self._cpp_controller, "clear_obstacles"
        ):
            self._cpp_controller.clear_obstacles()
        self.enable_collision_avoidance = False

    def set_scan_attitude_context(
        self,
        center: tuple[float, float, float] | None,
        axis: tuple[float, float, float] | None,
        direction: str = "CW",
    ) -> None:
        """Configure scan attitude context (+Z aligns with mission scan axis)."""
        if axis is None:
            if hasattr(self._cpp_controller, "clear_scan_attitude_context"):
                self._cpp_controller.clear_scan_attitude_context()
            self._scan_attitude_enabled = False
            return

        if hasattr(self._cpp_controller, "set_scan_attitude_context"):
            if center is None:
                c = np.array([np.nan, np.nan, np.nan], dtype=float)
            else:
                c = np.array(center, dtype=float)
            a = np.array(axis, dtype=float)
            self._cpp_controller.set_scan_attitude_context(c, a, str(direction))
        self._scan_attitude_enabled = True

    def set_runtime_mode(self, mode: str | None) -> None:
        """Set runtime V6 mode in the C++ controller core."""
        mode_name = str(mode or "TRACK").upper()
        self._runtime_mode = mode_name
        if hasattr(self._cpp_controller, "set_runtime_mode"):
            try:
                self._cpp_controller.set_runtime_mode(mode_name)
            except Exception:
                logger.debug(
                    "Failed to set runtime mode in C++ MPC core", exc_info=True
                )

    @staticmethod
    def _classify_solver_fallback_reason(
        *,
        status: int,
        solver_status: Any,
        timeout: bool,
        time_limit_exceeded: bool,
    ) -> str | None:
        if int(status) == 1:
            return None
        if bool(timeout) or bool(time_limit_exceeded):
            return "solver_timeout"
        if solver_status is not None:
            return f"solver_non_success_{solver_status}"
        return "solver_non_success"

    def _project_onto_path(
        self, position: np.ndarray
    ) -> tuple[float, np.ndarray, float]:
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
                logger.debug(
                    "C++ project_onto_path failed, using Python fallback", exc_info=True
                )

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

    def get_path_progress(self, position: np.ndarray | None = None) -> dict[str, float]:
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
            if hasattr(self._cpp_controller, "current_path_s"):
                try:
                    s_val = float(self._cpp_controller.current_path_s)
                except Exception:
                    s_val = float(self.s)
            else:
                s_val = float(self.s)
            if self._last_path_projection:
                path_error = float(
                    self._last_path_projection.get("path_error", float("inf"))
                )
                endpoint_error = float(
                    self._last_path_projection.get("endpoint_error", float("inf"))
                )
            else:
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
                    logger.debug(
                        "C++ project_onto_path failed in get_path_progress, using Python fallback",
                        exc_info=True,
                    )
                    s_val, _, path_error = self._project_onto_path(position)
                    endpoint = np.array(self._path_data[-1][1:4], dtype=float)
                    endpoint_error = float(np.linalg.norm(pos - endpoint))
            else:
                s_val, _, path_error = self._project_onto_path(position)
                endpoint = np.array(self._path_data[-1][1:4], dtype=float)
                endpoint_error = float(np.linalg.norm(pos - endpoint))

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
        self, s_query: float | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Get the path reference position and tangent for a given arc-length.

        Returns:
            Tuple of (position, unit_tangent). Falls back to zeros if path not set.
        """
        q_guess = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        pos_ref, tan_ref, _ = self.get_path_reference_state(
            s_query=s_query, q_current=q_guess
        )
        return pos_ref, tan_ref

    def get_path_reference_state(
        self,
        s_query: float | None = None,
        q_current: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get path reference position/tangent/quaternion for a given arc-length.

        Uses C++ reference-frame generation when available so viewer/ghost and MPC
        cost share the same attitude logic.
        """
        if not self._path_data or len(self._path_data) < 2:
            return (
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        s_val = float(self.s if s_query is None else s_query)
        if hasattr(self, "_path_length"):
            s_val = max(0.0, min(s_val, float(self._path_length)))

        q_curr = (
            np.array(q_current, dtype=float)
            if q_current is not None
            else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        )
        if hasattr(self._cpp_controller, "get_reference_at_s"):
            try:
                pos, tangent, q_ref = self._cpp_controller.get_reference_at_s(
                    s_val, q_curr
                )
                return (
                    np.array(pos, dtype=float),
                    np.array(tangent, dtype=float),
                    np.array(q_ref, dtype=float),
                )
            except Exception:
                logger.debug(
                    "C++ get_reference_at_s failed, using Python fallback",
                    exc_info=True,
                )

        # At exact path end, use the final non-degenerate segment direction
        # (second-last waypoint heading) because no forward segment exists.
        if (
            self._scan_attitude_enabled
            and self._path_length > 0.0
            and s_val >= (self._path_length - 1e-9)
        ):
            for i in range(len(self._path_data) - 1, 0, -1):
                x_prev, y_prev, z_prev = self._path_data[i - 1][1:4]
                x_cur, y_cur, z_cur = self._path_data[i][1:4]
                last_tangent = np.array(
                    [x_cur - x_prev, y_cur - y_prev, z_cur - z_prev], dtype=float
                )
                last_norm = np.linalg.norm(last_tangent)
                if last_norm > 1e-9:
                    p_end = np.array(self._path_data[-1][1:4], dtype=float)
                    return p_end, (last_tangent / last_norm), q_curr

        # Find the segment that contains s_val
        idx = 0
        while idx + 1 < len(self._path_data) and self._path_data[idx + 1][0] <= s_val:
            idx += 1

        # Use forward segment for heading (face next waypoint). At exact interior
        # waypoints this chooses [idx -> idx+1]. At terminal s, keep second-last
        # heading using [last-1 -> last].
        if idx >= len(self._path_data) - 1:
            idx = len(self._path_data) - 2

        s0, x0, y0, z0 = self._path_data[idx]
        s1, x1, y1, z1 = self._path_data[idx + 1]

        seg_len = s1 - s0
        if seg_len <= 1e-9:
            pos = np.array([x0, y0, z0], dtype=float)
            tangent = np.array([0.0, 0.0, 0.0], dtype=float)
            return pos, tangent, q_curr

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

        return pos, tangent, q_curr

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
        mpc_core = getattr(cfg, "mpc_core", None)
        controller_contracts = getattr(cfg, "controller_contracts", None)
        self.controller_core = "v6"
        self.solver_backend = str(getattr(mpc_core, "solver_backend", "OSQP")).upper()
        if self.solver_backend != "OSQP":
            logger.warning(
                "mpc_core.solver_backend '%s' is unsupported; using OSQP.",
                self.solver_backend,
            )
            self.solver_backend = "OSQP"

        self.solver_type = str(getattr(mpc, "solver_type", "OSQP"))
        if self.solver_type.upper() != "OSQP":
            logger.warning(
                "MPC solver_type '%s' not supported, using OSQP.", self.solver_type
            )
            self.solver_type = "OSQP"
        if self.solver_type.upper() != self.solver_backend:
            logger.info(
                "Aligning solver_type '%s' to mpc_core backend '%s'.",
                self.solver_type,
                self.solver_backend,
            )
            self.solver_type = self.solver_backend
        self.verbose_mpc = getattr(mpc, "verbose_mpc", False)

        self.Q_angvel = mpc.q_angular_velocity
        self.Q_attitude = getattr(mpc, "Q_attitude", 0.0)
        self.Q_axis_align = getattr(mpc, "Q_axis_align", 0.0)
        self.R_thrust = mpc.r_thrust
        self.R_rw_torque = mpc.r_rw_torque if hasattr(mpc, "r_rw_torque") else 0.1
        self.thrust_l1_weight = getattr(mpc, "thrust_l1_weight", 0.0)
        self.thrust_pair_weight = getattr(mpc, "thrust_pair_weight", 0.0)
        self.coast_pos_tolerance = getattr(mpc, "coast_pos_tolerance", 0.0)
        self.coast_vel_tolerance = getattr(mpc, "coast_vel_tolerance", 0.0)
        self.coast_min_speed = getattr(mpc, "coast_min_speed", 0.0)
        self.max_linear_velocity = getattr(mpc, "max_linear_velocity", 0.0)
        self.max_angular_velocity = getattr(mpc, "max_angular_velocity", 0.0)
        self.enable_delta_u_coupling = bool(
            getattr(mpc, "enable_delta_u_coupling", False)
        )
        self.enable_gyro_jacobian = bool(getattr(mpc, "enable_gyro_jacobian", False))
        self.enable_auto_state_bounds = bool(
            getattr(mpc, "enable_auto_state_bounds", False)
        )
        self.enable_collision_avoidance = bool(
            getattr(mpc, "enable_collision_avoidance", False)
        )

        # Path Following. - General Path MPCC
        self.mode_path_following = True  # Always True now
        self.Q_contour = mpc.Q_contour
        self.Q_progress = mpc.Q_progress
        self.progress_reward = getattr(mpc, "progress_reward", 0.0)
        self.Q_smooth = mpc.Q_smooth
        self.Q_lag = getattr(mpc, "Q_lag", 0.0)
        self.Q_lag_default = getattr(mpc, "Q_lag_default", -1.0)
        self.Q_velocity_align = getattr(mpc, "Q_velocity_align", 0.0)
        self.Q_s_anchor = getattr(mpc, "Q_s_anchor", -1.0)
        self.Q_terminal_pos = getattr(mpc, "Q_terminal_pos", 0.0)
        self.Q_terminal_s = getattr(mpc, "Q_terminal_s", 0.0)
        self.path_speed = mpc.path_speed
        self.path_speed_min = getattr(mpc, "path_speed_min", 0.0)
        self.path_speed_max = getattr(mpc, "path_speed_max", 0.0)
        self.progress_taper_distance = getattr(mpc, "progress_taper_distance", 0.0)
        self.progress_slowdown_distance = getattr(
            mpc, "progress_slowdown_distance", 0.0
        )
        self.recover_contour_scale = float(
            getattr(mpc_core, "recover_contour_scale", 2.0)
        )
        self.recover_lag_scale = float(getattr(mpc_core, "recover_lag_scale", 2.0))
        self.recover_progress_scale = float(
            getattr(mpc_core, "recover_progress_scale", 0.6)
        )
        self.recover_attitude_scale = float(
            getattr(mpc_core, "recover_attitude_scale", 0.8)
        )
        self.settle_progress_scale = float(
            getattr(mpc_core, "settle_progress_scale", 0.0)
        )
        self.settle_terminal_pos_scale = float(
            getattr(mpc_core, "settle_terminal_pos_scale", 2.0)
        )
        self.settle_terminal_attitude_scale = float(
            getattr(mpc_core, "settle_terminal_attitude_scale", 1.5)
        )
        self.settle_velocity_align_scale = float(
            getattr(mpc_core, "settle_velocity_align_scale", 1.5)
        )
        self.settle_angular_velocity_scale = float(
            getattr(mpc_core, "settle_angular_velocity_scale", 2.0)
        )
        self.hold_smoothness_scale = float(
            getattr(mpc_core, "hold_smoothness_scale", 1.5)
        )
        self.hold_thruster_pair_scale = float(
            getattr(mpc_core, "hold_thruster_pair_scale", 1.2)
        )
        self.solver_fallback_hold_s = float(
            getattr(controller_contracts, "solver_fallback_hold_s", 0.30)
        )
        self.solver_fallback_decay_s = float(
            getattr(controller_contracts, "solver_fallback_decay_s", 0.70)
        )
        self.solver_fallback_zero_after_s = float(
            getattr(controller_contracts, "solver_fallback_zero_after_s", 1.00)
        )
        self.enable_thruster_hysteresis = bool(
            getattr(mpc, "enable_thruster_hysteresis", True)
        )
        self.thruster_hysteresis_on = float(
            getattr(mpc, "thruster_hysteresis_on", 0.015)
        )
        self.thruster_hysteresis_off = float(
            getattr(mpc, "thruster_hysteresis_off", 0.007)
        )

        # Orbital parameters (for MPC linearization)
        try:
            from satellite_control.config.orbital_config import OrbitalConfig

            orbital_cfg = getattr(physics, "orbital", None)
            if orbital_cfg is not None:
                self.orbital_mu = float(getattr(orbital_cfg, "mu", OrbitalConfig().mu))
                self.orbital_radius = float(
                    getattr(
                        orbital_cfg, "orbital_radius", OrbitalConfig().orbital_radius
                    )
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
            logger.warning(
                "Failed to load orbital config, using Earth LEO defaults", exc_info=True
            )
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

    def split_control(self, control: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Split control vector into RW torques and thruster commands."""
        rw_torques = control[: self.num_rw_axes]
        thruster_cmds = control[self.num_rw_axes :]
        return rw_torques, thruster_cmds

    def get_solver_stats(self) -> dict[str, Any]:
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
        previous_thrusters: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Compute optimal control action via C++ backend."""
        # C++ owns path-progress continuity and projection filtering.
        s_for_state = float(self.s)
        if self._path_set and hasattr(self._cpp_controller, "current_path_s"):
            try:
                s_for_state = float(self._cpp_controller.current_path_s)
                self.s = s_for_state
            except Exception:
                logger.debug("Failed to read C++ current_path_s", exc_info=True)

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
                logger.debug("Warm-start control failed", exc_info=True)

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
                logger.debug("Path reference override failed", exc_info=True)
            result = self._cpp_controller.get_control_action(x_input, x_target)

        self.solve_times.append(result.solve_time)

        # Process output controls
        u_out = np.array(result.u)

        # Extract virtual control v_s (last element)
        v_s = float(u_out[-1]) if u_out.size > 0 else 0.0

        path_s = float(getattr(result, "path_s", s_for_state))
        path_s_proj = getattr(result, "path_s_proj", None)
        path_error = getattr(result, "path_error", None)
        endpoint_error = getattr(result, "path_endpoint_error", None)
        path_s_pred_raw = getattr(result, "path_s_pred", None)
        if path_s_pred_raw is None:
            path_s_pred = path_s + v_s * self.dt
            if self._path_set and self._path_length > 0.0:
                path_s_pred = max(
                    0.0, min(float(path_s_pred), float(self._path_length))
                )
        else:
            path_s_pred = float(path_s_pred_raw)
        # Use current-step path progress for external reference consumers
        # (viewer ghost/reference), while still exposing predicted progress
        # separately via `path_s_pred` telemetry.
        self.s = path_s
        self._last_path_projection = {
            "s": path_s,
            "s_proj": float(path_s_proj) if path_s_proj is not None else None,
            "path_error": float(path_error) if path_error is not None else None,
            "endpoint_error": (
                float(endpoint_error) if endpoint_error is not None else None
            ),
            "s_pred": path_s_pred,
        }

        # Strip virtual control from output so simulation gets only physical actuators
        u_phys = u_out[:-1] if u_out.size > 0 else u_out

        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        progress = float(path_s / path_len) if path_len > 1e-9 else 0.0
        remaining = float(path_len - path_s) if path_len > 0 else 0.0
        status_code = int(getattr(result, "status", -1))
        solver_status = getattr(result, "solver_status", None)
        timeout = bool(getattr(result, "timeout", False))
        solve_time = float(getattr(result, "solve_time", 0.0))
        time_limit_exceeded = bool(timeout) or (
            solve_time > float(self.solver_time_limit)
        )
        solver_fallback = bool(status_code != 1)
        fallback_active = bool(getattr(result, "fallback_active", False))
        fallback_age_s = float(getattr(result, "fallback_age_s", 0.0))
        fallback_scale = float(getattr(result, "fallback_scale", 0.0))
        fallback_reason = self._classify_solver_fallback_reason(
            status=status_code,
            solver_status=solver_status,
            timeout=timeout,
            time_limit_exceeded=time_limit_exceeded,
        )

        extras = {
            "status": status_code,
            "status_name": "SUCCESS" if status_code == 1 else "FAILED",
            "solver_status": solver_status,
            "timeout": timeout,
            "solve_time": solve_time,
            "iterations": getattr(result, "iterations", None),
            "objective_value": getattr(result, "objective", None),
            "optimality_gap": None,
            "solver_type": self.solver_type,
            "solver_backend": self.solver_backend,
            "solver_time_limit": self.solver_time_limit,
            "solver_fallback": solver_fallback,
            "solver_fallback_reason": fallback_reason,
            "solver_success": not solver_fallback,
            "time_limit_exceeded": time_limit_exceeded,
            "fallback_active": fallback_active,
            "fallback_age_s": fallback_age_s,
            "fallback_scale": fallback_scale,
            "controller_core": self.controller_core,
            "path_s": path_s,
            "path_s_proj": float(path_s_proj) if path_s_proj is not None else None,
            "path_v_s": v_s,
            "path_progress": progress,
            "path_remaining": remaining,
            "path_error": float(path_error) if path_error is not None else None,
            "path_endpoint_error": (
                float(endpoint_error) if endpoint_error is not None else None
            ),
            "path_s_pred": path_s_pred,
        }
        return u_phys, extras

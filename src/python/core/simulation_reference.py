"""
Path-following reference state computation.

Extracted from SatelliteMPCLinearizedSimulation.update_path_reference_state
to reduce simulation.py size while keeping the public API unchanged.
"""

import logging
import math
from typing import TYPE_CHECKING

import numpy as np
from utils.orientation_utils import quat_wxyz_from_basis

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from core.simulation import (
        SatelliteMPCLinearizedSimulation,
    )


def _norm3(v: np.ndarray) -> float:
    """Fast Euclidean norm for 3-element vectors (avoids numpy dispatch overhead)."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _normalize_or_default(v: np.ndarray, default: np.ndarray) -> np.ndarray:
    """Return unit vector, or default when norm is near zero."""
    n = _norm3(v)
    if n <= 1e-9:
        return np.array(default, dtype=float)
    return v / n


def _rotate_vec_by_quat_wxyz(q_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate vector by quaternion [w, x, y, z]."""
    q = np.array(q_wxyz, dtype=float).reshape(-1)
    v = np.array(vec, dtype=float).reshape(-1)
    if q.size < 4 or v.size < 3:
        return np.array([0.0, 0.0, 0.0], dtype=float)

    q = q[:4]
    v = v[:3]
    q_norm = float(np.linalg.norm(q))
    if q_norm <= 1e-12:
        return v.copy()
    q = q / q_norm
    s = float(q[0])
    u = q[1:4]
    # Equivalent to q * [0,v] * q_conj, expanded without temporary quaternions.
    return (
        2.0 * float(np.dot(u, v)) * u
        + (s * s - float(np.dot(u, u))) * v
        + 2.0 * s * np.cross(u, v)
    )


def _choose_seed_axis(x_axis: np.ndarray) -> np.ndarray:
    """Choose a fallback axis least aligned with x_axis."""
    candidates = (
        np.array([1.0, 0.0, 0.0], dtype=float),
        np.array([0.0, 1.0, 0.0], dtype=float),
        np.array([0.0, 0.0, 1.0], dtype=float),
    )
    best = candidates[0]
    best_abs_dot = abs(float(np.dot(x_axis, best)))
    for cand in candidates[1:]:
        cand_abs_dot = abs(float(np.dot(x_axis, cand)))
        if cand_abs_dot < best_abs_dot:
            best = cand
            best_abs_dot = cand_abs_dot
    return best


def _compute_continuous_path_frame(
    sim: "SatelliteMPCLinearizedSimulation",
    x_axis: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Build a right-handed orthonormal frame with continuity across updates.

    Uses the previous reference z-axis projected onto the plane orthogonal to
    current tangent (x_axis). Falls back to a minimally aligned global seed
    when needed.
    """
    prev_z = getattr(sim, "_ref_prev_z_axis", None)
    if isinstance(prev_z, np.ndarray) and prev_z.shape == (3,):
        z_seed = prev_z.astype(float, copy=False)
    else:
        z_seed = _choose_seed_axis(x_axis)

    z_axis = z_seed - np.dot(z_seed, x_axis) * x_axis
    z_norm = _norm3(z_axis)
    if z_norm <= 1e-9:
        z_seed = _choose_seed_axis(x_axis)
        z_axis = z_seed - np.dot(z_seed, x_axis) * x_axis
        z_norm = _norm3(z_axis)
    if z_norm <= 1e-9:
        z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        z_axis = z_axis / z_norm

    y_axis = np.cross(z_axis, x_axis)
    y_norm = _norm3(y_axis)
    if y_norm <= 1e-9:
        y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
    else:
        y_axis = y_axis / y_norm

    z_axis = np.cross(x_axis, y_axis)
    z_norm2 = _norm3(z_axis)
    if z_norm2 > 1e-9:
        z_axis = z_axis / z_norm2

    prev_y = getattr(sim, "_ref_prev_y_axis", None)
    if isinstance(prev_y, np.ndarray) and prev_y.shape == (3,):
        if float(np.dot(prev_y, y_axis)) < 0.0:
            y_axis = -y_axis
            z_axis = -z_axis

    sim._ref_prev_y_axis = y_axis.copy()
    sim._ref_prev_z_axis = z_axis.copy()
    return x_axis, y_axis, z_axis


def _get_scan_attitude_context(
    sim: "SatelliteMPCLinearizedSimulation",
) -> tuple[np.ndarray, np.ndarray, bool] | None:
    """Return scan center/axis/direction when scan-attitude mode is active."""
    mission_state = None
    if hasattr(sim, "_get_mission_state"):
        mission_state = sim._get_mission_state()
    if mission_state is None and sim.simulation_config is not None:
        mission_state = getattr(sim.simulation_config, "mission_state", None)
    if mission_state is None:
        return None

    center = getattr(mission_state, "scan_attitude_center", None)
    axis = getattr(mission_state, "scan_attitude_axis", None)
    if center is None or axis is None:
        return None

    center_arr = np.array(center, dtype=float).reshape(-1)
    axis_arr = np.array(axis, dtype=float).reshape(-1)
    if center_arr.size < 3 or axis_arr.size < 3:
        return None
    center_vec = center_arr[:3]
    axis_vec = _normalize_or_default(axis_arr[:3], np.array([0.0, 0.0, 1.0]))

    direction_raw = str(getattr(mission_state, "scan_attitude_direction", "CW"))
    scan_direction_cw = direction_raw.strip().upper() != "CCW"
    return center_vec, axis_vec, scan_direction_cw


def _compute_scan_path_frame(
    pos_ref: np.ndarray,
    x_axis: np.ndarray,
    center: np.ndarray,
    scan_axis: np.ndarray,
    scan_direction_cw: bool,
    current_state: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Build scan frame to match MPC scan-attitude rule:
    +X along path, +Z on scan-axis line, +Y toward object center.
    """
    tangent_dir = _normalize_or_default(
        np.array(x_axis, dtype=float), np.array([1.0, 0.0, 0.0], dtype=float)
    )
    z_line = _normalize_or_default(scan_axis, np.array([0.0, 0.0, 1.0], dtype=float))

    radial_in = center - pos_ref
    radial_in = radial_in - float(np.dot(radial_in, z_line)) * z_line
    radial_norm = _norm3(radial_in)
    has_radial = radial_norm > 1e-9

    if has_radial:
        radial_dir = radial_in / radial_norm
    else:
        radial_dir = np.array([0.0, 1.0, 0.0], dtype=float)
        if current_state.shape[0] >= 7:
            q_curr = np.array(current_state[3:7], dtype=float)
            if float(np.linalg.norm(q_curr)) > 1e-9:
                curr_y = _rotate_vec_by_quat_wxyz(q_curr, np.array([0.0, 1.0, 0.0]))
                curr_y = curr_y - float(np.dot(curr_y, z_line)) * z_line
                curr_y_norm = _norm3(curr_y)
                if curr_y_norm > 1e-9:
                    radial_dir = curr_y / curr_y_norm
                    has_radial = True

    if not has_radial:
        ref = (
            np.array([0.0, 0.0, 1.0], dtype=float)
            if abs(float(z_line[2])) < 0.9
            else np.array([1.0, 0.0, 0.0], dtype=float)
        )
        radial_dir = np.cross(z_line, ref)
        radial_dir = _normalize_or_default(radial_dir, np.array([0.0, 1.0, 0.0]))
        has_radial = True

    t_plane = x_axis - float(np.dot(x_axis, z_line)) * z_line
    t_plane_norm = _norm3(t_plane)
    if t_plane_norm > 1e-9:
        x_axis = t_plane / t_plane_norm
        z_axis = z_line.copy()
        y_axis = np.cross(z_axis, x_axis)
    else:
        z_axis = z_line.copy()
        y_axis = radial_dir.copy()
        if scan_direction_cw:
            x_axis = np.cross(z_axis, y_axis)
        else:
            x_axis = np.cross(y_axis, z_axis)

    x_axis = x_axis - float(np.dot(x_axis, z_axis)) * z_axis
    x_norm = _norm3(x_axis)
    if x_norm <= 1e-9:
        if scan_direction_cw:
            x_axis = np.cross(z_axis, radial_dir)
        else:
            x_axis = np.cross(radial_dir, z_axis)
        x_norm = _norm3(x_axis)
        if x_norm <= 1e-9:
            x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
            x_norm = 1.0
    x_axis = x_axis / x_norm

    y_axis = np.cross(z_axis, x_axis)
    y_norm = _norm3(y_axis)
    if y_norm > 1e-9:
        y_axis = y_axis / y_norm
    else:
        y_axis = radial_dir.copy()

    x_axis = np.cross(y_axis, z_axis)
    x_axis = _normalize_or_default(x_axis, np.array([1.0, 0.0, 0.0], dtype=float))
    z_axis = _normalize_or_default(z_axis, np.array([0.0, 0.0, 1.0], dtype=float))
    y_axis = np.cross(z_axis, x_axis)
    y_axis = _normalize_or_default(y_axis, np.array([0.0, 1.0, 0.0], dtype=float))
    x_axis = np.cross(y_axis, z_axis)
    x_axis = _normalize_or_default(x_axis, np.array([1.0, 0.0, 0.0], dtype=float))

    # Keep +X forward along path travel (avoid backward branch selection).
    if float(np.dot(x_axis, tangent_dir)) < 0.0:
        x_axis = -x_axis
        y_axis = -y_axis

    return x_axis, y_axis, z_axis


# Module-level buffer to avoid per-call allocation of np.zeros(13)
_REF_BUF = np.zeros(13, dtype=np.float64)


def update_path_reference_state(
    sim: "SatelliteMPCLinearizedSimulation",
    current_state: np.ndarray,
) -> None:
    """
    Update the reference state from the MPCC path data.

    Sets ``sim.reference_state`` to a 13-element vector that tracks the
    current path position, derives a forward-looking orientation from the
    path tangent, and mirrors V6 reference-speed ownership from MPC runtime.

    Args:
        sim: The simulation instance whose ``reference_state`` will be updated.
        current_state: Current state vector
            ``[x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]``.
    """
    if not hasattr(sim, "mpc_controller") or not sim.mpc_controller:
        sim.reference_state = current_state[:13].copy()
        return

    # Always MPCC mode
    q_ref_from_controller = None
    can_query_cpp_ref = hasattr(sim.mpc_controller, "_cpp_controller") and hasattr(
        sim.mpc_controller._cpp_controller, "get_reference_at_s"
    )
    if can_query_cpp_ref and hasattr(sim.mpc_controller, "get_path_reference_state"):
        try:
            q_curr = (
                np.array(current_state[3:7], dtype=float)
                if current_state.shape[0] >= 7
                else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            )
            pos_ref, tangent, q_ref_from_controller = (
                sim.mpc_controller.get_path_reference_state(q_current=q_curr)
            )
        except Exception:
            logger.debug(
                "C++ reference frame query failed, using Python frame fallback",
                exc_info=True,
            )
            pos_ref, tangent = sim.mpc_controller.get_path_reference()
            q_ref_from_controller = None
    else:
        pos_ref, tangent = sim.mpc_controller.get_path_reference()

    reference_state = _REF_BUF
    reference_state[:] = 0.0
    reference_state[0:3] = pos_ref

    tangent_norm = _norm3(tangent)
    if tangent_norm > 1e-6:
        if (
            q_ref_from_controller is not None
            and np.linalg.norm(q_ref_from_controller) > 1e-9
        ):
            q_norm = float(np.linalg.norm(q_ref_from_controller))
            reference_state[3:7] = np.array(q_ref_from_controller, dtype=float) / q_norm
        else:
            x_axis = tangent / tangent_norm
            scan_context = _get_scan_attitude_context(sim)
            if scan_context is not None:
                scan_center, scan_axis, scan_direction_cw = scan_context
                x_axis, y_axis, z_axis = _compute_scan_path_frame(
                    pos_ref=np.array(pos_ref, dtype=float),
                    x_axis=np.array(x_axis, dtype=float),
                    center=scan_center,
                    scan_axis=scan_axis,
                    scan_direction_cw=scan_direction_cw,
                    current_state=current_state,
                )
            else:
                x_axis, y_axis, z_axis = _compute_continuous_path_frame(sim, x_axis)

            reference_state[3:7] = quat_wxyz_from_basis(x_axis, y_axis, z_axis)
    else:
        # Maintain current orientation if stationary
        reference_state[3:7] = current_state[3:7]

    # --- Read MPC config (cached on sim to avoid repeated getattr per step) ---
    if not hasattr(sim, "_ref_config_cached"):
        sim._ref_config_cached = True
        if sim.simulation_config is not None:
            mpc_cfg = sim.simulation_config.app_config.mpc
            sim._ref_path_speed = float(mpc_cfg.path_speed)
            sim._ref_path_speed_min = float(getattr(mpc_cfg, "path_speed_min", 0.0))
            sim._ref_path_speed_max = float(getattr(mpc_cfg, "path_speed_max", 0.0))
        else:
            sim._ref_path_speed = 0.0
            sim._ref_path_speed_min = 0.0
            sim._ref_path_speed_max = 0.0

    path_speed = float(getattr(sim, "_ref_path_speed", 0.0))
    path_speed_min = float(getattr(sim, "_ref_path_speed_min", 0.0))
    path_speed_max = float(getattr(sim, "_ref_path_speed_max", 0.0))
    current_mode = str(
        getattr(getattr(sim, "v6_mode_state", None), "current_mode", "TRACK")
    ).upper()

    v_ref = path_speed
    projection_cache = getattr(sim.mpc_controller, "_last_path_projection", None)
    if isinstance(projection_cache, dict):
        path_v_s = projection_cache.get("path_v_s")
        if path_v_s is not None:
            try:
                path_v_s_val = float(path_v_s)
                if np.isfinite(path_v_s_val):
                    v_ref = path_v_s_val
            except (TypeError, ValueError):
                logger.debug(
                    "Invalid cached path_v_s for reference velocity", exc_info=True
                )

    if path_speed_max > 0.0 and path_speed_min > path_speed_max:
        path_speed_min = 0.0
    if path_speed_min > 0.0:
        v_ref = max(v_ref, path_speed_min)
    if path_speed_max > 0.0:
        v_ref = min(v_ref, path_speed_max)

    if current_mode in {"SETTLE", "HOLD", "COMPLETE"}:
        # V6 explicit endpoint behavior: SETTLE/HOLD use zero velocity targets.
        v_ref = 0.0

    reference_state[7:10] = tangent * v_ref

    sim.reference_state = reference_state.copy()

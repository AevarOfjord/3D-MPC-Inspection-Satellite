"""
Path-following reference state computation.

Extracted from SatelliteMPCLinearizedSimulation.update_path_reference_state
to reduce simulation.py size while keeping the public API unchanged.
"""

import logging
import math
from typing import TYPE_CHECKING

import numpy as np

from src.satellite_control.utils.orientation_utils import quat_wxyz_from_basis

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from src.satellite_control.core.simulation import (
        SatelliteMPCLinearizedSimulation,
    )


def _norm3(v: np.ndarray) -> float:
    """Fast Euclidean norm for 3-element vectors (avoids numpy dispatch overhead)."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


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
    path tangent, and tapers the reference velocity to zero near the path
    end so that completion tolerances can be satisfied.

    Args:
        sim: The simulation instance whose ``reference_state`` will be updated.
        current_state: Current state vector
            ``[x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]``.
    """
    if not hasattr(sim, "mpc_controller") or not sim.mpc_controller:
        sim.reference_state = current_state[:13].copy()
        return

    # Always MPCC mode
    pos_ref, tangent = sim.mpc_controller.get_path_reference()
    reference_state = _REF_BUF
    reference_state[:] = 0.0
    reference_state[0:3] = pos_ref

    tangent_norm = _norm3(tangent)
    if tangent_norm > 1e-6:
        x_axis = tangent / tangent_norm
        up = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(x_axis, up))) > 0.95:
            up = np.array([0.0, 1.0, 0.0], dtype=float)
        z_axis = up - np.dot(up, x_axis) * x_axis
        z_norm = _norm3(z_axis)
        if z_norm > 1e-6:
            z_axis = z_axis / z_norm
        else:
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
        y_axis = np.cross(z_axis, x_axis)
        y_norm = _norm3(y_axis)
        if y_norm > 1e-6:
            y_axis = y_axis / y_norm
        else:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
        z_axis = np.cross(x_axis, y_axis)
        z_norm2 = _norm3(z_axis)
        if z_norm2 > 1e-6:
            z_axis = z_axis / z_norm2

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
            sim._ref_taper_dist = float(
                getattr(mpc_cfg, "progress_taper_distance", 0.0) or 0.0
            )
            sim._ref_coast_pos_tol = float(
                getattr(mpc_cfg, "coast_pos_tolerance", 0.0) or 0.0
            )
            sim._ref_coast_vel_tol = float(
                getattr(mpc_cfg, "coast_vel_tolerance", 0.0) or 0.0
            )
            sim._ref_coast_min_speed = float(
                getattr(mpc_cfg, "coast_min_speed", 0.0) or 0.0
            )
        else:
            sim._ref_path_speed = 0.0
            sim._ref_taper_dist = 0.0
            sim._ref_coast_pos_tol = 0.0
            sim._ref_coast_vel_tol = 0.0
            sim._ref_coast_min_speed = 0.0

    path_speed = sim._ref_path_speed
    taper_dist = sim._ref_taper_dist
    coast_pos_tol = sim._ref_coast_pos_tol
    coast_vel_tol = sim._ref_coast_vel_tol
    coast_min_speed = sim._ref_coast_min_speed

    # --- Taper speed near path end ---
    speed_scale = 1.0
    remaining = None
    try:
        path_len = sim._get_mission_path_length(compute_if_missing=True)
        s_val = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)
        if path_len > 0.0:
            remaining = max(0.0, path_len - s_val)
            if taper_dist <= 0.0:
                pos_tol = float(getattr(sim, "position_tolerance", 0.05))
                taper_dist = max(pos_tol, path_speed * sim.control_update_interval)
            if taper_dist > 1e-6:
                speed_scale = max(0.0, min(1.0, remaining / taper_dist))
    except Exception:
        logger.debug(
            "Speed tapering calculation failed, using scale=1.0", exc_info=True
        )
        speed_scale = 1.0

    pos_tol = float(getattr(sim, "position_tolerance", 0.05))
    at_path_end = remaining is not None and remaining <= max(pos_tol, 1e-6)

    # Coasting bias: match reference speed to current along-track speed when on-path.
    v_ref = path_speed * speed_scale
    if (not at_path_end) and coast_pos_tol > 0.0 and tangent_norm > 1e-6:
        pos_err = _norm3(current_state[:3] - pos_ref)
        v_curr = current_state[7:10]
        v_along = float(np.dot(v_curr, tangent))
        v_perp = v_curr - v_along * tangent
        if (
            pos_err <= coast_pos_tol
            and _norm3(v_perp) <= coast_vel_tol
            and v_along >= 0.0
        ):
            v_ref = max(coast_min_speed, v_along)
    if at_path_end:
        # Force a full stop at the end of the path.
        v_ref = 0.0

    reference_state[7:10] = tangent * v_ref

    # At the end of the path, don't enforce a specific attitude; use current.
    if at_path_end:
        reference_state[3:7] = current_state[3:7]

    sim.reference_state = reference_state.copy()

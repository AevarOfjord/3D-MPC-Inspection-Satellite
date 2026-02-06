"""
Path-following reference state computation.

Extracted from SatelliteMPCLinearizedSimulation.update_path_reference_state
to reduce simulation.py size while keeping the public API unchanged.
"""

from typing import TYPE_CHECKING

import numpy as np

from src.satellite_control.utils.orientation_utils import quat_wxyz_from_basis

if TYPE_CHECKING:
    from src.satellite_control.core.simulation import (
        SatelliteMPCLinearizedSimulation,
    )


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
    reference_state = np.zeros(13, dtype=float)
    reference_state[0:3] = pos_ref

    if np.linalg.norm(tangent) > 1e-6:
        x_axis = tangent / np.linalg.norm(tangent)
        up = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(x_axis, up))) > 0.95:
            up = np.array([0.0, 1.0, 0.0], dtype=float)
        z_axis = up - np.dot(up, x_axis) * x_axis
        if np.linalg.norm(z_axis) > 1e-6:
            z_axis = z_axis / np.linalg.norm(z_axis)
        else:
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
        y_axis = np.cross(z_axis, x_axis)
        if np.linalg.norm(y_axis) > 1e-6:
            y_axis = y_axis / np.linalg.norm(y_axis)
        else:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
        z_axis = np.cross(x_axis, y_axis)
        if np.linalg.norm(z_axis) > 1e-6:
            z_axis = z_axis / np.linalg.norm(z_axis)

        reference_state[3:7] = quat_wxyz_from_basis(x_axis, y_axis, z_axis)
    else:
        # Maintain current orientation if stationary
        reference_state[3:7] = current_state[3:7]

    # --- Read MPC config ---
    path_speed = 0.0
    taper_dist = 0.0
    coast_pos_tol = 0.0
    coast_vel_tol = 0.0
    coast_min_speed = 0.0
    if sim.simulation_config is not None:
        path_speed = float(sim.simulation_config.app_config.mpc.path_speed)
        taper_dist = float(
            getattr(sim.simulation_config.app_config.mpc, "progress_taper_distance", 0.0)
            or 0.0
        )
        coast_pos_tol = float(
            getattr(sim.simulation_config.app_config.mpc, "coast_pos_tolerance", 0.0)
            or 0.0
        )
        coast_vel_tol = float(
            getattr(sim.simulation_config.app_config.mpc, "coast_vel_tolerance", 0.0)
            or 0.0
        )
        coast_min_speed = float(
            getattr(sim.simulation_config.app_config.mpc, "coast_min_speed", 0.0)
            or 0.0
        )

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
        speed_scale = 1.0

    pos_tol = float(getattr(sim, "position_tolerance", 0.05))
    at_path_end = remaining is not None and remaining <= max(pos_tol, 1e-6)

    # Coasting bias: match reference speed to current along-track speed when on-path.
    v_ref = path_speed * speed_scale
    if (not at_path_end) and coast_pos_tol > 0.0 and np.linalg.norm(tangent) > 1e-6:
        pos_err = float(np.linalg.norm(current_state[:3] - pos_ref))
        v_curr = current_state[7:10]
        v_along = float(np.dot(v_curr, tangent))
        v_perp = v_curr - v_along * tangent
        if (
            pos_err <= coast_pos_tol
            and np.linalg.norm(v_perp) <= coast_vel_tol
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

    sim.reference_state = reference_state

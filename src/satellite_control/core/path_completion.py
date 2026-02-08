"""Path completion checks for simulation runtime."""

import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


def check_path_complete(sim: Any) -> bool:
    """Return True when path progress and terminal state conditions are met."""
    if not hasattr(sim, "mpc_controller") or not sim.mpc_controller:
        return False

    path_len = sim._get_mission_path_length(compute_if_missing=True)
    if path_len <= 0.0:
        return False

    pos = None
    if hasattr(sim.satellite, "position"):
        pos = np.array(sim.satellite.position, dtype=float)
    else:
        try:
            pos = sim.get_current_state()[:3]
        except Exception:
            logger.debug("Failed to get position from state", exc_info=True)
            pos = None

    # Default to MPC controller's internal state 's' if available.
    # This is critical for closed-loop paths where geometric projection is ambiguous.
    path_s = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)

    endpoint_error = float("inf")

    # We still want endpoint_error from projection if available, but we trust
    # the internal 's' state for progress if it exists (MPCC mode).
    if hasattr(sim.mpc_controller, "get_path_progress") and pos is not None:
        metrics = sim.mpc_controller.get_path_progress(pos)
        if isinstance(metrics, dict):
            # Only update path_s from metrics if we don't trust the internal state
            # or if internal state is 0 and we want to rely on geometry (optional).
            # For MPCC, internal state is authoritative.
            # If path_s is 0 (start), geometric projection might return L (end) for loops.
            # So we prefer the internal state.

            # Update endpoint error
            endpoint_error = float(metrics.get("endpoint_error", endpoint_error))

            # Fallback: if sim.mpc_controller doesn't have attribute 's', use metrics
            if not hasattr(sim.mpc_controller, "s"):
                path_s = float(metrics.get("s", path_s))

    elif pos is not None:
        try:
            path = sim._get_mission_path_waypoints()
            end_pt = np.array(path[-1], dtype=float)
            endpoint_error = float(np.linalg.norm(pos - end_pt))
        except Exception:
            logger.debug("Failed to compute endpoint error", exc_info=True)
            endpoint_error = float("inf")

    pos_tol = float(getattr(sim, "position_tolerance", 0.05))
    progress_ok = path_s >= (path_len - pos_tol)
    pos_ok = endpoint_error <= pos_tol

    state_ok = None
    if hasattr(sim, "state_validator") and sim.state_validator is not None:
        try:
            current_state = sim.get_current_state()[:13]
            reference_state = (
                sim.reference_state if sim.reference_state is not None else np.zeros(13)
            )
            state_ok = sim.state_validator.check_reference_reached(
                current_state, reference_state
            )
        except Exception:
            logger.debug("State validator check failed", exc_info=True)
            state_ok = None

    if state_ok is None:
        state_ok = pos_ok
    else:
        state_ok = bool(state_ok or pos_ok)

    return bool(progress_ok and state_ok)

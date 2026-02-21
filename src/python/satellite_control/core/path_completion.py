"""Path completion checks for simulation runtime."""

import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


def get_path_completion_status(sim: Any) -> dict[str, Any]:
    """Compute strict path-end completion status and per-threshold booleans."""
    if not hasattr(sim, "mpc_controller") or not sim.mpc_controller:
        return {
            "progress_ok": False,
            "position_ok": False,
            "angle_ok": False,
            "velocity_ok": False,
            "angular_velocity_ok": False,
            "state_ok": False,
            "validator_available": False,
            "path_s": 0.0,
            "path_length": 0.0,
            "path_error": float("inf"),
            "endpoint_error": float("inf"),
        }

    path_len = sim._get_mission_path_length(compute_if_missing=True)
    if path_len <= 0.0:
        return {
            "progress_ok": False,
            "position_ok": False,
            "angle_ok": False,
            "velocity_ok": False,
            "angular_velocity_ok": False,
            "state_ok": False,
            "validator_available": False,
            "path_s": 0.0,
            "path_length": float(path_len),
            "path_error": float("inf"),
            "endpoint_error": float("inf"),
        }

    pos = None
    if hasattr(sim.satellite, "position"):
        pos = np.array(sim.satellite.position, dtype=float)
    else:
        try:
            pos = sim.get_current_state()[:3]
        except Exception:
            logger.debug("Failed to get position from state", exc_info=True)
            pos = None

    path_s = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)
    path_error = float("inf")
    endpoint_error = float("inf")
    if hasattr(sim.mpc_controller, "get_path_progress") and pos is not None:
        metrics = sim.mpc_controller.get_path_progress(pos)
        if isinstance(metrics, dict):
            path_s = float(metrics.get("s", path_s))
            path_error = float(metrics.get("path_error", path_error))
            endpoint_error = float(metrics.get("endpoint_error", endpoint_error))
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
    pos_ok = bool(endpoint_error <= pos_tol)
    validator_available = False
    threshold_status = {
        "position": pos_ok,
        "angle": False,
        "velocity": False,
        "angular_velocity": False,
    }
    state_ok: bool | None = None
    if hasattr(sim, "state_validator") and sim.state_validator is not None:
        try:
            validator_available = True
            current_state = sim.get_current_state()[:13]
            reference_state = (
                sim.reference_state if sim.reference_state is not None else np.zeros(13)
            )
            if hasattr(sim.state_validator, "check_within_tolerances"):
                checks = sim.state_validator.check_within_tolerances(
                    current_state, reference_state
                )
                if isinstance(checks, dict):
                    threshold_status["position"] = bool(
                        checks.get("position", threshold_status["position"])
                    )
                    threshold_status["angle"] = bool(checks.get("angle", False))
                    threshold_status["velocity"] = bool(checks.get("velocity", False))
                    threshold_status["angular_velocity"] = bool(
                        checks.get("angular_velocity", False)
                    )
                    state_ok = bool(
                        threshold_status["position"]
                        and threshold_status["angle"]
                        and threshold_status["velocity"]
                        and threshold_status["angular_velocity"]
                    )
            if state_ok is None:
                raw_state_ok = sim.state_validator.check_reference_reached(
                    current_state, reference_state
                )
                state_ok = bool(raw_state_ok)
                threshold_status["position"] = bool(
                    threshold_status["position"] and state_ok
                )
                threshold_status["angle"] = bool(state_ok)
                threshold_status["velocity"] = bool(state_ok)
                threshold_status["angular_velocity"] = bool(state_ok)
        except Exception:
            logger.debug("State validator check failed", exc_info=True)
            state_ok = None
            validator_available = False

    if state_ok is None:
        state_ok = pos_ok
    if not validator_available:
        threshold_status["position"] = bool(pos_ok)
        threshold_status["angle"] = bool(pos_ok)
        threshold_status["velocity"] = bool(pos_ok)
        threshold_status["angular_velocity"] = bool(pos_ok)

    return {
        "progress_ok": bool(progress_ok),
        "position_ok": bool(threshold_status["position"]),
        "angle_ok": bool(threshold_status["angle"]),
        "velocity_ok": bool(threshold_status["velocity"]),
        "angular_velocity_ok": bool(threshold_status["angular_velocity"]),
        "state_ok": bool(state_ok),
        "validator_available": bool(validator_available),
        "path_s": float(path_s),
        "path_length": float(path_len),
        "path_error": float(path_error),
        "endpoint_error": float(endpoint_error),
    }


def check_path_complete(sim: Any) -> bool:
    """Return True when strict terminal contract is satisfied at path end."""
    status = get_path_completion_status(sim)
    progress_ok = bool(status.get("progress_ok", False))
    state_ok = bool(status.get("state_ok", False))
    validator_available = bool(status.get("validator_available", False))
    position_ok = bool(status.get("position_ok", False))

    if validator_available:
        return bool(progress_ok and state_ok)
    return bool(progress_ok and position_ok)

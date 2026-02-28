"""Path completion checks for simulation runtime."""

import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


def get_path_completion_status(sim: Any) -> dict[str, Any]:
    """Compute strict path-end completion status and per-threshold booleans."""
    if getattr(sim, "mpc_controller", None) is None:
        return {
            "progress_ok": False,
            "position_ok": False,
            "angle_ok": False,
            "velocity_ok": False,
            "angular_velocity_ok": False,
            "position_hold_ok": False,
            "angle_hold_ok": False,
            "velocity_hold_ok": False,
            "angular_velocity_hold_ok": False,
            "terminal_gate_fail_reason": "progress",
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
            "position_hold_ok": False,
            "angle_hold_ok": False,
            "velocity_hold_ok": False,
            "angular_velocity_hold_ok": False,
            "terminal_gate_fail_reason": "progress",
            "state_ok": False,
            "validator_available": False,
            "path_s": 0.0,
            "path_length": float(path_len),
            "path_error": float("inf"),
            "endpoint_error": float("inf"),
        }

    pos = None
    if getattr(sim.satellite, "position", None) is not None:
        pos = np.array(sim.satellite.position, dtype=float)
    else:
        try:
            pos = sim.get_current_state()[:3]
        except Exception:
            logger.debug("Failed to get position from state", exc_info=True)
            pos = None

    path_s_controller = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)
    path_s = float(path_s_controller)
    path_error = float("inf")
    endpoint_error = float("inf")
    if pos is not None:
        metrics = sim.mpc_controller.get_path_progress(pos)
        if isinstance(metrics, dict):
            path_s = float(metrics.get("s", path_s))
            path_error = float(metrics.get("path_error", path_error))
            endpoint_error = float(metrics.get("endpoint_error", endpoint_error))

    pos_tol = float(getattr(sim, "position_tolerance", 0.05))
    path_s_progress = max(path_s, path_s_controller)
    progress_ok = path_s_progress >= (path_len - pos_tol)
    pos_ok = bool(endpoint_error <= pos_tol)
    validator_available = False
    threshold_status = {
        "position": pos_ok,
        "angle": False,
        "velocity": False,
        "angular_velocity": False,
    }
    hold_threshold_status = dict(threshold_status)
    state_ok: bool | None = None
    state_validator = getattr(sim, "state_validator", None)
    if state_validator is not None:
        try:
            validator_available = True
            current_state = sim.get_current_state()[:13]
            reference_state = (
                sim.reference_state if sim.reference_state is not None else np.zeros(13)
            )
            check_within_tolerances = getattr(
                state_validator, "check_within_tolerances", None
            )
            if callable(check_within_tolerances):
                checks = check_within_tolerances(current_state, reference_state)
                hold_checks = checks
                try:
                    hold_checks = check_within_tolerances(
                        current_state,
                        reference_state,
                        hysteresis_mode="hold",
                    )
                except TypeError:
                    hold_checks = checks
                if isinstance(checks, dict):
                    threshold_status["position"] = bool(
                        checks.get("position", threshold_status["position"])
                    )
                    threshold_status["angle"] = bool(checks.get("angle", False))
                    threshold_status["velocity"] = bool(checks.get("velocity", False))
                    threshold_status["angular_velocity"] = bool(
                        checks.get("angular_velocity", False)
                    )
                    if isinstance(hold_checks, dict):
                        hold_threshold_status["position"] = bool(
                            hold_checks.get("position", threshold_status["position"])
                        )
                        hold_threshold_status["angle"] = bool(
                            hold_checks.get("angle", threshold_status["angle"])
                        )
                        hold_threshold_status["velocity"] = bool(
                            hold_checks.get("velocity", threshold_status["velocity"])
                        )
                        hold_threshold_status["angular_velocity"] = bool(
                            hold_checks.get(
                                "angular_velocity",
                                threshold_status["angular_velocity"],
                            )
                        )
                    else:
                        hold_threshold_status = dict(threshold_status)
                    state_ok = bool(
                        threshold_status["position"]
                        and threshold_status["angle"]
                        and threshold_status["velocity"]
                        and threshold_status["angular_velocity"]
                    )
            if state_ok is None:
                raw_state_ok = state_validator.check_reference_reached(
                    current_state, reference_state
                )
                state_ok = bool(raw_state_ok)
                threshold_status["position"] = bool(
                    threshold_status["position"] and state_ok
                )
                threshold_status["angle"] = bool(state_ok)
                threshold_status["velocity"] = bool(state_ok)
                threshold_status["angular_velocity"] = bool(state_ok)
                hold_threshold_status = dict(threshold_status)
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
        hold_threshold_status = dict(threshold_status)

    if not progress_ok:
        fail_reason = "progress"
    elif not threshold_status["position"]:
        fail_reason = "position"
    elif not threshold_status["angle"]:
        fail_reason = "angle"
    elif not threshold_status["velocity"]:
        fail_reason = "velocity"
    elif not threshold_status["angular_velocity"]:
        fail_reason = "angular_velocity"
    else:
        fail_reason = "none"

    return {
        "progress_ok": bool(progress_ok),
        "position_ok": bool(threshold_status["position"]),
        "angle_ok": bool(threshold_status["angle"]),
        "velocity_ok": bool(threshold_status["velocity"]),
        "angular_velocity_ok": bool(threshold_status["angular_velocity"]),
        "position_hold_ok": bool(hold_threshold_status["position"]),
        "angle_hold_ok": bool(hold_threshold_status["angle"]),
        "velocity_hold_ok": bool(hold_threshold_status["velocity"]),
        "angular_velocity_hold_ok": bool(hold_threshold_status["angular_velocity"]),
        "terminal_gate_fail_reason": str(fail_reason),
        "state_ok": bool(state_ok),
        "validator_available": bool(validator_available),
        "path_s": float(path_s),
        "path_s_controller": float(path_s_controller),
        "path_s_progress": float(path_s_progress),
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

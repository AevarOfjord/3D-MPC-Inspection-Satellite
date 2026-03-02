"""Control-loop helpers extracted from the main simulation class."""

from __future__ import annotations

import logging
import time
from typing import Any

logger = logging.getLogger(__name__)

import numpy as np
from runtime.policy import (
    compute_pointing_errors_deg,
    resolve_object_visible_side,
    resolve_pointing_context,
)


def _resolve_waypoint_hold(
    sim: Any,
    *,
    path_s: float,
) -> tuple[bool, float]:
    """Evaluate per-waypoint hold schedule and optionally activate a hold."""
    mission_state = getattr(
        getattr(sim, "simulation_config", None), "mission_state", None
    )
    if mission_state is None:
        return False, float(path_s)

    schedule = list(getattr(mission_state, "path_hold_schedule", []) or [])
    waypoints = list(getattr(mission_state, "path_waypoints", []) or [])
    if not schedule or len(waypoints) < 2:
        mission_state.path_hold_active_index = None
        mission_state.path_hold_started_at_s = None
        return False, float(path_s)

    arc = getattr(sim, "_studio_path_arc_lengths", None)
    if arc is None or len(arc) != len(waypoints):
        arc = [0.0]
        total = 0.0
        for i in range(1, len(waypoints)):
            p0 = np.array(waypoints[i - 1], dtype=float)
            p1 = np.array(waypoints[i], dtype=float)
            total += float(np.linalg.norm(p1 - p0))
            arc.append(total)
        sim._studio_path_arc_lengths = arc

    active_idx = getattr(mission_state, "path_hold_active_index", None)
    if active_idx is not None:
        hold_s = float(arc[min(max(0, int(active_idx)), len(arc) - 1)])
        hold_started = getattr(mission_state, "path_hold_started_at_s", None)
        hold_cfg = next(
            (
                item
                for item in schedule
                if int(item.get("path_index", -1)) == int(active_idx)
            ),
            None,
        )
        duration = float(hold_cfg.get("duration_s", 0.0)) if hold_cfg else 0.0
        elapsed = (
            0.0
            if hold_started is None
            else max(0.0, float(sim.simulation_time) - float(hold_started))
        )
        if elapsed >= duration:
            completed_list = list(
                getattr(mission_state, "path_hold_completed", []) or []
            )
            idx_completed = int(active_idx)
            if idx_completed not in completed_list:
                completed_list.append(idx_completed)
            mission_state.path_hold_completed = completed_list
            mission_state.path_hold_active_index = None
            mission_state.path_hold_started_at_s = None
            return False, float(path_s)
        return True, hold_s

    completed = set(getattr(mission_state, "path_hold_completed", []) or [])
    trigger_tol_m = 0.05
    for item in schedule:
        idx = int(item.get("path_index", 0))
        if idx in completed:
            continue
        if idx < 0 or idx >= len(arc):
            continue
        hold_s = float(arc[idx])
        if float(path_s) + trigger_tol_m >= hold_s:
            mission_state.path_hold_active_index = idx
            mission_state.path_hold_started_at_s = float(sim.simulation_time)
            return True, hold_s
        break

    return False, float(path_s)


def _set_runtime_pointing_context(
    sim: Any,
    *,
    current_state: np.ndarray,
    path_s: float,
    force_transit_free: bool = False,
) -> tuple[Any, float | None, float | None, str | None, str]:
    """Resolve and forward current pointing context to MPC core."""
    context = resolve_pointing_context(
        sim=sim,
        current_state=current_state,
        path_s=float(path_s),
        force_transit_free=bool(force_transit_free),
    )
    pointing_policy = str(getattr(context, "policy", "scan_locked")).strip().lower()
    enforced = bool(getattr(context, "enforced", True))
    if str(getattr(context, "source", "")).strip().lower() == "non_scan_minimal_twist":
        enforced = False
        pointing_policy = "transit_free"
    if not enforced:
        sim.mpc_controller.set_scan_attitude_context(None, None, "CW")
    else:
        center_payload = (
            None
            if context.center_world is None
            else (
                float(context.center_world[0]),
                float(context.center_world[1]),
                float(context.center_world[2]),
            )
        )
        axis_payload = (
            float(context.axis_world[0]),
            float(context.axis_world[1]),
            float(context.axis_world[2]),
        )
        direction = "CW" if context.direction_cw else "CCW"
        sim.mpc_controller.set_scan_attitude_context(
            center_payload,
            axis_payload,
            direction,
        )

    x_error_deg = None
    z_error_deg = None
    if enforced:
        try:
            q_curr = (
                np.array(current_state[3:7], dtype=float)
                if current_state.size >= 7
                else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            )
            _, _, q_ref = sim.mpc_controller.get_path_reference_state(
                s_query=float(path_s),
                q_current=q_curr,
            )
            if q_ref is not None:
                x_error_deg, z_error_deg = compute_pointing_errors_deg(
                    current_quat_wxyz=q_curr,
                    reference_quat_wxyz=np.array(q_ref, dtype=float),
                )
        except Exception:
            logger.warning("Pointing context resolution failed", exc_info=True)
            x_error_deg = None
            z_error_deg = None

    visible_side = resolve_object_visible_side(
        current_state=current_state, context=context
    )
    return context, x_error_deg, z_error_deg, visible_side, pointing_policy


def _update_mode_state(sim: Any, current_state: np.ndarray) -> None:
    """Refresh controller mode state from latest path/gate metrics."""
    mode_manager = getattr(sim, "mode_manager", None)
    if mode_manager is None:
        return

    path_s_controller = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)
    path_s_projected = float(path_s_controller)
    contour_error = 0.0
    endpoint_error = None

    try:
        metrics = sim.mpc_controller.get_path_progress(current_state[:3])
        if isinstance(metrics, dict):
            path_s_projected = float(metrics.get("s", path_s_projected))
            contour_error = float(metrics.get("path_error", contour_error) or 0.0)
            endpoint_error = metrics.get("endpoint_error")
    except Exception:
        contour_error = 0.0

    contracts_cfg = getattr(
        getattr(getattr(sim, "simulation_config", None), "app_config", None),
        "controller_contracts",
        None,
    )
    lead_cap_m = max(
        0.0,
        float(getattr(contracts_cfg, "path_projection_lead_cap_m", 0.25) or 0.25),
    )
    path_s_projected_raw = float(path_s_projected)
    path_s_projected_capped = min(
        path_s_projected_raw, float(path_s_controller) + float(lead_cap_m)
    )

    path_len = float(sim._get_mission_path_length(compute_if_missing=True) or 0.0)
    path_s_progress = max(path_s_projected_capped, path_s_controller)
    if path_len > 0.0:
        path_s_progress = float(np.clip(path_s_progress, 0.0, path_len))
    else:
        path_s_progress = max(0.0, path_s_progress)
    sim.path_progress_debug = {
        "path_s_fused": float(path_s_progress),
        "path_s_projected_raw": float(path_s_projected_raw),
        "path_s_projected_capped": float(path_s_projected_capped),
        "path_s_controller": float(path_s_controller),
        "projection_lead_cap_m": float(lead_cap_m),
    }
    pos_tol = float(getattr(sim, "position_tolerance", 0.1) or 0.1)
    at_path_end = bool(path_len > 0.0 and path_s_progress >= (path_len - pos_tol))
    mode_for_override = str(
        getattr(
            getattr(sim, "mode_state", None),
            "current_mode",
            getattr(getattr(mode_manager, "state", None), "current_mode", "TRACK"),
        )
    ).upper()
    force_transit_free = bool(at_path_end and mode_for_override in {"SETTLE", "HOLD"})
    gate = getattr(sim, "completion_gate", None)
    completion_gate_state_ok = bool(getattr(gate, "all_thresholds_ok", False))
    completion_reached = bool(getattr(sim, "completion_reached", False))
    solver_health = getattr(sim, "solver_health", None)
    # Mode transitions should react to active/recent fallback, not to a
    # historical hard-limit latch that can persist for the whole run.
    solver_fallback_active = bool(
        solver_health is not None and getattr(solver_health, "fallback_active", False)
    )
    solver_fallback_age_s = float(
        getattr(solver_health, "fallback_age_s", 0.0)
        if solver_health is not None
        else 0.0
    )
    solver_fallback_count = int(
        getattr(solver_health, "fallback_count", 0) if solver_health is not None else 0
    )
    solver_recent_fallback = bool(
        solver_fallback_count > 0
        and solver_fallback_age_s > 0.0
        and solver_fallback_age_s <= 1.0
    )
    solver_degraded = bool(solver_fallback_active or solver_recent_fallback)
    solver_fallback_reason = (
        str(getattr(solver_health, "last_fallback_reason", "") or "")
        if solver_health is not None
        else None
    )
    if not solver_fallback_reason:
        solver_fallback_reason = None

    pointing_scope = (
        str(getattr(contracts_cfg, "pointing_scope", "all_missions")).strip().lower()
    )
    pointing_enabled = bool(getattr(contracts_cfg, "enable_pointing_contract", True))
    apply_pointing = pointing_enabled and pointing_scope in {
        "all_missions",
        "config_toggle",
    }
    if pointing_scope == "scan_only":
        mission_state = getattr(
            getattr(sim, "simulation_config", None), "mission_state", None
        )
        spans = list(getattr(mission_state, "pointing_path_spans", []) or [])
        initial_axis = np.array(
            getattr(mission_state, "scan_attitude_axis", (0.0, 0.0, 0.0)),
            dtype=float,
        )
        has_initial_axis = bool(
            initial_axis.size >= 3
            and np.all(np.isfinite(initial_axis[:3]))
            and float(np.linalg.norm(initial_axis[:3])) > 1e-9
        )
        apply_pointing = pointing_enabled and (len(spans) > 0 or has_initial_axis)

    context_source = None
    context = None
    x_error_deg = None
    z_error_deg = None
    visible_side = None
    pointing_policy = "transit_free"
    if apply_pointing:
        (
            context,
            x_error_deg,
            z_error_deg,
            visible_side,
            pointing_policy,
        ) = _set_runtime_pointing_context(
            sim,
            current_state=current_state,
            path_s=path_s_progress,
            force_transit_free=force_transit_free,
        )
        context_source = getattr(context, "source", None)
    else:
        sim.mpc_controller.set_scan_attitude_context(None, None, "CW")
        context_source = "pointing_disabled"

    pointing_guardrail = getattr(sim, "pointing_guardrail", None)
    pointing_guardrail_status = None
    guardrail_enforced = bool(apply_pointing and pointing_policy != "transit_free")
    if pointing_guardrail is not None and not guardrail_enforced:
        pointing_guardrail.reset()
    if pointing_guardrail is not None and guardrail_enforced:
        pointing_guardrail_status = pointing_guardrail.update(
            sim_time_s=float(sim.simulation_time),
            x_error_deg=x_error_deg,
            z_error_deg=z_error_deg,
        )

    pointing_guardrail_breached = bool(
        getattr(pointing_guardrail_status, "breached", False)
    )
    pointing_reason = getattr(pointing_guardrail_status, "last_reason", None)
    if pointing_guardrail_breached and not solver_fallback_reason:
        solver_fallback_reason = (
            str(pointing_reason)
            if isinstance(pointing_reason, str) and pointing_reason
            else "pointing_guardrail"
        )
    solver_degraded_for_mode = solver_degraded or pointing_guardrail_breached

    sim.pointing_status = {
        "pointing_context_source": context_source,
        "pointing_policy": str(pointing_policy),
        "pointing_axis_world": (
            list(getattr(context, "axis_world", [0.0, 0.0, 1.0]))
            if apply_pointing
            else [0.0, 0.0, 1.0]
        ),
        "z_axis_error_deg": float(z_error_deg) if z_error_deg is not None else 0.0,
        "x_axis_error_deg": float(x_error_deg) if x_error_deg is not None else 0.0,
        "pointing_guardrail_breached": pointing_guardrail_breached,
        "object_visible_side": visible_side,
        "pointing_guardrail_reason": pointing_reason,
    }

    sim.mode_state = mode_manager.update(
        sim_time_s=float(sim.simulation_time),
        contour_error_m=contour_error,
        path_s=path_s_progress,
        path_len=path_len,
        position_tolerance_m=pos_tol,
        completion_gate_state_ok=completion_gate_state_ok,
        completion_reached=completion_reached,
        solver_degraded=solver_degraded_for_mode,
        solver_fallback_reason=solver_fallback_reason,
    )
    hold_active, hold_s = _resolve_waypoint_hold(sim, path_s=path_s_progress)
    if hold_active:
        try:
            hold_s_clamped = float(
                min(max(0.0, hold_s), path_len if path_len > 0.0 else hold_s)
            )
            set_current_path_s = getattr(sim.mpc_controller, "set_current_path_s", None)
            if callable(set_current_path_s):
                set_current_path_s(hold_s_clamped)
            else:
                sim.mpc_controller.s = hold_s_clamped
        except Exception:
            pass
        sim.mode_state.current_mode = "HOLD"
    sim.mode_profile = mode_manager.profile_for_mode(sim.mode_state.current_mode)
    mission_state = getattr(
        getattr(sim, "simulation_config", None), "mission_state", None
    )
    sim.studio_hold_status = {
        "active": bool(hold_active),
        "hold_s": float(hold_s),
        "hold_index": getattr(mission_state, "path_hold_active_index", None),
        "hold_started_at_s": getattr(mission_state, "path_hold_started_at_s", None),
    }

    mode_timeline = getattr(sim, "mode_timeline", None)
    if mode_timeline is not None:
        sim._append_capped_history(
            mode_timeline,
            {
                "time_s": float(sim.simulation_time),
                "mode": str(sim.mode_state.current_mode),
                "time_in_mode_s": float(sim.mode_state.time_in_mode_s),
                "path_s": float(path_s_progress),
                "path_s_proj": float(path_s_projected),
                "path_s_proj_raw": float(path_s_projected_raw),
                "path_s_proj_capped": float(path_s_projected_capped),
                "path_projection_lead_cap_m": float(lead_cap_m),
                "path_s_controller": float(path_s_controller),
                "path_error_m": float(contour_error),
                "endpoint_error_m": (
                    float(endpoint_error) if endpoint_error is not None else None
                ),
                "x_axis_error_deg": (
                    float(x_error_deg) if x_error_deg is not None else None
                ),
                "z_axis_error_deg": (
                    float(z_error_deg) if z_error_deg is not None else None
                ),
                "pointing_guardrail_breached": bool(pointing_guardrail_breached),
                "pointing_policy": str(pointing_policy),
                "pointing_context_source": context_source,
            },
        )


def update_mpc_control_step(sim: Any) -> None:
    """Run one MPC control update when control timing permits."""
    # Force MPC to send commands at fixed intervals
    if sim.simulation_time < sim.next_control_simulation_time:
        return

    current_state = sim.get_current_state()
    _update_mode_state(sim, current_state)
    reference_scheduler = getattr(sim, "reference_scheduler", None)
    if reference_scheduler is not None:
        try:
            horizon = int(getattr(sim.mpc_controller, "prediction_horizon", 10))
            sim.reference_slice = reference_scheduler.build_slice(
                sim=sim,
                current_state=current_state,
                mode=str(
                    getattr(getattr(sim, "mode_state", None), "current_mode", "TRACK")
                ),
                horizon=max(1, horizon),
                dt=float(sim.control_update_interval),
            )
        except Exception:
            sim.reference_slice = None

    # Delegate to MPCRunner
    if getattr(sim, "mpc_runner", None) is None:
        from runtime.mpc_runner import MPCRunner

        # Initialize MPC Runner wrapper
        sim.mpc_runner = MPCRunner(
            mpc_controller=sim.mpc_controller,
            config=sim.structured_config,
            state_validator=sim.state_validator,
            actuator_policy=getattr(sim, "actuator_policy", None),
        )
    sim.mpc_runner.set_mode_state(getattr(sim, "mode_state", None))

    mpc_start_sim_time = sim.simulation_time
    mpc_start_wall_time = time.perf_counter()

    # Compute action
    (
        thruster_action,
        rw_torque_norm,
        mpc_info,
        mpc_computation_time,
        command_sent_wall_time,
    ) = sim.mpc_runner.compute_control_action(
        true_state=current_state,
        previous_thrusters=sim.previous_thrusters,
    )

    mode_state = getattr(sim, "mode_state", None)
    if mpc_info is None:
        mpc_info = {}
    if mode_state is not None:
        mpc_info["mode_state"] = str(getattr(mode_state, "current_mode", "TRACK"))
        mpc_info["mode_time_in_mode_s"] = float(
            getattr(mode_state, "time_in_mode_s", 0.0)
        )
    mpc_info["controller_core"] = str(getattr(sim, "controller_core_mode", "sqp"))
    gate = getattr(sim, "completion_gate", None)
    if gate is not None:
        mpc_info["completion_gate_position_ok"] = bool(
            getattr(gate, "position_ok", False)
        )
        mpc_info["completion_gate_angle_ok"] = bool(getattr(gate, "angle_ok", False))
        mpc_info["completion_gate_velocity_ok"] = bool(
            getattr(gate, "velocity_ok", False)
        )
        mpc_info["completion_gate_angular_velocity_ok"] = bool(
            getattr(gate, "angular_velocity_ok", False)
        )
        mpc_info["completion_gate_hold_elapsed_s"] = float(
            getattr(gate, "hold_elapsed_s", 0.0)
        )
        mpc_info["completion_gate_hold_required_s"] = float(
            getattr(gate, "hold_required_s", 0.0)
        )
        mpc_info["completion_gate_last_breach_reason"] = getattr(
            gate, "last_breach_reason", None
        )
        mpc_info["terminal_gate_fail_reason"] = getattr(gate, "fail_reason", "none")
        mpc_info["hold_timer_s"] = float(getattr(gate, "hold_elapsed_s", 0.0))
        mpc_info["hold_reset_count"] = int(getattr(gate, "hold_reset_count", 0))
    pointing_status = getattr(sim, "pointing_status", None)
    if isinstance(pointing_status, dict):
        mpc_info["pointing_policy_active"] = pointing_status.get("pointing_policy")
        mpc_info["pointing_context_source_active"] = pointing_status.get(
            "pointing_context_source"
        )
        mpc_info["pointing_context_source"] = pointing_status.get(
            "pointing_context_source"
        )
        mpc_info["pointing_axis_world"] = pointing_status.get("pointing_axis_world")
        mpc_info["z_axis_error_deg"] = float(
            pointing_status.get("z_axis_error_deg", 0.0) or 0.0
        )
        mpc_info["x_axis_error_deg"] = float(
            pointing_status.get("x_axis_error_deg", 0.0) or 0.0
        )
        mpc_info["pointing_guardrail_breached"] = bool(
            pointing_status.get("pointing_guardrail_breached", False)
        )
        mpc_info["pointing_guardrail_reason"] = pointing_status.get(
            "pointing_guardrail_reason"
        )
        mpc_info["object_visible_side"] = pointing_status.get("object_visible_side")
    hold_status = getattr(sim, "studio_hold_status", None)
    if isinstance(hold_status, dict):
        mpc_info["studio_hold_active"] = bool(hold_status.get("active", False))
        mpc_info["studio_hold_index"] = hold_status.get("hold_index")
    path_progress_debug = getattr(sim, "path_progress_debug", None)
    if isinstance(path_progress_debug, dict):
        mpc_info["path_s_fused"] = float(path_progress_debug.get("path_s_fused", 0.0))
        mpc_info["path_s_projected_raw"] = float(
            path_progress_debug.get("path_s_projected_raw", 0.0)
        )
        mpc_info["path_s_projected_capped"] = float(
            path_progress_debug.get("path_s_projected_capped", 0.0)
        )
        mpc_info["path_s_controller"] = float(
            path_progress_debug.get("path_s_controller", 0.0)
        )
        mpc_info["projection_lead_cap_m"] = float(
            path_progress_debug.get("projection_lead_cap_m", 0.0)
        )

    solver_health = getattr(sim, "solver_health", None)
    if solver_health is not None:
        fallback_reason = mpc_info.get("solver_fallback_reason")
        solver_health.fallback_active = bool(mpc_info.get("fallback_active", False))
        solver_health.fallback_age_s = float(mpc_info.get("fallback_age_s", 0.0) or 0.0)
        solver_health.fallback_scale = float(mpc_info.get("fallback_scale", 0.0) or 0.0)
        if bool(mpc_info.get("solver_fallback", False)):
            solver_health.fallback_count += 1
            reason_key = str(fallback_reason or "solver_fallback")
            solver_health.last_fallback_reason = reason_key
            solver_health.fallback_reasons[reason_key] = (
                int(solver_health.fallback_reasons.get(reason_key, 0)) + 1
            )
        if bool(mpc_info.get("time_limit_exceeded", False)):
            solver_health.hard_limit_breaches += 1
        if solver_health.hard_limit_breaches > 0:
            solver_health.status = "hard_limit_breach"
        elif solver_health.fallback_count > 0:
            solver_health.status = "degraded"
        else:
            solver_health.status = "ok"
        mpc_info["solver_health_status"] = solver_health.status
        mpc_info["solver_fallback_count"] = int(solver_health.fallback_count)
        mpc_info["solver_hard_limit_breaches"] = int(solver_health.hard_limit_breaches)
        mpc_info["solver_last_fallback_reason"] = solver_health.last_fallback_reason
        mpc_info["solver_fallback_active"] = bool(solver_health.fallback_active)
        mpc_info["solver_fallback_age_s"] = float(solver_health.fallback_age_s)
        mpc_info["solver_fallback_scale"] = float(solver_health.fallback_scale)

    # Track solve time for high-frequency logging
    if mpc_info:
        sim.last_solve_time = mpc_info.get("solve_time", 0.0)

    rw_torque_cmd = np.zeros(3, dtype=np.float64)
    # Per-wheel denormalization: each MPC output in [-1,1] is scaled by
    # that wheel's tau_max (matching the CasADi dynamics model).
    rw_limits = getattr(sim.mpc_controller, "rw_torque_limits", None)
    if rw_torque_norm is not None and rw_limits and len(rw_limits) > 0:
        n = min(len(rw_torque_norm), len(rw_limits), 3)
        for i in range(n):
            rw_torque_cmd[i] = rw_torque_norm[i] * rw_limits[i]
    sim.satellite.set_reaction_wheel_torque(rw_torque_cmd)

    # Update simulation state
    sim.last_control_update = sim.simulation_time
    sim.next_control_simulation_time += sim.control_update_interval
    sim.last_control_output = np.concatenate([thruster_action, rw_torque_cmd])
    thruster_copy = thruster_action.copy()
    sim.previous_thrusters = thruster_copy
    history_stride = int(getattr(sim, "history_downsample_stride", 1) or 1)
    downsample_counter = int(getattr(sim, "_control_history_downsample_counter", 0)) + 1
    sim._control_history_downsample_counter = downsample_counter
    if history_stride <= 1 or (downsample_counter % history_stride) == 0:
        sim._append_capped_history(sim.control_history, thruster_copy)
    sim.set_thruster_pattern(thruster_action)

    # Log data
    command_sent_sim_time = sim.simulation_time
    control_loop_duration = command_sent_wall_time - mpc_start_wall_time
    update_ref = getattr(sim, "update_path_reference_state", None)
    if callable(update_ref):
        try:
            # Keep terminal/CSV reference snapshot aligned with controller state
            # used for this control step.
            update_ref(current_state)
        except Exception:
            logger.debug(
                "Failed to refresh reference state before logging", exc_info=True
            )

    sim.log_simulation_step(
        mpc_start_sim_time=mpc_start_sim_time,
        command_sent_sim_time=command_sent_sim_time,
        current_state=current_state,
        thruster_action=thruster_action,
        mpc_info=mpc_info,
        mpc_computation_time=mpc_computation_time,
        control_loop_duration=control_loop_duration,
        rw_torque=rw_torque_cmd,
    )

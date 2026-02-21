"""Control-loop helpers extracted from the main simulation class."""

from __future__ import annotations

import time
from typing import Any

import numpy as np


def _update_v6_mode_state(sim: Any, current_state: np.ndarray) -> None:
    """Refresh V6 controller mode state from latest path/gate metrics."""
    mode_manager = getattr(sim, "v6_mode_manager", None)
    if mode_manager is None:
        return

    path_s = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)
    contour_error = 0.0
    endpoint_error = None

    try:
        if hasattr(sim.mpc_controller, "get_path_progress"):
            metrics = sim.mpc_controller.get_path_progress(current_state[:3])
            if isinstance(metrics, dict):
                path_s = float(metrics.get("s", path_s))
                contour_error = float(metrics.get("path_error", contour_error) or 0.0)
                endpoint_error = metrics.get("endpoint_error")
    except Exception:
        contour_error = 0.0

    path_len = float(sim._get_mission_path_length(compute_if_missing=True) or 0.0)
    pos_tol = float(getattr(sim, "position_tolerance", 0.1) or 0.1)
    gate = getattr(sim, "v6_completion_gate", None)
    completion_gate_state_ok = bool(getattr(gate, "all_thresholds_ok", False))
    completion_reached = bool(getattr(sim, "v6_completion_reached", False))
    solver_health = getattr(sim, "v6_solver_health", None)
    solver_degraded = bool(
        solver_health is not None
        and str(getattr(solver_health, "status", "ok")).lower() != "ok"
    )
    solver_fallback_reason = (
        str(getattr(solver_health, "last_fallback_reason", "") or "")
        if solver_health is not None
        else None
    )
    if not solver_fallback_reason:
        solver_fallback_reason = None

    sim.v6_mode_state = mode_manager.update(
        sim_time_s=float(sim.simulation_time),
        contour_error_m=contour_error,
        path_s=path_s,
        path_len=path_len,
        position_tolerance_m=pos_tol,
        completion_gate_state_ok=completion_gate_state_ok,
        completion_reached=completion_reached,
        solver_degraded=solver_degraded,
        solver_fallback_reason=solver_fallback_reason,
    )
    sim.v6_mode_profile = mode_manager.profile_for_mode(sim.v6_mode_state.current_mode)

    if hasattr(sim, "v6_mode_timeline"):
        sim._append_capped_history(
            sim.v6_mode_timeline,
            {
                "time_s": float(sim.simulation_time),
                "mode": str(sim.v6_mode_state.current_mode),
                "time_in_mode_s": float(sim.v6_mode_state.time_in_mode_s),
                "path_s": float(path_s),
                "path_error_m": float(contour_error),
                "endpoint_error_m": (
                    float(endpoint_error)
                    if endpoint_error is not None
                    else None
                ),
            },
        )


def update_mpc_control_step(sim: Any) -> None:
    """Run one MPC control update when control timing permits."""
    # Force MPC to send commands at fixed intervals
    if sim.simulation_time < sim.next_control_simulation_time:
        return

    current_state = sim.get_current_state()
    _update_v6_mode_state(sim, current_state)
    reference_scheduler = getattr(sim, "v6_reference_scheduler", None)
    if reference_scheduler is not None and hasattr(reference_scheduler, "build_slice"):
        try:
            horizon = int(
                getattr(sim.mpc_controller, "prediction_horizon", 10)
                if hasattr(sim, "mpc_controller")
                else 10
            )
            sim.v6_reference_slice = reference_scheduler.build_slice(
                sim=sim,
                current_state=current_state,
                mode=str(
                    getattr(getattr(sim, "v6_mode_state", None), "current_mode", "TRACK")
                ),
                horizon=max(1, horizon),
                dt=float(sim.control_update_interval),
            )
        except Exception:
            sim.v6_reference_slice = None

    # Delegate to MPCRunner
    if not hasattr(sim, "mpc_runner"):
        from satellite_control.core.mpc_runner import MPCRunner

        # Initialize MPC Runner wrapper
        sim.mpc_runner = MPCRunner(
            mpc_controller=sim.mpc_controller,
            config=sim.structured_config,
            state_validator=sim.state_validator,
            actuator_policy=getattr(sim, "v6_actuator_policy", None),
        )
    if hasattr(sim.mpc_runner, "set_mode_state"):
        sim.mpc_runner.set_mode_state(getattr(sim, "v6_mode_state", None))

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

    mode_state = getattr(sim, "v6_mode_state", None)
    if mpc_info is None:
        mpc_info = {}
    if mode_state is not None:
        mpc_info["mode_state"] = str(getattr(mode_state, "current_mode", "TRACK"))
        mpc_info["mode_time_in_mode_s"] = float(
            getattr(mode_state, "time_in_mode_s", 0.0)
        )
    mpc_info["controller_core"] = str(getattr(sim, "controller_core_mode", "v6"))
    gate = getattr(sim, "v6_completion_gate", None)
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

    solver_health = getattr(sim, "v6_solver_health", None)
    if solver_health is not None:
        fallback_reason = mpc_info.get("solver_fallback_reason")
        solver_health.fallback_active = bool(mpc_info.get("fallback_active", False))
        solver_health.fallback_age_s = float(mpc_info.get("fallback_age_s", 0.0) or 0.0)
        solver_health.fallback_scale = float(mpc_info.get("fallback_scale", 0.0) or 0.0)
        if bool(mpc_info.get("solver_fallback", False)):
            solver_health.fallback_count += 1
            reason_key = str(fallback_reason or "solver_fallback")
            solver_health.last_fallback_reason = reason_key
            if not isinstance(getattr(solver_health, "fallback_reasons", None), dict):
                solver_health.fallback_reasons = {}
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
        mpc_info["solver_hard_limit_breaches"] = int(
            solver_health.hard_limit_breaches
        )
        mpc_info["solver_last_fallback_reason"] = getattr(
            solver_health, "last_fallback_reason", None
        )
        mpc_info["solver_fallback_active"] = bool(
            getattr(solver_health, "fallback_active", False)
        )
        mpc_info["solver_fallback_age_s"] = float(
            getattr(solver_health, "fallback_age_s", 0.0)
        )
        mpc_info["solver_fallback_scale"] = float(
            getattr(solver_health, "fallback_scale", 0.0)
        )

    # Track solve time for high-frequency logging
    if mpc_info:
        sim.last_solve_time = mpc_info.get("solve_time", 0.0)

    rw_torque_cmd = np.zeros(3, dtype=np.float64)
    max_rw_torque = getattr(sim.mpc_controller, "max_rw_torque", 0.0)
    if rw_torque_norm is not None and max_rw_torque:
        rw_torque_cmd[: len(rw_torque_norm)] = rw_torque_norm * max_rw_torque
    # Cache the capability check for set_reaction_wheel_torque
    if not hasattr(sim, "_sat_has_rw_torque"):
        sim._sat_has_rw_torque = hasattr(sim.satellite, "set_reaction_wheel_torque")
    if sim._sat_has_rw_torque:
        sim.satellite.set_reaction_wheel_torque(rw_torque_cmd)

    # Update simulation state
    sim.last_control_update = sim.simulation_time
    sim.next_control_simulation_time += sim.control_update_interval
    sim.last_control_output = np.concatenate([thruster_action, rw_torque_cmd])
    thruster_copy = thruster_action.copy()
    sim.previous_thrusters = thruster_copy
    history_stride = int(getattr(sim, "history_downsample_stride", 1) or 1)
    if not hasattr(sim, "_control_history_downsample_counter"):
        sim._control_history_downsample_counter = 0
    sim._control_history_downsample_counter += 1
    if (
        history_stride <= 1
        or (sim._control_history_downsample_counter % history_stride) == 0
    ):
        sim._append_capped_history(sim.control_history, thruster_copy)
    sim.set_thruster_pattern(thruster_action)

    # Log data
    command_sent_sim_time = sim.simulation_time
    control_loop_duration = command_sent_wall_time - mpc_start_wall_time

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

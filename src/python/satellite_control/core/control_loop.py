"""Control-loop helpers extracted from the main simulation class."""

from __future__ import annotations

import time
from typing import Any

import numpy as np


def update_mpc_control_step(sim: Any) -> None:
    """Run one MPC control update when control timing permits."""
    # Force MPC to send commands at fixed intervals
    if sim.simulation_time < sim.next_control_simulation_time:
        return

    # Delegate to MPCRunner
    if not hasattr(sim, "mpc_runner"):
        from satellite_control.core.mpc_runner import MPCRunner

        # Initialize MPC Runner wrapper
        sim.mpc_runner = MPCRunner(
            mpc_controller=sim.mpc_controller,
            config=sim.structured_config,
            state_validator=sim.state_validator,
            max_command_history=getattr(sim, "history_max_steps", 0),
        )

    current_state = sim.get_current_state()
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

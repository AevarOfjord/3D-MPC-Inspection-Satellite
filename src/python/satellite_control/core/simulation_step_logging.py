"""Control-step logging logic for simulation runtime."""

import logging
import math
from typing import Any

import numpy as np

from satellite_control.utils.orientation_utils import (
    quat_angle_error,
    quat_wxyz_to_euler_xyz,
)


def _norm3(v: np.ndarray) -> float:
    """Fast Euclidean norm for 3-element vectors."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _fmt_position_mm(state: np.ndarray) -> str:
    """Format position as millimeters string."""
    x_mm = state[0] * 1000
    y_mm = state[1] * 1000
    z_mm = state[2] * 1000
    return f"[x:{x_mm:.0f}, y:{y_mm:.0f}, z:{z_mm:.0f}]mm"


def _fmt_angles_deg(state: np.ndarray) -> str:
    """Format quaternion from state as Euler angles in degrees."""
    q = np.array(state[3:7], dtype=float)
    if q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] == 0:
        q = np.array([1.0, 0.0, 0.0, 0.0])
    roll, pitch, yaw = quat_wxyz_to_euler_xyz(q)
    roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])
    return f"[Yaw:{yaw_deg:.1f}, Roll:{roll_deg:.1f}, Pitch:{pitch_deg:.1f}]°"


def log_simulation_step(
    sim: Any,
    logger_obj: logging.Logger,
    mpc_start_sim_time: float | None = None,
    command_sent_sim_time: float | None = None,
    current_state: np.ndarray | None = None,
    thruster_action: np.ndarray | None = None,
    mpc_info: dict[str, Any] | None = None,
    mpc_computation_time: float | None = None,
    control_loop_duration: float | None = None,
    rw_torque: np.ndarray | None = None,
    **legacy_kwargs: Any,
) -> None:
    """Log control-step data to terminal output and CSV files."""
    if mpc_start_sim_time is None:
        mpc_start_sim_time = legacy_kwargs.pop("mpc_start_time", None)
    if command_sent_sim_time is None:
        command_sent_sim_time = legacy_kwargs.pop(
            "command_sent_sim_time", legacy_kwargs.pop("command_sent_time", None)
        )
    if current_state is None:
        current_state = legacy_kwargs.pop("state", None)
    if thruster_action is None:
        thruster_action = legacy_kwargs.pop("thrusters", None)

    if current_state is None or thruster_action is None:
        raise ValueError(
            "log_simulation_step requires current_state and thruster_action"
        )

    if mpc_start_sim_time is None:
        mpc_start_sim_time = sim.simulation_time
    if command_sent_sim_time is None:
        command_sent_sim_time = sim.simulation_time

    if mpc_computation_time is None:
        mpc_computation_time = (
            float(mpc_info.get("solve_time", 0.0)) if mpc_info else 0.0
        )
    if control_loop_duration is None:
        control_loop_duration = 0.0

    stride = int(getattr(sim, "control_log_stride", 1) or 1)
    if not hasattr(sim, "_control_log_counter"):
        sim._control_log_counter = 0
    sim._control_log_counter += 1
    do_log = stride <= 1 or (sim._control_log_counter % stride) == 0

    # Store state history for summaries/plots.
    record_history = False
    if do_log:
        history_stride = int(getattr(sim, "history_downsample_stride", 1) or 1)
        if not hasattr(sim, "_history_downsample_counter"):
            sim._history_downsample_counter = 0
        sim._history_downsample_counter += 1
        record_history = (
            history_stride <= 1
            or (sim._history_downsample_counter % history_stride) == 0
        )
        if record_history:
            sim._append_capped_history(sim.state_history, current_state.copy())

    # Record performance metrics.
    solve_time = mpc_info.get("solve_time", 0.0) if mpc_info else 0.0
    timeout = mpc_info.get("timeout", False) if mpc_info else False
    sim.performance_monitor.record_mpc_solve(solve_time, timeout=timeout)

    # Record control loop time.
    timing_violation = mpc_computation_time > (sim.control_update_interval * 0.9)
    sim.performance_monitor.record_control_loop(
        control_loop_duration, timing_violation=timing_violation
    )

    # Print status with timing information.
    pos_error_scalar = _norm3(current_state[:3] - sim.reference_state[:3])
    ang_error_scalar = quat_angle_error(sim.reference_state[3:7], current_state[3:7])

    # Expose metrics for external telemetry.
    sim.last_solve_time = solve_time
    sim.last_pos_error = pos_error_scalar
    sim.last_ang_error = ang_error_scalar

    # Determine status message (path-only).
    mission_phase = "PATH_FOLLOWING"
    status_msg = f"Following Path (t={sim.simulation_time:.1f}s)"

    path_s = getattr(sim.mpc_controller, "s", None)
    path_len = sim._get_mission_path_length(compute_if_missing=True)
    if path_s is not None and path_len:
        status_msg = f"Following Path (s={path_s:.2f}/{path_len:.2f}m, t={sim.simulation_time:.1f}s)"

    # Prepare display variables and update command history.
    if thruster_action.ndim > 1:
        display_thrusters = thruster_action[0, :]
    else:
        display_thrusters = thruster_action

    active_thruster_ids = [int(x) for x in np.where(display_thrusters > 0.01)[0] + 1]
    if record_history:
        sim._append_capped_history(sim.command_history, active_thruster_ids)

    safe_reference = (
        sim.reference_state if sim.reference_state is not None else np.zeros(13)
    )
    if (
        safe_reference.shape[0] >= 7
        and (
            safe_reference[3] * safe_reference[3]
            + safe_reference[4] * safe_reference[4]
            + safe_reference[5] * safe_reference[5]
            + safe_reference[6] * safe_reference[6]
        )
        == 0
    ):
        safe_reference = safe_reference.copy()
        safe_reference[3] = 1.0

    ang_err_deg_scalar = np.degrees(ang_error_scalar)
    vel_error = 0.0
    ang_vel_error = 0.0
    if current_state.shape[0] >= 13 and safe_reference.shape[0] >= 13:
        vel_error = _norm3(current_state[7:10] - safe_reference[7:10])
        ang_vel_error = _norm3(current_state[10:13] - safe_reference[10:13])
    ang_vel_err_deg = np.degrees(ang_vel_error)
    solve_ms = mpc_info.get("solve_time", 0) * 1000 if mpc_info else 0.0

    # Show duty cycle for each active thruster (matching active_thruster_ids).
    thr_out = [round(float(display_thrusters[i - 1]), 2) for i in active_thruster_ids]
    rw_norm = np.zeros(3, dtype=float)
    if rw_torque is not None:
        rw_vals = np.array(rw_torque, dtype=float)
        rw_norm[: min(3, len(rw_vals))] = rw_vals[:3]
    rw_out_str = f"[{rw_norm[0]:.2f}, {rw_norm[1]:.2f}, {rw_norm[2]:.2f}]"

    # Calculate detailed error vectors
    # Position Error Vector (Current - Reference)
    pos_err_vec = (current_state[:3] - safe_reference[:3]) * 1000.0  # mm
    pos_err_str = (
        f"[x:{pos_err_vec[0]:.0f}, y:{pos_err_vec[1]:.0f}, z:{pos_err_vec[2]:.0f}]mm"
    )

    # Angle Error Vector (Current - Reference), wrapped to [-180, 180]
    # We need to get Euler angles for both and subtract
    q_curr = np.array(current_state[3:7], dtype=float)
    if np.dot(q_curr, q_curr) == 0:
        q_curr = np.array([1.0, 0.0, 0.0, 0.0])

    q_ref = np.array(safe_reference[3:7], dtype=float)
    if np.dot(q_ref, q_ref) == 0:
        q_ref = np.array([1.0, 0.0, 0.0, 0.0])

    curr_r, curr_p, curr_y = quat_wxyz_to_euler_xyz(q_curr)
    ref_r, ref_p, ref_y = quat_wxyz_to_euler_xyz(q_ref)

    # Difference in degrees
    diff_r = np.degrees(curr_r - ref_r)
    diff_p = np.degrees(curr_p - ref_p)
    diff_y = np.degrees(curr_y - ref_y)

    # Wrap to [-180, 180]
    diff_r = (diff_r + 180) % 360 - 180
    diff_p = (diff_p + 180) % 360 - 180
    diff_y = (diff_y + 180) % 360 - 180

    ang_err_str = f"[Yaw:{diff_y:.1f}, Roll:{diff_r:.1f}, Pitch:{diff_p:.1f}]°"

    # ANSI Color Codes
    BLUE = "\033[94m"
    YELLOW = "\033[93m"
    RESET = "\033[0m"
    CYAN = "\033[96m"

    BOLD = "\033[1m"

    # Time and Status Header
    header_line = f"{status_msg} | Solve: {solve_ms:.1f}ms"

    # Error Metrics formatted with colors
    # We can use simple thresholding for colors, or just keep labels colored for readability.
    # Let's stick to colored labels for now to ensure readability without confusing thresholds.

    row_pos = (
        f"{CYAN}POS ERR:{RESET} {pos_error_scalar:.3f}m".ljust(25)
        + f"{CYAN}| Vec:{RESET} {pos_err_str}"
    )
    row_ang = (
        f"{CYAN}ANG ERR:{RESET} {ang_err_deg_scalar:.1f}°".ljust(25)
        + f"{CYAN}| Vec:{RESET} {ang_err_str}"
    )
    row_vel = (
        f"{CYAN}VEL ERR:{RESET} {vel_error:.3f}m/s".ljust(25)
        + f"{CYAN}| Ang Vel Err:{RESET} {ang_vel_err_deg:.1f}°/s"
    )

    # State Data
    row_state_pos = (
        f"{BLUE}Pos:{RESET} {_fmt_position_mm(current_state)}".ljust(45)
        + f"{BLUE}(Ref:{RESET} {_fmt_position_mm(safe_reference)})"
    )
    row_state_ang = f"{BLUE}Ang:{RESET} {_fmt_angles_deg(current_state)}"

    # Actuators
    row_thrusters = f"{YELLOW}Thrusters {active_thruster_ids}:{RESET} {thr_out}"
    row_rw = f"{YELLOW}RW Torque [X,Y,Z]:{RESET}   {rw_out_str}"

    sep = "-" * 80
    block_sep = "=" * 80

    if do_log:
        logger_obj.info(
            f"\n{BOLD}{block_sep}{RESET}\n"
            f"{header_line}\n"
            f"{sep}\n"
            f"{row_pos}\n"
            f"{row_ang}\n"
            f"{row_vel}\n"
            f"{sep}\n"
            f"{row_state_pos}\n"
            f"{row_state_ang}\n"
            f"{sep}\n"
            f"{row_thrusters}\n"
            f"{row_rw}\n\n\n"
        )

    if do_log:
        # Delegate to SimulationLogger for control_data.csv output.
        if not hasattr(sim, "logger_helper"):
            from satellite_control.core.simulation_logger import SimulationLogger

            sim.logger_helper = SimulationLogger(sim.data_logger)

        previous_thruster_action: np.ndarray | None = (
            sim.previous_command if hasattr(sim, "previous_command") else None
        )

        # Update Context.
        # Update Context.
        mission_state = getattr(sim.simulation_config, "mission_state", None)
        frame_origin = None
        if mission_state and hasattr(mission_state, "frame_origin"):
            frame_origin = np.array(mission_state.frame_origin, dtype=float)

        sim.context.update_state(
            sim.simulation_time,
            current_state,
            sim.reference_state,
            frame_origin=frame_origin,
        )
        sim.context.step_number = sim.data_logger.current_step
        sim.context.mission_phase = mission_phase
        sim.context.previous_thruster_command = previous_thruster_action
        if rw_torque is not None:
            sim.context.rw_torque_command = np.array(rw_torque, dtype=np.float64)

        mpc_info_safe = mpc_info if mpc_info is not None else {}
        sim.logger_helper.log_step(
            sim.context,
            mpc_start_sim_time,
            command_sent_sim_time,
            thruster_action,
            mpc_info_safe,
            rw_torque=sim.context.rw_torque_command,
            solve_time=sim.last_solve_time,
        )

        # Log terminal message to CSV.
        # Restoring variables needed for CSV logging
        stabilization_time = None
        next_upd = sim.next_control_simulation_time

        terminal_entry = {
            "Time": sim.simulation_time,
            "Status": status_msg,
            "Stabilization_Time": (
                stabilization_time if stabilization_time is not None else ""
            ),
            "Position_Error_m": pos_error_scalar,
            "Angle_Error_deg": ang_err_deg_scalar,
            "Active_Thrusters": str(active_thruster_ids),
            "Solve_Time_s": mpc_computation_time,
            "Next_Update_s": next_upd,
        }
        sim.data_logger.log_terminal_message(terminal_entry)

    sim.previous_command = thruster_action.copy()

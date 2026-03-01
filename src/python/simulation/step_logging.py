"""Control-step logging logic for simulation runtime."""

import logging
import math
import os
import sys
from typing import Any

import numpy as np
from utils.orientation_utils import (
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


def _unwrap_euler_deg(
    curr_deg: np.ndarray, prev_unwrapped_deg: np.ndarray | None
) -> np.ndarray:
    """Unwrap Euler degrees to a continuous display series."""
    if prev_unwrapped_deg is None:
        return curr_deg.copy()
    delta = curr_deg - prev_unwrapped_deg
    delta = (delta + 180.0) % 360.0 - 180.0
    return prev_unwrapped_deg + delta


def _wrap_delta_deg(delta_deg: np.ndarray) -> np.ndarray:
    """Wrap degree deltas to [-180, 180] for readable terminal display."""
    return (delta_deg + 180.0) % 360.0 - 180.0


def _fmt_metric_row(
    label: str,
    value: float,
    unit: str,
    vec: str,
    *,
    color_prefix: str,
    color_reset: str,
    left_width: int = 26,
) -> str:
    """Format a metric row with aligned `| Vec:` separators."""
    value_str = f"{value:.3f}"
    if unit in {"°", "°/s"}:
        value_str = f"{value:.1f}"
    left_plain = f"{label}: {value_str}{unit}"
    pad = max(1, left_width - len(left_plain))
    left = f"{color_prefix}{label}:{color_reset} {value_str}{unit}" + (" " * pad)
    return left + f"{color_prefix}| Vec:{color_reset} {vec}"


def _fmt_state_row(
    label: str,
    value: str,
    *,
    color_prefix: str,
    color_reset: str,
    label_width: int = 8,
) -> str:
    """Format state lines with aligned labels for quick Sat/Ref comparison."""
    lbl = f"{label}:".ljust(label_width)
    return f"{color_prefix}{lbl}{color_reset} {value}"


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
) -> None:
    """Log control-step data to terminal output and CSV files."""
    if mpc_info is None:
        mpc_info = {}

    if current_state is None or thruster_action is None:
        raise ValueError(
            "log_simulation_step requires current_state and thruster_action"
        )

    if mpc_start_sim_time is None:
        mpc_start_sim_time = sim.simulation_time
    if command_sent_sim_time is None:
        command_sent_sim_time = sim.simulation_time

    if mpc_computation_time is None:
        mpc_computation_time = float(mpc_info.get("solve_time", 0.0))
    if control_loop_duration is None:
        control_loop_duration = 0.0

    stride = int(getattr(sim, "control_log_stride", 1) or 1)
    control_log_counter = int(getattr(sim, "_control_log_counter", 0)) + 1
    sim._control_log_counter = control_log_counter
    do_log = stride <= 1 or (control_log_counter % stride) == 0

    # Store state history for summaries/plots.
    record_history = False
    if do_log:
        history_stride = int(getattr(sim, "history_downsample_stride", 1) or 1)
        history_counter = int(getattr(sim, "_history_downsample_counter", 0)) + 1
        sim._history_downsample_counter = history_counter
        record_history = history_stride <= 1 or (history_counter % history_stride) == 0
        if record_history:
            sim._append_capped_history(sim.state_history, current_state.copy())

    # Record performance metrics.
    solve_time = mpc_info.get("solve_time", 0.0)
    timeout = mpc_info.get("timeout", False)
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
    mode_label = str(getattr(getattr(sim, "mode_state", None), "current_mode", "TRACK"))
    status_msg = f"Following Path [{mode_label}] (t={sim.simulation_time:.1f}s)"

    path_s_ref = mpc_info.get("path_s", getattr(sim.mpc_controller, "s", None))
    path_s_sat = mpc_info.get("path_s_proj")
    if path_s_sat is None:
        try:
            path_metrics = sim.mpc_controller.get_path_progress(current_state[:3])
            if isinstance(path_metrics, dict):
                path_s_sat = path_metrics.get("s")
        except Exception:
            path_s_sat = None
    path_len = sim._get_mission_path_length(compute_if_missing=True)
    if path_s_sat is not None and path_s_ref is not None and path_len:
        status_msg = (
            f"Following Path [{mode_label}] "
            f"(sat s={float(path_s_sat):.2f}/{path_len:.2f}m, "
            f"ref s={float(path_s_ref):.2f}/{path_len:.2f}m, "
            f"t={sim.simulation_time:.1f}s)"
        )
    elif path_s_ref is not None and path_len:
        status_msg = (
            f"Following Path [{mode_label}] "
            f"(ref s={float(path_s_ref):.2f}/{path_len:.2f}m, "
            f"t={sim.simulation_time:.1f}s)"
        )

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
    vel_err_vec = np.zeros(3, dtype=float)
    ang_vel_err_vec = np.zeros(3, dtype=float)
    if current_state.shape[0] >= 13 and safe_reference.shape[0] >= 13:
        vel_err_vec = np.array(current_state[7:10] - safe_reference[7:10], dtype=float)
        ang_vel_err_vec = np.array(
            current_state[10:13] - safe_reference[10:13], dtype=float
        )
        vel_error = _norm3(vel_err_vec)
        ang_vel_error = _norm3(ang_vel_err_vec)
    ang_vel_err_deg = np.degrees(ang_vel_error)
    ang_vel_err_vec_deg = np.degrees(ang_vel_err_vec)
    solve_ms = mpc_info.get("solve_time", 0) * 1000

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

    curr_euler_deg = np.degrees(np.array([curr_r, curr_p, curr_y], dtype=float))
    ref_euler_deg = np.degrees(np.array([ref_r, ref_p, ref_y], dtype=float))

    if getattr(sim, "_terminal_curr_euler_unwrapped_deg", None) is None:
        sim._terminal_curr_euler_unwrapped_deg = None
    if getattr(sim, "_terminal_ref_euler_unwrapped_deg", None) is None:
        sim._terminal_ref_euler_unwrapped_deg = None

    curr_unwrapped_deg = _unwrap_euler_deg(
        curr_euler_deg, sim._terminal_curr_euler_unwrapped_deg
    )
    ref_unwrapped_deg = _unwrap_euler_deg(
        ref_euler_deg, sim._terminal_ref_euler_unwrapped_deg
    )
    sim._terminal_curr_euler_unwrapped_deg = curr_unwrapped_deg.copy()
    sim._terminal_ref_euler_unwrapped_deg = ref_unwrapped_deg.copy()

    # Readable display deltas (current - reference), wrapped to principal range.
    diff_deg = _wrap_delta_deg(curr_euler_deg - ref_euler_deg)
    diff_r = float(diff_deg[0])
    diff_p = float(diff_deg[1])
    diff_y = float(diff_deg[2])

    ang_err_str = f"[Yaw:{diff_y:.1f}, Roll:{diff_r:.1f}, Pitch:{diff_p:.1f}]°"
    vel_err_str = (
        f"[vx:{vel_err_vec[0]:.3f}, vy:{vel_err_vec[1]:.3f}, "
        f"vz:{vel_err_vec[2]:.3f}]m/s"
    )
    ang_vel_err_str = (
        f"[wx:{ang_vel_err_vec_deg[0]:.1f}, wy:{ang_vel_err_vec_deg[1]:.1f}, "
        f"wz:{ang_vel_err_vec_deg[2]:.1f}]°/s"
    )

    # ANSI colors only when terminal supports them and NO_COLOR is not set.
    use_color = bool(getattr(sys.stdout, "isatty", lambda: False)()) and not bool(
        os.environ.get("NO_COLOR")
    )
    BLUE = "\033[94m" if use_color else ""
    YELLOW = "\033[93m" if use_color else ""
    RESET = "\033[0m" if use_color else ""
    CYAN = "\033[96m" if use_color else ""
    BOLD = "\033[1m" if use_color else ""

    # Time and Status Header
    status_split_idx = status_msg.find(" (")
    if status_split_idx > 0:
        header_line_top = status_msg[:status_split_idx]
        header_line_bottom = (
            status_msg[status_split_idx:].lstrip() + f" | Solve: {solve_ms:.1f}ms"
        )
    else:
        header_line_top = status_msg
        header_line_bottom = f"(t={sim.simulation_time:.1f}s) | Solve: {solve_ms:.1f}ms"

    # Error Metrics formatted with colors
    # We can use simple thresholding for colors, or just keep labels colored for readability.
    # Let's stick to colored labels for now to ensure readability without confusing thresholds.

    row_pos = _fmt_metric_row(
        "POS ERR",
        pos_error_scalar,
        "m",
        pos_err_str,
        color_prefix=CYAN,
        color_reset=RESET,
    )
    row_ang = _fmt_metric_row(
        "ANG ERR",
        ang_err_deg_scalar,
        "°",
        ang_err_str,
        color_prefix=CYAN,
        color_reset=RESET,
    )
    row_vel = _fmt_metric_row(
        "VEL ERR",
        vel_error,
        "m/s",
        vel_err_str,
        color_prefix=CYAN,
        color_reset=RESET,
    )
    row_ang_vel = _fmt_metric_row(
        "ANG VEL ERR",
        ang_vel_err_deg,
        "°/s",
        ang_vel_err_str,
        color_prefix=CYAN,
        color_reset=RESET,
    )

    # State Data (satellite and reference broken into explicit sections)
    row_sat_pos = _fmt_state_row(
        "Sat Pos",
        _fmt_position_mm(current_state),
        color_prefix=BLUE,
        color_reset=RESET,
    )
    row_sat_ang = _fmt_state_row(
        "Sat Ang",
        f"[Yaw:{curr_euler_deg[2]:.1f}, Roll:{curr_euler_deg[0]:.1f}, Pitch:{curr_euler_deg[1]:.1f}]°",
        color_prefix=BLUE,
        color_reset=RESET,
    )
    row_ref_pos = _fmt_state_row(
        "Ref Pos",
        _fmt_position_mm(safe_reference),
        color_prefix=BLUE,
        color_reset=RESET,
    )
    row_ref_ang = _fmt_state_row(
        "Ref Ang",
        f"[Yaw:{ref_euler_deg[2]:.1f}, Roll:{ref_euler_deg[0]:.1f}, Pitch:{ref_euler_deg[1]:.1f}]°",
        color_prefix=BLUE,
        color_reset=RESET,
    )

    # Actuators
    row_thrusters = f"{YELLOW}Thrusters {active_thruster_ids}:{RESET} {thr_out}"
    row_rw = f"{YELLOW}RW Torque [X,Y,Z]:{RESET}   {rw_out_str}"

    sep = "-" * 80
    block_sep = "=" * 80

    if do_log:
        logger_obj.info(
            f"\n{BOLD}{block_sep}{RESET}\n"
            f"{header_line_top}\n"
            f"{header_line_bottom}\n"
            f"{sep}\n"
            f"{row_pos}\n"
            f"{row_ang}\n"
            f"{row_vel}\n"
            f"{row_ang_vel}\n"
            f"{sep}\n"
            f"\n"
            f"{row_sat_pos}\n"
            f"{row_sat_ang}\n"
            f"{row_ref_pos}\n"
            f"{row_ref_ang}\n"
            f"{sep}\n"
            f"\n"
            f"{row_thrusters}\n"
            f"{row_rw}\n\n\n"
        )

    if do_log:
        # Delegate to SimulationLogger for control_data.csv output.
        if getattr(sim, "logger_helper", None) is None:
            from simulation.logger import SimulationLogger

            sim.logger_helper = SimulationLogger(sim.data_logger)

        previous_thruster_action: np.ndarray | None = sim.previous_command

        # Update Context.
        # Update Context.
        mission_state = sim.simulation_config.mission_state
        frame_origin = None
        if mission_state is not None:
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

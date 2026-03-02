from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
from simulation.step_logging import log_simulation_step


def _make_sim(*, thruster_forces=None, rw_torque_limits=None):
    perf = SimpleNamespace(
        record_mpc_solve=lambda *args, **kwargs: None,
        record_control_loop=lambda *args, **kwargs: None,
    )
    context = SimpleNamespace(
        update_state=lambda *args, **kwargs: None,
        step_number=0,
        mission_phase="",
        previous_thruster_command=None,
        rw_torque_command=np.zeros(3, dtype=float),
    )
    return SimpleNamespace(
        simulation_time=370.5,
        control_log_stride=1,
        _control_log_counter=0,
        history_downsample_stride=1,
        _history_downsample_counter=0,
        _append_capped_history=lambda hist, value: hist.append(value),
        state_history=[],
        command_history=[],
        reference_state=np.array(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            dtype=float,
        ),
        mode_state=SimpleNamespace(current_mode="HOLD"),
        _get_mission_path_length=lambda compute_if_missing=True: 44.97,
        mpc_controller=SimpleNamespace(
            s=44.97,
            thruster_forces=thruster_forces,
            rw_torque_limits=rw_torque_limits,
        ),
        performance_monitor=perf,
        logger_helper=SimpleNamespace(log_step=MagicMock()),
        data_logger=SimpleNamespace(current_step=1, log_terminal_message=MagicMock()),
        simulation_config=SimpleNamespace(
            mission_state=SimpleNamespace(frame_origin=(0.0, 0.0, 0.0))
        ),
        context=context,
        previous_command=np.zeros(6, dtype=float),
        control_update_interval=0.1,
        next_control_simulation_time=370.6,
        last_solve_time=0.0,
        last_pos_error=0.0,
        last_ang_error=0.0,
    )


def test_step_logging_prints_physical_actuator_rows():
    sim = _make_sim(
        thruster_forces=[1.0, 1.5, 2.0, 2.5, 3.0, 3.5],
        rw_torque_limits=[0.1, 0.2, 0.4],
    )
    logger = MagicMock()
    current_state = np.array(
        [
            -2.003,
            4.911,
            3.641,
            1.0,
            0.0,
            0.0,
            0.0,
            -0.002,
            0.004,
            0.003,
            0.0,
            0.0,
            0.0,
        ],
        dtype=float,
    )
    thruster_action = np.array([0.33, 0.0, 0.0, 0.10, 0.35, 0.0], dtype=float)
    rw_torque = np.array([0.05, 0.04, -0.20], dtype=float)
    mpc_info = {"solve_time": 0.0083, "path_s": 44.97, "path_s_proj": 44.90}

    log_simulation_step(
        sim=sim,
        logger_obj=logger,
        current_state=current_state,
        thruster_action=thruster_action,
        mpc_info=mpc_info,
        rw_torque=rw_torque,
    )

    printed = logger.info.call_args[0][0]
    assert "Thrusters Activity [1, 4, 5]: [0.33, 0.10, 0.35]" in printed
    assert "Thruster Force [1, 4, 5]: [0.33, 0.25, 1.05]N" in printed
    assert "RW Activity [X,Y,Z]:   [0.50, 0.20, -0.50]" in printed
    assert "RW Torque [X,Y,Z]:   [0.05, 0.04, -0.20]N*m" in printed


def test_step_logging_actuator_rows_fallback_without_limits():
    sim = _make_sim(thruster_forces=None, rw_torque_limits=None)
    logger = MagicMock()
    current_state = np.array([0.0] * 13, dtype=float)
    current_state[3] = 1.0
    thruster_action = np.array([0.33, 0.0, 0.0, 0.10, 0.35, 0.0], dtype=float)
    rw_torque = np.array([0.05, 0.04, -0.20], dtype=float)
    mpc_info = {"solve_time": 0.0083, "path_s": 1.0, "path_s_proj": 0.9}

    log_simulation_step(
        sim=sim,
        logger_obj=logger,
        current_state=current_state,
        thruster_action=thruster_action,
        mpc_info=mpc_info,
        rw_torque=rw_torque,
    )

    printed = logger.info.call_args[0][0]
    assert "Thruster Force [1, 4, 5]: [0.00, 0.00, 0.00]N" in printed
    assert "RW Activity [X,Y,Z]:   [0.00, 0.00, 0.00]" in printed


def test_step_logging_actuator_rows_with_no_active_thrusters():
    sim = _make_sim(
        thruster_forces=[1.0, 1.5, 2.0, 2.5, 3.0, 3.5],
        rw_torque_limits=[0.1, 0.2, 0.4],
    )
    logger = MagicMock()
    current_state = np.array([0.0] * 13, dtype=float)
    current_state[3] = 1.0
    thruster_action = np.zeros(6, dtype=float)
    rw_torque = np.zeros(3, dtype=float)
    mpc_info = {"solve_time": 0.0083, "path_s": 1.0, "path_s_proj": 0.9}

    log_simulation_step(
        sim=sim,
        logger_obj=logger,
        current_state=current_state,
        thruster_action=thruster_action,
        mpc_info=mpc_info,
        rw_torque=rw_torque,
    )

    printed = logger.info.call_args[0][0]
    assert "Thrusters Activity []: []" in printed
    assert "Thruster Force []: []N" in printed

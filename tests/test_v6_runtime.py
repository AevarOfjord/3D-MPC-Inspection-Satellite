"""Unit tests for V6 runtime mode/gate/policy helpers."""

import numpy as np
import pytest
from core.simulation_reference import update_path_reference_state
from core.v6_controller_runtime import (
    ActuatorPolicyV6,
    ControllerModeManagerV6,
    PointingGuardrailV6,
    TerminalSupervisorV6,
    compute_pointing_errors_deg,
    compute_runtime_path_speed,
    estimate_required_duration_s,
    resolve_pointing_context_v6,
)


def test_mode_manager_track_recover_hysteresis() -> None:
    manager = ControllerModeManagerV6(
        recover_enter_error_m=0.20,
        recover_enter_hold_s=0.5,
        recover_exit_error_m=0.10,
        recover_exit_hold_s=1.0,
    )
    manager.reset(sim_time_s=0.0)

    state = manager.update(
        sim_time_s=0.2,
        contour_error_m=0.25,
        path_s=1.0,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=False,
        completion_reached=False,
    )
    assert state.current_mode == "TRACK"

    state = manager.update(
        sim_time_s=0.8,
        contour_error_m=0.25,
        path_s=1.5,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=False,
        completion_reached=False,
    )
    assert state.current_mode == "RECOVER"

    state = manager.update(
        sim_time_s=1.0,
        contour_error_m=0.08,
        path_s=2.0,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=False,
        completion_reached=False,
    )
    assert state.current_mode == "RECOVER"

    state = manager.update(
        sim_time_s=2.1,
        contour_error_m=0.08,
        path_s=2.2,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=False,
        completion_reached=False,
    )
    assert state.current_mode == "TRACK"


def test_mode_manager_settle_hold_complete_transitions() -> None:
    manager = ControllerModeManagerV6()
    manager.reset(sim_time_s=0.0)

    state = manager.update(
        sim_time_s=3.0,
        contour_error_m=0.05,
        path_s=9.95,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=False,
        completion_reached=False,
    )
    assert state.current_mode == "SETTLE"

    state = manager.update(
        sim_time_s=3.2,
        contour_error_m=0.04,
        path_s=10.0,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=True,
        completion_reached=False,
    )
    assert state.current_mode == "HOLD"

    state = manager.update(
        sim_time_s=13.5,
        contour_error_m=0.0,
        path_s=10.0,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=True,
        completion_reached=True,
    )
    assert state.current_mode == "COMPLETE"


def test_mode_manager_solver_degraded_triggers_recover() -> None:
    manager = ControllerModeManagerV6(
        recover_enter_error_m=0.20,
        recover_enter_hold_s=0.5,
    )
    manager.reset(sim_time_s=0.0)

    state = manager.update(
        sim_time_s=0.1,
        contour_error_m=0.02,
        path_s=1.0,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=False,
        completion_reached=False,
        solver_degraded=True,
        solver_fallback_reason="solver_timeout",
    )
    assert state.current_mode == "TRACK"

    state = manager.update(
        sim_time_s=0.4,
        contour_error_m=0.02,
        path_s=1.2,
        path_len=10.0,
        position_tolerance_m=0.1,
        completion_gate_state_ok=False,
        completion_reached=False,
        solver_degraded=True,
        solver_fallback_reason="solver_timeout",
    )
    assert state.current_mode == "RECOVER"


def test_terminal_supervisor_hold_reset_and_completion() -> None:
    supervisor = TerminalSupervisorV6(hold_required_s=10.0)

    gate = supervisor.evaluate(
        sim_time_s=1.0,
        progress_ok=True,
        position_ok=True,
        angle_ok=True,
        velocity_ok=True,
        angular_velocity_ok=True,
    )
    assert gate.complete is False
    assert gate.hold_elapsed_s == pytest.approx(0.0)

    gate = supervisor.evaluate(
        sim_time_s=8.0,
        progress_ok=True,
        position_ok=True,
        angle_ok=False,
        velocity_ok=True,
        angular_velocity_ok=True,
    )
    assert gate.complete is False
    assert gate.hold_elapsed_s == pytest.approx(0.0)
    assert gate.last_breach_reason == "angle_error"

    gate = supervisor.evaluate(
        sim_time_s=9.0,
        progress_ok=True,
        position_ok=True,
        angle_ok=True,
        velocity_ok=True,
        angular_velocity_ok=True,
    )
    assert gate.complete is False

    gate = supervisor.evaluate(
        sim_time_s=19.0,
        progress_ok=True,
        position_ok=True,
        angle_ok=True,
        velocity_ok=True,
        angular_velocity_ok=True,
    )
    assert gate.complete is True


def test_runtime_speed_policy_and_duration_estimator() -> None:
    speed = compute_runtime_path_speed(
        non_hold_segment_caps=[0.12, 0.08, 0.10],
        default_path_speed=0.2,
        path_speed_min=0.05,
        path_speed_max=0.09,
    )
    assert speed == pytest.approx(0.08)

    duration = estimate_required_duration_s(
        path_length_m=12.0,
        path_speed_mps=speed,
        hold_duration_s=10.0,
        margin_s=30.0,
    )
    assert duration == pytest.approx((12.0 / 0.08) + 10.0 + 30.0)


def test_reference_velocity_prefers_mpc_progress_signal_and_clamps() -> None:
    class _MPCCfg:
        path_speed = 0.05
        path_speed_min = 0.02
        path_speed_max = 0.08

    class _AppCfg:
        mpc = _MPCCfg()

    class _SimCfg:
        app_config = _AppCfg()

    class _Mpc:
        _cpp_controller = type(
            "_CppRef",
            (),
            {"get_reference_at_s": lambda self, s_query, q_current: None},
        )()
        _last_path_projection = {"path_v_s": 0.12}

        @staticmethod
        def get_path_reference_state(q_current=None):
            return (
                np.array([0.0, 0.0, 0.0], dtype=float),
                np.array([1.0, 0.0, 0.0], dtype=float),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

    class _Mode:
        current_mode = "TRACK"

    class _Sim:
        mpc_controller = _Mpc()
        simulation_config = _SimCfg()
        v6_mode_state = _Mode()
        reference_state = None

    current_state = np.array(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dtype=float,
    )
    sim = _Sim()
    update_path_reference_state(sim, current_state)
    assert sim.reference_state[7] == pytest.approx(0.08)
    assert sim.reference_state[8] == pytest.approx(0.0)
    assert sim.reference_state[9] == pytest.approx(0.0)


def test_reference_velocity_forces_zero_in_settle_hold_complete() -> None:
    class _MPCCfg:
        path_speed = 0.05
        path_speed_min = 0.0
        path_speed_max = 0.08

    class _AppCfg:
        mpc = _MPCCfg()

    class _SimCfg:
        app_config = _AppCfg()

    class _Mpc:
        _cpp_controller = type(
            "_CppRef",
            (),
            {"get_reference_at_s": lambda self, s_query, q_current: None},
        )()
        _last_path_projection = {"path_v_s": 0.06}

        @staticmethod
        def get_path_reference_state(q_current=None):
            return (
                np.array([0.0, 0.0, 0.0], dtype=float),
                np.array([1.0, 0.0, 0.0], dtype=float),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

    class _Mode:
        current_mode = "SETTLE"

    class _Sim:
        mpc_controller = _Mpc()
        simulation_config = _SimCfg()
        v6_mode_state = _Mode()
        reference_state = None

    current_state = np.array(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dtype=float,
    )
    sim = _Sim()
    update_path_reference_state(sim, current_state)
    assert np.allclose(sim.reference_state[7:10], np.array([0.0, 0.0, 0.0]))


def test_actuator_policy_mode_behavior() -> None:
    policy = ActuatorPolicyV6(
        enable_thruster_hysteresis=True,
        thruster_hysteresis_on=0.02,
        thruster_hysteresis_off=0.01,
        terminal_bypass_band_m=0.2,
    )

    prev = np.array([0.0], dtype=np.float64)
    cmd = np.array([0.015], dtype=np.float64)
    out = policy.apply(cmd, prev, mode="TRACK", endpoint_error_m=0.5)
    assert np.allclose(out, np.array([0.0]))

    cmd_on = np.array([0.03], dtype=np.float64)
    out = policy.apply(cmd_on, prev, mode="TRACK", endpoint_error_m=0.5)
    assert np.allclose(out, np.array([0.03]))

    # SETTLE within bypass band preserves fine command (no hysteresis suppression).
    prev = np.array([0.03], dtype=np.float64)
    cmd_fine = np.array([0.015], dtype=np.float64)
    out = policy.apply(cmd_fine, prev, mode="SETTLE", endpoint_error_m=0.05)
    assert np.allclose(out, np.array([0.015]))

    # COMPLETE mode always zeros commands.
    out = policy.apply(cmd_fine, prev, mode="COMPLETE", endpoint_error_m=0.05)
    assert np.allclose(out, np.array([0.0]))


def test_mode_profile_uses_configured_multipliers() -> None:
    manager = ControllerModeManagerV6(
        recover_contour_scale=2.5,
        recover_lag_scale=2.3,
        recover_progress_scale=0.55,
        recover_attitude_scale=0.7,
        settle_progress_scale=0.0,
        settle_terminal_pos_scale=2.2,
        settle_terminal_attitude_scale=1.6,
        settle_velocity_align_scale=1.4,
        settle_angular_velocity_scale=1.9,
        hold_smoothness_scale=1.7,
        hold_thruster_pair_scale=1.25,
    )
    recover = manager.profile_for_mode("RECOVER")
    assert recover.contour_scale == pytest.approx(2.5)
    assert recover.lag_scale == pytest.approx(2.3)
    assert recover.progress_scale == pytest.approx(0.55)
    assert recover.attitude_scale == pytest.approx(0.7)

    hold = manager.profile_for_mode("HOLD")
    assert hold.terminal_pos_scale == pytest.approx(2.2)
    assert hold.terminal_attitude_scale == pytest.approx(1.6)
    assert hold.velocity_align_scale == pytest.approx(1.4)
    assert hold.angular_velocity_scale == pytest.approx(1.9)
    assert hold.smoothness_scale == pytest.approx(1.7)
    assert hold.thruster_pair_scale == pytest.approx(1.25)


def test_pointing_context_falls_back_to_lvlh_radial_axis() -> None:
    class _MissionState:
        path_frame = "LVLH"
        frame_origin = (100.0, 0.0, 0.0)
        pointing_path_spans: list[dict] = []

    class _SimConfig:
        mission_state = _MissionState()

    class _Sim:
        simulation_config = _SimConfig()

    state = np.array(
        [10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dtype=float,
    )
    context = resolve_pointing_context_v6(sim=_Sim(), current_state=state, path_s=0.5)
    assert context.source == "lvlh_radial_fallback"
    assert np.allclose(context.axis_world, np.array([1.0, 0.0, 0.0]), atol=1e-6)


def test_pointing_context_switches_across_span_boundaries() -> None:
    class _MissionState:
        path_frame = "LVLH"
        frame_origin = (0.0, 0.0, 0.0)
        pointing_path_spans = [
            {
                "s_start": 0.0,
                "s_end": 1.0,
                "scan_axis": [0.0, 0.0, 1.0],
                "scan_direction": "CW",
                "source_segment_index": 0,
                "context_source": "transfer_next_scan",
            },
            {
                "s_start": 1.0,
                "s_end": 2.0,
                "scan_axis": [0.0, 1.0, 0.0],
                "scan_direction": "CCW",
                "source_segment_index": 1,
                "context_source": "transfer_previous_scan",
            },
        ]

    class _SimConfig:
        mission_state = _MissionState()

    class _Sim:
        simulation_config = _SimConfig()

    state = np.array(
        [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dtype=float,
    )

    first = resolve_pointing_context_v6(sim=_Sim(), current_state=state, path_s=0.25)
    second = resolve_pointing_context_v6(sim=_Sim(), current_state=state, path_s=1.75)

    assert first.source == "transfer_next_scan"
    assert np.allclose(first.axis_world, np.array([0.0, 0.0, 1.0]), atol=1e-6)
    assert first.direction_cw is True

    assert second.source == "transfer_previous_scan"
    assert np.allclose(second.axis_world, np.array([0.0, 1.0, 0.0]), atol=1e-6)
    assert second.direction_cw is False


def test_pointing_context_uses_nearest_valid_scan_axis_before_radial_fallback() -> None:
    class _MissionState:
        path_frame = "LVLH"
        frame_origin = (100.0, 0.0, 0.0)
        scan_attitude_axis = (0.0, 0.0, 1.0)
        pointing_path_spans = [
            {
                "s_start": 0.0,
                "s_end": 1.0,
                "scan_axis": None,
                "scan_direction": "CW",
                "source_segment_index": 0,
                "context_source": "transfer_none",
            },
            {
                "s_start": 1.0,
                "s_end": 2.0,
                "scan_axis": [0.0, 1.0, 0.0],
                "scan_direction": "CW",
                "source_segment_index": 1,
                "context_source": "scan_segment",
            },
        ]

    class _SimConfig:
        mission_state = _MissionState()

    class _Sim:
        simulation_config = _SimConfig()

    state = np.array(
        [10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dtype=float,
    )
    context = resolve_pointing_context_v6(sim=_Sim(), current_state=state, path_s=0.25)

    assert context.source != "lvlh_radial_fallback"
    assert np.allclose(context.axis_world, np.array([0.0, 1.0, 0.0]), atol=1e-6)


def test_pointing_guardrail_latch_and_clear() -> None:
    guardrail = PointingGuardrailV6(
        enabled=True,
        z_error_deg_max=4.0,
        x_error_deg_max=6.0,
        breach_hold_s=0.3,
        clear_hold_s=0.8,
    )
    status = guardrail.update(sim_time_s=0.0, x_error_deg=7.0, z_error_deg=1.0)
    assert status.breached is False
    status = guardrail.update(sim_time_s=0.35, x_error_deg=7.0, z_error_deg=1.0)
    assert status.breached is True
    status = guardrail.update(sim_time_s=0.6, x_error_deg=1.0, z_error_deg=1.0)
    assert status.breached is True
    status = guardrail.update(sim_time_s=1.45, x_error_deg=1.0, z_error_deg=1.0)
    assert status.breached is False


def test_compute_pointing_errors_decomposes_x_and_z_axes() -> None:
    # 90 deg yaw: x-axis rotates to +Y while z-axis stays +Z.
    q_ref = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    q_curr = np.array(
        [np.cos(np.pi / 4.0), 0.0, 0.0, np.sin(np.pi / 4.0)],
        dtype=float,
    )
    x_err, z_err = compute_pointing_errors_deg(
        current_quat_wxyz=q_curr,
        reference_quat_wxyz=q_ref,
    )
    assert x_err == pytest.approx(90.0, abs=1e-3)
    assert z_err == pytest.approx(0.0, abs=1e-3)

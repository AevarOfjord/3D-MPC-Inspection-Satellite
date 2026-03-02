"""
Unit tests for runtime.control_loop.update_mpc_control_step().

Uses minimal mocks so no C++ extensions or full simulation are needed.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from runtime.control_loop import _update_mode_state, update_mpc_control_step
from runtime.policy import SolverHealth


def _make_sim(
    *,
    simulation_time: float = 0.0,
    next_control_simulation_time: float = -1.0,
    mpc_info: dict | None = None,
    control_update_interval: float = 0.1,
) -> MagicMock:
    """
    Return a MagicMock sim with the minimum attributes set for
    update_mpc_control_step to run without crashing.

    Callers can override individual attributes after creation.
    """
    sim = MagicMock()
    sim.simulation_time = simulation_time
    sim.next_control_simulation_time = next_control_simulation_time

    # Skip the expensive _update_mode_state path
    sim.mode_manager = None
    # Skip reference scheduler
    sim.reference_scheduler = None

    sim.control_update_interval = control_update_interval
    sim.get_current_state.return_value = np.zeros(16)
    sim.mpc_controller.rw_torque_limits = []

    if mpc_info is None:
        mpc_info = {}
    sim.mpc_runner.compute_control_action.return_value = (
        np.zeros(8),  # thruster_action
        np.zeros(3),  # rw_torque_norm (normalised)
        mpc_info,
        0.005,  # mpc_computation_time
        0.005,  # command_sent_wall_time
    )

    return sim


class TestControlLoopTimingGate:
    def test_timing_gate_skips_when_not_ready(self):
        """Control step is a no-op when simulation_time < next_control_simulation_time."""
        sentinel = object()
        sim = SimpleNamespace(
            simulation_time=0.5,
            next_control_simulation_time=1.0,
            previous_thrusters=sentinel,
        )
        update_mpc_control_step(sim)

        # Function returned at the timing gate — no attributes mutated
        assert sim.previous_thrusters is sentinel


class TestSolverHealthTransitions:
    def test_ok_to_degraded_on_fallback(self):
        """solver_health transitions to 'degraded' after a fallback solve."""
        sim = _make_sim(
            mpc_info={"solver_fallback": True, "solver_fallback_reason": "test"},
        )
        sim.solver_health = SolverHealth()

        update_mpc_control_step(sim)

        assert sim.solver_health.status == "degraded"
        assert sim.solver_health.fallback_count == 1

    def test_degraded_to_hard_limit_on_time_exceeded(self):
        """solver_health escalates to 'hard_limit_breach' when time_limit_exceeded."""
        sim = _make_sim(mpc_info={"time_limit_exceeded": True})
        sim.solver_health = SolverHealth(fallback_count=1, status="degraded")

        update_mpc_control_step(sim)

        assert sim.solver_health.status == "hard_limit_breach"
        assert sim.solver_health.hard_limit_breaches == 1

    def test_control_time_advances_after_successful_step(self):
        """next_control_simulation_time is incremented by control_update_interval."""
        interval = 0.1
        sim = _make_sim(
            simulation_time=0.0,
            next_control_simulation_time=-1.0,
            control_update_interval=interval,
        )
        sim.solver_health = None  # skip health update

        update_mpc_control_step(sim)

        assert sim.next_control_simulation_time == pytest.approx(-1.0 + interval)

    def test_control_step_refreshes_reference_before_logging(self):
        """Control-step logging should use the current reference snapshot."""
        sim = _make_sim()
        sim.solver_health = None

        update_mpc_control_step(sim)

        sim.update_path_reference_state.assert_called_once()
        sim.log_simulation_step.assert_called_once()


def test_update_mode_state_uses_max_progress_for_mode_transitions():
    class _CaptureModeManager:
        def __init__(self):
            self.last_update_kwargs: dict | None = None

        def update(self, **kwargs):
            self.last_update_kwargs = kwargs
            return SimpleNamespace(current_mode="TRACK", time_in_mode_s=0.0)

        @staticmethod
        def profile_for_mode(_mode: str) -> SimpleNamespace:
            return SimpleNamespace()

    class _FakeMPC:
        def __init__(self):
            self.s = 10.0

        @staticmethod
        def get_path_progress(_pos: np.ndarray) -> dict[str, float]:
            return {"s": 9.6, "path_error": 0.0, "endpoint_error": 0.4}

        @staticmethod
        def set_scan_attitude_context(_center, _axis, _direction) -> None:
            return None

    mode_manager = _CaptureModeManager()
    mission_state = SimpleNamespace(
        path_hold_schedule=[],
        path_waypoints=[],
        path_hold_active_index=None,
        path_hold_started_at_s=None,
        path_hold_completed=[],
    )
    sim = SimpleNamespace(
        mode_manager=mode_manager,
        mpc_controller=_FakeMPC(),
        position_tolerance=0.1,
        completion_gate=SimpleNamespace(all_thresholds_ok=False),
        completion_reached=False,
        solver_health=SimpleNamespace(status="ok", last_fallback_reason=None),
        simulation_time=12.0,
        simulation_config=SimpleNamespace(
            mission_state=mission_state,
            app_config=SimpleNamespace(
                controller_contracts=SimpleNamespace(
                    enable_pointing_contract=False,
                    pointing_scope="all_missions",
                )
            ),
        ),
        pointing_guardrail=None,
        mode_timeline=None,
    )
    sim._get_mission_path_length = lambda compute_if_missing=True: 10.0
    sim._append_capped_history = lambda *_args, **_kwargs: None

    _update_mode_state(sim, np.zeros(13, dtype=float))

    assert mode_manager.last_update_kwargs is not None
    assert mode_manager.last_update_kwargs["path_s"] == pytest.approx(10.0)


def test_update_mode_state_caps_projection_lead_for_progress_fusion():
    class _CaptureModeManager:
        def __init__(self):
            self.last_update_kwargs: dict | None = None

        def update(self, **kwargs):
            self.last_update_kwargs = kwargs
            return SimpleNamespace(current_mode="TRACK", time_in_mode_s=0.0)

        @staticmethod
        def profile_for_mode(_mode: str) -> SimpleNamespace:
            return SimpleNamespace()

    class _FakeMPC:
        def __init__(self):
            self.s = 0.0

        @staticmethod
        def get_path_progress(_pos: np.ndarray) -> dict[str, float]:
            return {"s": 2.0, "path_error": 0.0, "endpoint_error": 0.4}

        @staticmethod
        def set_scan_attitude_context(_center, _axis, _direction) -> None:
            return None

    sim = SimpleNamespace(
        mode_manager=_CaptureModeManager(),
        mpc_controller=_FakeMPC(),
        position_tolerance=0.1,
        completion_gate=SimpleNamespace(all_thresholds_ok=False),
        completion_reached=False,
        solver_health=SimpleNamespace(status="ok", last_fallback_reason=None),
        simulation_time=12.0,
        simulation_config=SimpleNamespace(
            mission_state=SimpleNamespace(
                path_hold_schedule=[],
                path_waypoints=[],
                path_hold_active_index=None,
                path_hold_started_at_s=None,
                path_hold_completed=[],
            ),
            app_config=SimpleNamespace(
                controller_contracts=SimpleNamespace(
                    enable_pointing_contract=False,
                    pointing_scope="all_missions",
                    path_projection_lead_cap_m=0.25,
                )
            ),
        ),
        pointing_guardrail=None,
        mode_timeline=None,
    )
    sim._get_mission_path_length = lambda compute_if_missing=True: 10.0
    sim._append_capped_history = lambda *_args, **_kwargs: None

    _update_mode_state(sim, np.zeros(13, dtype=float))

    assert sim.mode_manager.last_update_kwargs is not None
    assert sim.mode_manager.last_update_kwargs["path_s"] == pytest.approx(0.25)
    assert sim.path_progress_debug["path_s_projected_capped"] == pytest.approx(0.25)
    assert sim.path_progress_debug["path_s_fused"] == pytest.approx(0.25)


def test_update_mode_state_ignores_historical_hard_limit_for_solver_degraded():
    class _CaptureModeManager:
        def __init__(self):
            self.last_update_kwargs: dict | None = None

        def update(self, **kwargs):
            self.last_update_kwargs = kwargs
            return SimpleNamespace(current_mode="TRACK", time_in_mode_s=0.0)

        @staticmethod
        def profile_for_mode(_mode: str) -> SimpleNamespace:
            return SimpleNamespace()

    class _FakeMPC:
        def __init__(self):
            self.s = 1.0

        @staticmethod
        def get_path_progress(_pos: np.ndarray) -> dict[str, float]:
            return {"s": 1.0, "path_error": 0.01, "endpoint_error": 1.0}

        @staticmethod
        def set_scan_attitude_context(_center, _axis, _direction) -> None:
            return None

    sim = SimpleNamespace(
        mode_manager=_CaptureModeManager(),
        mpc_controller=_FakeMPC(),
        position_tolerance=0.1,
        completion_gate=SimpleNamespace(all_thresholds_ok=False),
        completion_reached=False,
        solver_health=SimpleNamespace(
            status="hard_limit_breach",
            last_fallback_reason="solver_timeout",
            fallback_active=False,
            fallback_age_s=0.0,
            fallback_count=3,
        ),
        simulation_time=12.0,
        simulation_config=SimpleNamespace(
            mission_state=SimpleNamespace(
                path_hold_schedule=[],
                path_waypoints=[],
                path_hold_active_index=None,
                path_hold_started_at_s=None,
                path_hold_completed=[],
            ),
            app_config=SimpleNamespace(
                controller_contracts=SimpleNamespace(
                    enable_pointing_contract=False,
                    pointing_scope="all_missions",
                    path_projection_lead_cap_m=0.25,
                )
            ),
        ),
        pointing_guardrail=None,
        mode_timeline=None,
    )
    sim._get_mission_path_length = lambda compute_if_missing=True: 10.0
    sim._append_capped_history = lambda *_args, **_kwargs: None

    _update_mode_state(sim, np.zeros(13, dtype=float))

    assert sim.mode_manager.last_update_kwargs is not None
    assert sim.mode_manager.last_update_kwargs["solver_degraded"] is False


def test_update_mode_state_uses_cpp_path_setter_for_waypoint_hold():
    class _CaptureModeManager:
        def __init__(self):
            self.last_update_kwargs: dict | None = None

        def update(self, **kwargs):
            self.last_update_kwargs = kwargs
            return SimpleNamespace(current_mode="TRACK", time_in_mode_s=0.0)

        @staticmethod
        def profile_for_mode(_mode: str) -> SimpleNamespace:
            return SimpleNamespace()

    class _FakeMPC:
        def __init__(self):
            self.s = 0.9
            self.set_s_calls: list[float] = []

        @staticmethod
        def get_path_progress(_pos: np.ndarray) -> dict[str, float]:
            return {"s": 1.0, "path_error": 0.0, "endpoint_error": 0.01}

        @staticmethod
        def set_scan_attitude_context(_center, _axis, _direction) -> None:
            return None

        def set_current_path_s(self, s_value: float) -> None:
            self.set_s_calls.append(float(s_value))
            self.s = float(s_value)

    mission_state = SimpleNamespace(
        path_hold_schedule=[{"path_index": 1, "duration_s": 5.0}],
        path_waypoints=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        path_hold_active_index=None,
        path_hold_started_at_s=None,
        path_hold_completed=[],
    )
    sim = SimpleNamespace(
        mode_manager=_CaptureModeManager(),
        mpc_controller=_FakeMPC(),
        position_tolerance=0.1,
        completion_gate=SimpleNamespace(all_thresholds_ok=False),
        completion_reached=False,
        solver_health=SimpleNamespace(status="ok", last_fallback_reason=None),
        simulation_time=12.0,
        simulation_config=SimpleNamespace(
            mission_state=mission_state,
            app_config=SimpleNamespace(
                controller_contracts=SimpleNamespace(
                    enable_pointing_contract=False,
                    pointing_scope="all_missions",
                    path_projection_lead_cap_m=0.25,
                )
            ),
        ),
        pointing_guardrail=None,
        mode_timeline=None,
    )
    sim._get_mission_path_length = lambda compute_if_missing=True: 1.0
    sim._append_capped_history = lambda *_args, **_kwargs: None

    _update_mode_state(sim, np.zeros(13, dtype=float))

    assert sim.mpc_controller.set_s_calls
    assert sim.mpc_controller.set_s_calls[-1] == pytest.approx(1.0)
    assert sim.mode_state.current_mode == "HOLD"


def test_update_mode_state_forces_transit_free_pointing_near_terminal_settle():
    class _CaptureModeManager:
        def __init__(self):
            self.last_update_kwargs: dict | None = None
            self.state = SimpleNamespace(current_mode="SETTLE")

        def update(self, **kwargs):
            self.last_update_kwargs = kwargs
            return SimpleNamespace(current_mode="SETTLE", time_in_mode_s=0.0)

        @staticmethod
        def profile_for_mode(_mode: str) -> SimpleNamespace:
            return SimpleNamespace()

    class _FakeMPC:
        def __init__(self):
            self.s = 10.0
            self.scan_context_calls: list[tuple[object, object, str]] = []

        @staticmethod
        def get_path_progress(_pos: np.ndarray) -> dict[str, float]:
            return {"s": 9.95, "path_error": 0.0, "endpoint_error": 0.2}

        def set_scan_attitude_context(self, center, axis, direction) -> None:
            self.scan_context_calls.append((center, axis, direction))

    mode_manager = _CaptureModeManager()
    mission_state = SimpleNamespace(
        path_hold_schedule=[],
        path_waypoints=[],
        path_hold_active_index=None,
        path_hold_started_at_s=None,
        path_hold_completed=[],
        pointing_path_spans=[
            {
                "segment_type": "scan",
                "pointing_policy": "scan_locked",
                "s_start": 0.0,
                "s_end": 10.0,
                "scan_axis": [0.0, 0.0, 1.0],
                "scan_direction": "CW",
                "source_segment_index": 0,
                "context_source": "scan_segment",
            }
        ],
    )
    sim = SimpleNamespace(
        mode_manager=mode_manager,
        mpc_controller=_FakeMPC(),
        position_tolerance=0.1,
        completion_gate=SimpleNamespace(all_thresholds_ok=False),
        completion_reached=False,
        solver_health=SimpleNamespace(status="ok", last_fallback_reason=None),
        simulation_time=12.0,
        simulation_config=SimpleNamespace(
            mission_state=mission_state,
            app_config=SimpleNamespace(
                controller_contracts=SimpleNamespace(
                    enable_pointing_contract=True,
                    pointing_scope="all_missions",
                )
            ),
        ),
        pointing_guardrail=None,
        mode_timeline=None,
    )
    sim._get_mission_path_length = lambda compute_if_missing=True: 10.0
    sim._append_capped_history = lambda *_args, **_kwargs: None

    _update_mode_state(sim, np.zeros(13, dtype=float))

    assert sim.pointing_status["pointing_policy"] == "transit_free"
    assert sim.pointing_status["pointing_context_source"] == "terminal_settle_override"
    assert sim.mpc_controller.scan_context_calls
    assert sim.mpc_controller.scan_context_calls[-1] == (None, None, "CW")

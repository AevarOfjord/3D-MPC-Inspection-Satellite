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

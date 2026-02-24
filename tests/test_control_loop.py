"""
Unit tests for runtime.control_loop.update_mpc_control_step().

Uses minimal mocks so no C++ extensions or full simulation are needed.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from runtime.control_loop import update_mpc_control_step
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

"""
Termination contract tests for strict last-waypoint completion behavior.
"""

from types import SimpleNamespace

import numpy as np
from core.path_completion import check_path_complete
from core.simulation_loop import SimulationLoop


class _FakeMPCController:
    def __init__(self, s_val: float, endpoint_error: float):
        self.s = s_val
        self._endpoint_error = endpoint_error

    def get_path_progress(self, _position: np.ndarray) -> dict[str, float]:
        return {"s": float(self.s), "endpoint_error": float(self._endpoint_error)}


class _FakeStateValidator:
    def __init__(self, result: bool | None = None, raise_error: bool = False):
        self._result = result
        self._raise_error = raise_error

    def check_reference_reached(
        self, _current_state: np.ndarray, _reference_state: np.ndarray
    ) -> bool:
        if self._raise_error:
            raise RuntimeError("validator failed")
        return bool(self._result)


class _FakeCompletionSimulation:
    def __init__(
        self,
        *,
        path_len: float = 2.0,
        s_val: float = 2.0,
        endpoint_error: float = 0.05,
        position_tolerance: float = 0.1,
        validator: _FakeStateValidator | None = None,
    ):
        self.mpc_controller = _FakeMPCController(
            s_val=s_val, endpoint_error=endpoint_error
        )
        self.satellite = SimpleNamespace(position=np.array([2.0, 0.0, 0.0]))
        self.position_tolerance = position_tolerance
        self.state_validator = validator
        self.reference_state = np.zeros(13, dtype=float)
        self._path_len = path_len

    def _get_mission_path_length(self, compute_if_missing: bool = True) -> float:
        _ = compute_if_missing
        return float(self._path_len)

    def _get_mission_path_waypoints(self) -> list[tuple[float, float, float]]:
        return [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]

    def get_current_state(self) -> np.ndarray:
        state = np.zeros(13, dtype=float)
        state[0:3] = np.array([2.0, 0.0, 0.0], dtype=float)
        state[3] = 1.0  # Identity quaternion [w, x, y, z]
        return state


class _FakeLoopSimulation:
    def __init__(self, hold_s: float):
        self.simulation_config = SimpleNamespace(
            mission_state=SimpleNamespace(path_hold_end=hold_s)
        )
        self.trajectory_endpoint_reached_time = None
        self.simulation_time = 0.0
        self.is_running = True
        self._path_complete = False
        self.summary_calls = 0
        self.position_tolerance = 0.1
        self.reference_state = np.zeros(13, dtype=float)
        self._path_len = 2.0
        self.satellite = SimpleNamespace(position=np.array([2.0, 0.0, 0.0]))
        self.mpc_controller = _FakeMPCController(s_val=2.0, endpoint_error=0.05)
        self.state_validator = _ToggleValidator(self)

    def print_performance_summary(self) -> None:
        self.summary_calls += 1

    def _get_mission_path_length(self, compute_if_missing: bool = True) -> float:
        _ = compute_if_missing
        return float(self._path_len)

    def _get_mission_path_waypoints(self) -> list[tuple[float, float, float]]:
        return [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]

    def get_current_state(self) -> np.ndarray:
        state = np.zeros(13, dtype=float)
        state[0:3] = np.array([2.0, 0.0, 0.0], dtype=float)
        state[3] = 1.0  # Identity quaternion [w, x, y, z]
        return state


class _ToggleValidator:
    def __init__(self, sim: _FakeLoopSimulation):
        self._sim = sim

    def check_within_tolerances(
        self, _current_state: np.ndarray, _reference_state: np.ndarray
    ) -> dict[str, bool]:
        ok = bool(self._sim._path_complete)
        return {
            "position": ok,
            "angle": ok,
            "velocity": ok,
            "angular_velocity": ok,
        }


def test_strict_completion_rejects_position_only_pass():
    """Position-only pass should not complete when validator says terminal state is not met."""
    sim = _FakeCompletionSimulation(
        validator=_FakeStateValidator(result=False),
        endpoint_error=0.05,
        s_val=2.0,
    )
    assert check_path_complete(sim) is False


def test_strict_completion_accepts_all_thresholds():
    """Completion should succeed when path progress and state validator are both satisfied."""
    sim = _FakeCompletionSimulation(
        validator=_FakeStateValidator(result=True),
        endpoint_error=0.05,
        s_val=2.0,
    )
    assert check_path_complete(sim) is True


def test_completion_falls_back_to_position_if_validator_unavailable():
    """When validator is unavailable/fails, completion should use position fallback."""
    sim = _FakeCompletionSimulation(
        validator=_FakeStateValidator(result=False, raise_error=True),
        endpoint_error=0.05,
        s_val=2.0,
    )
    assert check_path_complete(sim) is True


def test_hold_timer_requires_continuous_10s_window():
    """Hold timer should reset on threshold break and terminate after continuous hold."""
    sim = _FakeLoopSimulation(hold_s=10.0)
    loop = SimulationLoop(sim)

    sim._path_complete = True
    sim.simulation_time = 1.0
    assert loop._check_path_following_completion() is False
    assert sim.trajectory_endpoint_reached_time == 1.0

    sim.simulation_time = 7.0
    assert loop._check_path_following_completion() is False
    assert sim.is_running is True

    # Break completion; timer must reset.
    sim._path_complete = False
    sim.simulation_time = 7.1
    assert loop._check_path_following_completion() is False
    assert sim.trajectory_endpoint_reached_time is None

    # Start a fresh hold window and complete it.
    sim._path_complete = True
    sim.simulation_time = 20.0
    assert loop._check_path_following_completion() is False
    assert sim.trajectory_endpoint_reached_time == 20.0

    sim.simulation_time = 30.0
    assert loop._check_path_following_completion() is True
    assert sim.is_running is False
    assert sim.summary_calls == 1

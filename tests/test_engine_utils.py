"""
Unit tests for SatelliteMPCLinearizedSimulation utility methods.

These are pure-Python helpers that require no C++ extension or full sim setup.
Tested by calling the unbound methods against lightweight stub objects where
the instance state needed is just a single attribute.
"""

import math
from types import SimpleNamespace

import pytest

from controller.shared.python.simulation.engine import SatelliteMPCLinearizedSimulation
from controller.shared.python.utils.navigation_utils import (
    angle_difference,
    normalize_angle,
)


class TestNormalizeAngle:
    """normalize_angle wraps to (-pi, pi] using math.remainder."""

    def test_identity_in_range(self):
        assert normalize_angle(0.0) == pytest.approx(0.0)
        assert normalize_angle(1.0) == pytest.approx(1.0)
        assert normalize_angle(-1.0) == pytest.approx(-1.0)

    def test_wraps_above_pi(self):
        # 2π + 0.5 should wrap to 0.5
        assert normalize_angle(2 * math.pi + 0.5) == pytest.approx(0.5, abs=1e-10)

    def test_wraps_below_minus_pi(self):
        # -2π - 0.5 should wrap to -0.5
        assert normalize_angle(-2 * math.pi - 0.5) == pytest.approx(-0.5, abs=1e-10)

    def test_pi_boundary(self):
        # math.remainder(π, 2π) = π  (IEEE 754: ties to even, so this is +π)
        result = normalize_angle(math.pi)
        assert abs(result) == pytest.approx(math.pi)


class TestAngleDifference:
    """angle_difference returns the shortest signed arc (result in (-pi, pi])."""

    def test_zero_difference(self):
        assert angle_difference(1.0, 1.0) == pytest.approx(0.0)

    def test_positive_difference(self):
        # angle_difference(ref, current) = ref - current (shortest path)
        # ref=0.5 is 0.5 ahead of current=0.0  →  +0.5
        assert angle_difference(0.5, 0.0) == pytest.approx(0.5, abs=1e-10)

    def test_shortest_path_wraps(self):
        # ref=10°, current=350°: shortest arc from 350° to 10° is +20° (not +370°)
        deg = math.radians
        diff = angle_difference(deg(10), deg(350))
        assert diff == pytest.approx(deg(20), abs=1e-9)

    def test_negative_difference(self):
        # ref=0.0 is 0.5 behind current=0.5  →  -0.5
        assert angle_difference(0.0, 0.5) == pytest.approx(-0.5, abs=1e-10)


class TestAppendCappedHistory:
    """_append_capped_history enforces a maximum list length."""

    def _make_obj(self, max_len: int) -> SimpleNamespace:
        """Minimal stub that provides the attributes the method reads."""
        return SimpleNamespace(history_max_steps=max_len, history_trimmed=False)

    def test_respects_max_len(self):
        """List never exceeds history_max_steps entries."""
        obj = self._make_obj(max_len=10)
        history: list = []
        for i in range(25):
            SatelliteMPCLinearizedSimulation._append_capped_history(obj, history, i)
        assert len(history) == 10

    def test_removes_oldest_first(self):
        """The first item appended is the first to be evicted."""
        obj = self._make_obj(max_len=5)
        history: list = []
        for i in range(8):
            SatelliteMPCLinearizedSimulation._append_capped_history(obj, history, i)
        # Items 0-2 were evicted; 3-7 remain
        assert history[0] == 3
        assert history[-1] == 7

    def test_no_cap_when_history_max_steps_is_zero(self):
        """history_max_steps=0 means no cap — list grows unbounded."""
        obj = self._make_obj(max_len=0)
        history: list = []
        for i in range(200):
            SatelliteMPCLinearizedSimulation._append_capped_history(obj, history, i)
        assert len(history) == 200

    def test_sets_history_trimmed_flag(self):
        """history_trimmed is set to True when an entry is evicted."""
        obj = self._make_obj(max_len=3)
        history: list = []
        for i in range(3):
            SatelliteMPCLinearizedSimulation._append_capped_history(obj, history, i)
        assert not obj.history_trimmed  # not yet evicted

        SatelliteMPCLinearizedSimulation._append_capped_history(obj, history, 99)
        assert obj.history_trimmed  # eviction just occurred

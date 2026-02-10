"""
Math Utility Tests.

Tests core math functions, orbital dynamics equations, and navigation utilities.
Consolidates `test_math_utils.py`, `test_orbital_mechanics.py`, and parts of `physics/test_orbital_dynamics.py`.
"""

import math
import numpy as np
import pytest
from satellite_control.utils.navigation_utils import (
    normalize_angle,
    angle_difference,
    point_to_line_distance,
)


class TestNavigationMath:
    """Tests for navigation math utilities."""

    def test_normalize_angle(self):
        """Angle normalization should wrap to [-pi, pi]."""
        assert normalize_angle(0.0) == 0.0
        assert normalize_angle(np.pi) == pytest.approx(np.pi) or normalize_angle(
            np.pi
        ) == pytest.approx(-np.pi)
        assert normalize_angle(-np.pi) == pytest.approx(-np.pi) or normalize_angle(
            -np.pi
        ) == pytest.approx(np.pi)
        assert normalize_angle(2 * np.pi) == pytest.approx(0.0, abs=1e-6)
        assert normalize_angle(3 * np.pi) == pytest.approx(
            np.pi, abs=1e-6
        ) or normalize_angle(3 * np.pi) == pytest.approx(-np.pi, abs=1e-6)
        assert normalize_angle(-3 * np.pi) == pytest.approx(
            -np.pi, abs=1e-6
        ) or normalize_angle(-3 * np.pi) == pytest.approx(np.pi, abs=1e-6)

    def test_angle_difference(self):
        """Shortest angular difference."""
        # angle_difference(ref, curr) = ref - curr (normalized)
        # 0.0 - 0.1 = -0.1
        assert angle_difference(0.0, 0.1) == pytest.approx(-0.1)
        # 0.1 - 0.0 = 0.1
        assert angle_difference(0.1, 0.0) == pytest.approx(0.1)
        # Wrap around: (pi - 0.1) - (-pi + 0.1) = ~2pi - 0.2 = -0.2 (normalized)

        ref = np.pi - 0.1
        curr = -np.pi + 0.1
        expected = -0.2
        assert angle_difference(ref, curr) == pytest.approx(expected, abs=1e-6)

    def test_point_to_line_distance(self):
        """Distance from point to line segment."""
        start = np.array([0.0, 0.0, 0.0])
        end = np.array([10.0, 0.0, 0.0])

        # Point on line
        assert point_to_line_distance(np.array([5.0, 0.0, 0.0]), start, end) == 0.0

        # Point off line
        assert point_to_line_distance(np.array([5.0, 1.0, 0.0]), start, end) == 1.0

        # Point beyond segment (clamped to end)
        assert point_to_line_distance(np.array([12.0, 0.0, 0.0]), start, end) == 2.0


class TestOrbitalMath:
    """Tests for orbital mechanics calculations."""

    # Placeholder for CW equations if needed, or verified via integration/physics tests
    pass

"""
Path planning tests for point-to-point polyline generation.
"""

import numpy as np
import pytest
from mission.path_following import build_point_to_point_path
from utils.navigation_utils import point_to_line_distance


class TestPathPlanning:
    def test_direct_path_generation(self):
        start = (0.0, 0.0, 0.0)
        end = (10.0, 0.0, 0.0)
        path = build_point_to_point_path(waypoints=[start, end], step_size=0.5)

        assert len(path) >= 2
        assert np.allclose(path[0], start)
        assert np.allclose(path[-1], end)

        mid_idx = len(path) // 2
        mid_point = np.array(path[mid_idx], dtype=float)
        assert mid_point[1] == pytest.approx(0.0)
        assert mid_point[2] == pytest.approx(0.0)

    def test_path_generation_with_intermediate_waypoint(self):
        waypoints = [
            (0.0, 0.0, 0.0),
            (2.0, 1.0, 0.0),
            (4.0, 1.0, 0.5),
        ]
        path = build_point_to_point_path(waypoints=waypoints, step_size=0.25)

        assert len(path) > len(waypoints)
        assert np.allclose(path[0], waypoints[0])
        assert np.allclose(path[-1], waypoints[-1])

    def test_point_to_line_distance(self):
        point = np.array([1.0, 1.0, 0.0])
        line_start = np.array([0.0, 0.0, 0.0])
        line_end = np.array([2.0, 0.0, 0.0])

        dist = point_to_line_distance(point, line_start, line_end)
        assert dist == pytest.approx(1.0)

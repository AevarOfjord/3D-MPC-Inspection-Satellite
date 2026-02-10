"""
Path Planning and Obstacle Avoidance Tests.

Tests path generation, waypoint navigation, and safety checks around obstacles.
Consolidates `test_path_following_obstacles.py` and parts of `test_navigation_utils.py`.
"""

import numpy as np
import pytest
from satellite_control.mission.path_following import build_point_to_point_path
from satellite_control.utils.navigation_utils import calculate_safe_path_to_waypoint


class TestPathPlanning:
    """Tests for path generation and obstacle avoidance."""

    def test_direct_path_no_obstacles(self):
        """Should generate a straight line when space is clear."""
        start = (0.0, 0.0, 0.0)
        end = (10.0, 0.0, 0.0)

        # Function expects explicit obstacles list or None
        # Signature: waypoints (list of points), obstacles, ...
        path = build_point_to_point_path(waypoints=[start, end], obstacles=[])

        # Should be just start and end (or sampled points on line)
        assert len(path) >= 2
        # Check midpoint lies on line
        mid_idx = len(path) // 2
        mid_point = np.array(path[mid_idx])
        assert mid_point[1] == pytest.approx(0.0)
        assert mid_point[2] == pytest.approx(0.0)

    def test_obstacle_avoidance_arc(self):
        """Should detour around an obstacle blocking the path."""
        start = [-5.0, 0.0, 0.0]
        end = [5.0, 0.0, 0.0]

        # Obstacle at origin, radius 1.0 (x, y, z, r) or (x, y, r)
        obstacles = [(0.0, 0.0, 0.0, 1.0)]

        path = build_point_to_point_path(
            waypoints=[start, end],
            obstacles=obstacles,
            step_size=0.5,  # Larger step to see deviation clearly with fewer points
        )

        # Check that no point is inside the obstacle (with safety margin)
        obstacle_pos = np.array([0.0, 0.0, 0.0])
        radius = 1.0

        min_dist = min(np.linalg.norm(np.array(p) - obstacle_pos) for p in path)
        assert min_dist >= radius, (
            f"Path intersects obstacle! Min dist {min_dist} < radius {radius}"
        )

        # Check that path actually deviates (y or z should be non-zero)
        max_deviation = max(np.linalg.norm(np.array(p)[1:]) for p in path)
        assert max_deviation > 0.5, "Path did not deviate enough to avoid obstacle"

    def test_safe_path_calculation(self):
        """Test the low-level safe path calculator."""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 0.0, 0.0])
        # Function expects (x, y, z, r) tuples in list
        obstacles = [(5.0, 0.0, 0.0, 2.0)]

        # Signature: start, end, obstacles, safety_radius
        waypoints = calculate_safe_path_to_waypoint(
            start_pos=start, end_pos=goal, all_obstacles=obstacles, safety_radius=0.5
        )

        # Should have intermediate waypoints
        assert len(waypoints) > 2

        # Check intermediate points area safe
        for wp in waypoints:
            dist = np.linalg.norm(np.array(wp) - np.array([5.0, 0.0, 0.0]))
            # Waypoints shouldn't be inside obstacle + safety
            if not np.allclose(wp, start) and not np.allclose(wp, goal):
                assert dist >= 2.0 + 0.5

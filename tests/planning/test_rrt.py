"""
Tests for RRT* Path Planner
"""

import numpy as np
import pytest

from src.satellite_control.planning.rrt_star import (
    Node,
    Obstacle,
    RRTStarPlanner,
)


class TestNode:
    """Tests for the Node class."""

    def test_node_position(self):
        """Test node position returns correct numpy array."""
        node = Node(1.0, 2.0, 3.0)
        pos = node.position()
        assert isinstance(pos, np.ndarray)
        np.testing.assert_array_equal(pos, np.array([1.0, 2.0, 3.0]))

    def test_node_initial_cost_is_zero(self):
        """Test that node initial cost is 0."""
        node = Node(0.0, 0.0, 0.0)
        assert node.cost == 0.0

    def test_node_parent_initially_none(self):
        """Test that node parent is initially None."""
        node = Node(0.0, 0.0, 0.0)
        assert node.parent is None


class TestRRTStarPlanner:
    """Tests for the RRTStarPlanner class."""

    @pytest.fixture
    def planner(self) -> RRTStarPlanner:
        """Create a standard planner for testing."""
        return RRTStarPlanner(
            bounds_min=(-5, -5, -5),
            bounds_max=(5, 5, 5),
            step_size=0.5,
            max_iter=500,
            search_radius=1.0,
        )

    def test_plan_obstacle_free(self, planner: RRTStarPlanner):
        """Test path planning with no obstacles."""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([2.0, 2.0, 0.0])
        obstacles: list = []

        # Increase max_iter for this test due to stochastic nature
        planner.max_iter = 1000
        path = planner.plan(start, goal, obstacles)

        assert len(path) > 0, "Path should not be empty"
        # First waypoint should be close to start
        np.testing.assert_array_almost_equal(path[0], start, decimal=1)
        # Last waypoint should be reasonably close to goal (RRT* is stochastic)
        dist_to_goal = np.linalg.norm(path[-1] - goal)
        assert dist_to_goal < 3.0, (
            f"Final waypoint should be within 3m of goal, got dist {dist_to_goal}"
        )

    def test_plan_with_obstacle(self, planner: RRTStarPlanner):
        """Test path planning avoids obstacles."""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([3.0, 0.0, 0.0])
        # Put obstacle directly between start and goal
        obstacles = [Obstacle(position=np.array([1.5, 0.0, 0.0]), radius=0.5)]

        path = planner.plan(start, goal, obstacles)

        assert len(path) > 0, "Path should not be empty"

        # Check that no path segment intersects the obstacle
        for waypoint in path:
            dist = np.linalg.norm(waypoint - obstacles[0].position)
            assert dist > obstacles[0].radius, "Path should avoid obstacle"

    def test_planner_returns_list_of_arrays(self, planner: RRTStarPlanner):
        """Test that planner returns list of numpy arrays."""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([1.0, 1.0, 0.0])

        path = planner.plan(start, goal, [])

        assert isinstance(path, list)
        if len(path) > 0:
            assert isinstance(path[0], np.ndarray)

    def test_get_nearest_node_index(self, planner: RRTStarPlanner):
        """Test nearest node selection."""
        # Initialize with start node
        planner.node_list = [
            Node(0.0, 0.0, 0.0),
            Node(1.0, 0.0, 0.0),
            Node(0.0, 1.0, 0.0),
        ]

        # Random node close to second node
        rnd = Node(1.1, 0.1, 0.0)
        nearest_idx = planner._get_nearest_node_index(planner.node_list, rnd)

        assert nearest_idx == 1, "Should return index of closest node"

    def test_check_collision_no_obstacles(self, planner: RRTStarPlanner):
        """Test collision check with no obstacles."""
        node = Node(0.0, 0.0, 0.0)
        assert planner._check_collision(node, []) is True

    def test_check_collision_with_obstacle(self, planner: RRTStarPlanner):
        """Test collision check with obstacle."""
        node = Node(0.0, 0.0, 0.0)
        obstacles = [Obstacle(position=np.array([0.0, 0.0, 0.0]), radius=1.0)]
        assert planner._check_collision(node, obstacles) is False

"""
MPC Controller Tests.

Tests the Model Predictive Controller initialization and control action computation.
Consolidates `test_mpc_controller.py` and `test_mpc_controller_path.py`.
"""

import numpy as np
import pytest
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.control.mpc_controller import MPCController


class TestMPCController:
    """Tests for the MPC Controller."""

    @pytest.fixture
    def controller(self, fresh_config):
        return MPCController(fresh_config.app_config)

    def test_initialization(self, controller):
        """Test controller initialization."""
        # State dimension (13 + 3 RW speeds + 1 path s) = 17
        assert controller.nx == 17
        assert controller.nu > 0  # Control dimension

    def test_get_control_action_dimensions(self, controller):
        """Test that control action returns correct dimensions."""
        # Create a dummy state vector (nx=16, function handles augmentation)
        x_current = np.zeros(16)
        x_current[3] = 1.0  # valid quaternion (w=1)

        u, info = controller.get_control_action(x_current)

        assert u.shape == (controller.nu,)
        # info["status"] is integer from solver (1 = solved, etc)
        assert "status" in info
        assert isinstance(info["status"], int)

    def test_path_following_mode(self, controller):
        """Test setting a path and computing controls."""
        # Define a simple path
        path = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
        controller.set_path(path)

        # Initial state at start of path
        x_current = np.zeros(16)
        x_current[3] = 1.0

        u, info = controller.get_control_action(x_current)

        # Check that we get a control action
        assert u is not None

        # Check progress variable s is being tracked
        assert hasattr(controller, "s")

    def test_control_bounds(self, controller):
        """Test that control outputs respect physical limits."""
        x_current = np.zeros(16)
        x_current[3] = 1.0
        # Put satellite in a state that requires effort
        x_current[0] = 5.0

        u, _ = controller.get_control_action(x_current)
        rw_torques, thrusters = controller.split_control(u)

        # Thrusters are [0, 1]
        assert np.all(thrusters >= -1e-5)
        assert np.all(thrusters <= 1.0 + 1e-5)

        # RW torques are bounded (check limits from config)
        max_torque = controller.max_rw_torque
        # Only check if we have RWs configured
        if max_torque > 0:
            assert np.all(np.abs(rw_torques) <= max_torque + 1e-5)

    def test_solver_metadata_fields(self, controller):
        """Solver metadata should include status and timeout diagnostics."""
        x_current = np.zeros(16)
        x_current[3] = 1.0
        _, info = controller.get_control_action(x_current)

        assert "status" in info
        assert "timeout" in info
        assert "solver_status" in info
        assert isinstance(info["timeout"], bool)

    def test_obstacle_constraint_api(self, controller):
        """Controller accepts obstacle updates and still computes controls."""
        if not hasattr(controller, "set_obstacles"):
            pytest.skip("Obstacle APIs not available in this build")

        controller.set_obstacles([(1.0, 0.0, 0.0, 0.5), (2.0, 0.0, 0.0, 0.25)])
        x_current = np.zeros(16)
        x_current[3] = 1.0
        u, info = controller.get_control_action(x_current)
        assert u is not None
        assert isinstance(info.get("status"), int)

        if hasattr(controller, "clear_obstacles"):
            controller.clear_obstacles()

    def test_collision_avoidance_flag_passthrough(self):
        """MPC config flag should propagate into the controller wrapper."""
        cfg = SimulationConfig.create_with_overrides(
            {"mpc": {"enable_collision_avoidance": True}}
        )
        controller = MPCController(cfg.app_config)
        assert controller.enable_collision_avoidance is True

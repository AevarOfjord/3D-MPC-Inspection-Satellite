"""
MPC Controller Tests.

Tests the Model Predictive Controller initialization and control action computation.
Consolidates `test_mpc_controller.py` and `test_mpc_controller_path.py`.
"""

import numpy as np
import pytest
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.control.mpc_controller import MPCController
from satellite_control.core.mpc_runner import MPCRunner


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

    def test_cpp_binding_exposes_v6_mode_profile_fields(self):
        """C++ binding should expose V6 mode profile params and runtime mode setter."""
        from satellite_control.cpp import _cpp_mpc

        params = _cpp_mpc.MPCParams()
        assert hasattr(params, "recover_contour_scale")
        assert hasattr(params, "settle_progress_scale")
        assert hasattr(params, "hold_smoothness_scale")
        assert hasattr(_cpp_mpc.MPCControllerCpp, "set_runtime_mode")

    def test_v6_core_is_default_and_runtime_mode_is_settable(self):
        """V6 core should be active and runtime mode updates should be accepted."""
        cfg = SimulationConfig.create_with_overrides({})
        controller = MPCController(cfg.app_config)
        assert controller.controller_core == "v6"
        controller.set_runtime_mode("RECOVER")
        assert controller._runtime_mode == "RECOVER"


class _DummyMPCController:
    def __init__(self, next_action: np.ndarray):
        self.num_thrusters = int(next_action.size)
        self.num_rw_axes = 0
        self._next_action = np.array(next_action, dtype=np.float64)
        self._next_info = {"status": 1}
        self.runtime_modes: list[str] = []

    def set_next_action(self, action: np.ndarray) -> None:
        self._next_action = np.array(action, dtype=np.float64)

    def set_next_info(self, info: dict[str, object]) -> None:
        payload = {"status": 1}
        payload.update(info)
        self._next_info = payload

    def set_runtime_mode(self, mode: str) -> None:
        self.runtime_modes.append(str(mode))

    def get_control_action(self, x_current, previous_thrusters=None):
        return self._next_action.copy(), dict(self._next_info)


class TestMPCRunnerHysteresis:
    def test_thruster_hysteresis_toggle_behavior(self):
        cfg = SimulationConfig.create_with_overrides(
            {
                "mpc": {
                    "enable_thruster_hysteresis": True,
                    "thruster_hysteresis_on": 0.02,
                    "thruster_hysteresis_off": 0.01,
                }
            }
        ).app_config
        dummy = _DummyMPCController(np.array([0.03, 0.03, 0.0], dtype=np.float64))
        runner = MPCRunner(dummy, config=cfg)
        state = np.zeros(16, dtype=np.float64)

        thrusters, _, _, _, _ = runner.compute_control_action(
            state, runner.get_previous_thrusters()
        )
        assert np.allclose(thrusters, np.array([0.03, 0.03, 0.0]))

        dummy.set_next_action(np.array([0.015, 0.009, 0.0], dtype=np.float64))
        thrusters, _, _, _, _ = runner.compute_control_action(
            state, runner.get_previous_thrusters()
        )
        assert np.allclose(thrusters, np.array([0.015, 0.0, 0.0]))

        dummy.set_next_action(np.array([0.011, 0.015, 0.0], dtype=np.float64))
        thrusters, _, _, _, _ = runner.compute_control_action(
            state, runner.get_previous_thrusters()
        )
        assert np.allclose(thrusters, np.array([0.011, 0.0, 0.0]))

    def test_thruster_hysteresis_disabled_passthrough(self):
        cfg = SimulationConfig.create_with_overrides(
            {"mpc": {"enable_thruster_hysteresis": False}}
        ).app_config
        dummy = _DummyMPCController(np.array([0.015, 0.0], dtype=np.float64))
        runner = MPCRunner(dummy, config=cfg)
        state = np.zeros(16, dtype=np.float64)

        thrusters, _, _, _, _ = runner.compute_control_action(
            state, runner.get_previous_thrusters()
        )
        assert np.allclose(thrusters, np.array([0.015, 0.0]))

    def test_terminal_settling_bypasses_hysteresis(self):
        cfg = SimulationConfig.create_with_overrides(
            {
                "mpc": {
                    "enable_thruster_hysteresis": True,
                    "thruster_hysteresis_on": 0.02,
                    "thruster_hysteresis_off": 0.01,
                }
            }
        ).app_config
        dummy = _DummyMPCController(np.array([0.03, 0.0], dtype=np.float64))
        runner = MPCRunner(dummy, config=cfg)
        state = np.zeros(16, dtype=np.float64)

        # First step turns channel on.
        thrusters, _, _, _, _ = runner.compute_control_action(
            state, runner.get_previous_thrusters()
        )
        assert np.allclose(thrusters, np.array([0.03, 0.0]))

        # Near endpoint: allow small command below "on" threshold for fine settling.
        dummy.set_next_action(np.array([0.015, 0.0], dtype=np.float64))
        dummy.set_next_info({"path_endpoint_error": 0.05})
        thrusters, _, _, _, _ = runner.compute_control_action(
            state, runner.get_previous_thrusters()
        )
        assert np.allclose(thrusters, np.array([0.015, 0.0]))

    def test_runtime_mode_forwarded_to_controller(self):
        cfg = SimulationConfig.create_with_overrides({}).app_config
        dummy = _DummyMPCController(np.array([0.02], dtype=np.float64))
        runner = MPCRunner(dummy, config=cfg)
        state = np.zeros(16, dtype=np.float64)

        class _ModeState:
            current_mode = "RECOVER"

        runner.set_mode_state(_ModeState())
        runner.compute_control_action(state, runner.get_previous_thrusters())
        assert dummy.runtime_modes
        assert dummy.runtime_modes[-1] == "RECOVER"

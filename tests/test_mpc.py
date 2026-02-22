"""
MPC Controller Tests.

Tests the Model Predictive Controller initialization and control action computation.
Consolidates `test_mpc_controller.py` and `test_mpc_controller_path.py`.
"""

import numpy as np
import pytest
from config.simulation_config import SimulationConfig
from control.mpc_controller import MPCController
from core.mpc_runner import MPCRunner

import cpp


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

    def test_cpp_binding_exposes_v6_mode_profile_fields(self):
        """C++ binding should expose V6 mode profile params and runtime mode setter."""
        from cpp import _cpp_mpc

        params = _cpp_mpc.MPCParams()
        assert hasattr(params, "recover_contour_scale")
        assert hasattr(params, "settle_progress_scale")
        assert hasattr(params, "hold_smoothness_scale")
        assert hasattr(params, "terminal_cost_profile")
        assert hasattr(params, "robustness_mode")
        assert hasattr(params, "constraint_tightening_scale")
        assert not hasattr(params, "coast_pos_tolerance")
        assert not hasattr(params, "coast_vel_tolerance")
        assert not hasattr(params, "coast_min_speed")
        assert not hasattr(params, "progress_taper_distance")
        assert not hasattr(params, "progress_slowdown_distance")
        assert hasattr(_cpp_mpc.MPCControllerCpp, "set_runtime_mode")

    def test_v6_core_is_default_and_runtime_mode_is_settable(self):
        """V6 core should be active and runtime mode updates should be accepted."""
        cfg = SimulationConfig.create_with_overrides({})
        controller = MPCController(cfg.app_config)
        assert controller.controller_core == "v6"
        controller.set_runtime_mode("RECOVER")
        assert controller._runtime_mode == "RECOVER"

    def test_reference_quaternion_keeps_z_locked_to_scan_axis(self, controller):
        """Scan context should lock +Z to configured axis while keeping +X path-forward."""
        controller.set_path([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
        controller.set_scan_attitude_context(
            center=None,
            axis=(0.0, 1.0, 0.0),
            direction="CW",
        )

        _p_ref, t_ref, q_ref = controller.get_path_reference_state(
            s_query=0.5,
            q_current=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )

        def _rotate(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
            w, x, y, z = q_wxyz
            q_vec = np.array([x, y, z], dtype=float)
            uv = np.cross(q_vec, v)
            uuv = np.cross(q_vec, uv)
            return v + 2.0 * (w * uv + uuv)

        q_ref = np.array(q_ref, dtype=float)
        q_ref /= np.linalg.norm(q_ref)
        x_axis = _rotate(q_ref, np.array([1.0, 0.0, 0.0], dtype=float))
        z_axis = _rotate(q_ref, np.array([0.0, 0.0, 1.0], dtype=float))

        assert np.dot(z_axis, np.array([0.0, 1.0, 0.0], dtype=float)) > 0.999
        assert np.dot(x_axis, np.array(t_ref, dtype=float)) > 0.999

    def test_tangent_uses_next_segment_at_interior_waypoint(self, controller):
        """At exact interior waypoint, heading should face next waypoint segment."""
        controller.set_path([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)])

        _p_ref, t_ref, _q_ref = controller.get_path_reference_state(
            s_query=1.0,
            q_current=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )
        t = np.array(t_ref, dtype=float)
        t /= np.linalg.norm(t)
        assert np.dot(t, np.array([0.0, 1.0, 0.0], dtype=float)) > 0.999

    def test_cpp_loader_uses_only_canonical_namespace(self, monkeypatch):
        """cpp extension loader should only probe the canonical cpp.* namespace."""
        attempted: list[str] = []

        def _fake_import(module_name: str):
            attempted.append(module_name)
            raise ImportError("not found")

        monkeypatch.setattr(cpp.importlib, "import_module", _fake_import)
        assert cpp._load_extension("_not_a_real_extension") is None
        assert attempted == ["cpp._not_a_real_extension"]

    def test_get_control_action_no_legacy_two_arg_fallback(self):
        """Wrapper should not retry legacy two-arg C++ get_control_action signature."""
        cfg = SimulationConfig.create_default()
        controller = MPCController(cfg.app_config)

        class _StrictCpp:
            def get_control_action(self, x_input):
                raise TypeError("signature mismatch")

        controller._cpp_controller = _StrictCpp()

        x_current = np.zeros(16, dtype=np.float64)
        x_current[3] = 1.0
        with pytest.raises(TypeError):
            controller.get_control_action(x_current)


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

    def split_control(self, control: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return np.zeros(0, dtype=np.float64), np.array(control, dtype=np.float64)


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

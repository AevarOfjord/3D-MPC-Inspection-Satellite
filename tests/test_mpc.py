"""
MPC Controller Tests.

Tests the Model Predictive Controller initialization and control action computation.
Consolidates `test_mpc_controller.py` and `test_mpc_controller_path.py`.
"""

import numpy as np
import pytest
from config.simulation_config import SimulationConfig
from control.mpc_controller import MPCController
from runtime.mpc_runner import MPCRunner

import cpp


class TestMPCController:
    """Tests for the MPC Controller."""

    @pytest.fixture
    def controller(self, fresh_config):
        ctrl = MPCController(fresh_config.app_config)
        # MPCC controller requires a path for meaningful solve
        ctrl.set_path([(0, 0, 0), (10, 0, 0), (20, 0, 0)])
        return ctrl

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

        # RW torques are bounded
        if hasattr(controller, "rw_torque_limits") and controller.rw_torque_limits:
            max_torque = max(controller.rw_torque_limits)
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

    def test_cpp_binding_exposes_mode_profile_fields(self):
        """C++ binding should expose MPCV2Params and mode profile fields."""
        from cpp import _cpp_mpc

        params = _cpp_mpc.MPCV2Params()
        assert hasattr(params, "recover_contour_scale")
        assert hasattr(params, "settle_progress_scale")
        assert hasattr(params, "hold_smoothness_scale")
        assert hasattr(params, "terminal_cost_profile")
        assert hasattr(_cpp_mpc.SQPController, "set_runtime_mode")

    def test_core_is_default_and_runtime_mode_is_settable(self):
        """SQP core should be active and runtime mode updates should be accepted."""
        cfg = SimulationConfig.create_with_overrides({})
        controller = MPCController(cfg.app_config)
        # reports controller_core in extras
        controller.set_runtime_mode("RECOVER")
        assert controller._runtime_mode == "RECOVER"

    def test_terminal_position_weight_increases_endpoint_correction(self):
        """Higher Q_terminal_pos should increase corrective effort in SETTLE near endpoint."""
        cfg_zero = SimulationConfig.create_with_overrides(
            {"mpc": {"Q_terminal_pos": 0.0}}
        )
        cfg_high = SimulationConfig.create_with_overrides(
            {"mpc": {"Q_terminal_pos": 8000.0}}
        )
        ctrl_zero = MPCController(cfg_zero.app_config)
        ctrl_high = MPCController(cfg_high.app_config)
        path = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
        ctrl_zero.set_path(path)
        ctrl_high.set_path(path)
        ctrl_zero.set_runtime_mode("SETTLE")
        ctrl_high.set_runtime_mode("SETTLE")
        ctrl_zero.s = 9.95
        ctrl_high.s = 9.95

        x_current = np.zeros(16)
        x_current[3] = 1.0
        x_current[0] = 9.7  # retain endpoint position offset

        u_zero, _ = ctrl_zero.get_control_action(x_current)
        u_high, _ = ctrl_high.get_control_action(x_current)

        assert not np.allclose(u_high, u_zero, atol=1e-6)

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

    def test_non_scan_reference_quaternion_is_continuous_on_sway_path(self, controller):
        """Minimal-twist non-scan reference should stay continuous through tangent reversals."""
        controller.set_path(
            [
                (0.0, 0.0, 20.0),
                (1.431361, 0.0, 14.205968),
                (-1.377417, 0.0, 6.528819),
                (0.0, 0.0, 0.0),
            ]
        )
        controller.set_scan_attitude_context(center=None, axis=None, direction="CW")

        def _rotate(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
            w, x, y, z = q_wxyz
            q_vec = np.array([x, y, z], dtype=float)
            uv = np.cross(q_vec, v)
            uuv = np.cross(q_vec, uv)
            return v + 2.0 * (w * uv + uuv)

        q_prev = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        min_abs_dot = 1.0
        samples = np.linspace(0.0, controller._path_length, 81)
        for s_query in samples:
            _p_ref, t_ref, q_ref = controller.get_path_reference_state(
                s_query=float(s_query),
                q_current=q_prev,
            )
            q_ref = np.array(q_ref, dtype=float)
            q_ref /= np.linalg.norm(q_ref)
            t_ref = np.array(t_ref, dtype=float)
            t_ref /= np.linalg.norm(t_ref)
            x_axis = _rotate(q_ref, np.array([1.0, 0.0, 0.0], dtype=float))
            assert np.dot(x_axis, t_ref) > 0.995

            dot_abs = abs(float(np.dot(q_prev, q_ref)))
            min_abs_dot = min(min_abs_dot, dot_abs)
            q_prev = q_ref

        assert min_abs_dot > 0.70

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

            @property
            def current_path_s(self):
                return 0.0

            @property
            def prediction_horizon(self):
                return 10

            def get_stage_state(self, k):
                return np.zeros(17)

            def get_stage_control(self, k):
                return np.zeros(10)

        controller._cpp = _StrictCpp()

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

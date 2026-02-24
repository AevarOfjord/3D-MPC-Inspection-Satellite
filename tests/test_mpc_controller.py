"""
Tests for MPC (CasADi + OSQP RTI-SQP) components.

Tests are structured bottom-up:
  1. CasADi symbolic dynamics accuracy
  2. CasADi Jacobian accuracy (finite-difference check)
  3. Cost function sanity
  4. C++ SQP controller (if built)
  5. Python wrapper end-to-end
"""

import pathlib
import sys

import numpy as np
import pytest

# Ensure src/ is importable
_SRC_DIR = str(pathlib.Path(__file__).resolve().parent.parent / "src" / "python")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

# --------------------------------------------------------------------------
# Skip markers
# --------------------------------------------------------------------------

try:
    import casadi  # noqa: F401

    HAS_CASADI = True
except ImportError:
    HAS_CASADI = False

skip_no_casadi = pytest.mark.skipif(not HAS_CASADI, reason="CasADi not installed")

try:
    from cpp import _cpp_mpc  # noqa: F401

    HAS_CPP_MPC = True
except ImportError:
    HAS_CPP_MPC = False

skip_no_cpp_mpc = pytest.mark.skipif(not HAS_CPP_MPC, reason="C++ MPC module not built")

# Default timestep for tests
DT = 0.05


def _make_params(dynamics_cls):
    """Build a realistic CasADi parameter vector for 6-thruster, 3-RW config."""
    return dynamics_cls.pack_params(
        mass=10.0,
        inertia_diag=np.array([0.15, 0.12, 0.18]),
        thruster_positions=[
            np.array([0.5, 0, 0]),
            np.array([-0.5, 0, 0]),
            np.array([0, 0.5, 0]),
            np.array([0, -0.5, 0]),
            np.array([0, 0, 0.5]),
            np.array([0, 0, -0.5]),
        ],
        thruster_directions=[
            np.array([1, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([0, 1, 0]),
            np.array([0, -1, 0]),
            np.array([0, 0, 1]),
            np.array([0, 0, -1]),
        ],
        thruster_forces=[1.0] * 6,
        rw_axes=[np.eye(3)[i] for i in range(3)],
        rw_torque_limits=[0.05] * 3,
        rw_inertias=[0.001] * 3,
        orbital_mu=3.986e14,
        orbital_radius=6.778e6,
    )


# ==========================================================================
# 1. CasADi Symbolic Dynamics
# ==========================================================================


@skip_no_casadi
class TestCasADiDynamics:
    """Test the CasADi symbolic dynamics model."""

    @pytest.fixture
    def dynamics(self):
        from control.codegen.satellite_dynamics import SatelliteDynamicsSymbolic

        return SatelliteDynamicsSymbolic(num_thrusters=6, num_rw=3)

    def test_state_dimension(self, dynamics):
        """f_discrete maps R^17 x R^10 x R^np x R^1 -> R^17."""
        f = dynamics.f_discrete
        assert f.n_in() == 4  # x, u, p, dt
        assert f.n_out() == 1
        nx = 17
        p = _make_params(dynamics.__class__)
        x = np.zeros(nx)
        x[3] = 1.0  # qw=1
        x[0] = 6778e3  # orbital position to avoid NaN
        u = np.zeros(10)
        result = np.array(f(x, u, p, DT)).ravel()
        assert result.shape == (nx,)

    def test_quaternion_normalisation(self, dynamics):
        """After integration, quaternion should be normalised."""
        f = dynamics.f_discrete
        p = _make_params(dynamics.__class__)
        x = np.zeros(17)
        x[3] = 1.0
        x[0] = 6778e3
        u = np.zeros(10)
        u[0] = 0.5  # RW torque

        x_next = np.array(f(x, u, p, DT)).ravel()
        q_norm = np.linalg.norm(x_next[3:7])
        assert abs(q_norm - 1.0) < 1e-8, f"Quaternion norm = {q_norm}"

    def test_zero_input_drift(self, dynamics):
        """Zero control, zero gravity -> position unchanged."""
        f = dynamics.f_discrete
        p = dynamics.__class__.pack_params(
            mass=10.0,
            inertia_diag=np.array([0.1, 0.1, 0.1]),
            thruster_positions=[np.zeros(3)] * 6,
            thruster_directions=[np.array([1, 0, 0])] * 6,
            thruster_forces=[1.0] * 6,
            rw_axes=[np.eye(3)[i] for i in range(3)],
            rw_torque_limits=[0.05] * 3,
            rw_inertias=[0.001] * 3,
            orbital_mu=0.0,
            orbital_radius=1e7,
        )
        x = np.zeros(17)
        x[3] = 1.0
        u = np.zeros(10)

        x_next = np.array(f(x, u, p, DT)).ravel()
        # Position should remain near zero (no velocity, no gravity)
        assert np.linalg.norm(x_next[:3]) < 1e-10

    def test_f_and_jacs_returns_correct_shapes(self, dynamics):
        """f_and_jacs should return (x_next, A, B)."""
        f_jac = dynamics.f_and_jacs
        nx, nu = 17, 10
        p = _make_params(dynamics.__class__)
        x = np.zeros(nx)
        x[3] = 1.0
        x[0] = 6778e3
        u = np.zeros(nu)

        result = f_jac(x, u, p, DT)
        x_next = np.array(result[0]).ravel()
        A = np.array(result[1])
        B = np.array(result[2])

        assert x_next.shape == (nx,)
        assert A.shape == (nx, nx)
        assert B.shape == (nx, nu)


# ==========================================================================
# 2. Jacobian Accuracy (finite-difference check)
# ==========================================================================


@skip_no_casadi
class TestJacobianAccuracy:
    """Verify CasADi AD Jacobians match finite-difference approximation."""

    @pytest.fixture
    def dynamics(self):
        from control.codegen.satellite_dynamics import SatelliteDynamicsSymbolic

        return SatelliteDynamicsSymbolic(num_thrusters=6, num_rw=3)

    @pytest.fixture
    def params(self, dynamics):
        return _make_params(dynamics.__class__)

    def _finite_diff_jac(self, f_func, x, u, p, dt, eps=1e-6):
        """Compute finite-difference Jacobian w.r.t. x and u."""
        nx = len(x)
        nu = len(u)
        f0 = np.array(f_func(x, u, p, dt)).ravel()
        nf = len(f0)

        A_fd = np.zeros((nf, nx))
        for i in range(nx):
            x_p = x.copy()
            x_p[i] += eps
            f_p = np.array(f_func(x_p, u, p, dt)).ravel()
            A_fd[:, i] = (f_p - f0) / eps

        B_fd = np.zeros((nf, nu))
        for i in range(nu):
            u_p = u.copy()
            u_p[i] += eps
            f_p = np.array(f_func(x, u_p, p, dt)).ravel()
            B_fd[:, i] = (f_p - f0) / eps

        return A_fd, B_fd

    def test_jacobian_A_accuracy(self, dynamics, params):
        """CasADi df/dx should match finite-difference within tolerance."""
        f_disc = dynamics.f_discrete
        f_jac = dynamics.f_and_jacs

        rng = np.random.default_rng(42)
        x = rng.uniform(-0.1, 0.1, 17)
        x[0] = 6778e3  # orbital position to avoid NaN in gravity
        x[3:7] = [1, 0, 0, 0]  # valid quaternion
        u = rng.uniform(-0.1, 0.1, 10)
        u[3:9] = np.clip(u[3:9], 0, 1)  # thrusters [0,1]

        result = f_jac(x, u, params, DT)
        A_casadi = np.array(result[1])

        A_fd, _ = self._finite_diff_jac(f_disc, x, u, params, DT, eps=1e-5)

        err = np.abs(A_casadi - A_fd)
        max_err = err.max()
        assert max_err < 1e-2, f"Max df/dx error: {max_err:.2e}"

    def test_jacobian_B_accuracy(self, dynamics, params):
        """CasADi df/du should match finite-difference within tolerance."""
        f_disc = dynamics.f_discrete
        f_jac = dynamics.f_and_jacs

        rng = np.random.default_rng(42)
        x = rng.uniform(-0.1, 0.1, 17)
        x[0] = 6778e3  # orbital position to avoid NaN in gravity
        x[3:7] = [1, 0, 0, 0]
        u = rng.uniform(-0.1, 0.1, 10)
        u[3:9] = np.clip(u[3:9], 0, 1)

        result = f_jac(x, u, params, DT)
        B_casadi = np.array(result[2])

        _, B_fd = self._finite_diff_jac(f_disc, x, u, params, DT, eps=1e-5)

        err = np.abs(B_casadi - B_fd)
        max_err = err.max()
        assert max_err < 1e-2, f"Max df/du error: {max_err:.2e}"


# ==========================================================================
# 3. Cost Functions
# ==========================================================================


@skip_no_casadi
class TestCostFunctions:
    """Test MPCC cost function components."""

    def test_contouring_cost_zero_at_path(self):
        """Contouring cost = 0 when satellite is on the path."""
        import casadi as ca
        from control.codegen.cost_functions import contouring_cost

        pos = ca.DM([1.0, 0.0, 0.0])
        p_ref = ca.DM([1.0, 0.0, 0.0])
        t_ref = ca.DM([0.0, 1.0, 0.0])

        cost = float(contouring_cost(pos, p_ref, t_ref, 1.0))
        assert abs(cost) < 1e-12

    def test_contouring_cost_positive_off_path(self):
        """Contouring cost > 0 when satellite is off the path."""
        import casadi as ca
        from control.codegen.cost_functions import contouring_cost

        pos = ca.DM([1.0, 0.0, 0.5])  # 0.5m off path
        p_ref = ca.DM([1.0, 0.0, 0.0])
        t_ref = ca.DM([0.0, 1.0, 0.0])

        cost = float(contouring_cost(pos, p_ref, t_ref, 1.0))
        assert cost > 0

    def test_lag_cost_zero_at_ref(self):
        """Lag cost = 0 when satellite is at path reference point."""
        import casadi as ca
        from control.codegen.cost_functions import lag_cost

        pos = ca.DM([1.0, 2.0, 0.0])
        p_ref = ca.DM([1.0, 2.0, 0.0])
        t_ref = ca.DM([0.0, 1.0, 0.0])

        cost = float(lag_cost(pos, p_ref, t_ref, 1.0))
        assert abs(cost) < 1e-12

    def test_stage_cost_H_shape(self):
        """Stage cost should build without errors."""
        from control.codegen.cost_functions import MPCCStageCost

        sc = MPCCStageCost(num_thrusters=6, num_rw=3)
        assert sc.stage_cost is not None
        assert sc.stage_cost_hess is not None

    def test_quat_error_shortest_path(self):
        """Quaternion error should choose shortest rotation path."""
        import casadi as ca
        from control.codegen.cost_functions import quat_error_vec

        q = ca.DM([1, 0, 0, 0])
        q_ref = ca.DM([-1, 0, 0, 0])  # equivalent orientation

        err = np.array(quat_error_vec(q, q_ref)).ravel()
        # Should be near zero (same orientation)
        assert np.linalg.norm(err) < 1e-6


# ==========================================================================
# 4. C++ SQP Controller
# ==========================================================================


@skip_no_cpp_mpc
class TestCppSQPController:
    """Test the C++ SQP controller directly via pybind11."""

    @pytest.fixture
    def controller(self):
        from cpp import _cpp_mpc

        sat = _cpp_mpc.SatelliteParams()
        sat.mass = 10.0
        sat.inertia = np.array([0.15, 0.12, 0.18])
        sat.num_thrusters = 6
        sat.num_rw = 3
        sat.thruster_positions = [
            np.array([0.5, 0, 0]),
            np.array([-0.5, 0, 0]),
            np.array([0, 0.5, 0]),
            np.array([0, -0.5, 0]),
            np.array([0, 0, 0.5]),
            np.array([0, 0, -0.5]),
        ]
        sat.thruster_directions = [
            np.array([1, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([0, 1, 0]),
            np.array([0, -1, 0]),
            np.array([0, 0, 1]),
            np.array([0, 0, -1]),
        ]
        sat.thruster_forces = [1.0] * 6
        sat.rw_torque_limits = [0.05] * 3
        sat.rw_inertia = [0.001] * 3
        sat.rw_speed_limits = [600.0] * 3
        sat.rw_axes = [np.eye(3)[i] for i in range(3)]
        sat.com_offset = np.zeros(3)
        sat.orbital_mu = 3.986e14
        sat.orbital_radius = 6.778e6
        sat.orbital_mean_motion = 0.0
        sat.use_two_body = False

        params = _cpp_mpc.MPCV2Params()
        params.prediction_horizon = 10
        params.control_horizon = 8
        params.dt = 0.1

        return _cpp_mpc.SQPController(sat, params)

    def test_construction(self, controller):
        """Controller should initialise without errors."""
        assert controller.num_controls == 10  # 3 RW + 6 thr + 1 v_s
        assert controller.prediction_horizon == 10

    def test_get_control_action_returns_result(self, controller):
        """get_control_action should return a ControlResult."""
        x = np.zeros(17)
        x[3] = 1.0  # valid quaternion
        result = controller.get_control_action(x)
        assert hasattr(result, "u")
        assert hasattr(result, "status")
        assert hasattr(result, "solve_time")
        assert len(result.u) == 10

    def test_path_data_round_trip(self, controller):
        """Set path data and verify path_length / has_path."""
        path = [
            [0.0, 0.0, 0.0, 0.0],
            [0.5, 0.5, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0],
        ]
        controller.set_path_data(path)
        assert controller.has_path
        assert abs(controller.path_length - 1.0) < 1e-6

    def test_project_onto_path(self, controller):
        """Projection should return closest point on path."""
        path = [
            [0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0],
        ]
        controller.set_path_data(path)

        s, pt, dist, ep_err = controller.project_onto_path(np.array([0.5, 0.0, 0.0]))
        assert 0.0 <= s <= 1.0
        assert dist >= 0.0

    def test_set_runtime_mode(self, controller):
        """Setting mode should not throw."""
        for mode in ["TRACK", "RECOVER", "SETTLE", "HOLD", "COMPLETE"]:
            controller.set_runtime_mode(mode)

    def test_stage_linearisation_injection(self, controller):
        """set_stage_linearisation should accept matrices without error."""
        A = np.eye(17) * 0.99
        B = np.zeros((17, 10))
        d = np.zeros(17)

        for k in range(controller.prediction_horizon):
            controller.set_stage_linearisation(k, A, B, d)

    def test_get_stage_state(self, controller):
        """get_stage_state should return correct dimension."""
        x = controller.get_stage_state(0)
        assert len(x) == 17


# ==========================================================================
# 5. Python Wrapper (end-to-end)
# ==========================================================================


@skip_no_casadi
@skip_no_cpp_mpc
class TestMPCController:
    """End-to-end test of the Python wrapper."""

    @pytest.fixture
    def app_config(self):
        from config.defaults import create_default_app_config

        return create_default_app_config()

    def test_construction(self, app_config):
        """MPCController should construct from default AppConfig."""
        from control.mpc_controller import MPCController

        ctrl = MPCController(app_config)
        assert ctrl.dt > 0
        assert ctrl.prediction_horizon > 0

    def test_get_control_action_shape(self, app_config):
        """Control output should have correct dimension."""
        from control.mpc_controller import MPCController

        ctrl = MPCController(app_config)

        x = np.zeros(17)
        x[3] = 1.0
        u, info = ctrl.get_control_action(x)

        # u should be physical controls only (no v_s)
        expected_nu = ctrl.num_rw_axes + ctrl.num_thrusters
        assert len(u) == expected_nu
        assert "status" in info
        assert "solve_time" in info

    def test_path_following_workflow(self, app_config):
        """Full workflow: set path -> solve -> get progress."""
        from control.mpc_controller import MPCController

        ctrl = MPCController(app_config)

        path = [(0, 0, 0), (1, 0, 0), (2, 0, 0)]
        ctrl.set_path(path)

        x = np.zeros(17)
        x[3] = 1.0
        u, info = ctrl.get_control_action(x)

        progress = ctrl.get_path_progress(position=np.array([0.5, 0, 0]))
        assert "s" in progress
        assert "progress" in progress

    def test_implements_controller_abc(self, app_config):
        """MPCController should be a valid Controller."""
        from control.base import Controller
        from control.mpc_controller import MPCController

        ctrl = MPCController(app_config)
        assert isinstance(ctrl, Controller)

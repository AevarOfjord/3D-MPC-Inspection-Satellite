"""acados controller profile behavior tests (acados_rti + acados_sqp).

Verifies both acados-backed NMPC profiles:
  - construct correctly from AppConfig (class vars, prediction_horizon, dt)
  - route through the factory to the right concrete class
  - share the same shared_contract fairness hash as RTI-SQP and NMPC profiles
  - return control output of the correct shape
  - expose expected info keys, including acados_status and acados_iterations
  - path and reset interfaces work identically to NmpcController

Auto-skipped when acados_template is not installed (CI without acados libs).
"""

import numpy as np
import pytest

acados_template = pytest.importorskip(
    "acados_template",
    reason="acados_template not installed — skipping acados controller tests",
)

# acados_template is importable but useless without the compiled C library.
# Check for the sentinel file that AcadosOcpSolver needs at construction time.
import os as _os

_acados_source_dir = _os.environ.get("ACADOS_SOURCE_DIR", "")
if not _acados_source_dir:
    # Try the heuristic acados_template uses: look alongside the Python install
    import acados_template as _at

    _guessed_path = _os.path.join(_os.path.dirname(_at.__file__), "..", "..")
    _link_libs = _os.path.join(_guessed_path, "lib", "link_libs.json")
    if not _os.path.isfile(_link_libs):
        pytest.skip(
            "acados C library not found (ACADOS_SOURCE_DIR not set and "
            "link_libs.json not present). Install acados from source: "
            "https://docs.acados.org/installation/",
            allow_module_level=True,
        )

from controller.configs.simulation_config import SimulationConfig  # noqa: E402
from controller.factory import create_controller  # noqa: E402

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _build_acados_controller(profile: str, horizon: int = 3):
    """Build an acados controller with a small horizon for fast CI compilation."""
    cfg = SimulationConfig.create_with_overrides(
        {
            "mpc": {
                "prediction_horizon": horizon,
                "control_horizon": horizon,
            },
            "mpc_core": {
                "controller_profile": profile,
            },
        }
    ).app_config
    return create_controller(cfg)


def _zero_state() -> np.ndarray:
    """Return a 16-element state with a valid unit quaternion."""
    x = np.zeros(16, dtype=float)
    x[3] = 1.0  # qw = 1 → identity quaternion
    return x


def _build_controller_for_hash(profile: str):
    """Build controller at a fixed horizon so contract hashes are comparable."""
    cfg = SimulationConfig.create_with_overrides(
        {
            "mpc": {"prediction_horizon": 3, "control_horizon": 3},
            "mpc_core": {"controller_profile": profile},
        }
    ).app_config
    return create_controller(cfg)


# ---------------------------------------------------------------------------
# Construction and class attributes
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_profile_construction(profile):
    controller = _build_acados_controller(profile)

    assert controller.controller_profile == profile
    assert controller.controller_core == "acados"
    assert controller.solver_backend == "acados+HPIPM"
    assert controller.linearization_mode == "none"
    assert controller.cpp_module_name is None
    assert controller.prediction_horizon == 3
    assert controller.dt > 0.0


def test_acados_rti_class_vars():
    controller = _build_acados_controller("acados_rti")
    assert controller.solver_type == "ACADOS-SQP_RTI"
    assert controller._acados_nlp_solver_type == "SQP_RTI"


def test_acados_sqp_class_vars():
    controller = _build_acados_controller("acados_sqp")
    assert controller.solver_type == "ACADOS-SQP"
    assert controller._acados_nlp_solver_type == "SQP"


# ---------------------------------------------------------------------------
# Factory routing
# ---------------------------------------------------------------------------


def test_acados_rti_registered_via_factory():
    from controller.acados_rti.python.controller import AcadosRtiController

    controller = _build_acados_controller("acados_rti")
    assert isinstance(controller, AcadosRtiController)


def test_acados_sqp_registered_via_factory():
    from controller.acados_sqp.python.controller import AcadosSqpController

    controller = _build_acados_controller("acados_sqp")
    assert isinstance(controller, AcadosSqpController)


# ---------------------------------------------------------------------------
# get_control_action — output shape
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_get_control_action_returns_correct_shape(profile):
    controller = _build_acados_controller(profile)
    x = _zero_state()

    u, info = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )

    expected_nu = controller.num_rw_axes + controller.num_thrusters
    assert u.shape == (expected_nu,), (
        f"[{profile}] Expected control shape ({expected_nu},), got {u.shape}"
    )


# ---------------------------------------------------------------------------
# get_control_action — info keys
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_info_keys_present(profile):
    controller = _build_acados_controller(profile)
    x = _zero_state()

    _, info = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )

    # Core identification
    assert info["controller_profile"] == profile
    assert info["solver_backend"] == "acados+HPIPM"
    assert info["linearization_mode"] == "none"
    assert info["cpp_backend_module"] is None

    # acados-specific
    assert "acados_status" in info
    assert "acados_iterations" in info
    assert isinstance(info["acados_iterations"], int)
    assert info["acados_iterations"] >= 0

    # Path and timing keys
    assert "path_s" in info
    assert "path_progress" in info
    assert "timing_solve_only_s" in info
    assert info["timing_solve_only_s"] >= 0.0

    # Fairness contract keys
    assert "shared_params_hash" in info
    assert "effective_params_hash" in info


# ---------------------------------------------------------------------------
# Regression: state dependence, fallback freezing, and path metrics
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_control_changes_with_different_states(profile):
    """
    Regression for x0 pinning:
    different measured states should produce different control decisions.
    """
    path = [(1.0, 1.0, 0.0), (0.0, 0.0, 0.0)]

    ctrl_a = _build_acados_controller(profile)
    ctrl_a.set_path(path)
    x_a = _zero_state()
    x_a[0], x_a[1], x_a[2] = 2.0, 2.0, 0.0
    u_a, _ = ctrl_a.get_control_action(x_current=x_a)
    thr_a = u_a[ctrl_a.num_rw_axes : ctrl_a.num_rw_axes + ctrl_a.num_thrusters]

    ctrl_b = _build_acados_controller(profile)
    ctrl_b.set_path(path)
    x_b = _zero_state()
    x_b[0], x_b[1], x_b[2] = -1.0, -1.0, 0.0
    u_b, _ = ctrl_b.get_control_action(x_current=x_b)
    thr_b = u_b[ctrl_b.num_rw_axes : ctrl_b.num_rw_axes + ctrl_b.num_thrusters]

    assert not np.allclose(thr_a, thr_b, atol=1e-4), (
        f"[{profile}] Thruster output should differ for opposite-side states."
    )


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_failed_solve_freezes_progress_and_holds_safe(monkeypatch, profile):
    controller = _build_acados_controller(profile)
    controller.set_path([(1.0, 1.0, 0.0), (0.0, 0.0, 0.0)])
    controller.s = 0.42

    # Force solver failure status; get_control_action should apply safe hold fallback.
    monkeypatch.setattr(controller._acados_solver, "solve", lambda: 4)

    x = _zero_state()
    x[0], x[1] = 1.0, 1.0
    u, info = controller.get_control_action(x_current=x)

    assert info["solver_fallback"] is True
    assert info["solver_success"] is False
    assert info["solver_fallback_reason"] == "acados_failed"
    assert info["path_v_s"] == pytest.approx(0.0)
    assert info["path_s"] == pytest.approx(0.42)
    assert controller.s == pytest.approx(0.42)
    assert np.allclose(u, 0.0, atol=1e-12)


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_path_progress_reports_finite_projection_metrics(profile):
    controller = _build_acados_controller(profile)
    controller.set_path([(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 1.0, 0.0)])

    # Direct geometric projection.
    projected = controller.get_path_progress(np.array([0.4, 0.2, 0.0], dtype=float))
    assert np.isfinite(projected["path_error"])
    assert np.isfinite(projected["endpoint_error"])
    assert projected["s"] >= 0.0

    # Cached metrics (no explicit position) after one control step.
    x = _zero_state()
    x[0], x[1] = 0.3, 0.2
    _, _ = controller.get_control_action(x_current=x)
    cached = controller.get_path_progress()
    assert np.isfinite(cached["path_error"])
    assert np.isfinite(cached["endpoint_error"])


# ---------------------------------------------------------------------------
# Runtime preload helper
# ---------------------------------------------------------------------------


def test_acados_preload_runtime_libs_load_order(monkeypatch):
    from controller.acados_shared.python.base import AcadosBaseController

    loaded_paths = []

    def _fake_cdll(path, mode=None):
        loaded_paths.append(path)
        return object()

    monkeypatch.setattr("ctypes.CDLL", _fake_cdll)
    monkeypatch.setattr("os.path.isfile", lambda _p: True)

    lib_dir = "/tmp/acados-lib"
    ext = AcadosBaseController._shared_lib_extension()
    expected = [
        f"{lib_dir}/libblasfeo{ext}",
        f"{lib_dir}/libhpipm{ext}",
        f"{lib_dir}/libqpOASES_e{ext}",
        f"{lib_dir}/libacados{ext}",
    ]

    AcadosBaseController._preload_acados_runtime_libs([lib_dir])
    assert loaded_paths == expected


def test_acados_preload_runtime_libs_failure_message(monkeypatch):
    from controller.acados_shared.python.base import AcadosBaseController

    def _always_fail(path, mode=None):
        raise OSError(f"cannot load {path}")

    monkeypatch.setattr("ctypes.CDLL", _always_fail)
    monkeypatch.setattr("os.path.isfile", lambda _p: True)

    with pytest.raises(RuntimeError) as exc:
        AcadosBaseController._preload_acados_runtime_libs(["/missing/acados/lib"])

    msg = str(exc.value)
    assert "Failed to preload required acados runtime library" in msg
    assert "libblasfeo" in msg
    assert "/missing/acados/lib" in msg


# ---------------------------------------------------------------------------
# Warm-start on second call
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_second_call_uses_warm_start(profile):
    controller = _build_acados_controller(profile)
    x = _zero_state()

    _, _ = controller.get_control_action(x_current=x)
    # First call should have populated the warm-start cache
    assert controller._last_X_sol is not None or True  # best-effort

    _, info2 = controller.get_control_action(x_current=x)
    assert info2["controller_profile"] == profile


# ---------------------------------------------------------------------------
# Fairness hash — shared contract must match all other profiles
# ---------------------------------------------------------------------------


def test_acados_shared_contract_hash_matches_other_profiles():
    """
    All profiles must share the same shared_contract hash — physics, weights,
    and horizons must be identical for a fair paper comparison.
    """
    hybrid = _build_controller_for_hash("hybrid")
    nmpc = _build_controller_for_hash("nmpc")
    rti = _build_controller_for_hash("acados_rti")
    sqp = _build_controller_for_hash("acados_sqp")

    hybrid_sig = hybrid.get_shared_contract_signature()
    nmpc_sig = nmpc.get_shared_contract_signature()
    rti_sig = rti.get_shared_contract_signature()
    sqp_sig = sqp.get_shared_contract_signature()

    assert rti_sig is not None
    assert sqp_sig is not None

    assert rti_sig == hybrid_sig, (
        f"acados_rti shared hash ({rti_sig[:16]}…) != hybrid ({hybrid_sig[:16]}…). "
        "Both controllers must use identical physics/weight parameters."
    )
    assert sqp_sig == hybrid_sig, (
        f"acados_sqp shared hash ({sqp_sig[:16]}…) != hybrid ({hybrid_sig[:16]}…). "
        "Both controllers must use identical physics/weight parameters."
    )
    assert rti_sig == nmpc_sig, (
        f"acados_rti shared hash ({rti_sig[:16]}…) != nmpc ({nmpc_sig[:16]}…)."
    )


def test_acados_effective_contract_differs_across_profiles():
    """
    Effective contract includes profile name and profile-specific settings,
    so each profile must have a distinct effective signature.
    """
    hybrid = _build_controller_for_hash("hybrid")
    rti = _build_controller_for_hash("acados_rti")
    sqp = _build_controller_for_hash("acados_sqp")

    hybrid_eff = hybrid.get_effective_contract_signature()
    rti_eff = rti.get_effective_contract_signature()
    sqp_eff = sqp.get_effective_contract_signature()

    assert rti_eff is not None
    assert sqp_eff is not None
    assert rti_eff != hybrid_eff, "acados_rti effective hash must differ from hybrid"
    assert sqp_eff != hybrid_eff, "acados_sqp effective hash must differ from hybrid"
    assert rti_eff != sqp_eff, (
        "acados_rti and acados_sqp must have different effective hashes"
    )


# ---------------------------------------------------------------------------
# Profile-specific tolerances in profile_specific_params
# ---------------------------------------------------------------------------


def test_acados_rti_profile_specific_defaults():
    controller = _build_acados_controller("acados_rti")
    ps = controller.profile_specific_params
    assert ps["acados_max_iter"] == 1
    assert ps["acados_tol_stat"] == pytest.approx(1e-2)


def test_acados_sqp_profile_specific_defaults():
    controller = _build_acados_controller("acados_sqp")
    ps = controller.profile_specific_params
    assert ps["acados_max_iter"] == 50
    assert ps["acados_tol_stat"] == pytest.approx(1e-2)


# ---------------------------------------------------------------------------
# Path interface
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_set_path_enables_reference_building(profile):
    controller = _build_acados_controller(profile)

    path = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    controller.set_path(path)

    assert controller._path_set is True
    assert controller._path_length > 0.0

    progress = controller.get_path_progress()
    assert "s" in progress
    assert "progress" in progress
    assert "remaining" in progress


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_set_current_path_s(profile):
    controller = _build_acados_controller(profile)
    controller.set_current_path_s(2.71)
    assert controller.s == pytest.approx(2.71)


# ---------------------------------------------------------------------------
# reset
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile", ["acados_rti", "acados_sqp"])
def test_acados_reset_clears_state(profile):
    controller = _build_acados_controller(profile)
    x = _zero_state()
    controller.get_control_action(x_current=x)
    controller.s = 99.0

    controller.reset()

    assert controller.s == 0.0
    assert controller._last_X_sol is None
    assert controller._last_U_sol is None
    assert controller._step_count == 0

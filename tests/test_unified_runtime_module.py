"""Unified C++ runtime module contract tests."""

import numpy as np
import pytest


def _import_runtime_module():
    from controller.shared.python import cpp

    runtime_mod = getattr(cpp, "_cpp_mpc_runtime", None)
    if runtime_mod is None:
        pytest.skip("_cpp_mpc_runtime extension is not available")
    return cpp, runtime_mod


def test_runtime_module_exports_capability_flags():
    cpp, runtime_mod = _import_runtime_module()

    assert hasattr(runtime_mod, "HAS_ACADOS_BACKEND")
    assert hasattr(runtime_mod, "HAS_IPOPT_BACKEND")
    assert hasattr(runtime_mod, "HAS_ACADOS_DEPENDENCIES")
    assert hasattr(runtime_mod, "HAS_IPOPT_DEPENDENCIES")
    assert hasattr(runtime_mod, "HAS_CASADI_CPP_DEPENDENCIES")
    assert cpp.HAS_ACADOS_BACKEND == bool(runtime_mod.HAS_ACADOS_BACKEND)
    assert cpp.HAS_IPOPT_BACKEND == bool(runtime_mod.HAS_IPOPT_BACKEND)
    assert cpp.HAS_ACADOS_DEPENDENCIES == bool(runtime_mod.HAS_ACADOS_DEPENDENCIES)
    assert cpp.HAS_IPOPT_DEPENDENCIES == bool(runtime_mod.HAS_IPOPT_DEPENDENCIES)
    assert cpp.HAS_CASADI_CPP_DEPENDENCIES == bool(
        runtime_mod.HAS_CASADI_CPP_DEPENDENCIES
    )


def test_runtime_unavailable_backend_returns_deterministic_error():
    _cpp, runtime_mod = _import_runtime_module()

    sat = runtime_mod.SatelliteParams()
    sat.dt = 0.05
    sat.mass = 1.0
    sat.inertia = np.array([1.0, 1.0, 1.0], dtype=float)
    sat.num_thrusters = 6
    sat.num_rw = 3
    sat.thruster_positions = [np.zeros(3, dtype=float) for _ in range(6)]
    sat.thruster_directions = [np.array([1.0, 0.0, 0.0], dtype=float) for _ in range(6)]
    sat.thruster_forces = [1.0 for _ in range(6)]
    sat.rw_torque_limits = [1.0, 1.0, 1.0]
    sat.rw_inertia = [1.0, 1.0, 1.0]
    sat.rw_speed_limits = [1.0, 1.0, 1.0]
    sat.rw_axes = [np.array([1.0, 0.0, 0.0], dtype=float) for _ in range(3)]
    sat.com_offset = np.zeros(3, dtype=float)
    sat.orbital_mean_motion = 0.0
    sat.orbital_mu = 3.986004418e14
    sat.orbital_radius = 6.778e6
    sat.use_two_body = True

    mpc = runtime_mod.MPCV2Params()
    cfg = runtime_mod.RuntimeConfig()
    cfg.profile = runtime_mod.RuntimeProfile.CPP_NONLINEAR_RTI_HPIPM

    runtime = runtime_mod.UnifiedMpcRuntime(sat, mpc, cfg)
    result = runtime.solve_step(np.zeros(17, dtype=float))

    assert runtime.backend_available is False
    assert result.solver_status == "unavailable_backend"
    assert isinstance(result.unavailable_reason, str)
    assert "acados" in result.unavailable_reason.lower()

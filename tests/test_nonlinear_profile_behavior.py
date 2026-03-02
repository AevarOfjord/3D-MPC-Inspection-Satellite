"""Nonlinear profile behavior tests."""

import numpy as np

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller


def test_nonlinear_fail_closed_on_invalid_linearization():
    cfg = SimulationConfig.create_default().app_config.model_copy(deep=True)
    cfg.mpc_core.controller_profile = "nonlinear"
    controller = create_controller(cfg)

    # Force non-finite Jacobians to trigger strict nonlinear integrity failure.
    def bad_f_and_jacs(x, u, p, dt):  # noqa: ANN001
        nx = len(x)
        nu = len(u)
        return (
            np.full(nx, np.nan),
            np.full((nx, nx), np.nan),
            np.full((nx, nu), np.nan),
        )

    controller._f_and_jacs = bad_f_and_jacs  # noqa: SLF001 - intentional test hook

    x = np.zeros(16, dtype=float)
    x[3] = 1.0
    u, info = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )

    assert info["controller_profile"] == "nonlinear"
    assert info["linearization_mode"] == "nonlinear_exact_stage"
    assert info["linearization_integrity_failure"] is True
    assert info["solver_fallback_reason"] == "linearization_integrity_failed"
    assert info["solver_success"] is False
    assert info["fallback_active"] is True
    assert u.shape[0] == controller.num_rw_axes + controller.num_thrusters

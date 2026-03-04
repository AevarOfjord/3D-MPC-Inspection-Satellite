"""Linear profile behavior tests."""

import numpy as np

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller


def test_linear_profile_uses_frozen_step_linearization():
    cfg = SimulationConfig.create_default().app_config.model_copy(deep=True)
    cfg.mpc_core.controller_profile = "cpp_linearized_rti_osqp"
    controller = create_controller(cfg)

    call_count = {"n": 0}

    def frozen_linearization(x, u, p, dt):  # noqa: ANN001
        call_count["n"] += 1
        nx = len(x)
        nu = len(u)
        x_next = np.array(x, dtype=float).copy()
        A = np.eye(nx, dtype=float)
        B = np.zeros((nx, nu), dtype=float)
        return x_next, A, B

    controller._f_and_jacs = frozen_linearization  # noqa: SLF001 - intentional test hook

    x = np.zeros(16, dtype=float)
    x[3] = 1.0
    _, info = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )

    assert info["controller_profile"] == "cpp_linearized_rti_osqp"
    assert info["linearization_mode"] == "linear_frozen_step"
    assert info["cpp_backend_module"] == "_cpp_mpc_runtime"
    assert info["linearization_attempted_stages"] == controller.prediction_horizon
    assert info["linearization_failed_stages"] == 0
    assert call_count["n"] == 1

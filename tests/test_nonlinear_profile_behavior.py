"""Nonlinear profile behavior tests."""

import numpy as np

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller


def _build_nonlinear_controller():
    cfg = SimulationConfig.create_with_overrides(
        {
            "mpc": {
                "prediction_horizon": 8,
                "control_horizon": 8,
            },
            "mpc_core": {
                "controller_profile": "nonlinear",
            },
        }
    ).app_config
    return create_controller(cfg)


def test_nonlinear_profile_uses_mixed_cpp_python_backend():
    controller = _build_nonlinear_controller()

    x = np.zeros(16, dtype=float)
    x[3] = 1.0
    u, info = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )

    assert info["controller_profile"] == "nonlinear"
    assert info["linearization_mode"] == "nonlinear_exact_stage"
    assert info["solver_backend"] == "CasADi+OSQP"
    assert info["cpp_backend_module"] == "_cpp_mpc_nonlinear"
    assert u.shape[0] == controller.num_rw_axes + controller.num_thrusters


def test_nonlinear_profile_reports_solver_iterations():
    controller = _build_nonlinear_controller()

    x = np.zeros(16, dtype=float)
    x[3] = 1.0
    _, info = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )

    assert info["iterations"] is not None
    assert int(info["sqp_iterations"]) >= 0
    assert int(info["linearization_attempted_stages"]) >= 0

    # Second call exercises warm-start shift path.
    _, info2 = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )
    assert info2["controller_profile"] == "nonlinear"

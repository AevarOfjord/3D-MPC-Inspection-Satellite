"""Controller metadata contract tests."""

import numpy as np
import pytest

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller


@pytest.mark.parametrize(
    ("profile", "expected_mode"),
    [
        ("hybrid", "hybrid_tolerant_stage"),
        ("nonlinear", "nonlinear_exact_stage"),
        ("linear", "linear_frozen_step"),
    ],
)
def test_controller_emits_profile_and_linearization_metadata(
    profile: str, expected_mode: str
):
    cfg = SimulationConfig.create_default().app_config.model_copy(deep=True)
    cfg.mpc_core.controller_profile = profile
    controller = create_controller(cfg)
    x = np.zeros(16, dtype=float)
    x[3] = 1.0
    _, info = controller.get_control_action(
        x_current=x,
        previous_thrusters=np.zeros(controller.num_thrusters, dtype=float),
    )

    assert info["controller_profile"] == profile
    assert info["controller_core"] == controller.controller_core
    assert info["solver_backend"] == controller.solver_backend
    assert info["linearization_mode"] == expected_mode
    assert isinstance(info["shared_params_hash"], str) and info["shared_params_hash"]
    assert (
        isinstance(info["effective_params_hash"], str) and info["effective_params_hash"]
    )
    assert isinstance(info["override_diff"], dict)

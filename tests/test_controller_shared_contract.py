"""Shared MPC parameter contract tests across controller profiles."""

import pytest

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller


def _build_controller(profile: str):
    cfg = SimulationConfig.create_default().app_config.model_copy(deep=True)
    cfg.mpc_core.controller_profile = profile
    return create_controller(cfg)


def test_shared_contract_signature_matches_across_profiles():
    hybrid = _build_controller("hybrid")
    nonlinear = _build_controller("nonlinear")
    linear = _build_controller("linear")

    assert hybrid.get_shared_contract_signature()
    assert (
        hybrid.get_shared_contract_signature()
        == nonlinear.get_shared_contract_signature()
    )
    assert (
        hybrid.get_shared_contract_signature() == linear.get_shared_contract_signature()
    )


def test_shared_contract_is_immutable():
    hybrid = _build_controller("hybrid")
    with pytest.raises(TypeError):
        hybrid.shared_contract.mpc["Q_contour"] = 1.0

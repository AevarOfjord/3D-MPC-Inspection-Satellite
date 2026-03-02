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


def test_effective_contract_changes_only_with_profile_overrides():
    base_cfg = SimulationConfig.create_default().app_config.model_copy(deep=True)
    base_cfg.mpc_core.controller_profile = "nonlinear"
    baseline = create_controller(base_cfg)

    tuned_cfg = base_cfg.model_copy(deep=True)
    tuned_cfg.mpc_profile_overrides.nonlinear.base_overrides["Q_contour"] = (
        float(tuned_cfg.mpc.Q_contour) + 111.0
    )
    tuned_cfg.mpc_profile_overrides.nonlinear.profile_specific["sqp_max_iter"] = 3
    tuned = create_controller(tuned_cfg)

    # Fairness baseline remains identical across profile variants.
    assert (
        baseline.get_shared_contract_signature()
        == tuned.get_shared_contract_signature()
    )
    # Effective contract must change when explicit deltas are applied.
    assert (
        baseline.get_effective_contract_signature()
        != tuned.get_effective_contract_signature()
    )
    assert tuned.profile_override_diff["Q_contour"] == pytest.approx(
        float(tuned_cfg.mpc.Q_contour) + 111.0
    )

"""
Configuration Tests.

Tests configuration validation, presets, and model integrity.
Consolidates `test_config_validation.py` and `test_presets.py`.
"""

from pathlib import Path

import pytest
from pydantic import ValidationError
from satellite_control.config import physics as physics_cfg
from satellite_control.config.io import ConfigIO
from satellite_control.config.mission_state import DEFAULT_PATH_HOLD_END_S
from satellite_control.config.models import (
    ActuatorPolicyParams,
    ControllerContractsParams,
    MPCParams,
    SatellitePhysicalParams,
)
from satellite_control.config.presets import list_presets, load_preset
from satellite_control.config.simulation_config import SimulationConfig


class TestConfigValidation:
    """Tests for configuration validation logic."""

    def test_physical_params_validation(self):
        """Test physical parameter validation."""
        # valid config
        thruster_ids = range(1, len(physics_cfg.THRUSTER_POSITIONS) + 1)
        params = SatellitePhysicalParams(
            total_mass=10.0,
            moment_of_inertia=1.0,
            satellite_size=1.0,
            com_offset=(0, 0, 0),
            thruster_positions={
                i: physics_cfg.THRUSTER_POSITIONS[i] for i in thruster_ids
            },
            thruster_directions={
                i: physics_cfg.THRUSTER_DIRECTIONS[i] for i in thruster_ids
            },
            thruster_forces={i: physics_cfg.THRUSTER_FORCES[i] for i in thruster_ids},
            use_realistic_physics=False,
            damping_linear=0.0,
            damping_angular=0.0,
        )
        assert params.total_mass == 10.0

        # Invalid config (negative mass)
        with pytest.raises(ValidationError):
            SatellitePhysicalParams(
                total_mass=-10.0,
                moment_of_inertia=1.0,
                satellite_size=1.0,
                com_offset=(0, 0, 0),
                thruster_positions={
                    i: physics_cfg.THRUSTER_POSITIONS[i] for i in thruster_ids
                },
                thruster_directions={
                    i: physics_cfg.THRUSTER_DIRECTIONS[i] for i in thruster_ids
                },
                thruster_forces={
                    i: physics_cfg.THRUSTER_FORCES[i] for i in thruster_ids
                },
                use_realistic_physics=False,
                damping_linear=0.0,
                damping_angular=0.0,
            )

    def test_simulation_config_creation(self):
        """Test SimulationConfig creation and validation."""
        config = SimulationConfig.create_default()
        assert config.app_config.physics.total_mass > 0

    def test_preset_loading(self):
        """Test loading of configuration presets."""
        presets = list_presets()
        assert "balanced" in presets
        assert "fast" in presets

        # Test loading a specific preset
        config = load_preset("fast")
        # fast preset has path_speed set, check it in the dict
        assert config["mpc"]["path_speed"] >= 0.1

    def test_invalid_preset(self):
        """Test handling of invalid preset names."""
        with pytest.raises(ValueError):
            load_preset("nonexistent_preset")

    def test_mpc_performance_toggles_default_and_legacy_mapping(self):
        """Ensure MPC performance toggles are exposed in config and legacy maps."""
        config = SimulationConfig.create_default()
        mpc = config.app_config.mpc
        legacy = config.get_mpc_params()

        assert mpc.enable_delta_u_coupling is False
        assert mpc.enable_gyro_jacobian is False
        assert mpc.enable_auto_state_bounds is False
        assert legacy["enable_delta_u_coupling"] is False
        assert legacy["enable_gyro_jacobian"] is False
        assert legacy["enable_auto_state_bounds"] is False

    def test_mpc_hysteresis_threshold_validation(self):
        """Hysteresis on-threshold must exceed off-threshold."""
        with pytest.raises(ValidationError):
            MPCParams(thruster_hysteresis_on=0.01, thruster_hysteresis_off=0.01)
        with pytest.raises(ValidationError):
            MPCParams(thruster_hysteresis_on=0.005, thruster_hysteresis_off=0.007)

    def test_removed_mpc_dead_knobs_absent_from_schema(self):
        """Removed V6 cleanup knobs should not exist in canonical MPC schema."""
        removed = {
            "coast_pos_tolerance",
            "coast_vel_tolerance",
            "coast_min_speed",
            "progress_taper_distance",
            "progress_slowdown_distance",
        }
        assert removed.isdisjoint(MPCParams.model_fields.keys())

    def test_removed_mpc_dead_knobs_only_exist_in_runner_deprecation_filter(self):
        """Runtime-critical sources should not reference removed MPC knobs."""
        removed = (
            "coast_pos_tolerance",
            "coast_vel_tolerance",
            "coast_min_speed",
            "progress_taper_distance",
            "progress_slowdown_distance",
        )
        repo_root = Path(__file__).resolve().parents[1]
        allowed = {
            repo_root / "src/python/satellite_control/dashboard/runner_manager.py"
        }

        for base in ("src/python/satellite_control", "src/cpp", "ui/src"):
            for path in (repo_root / base).rglob("*"):
                if not path.is_file():
                    continue
                if path.suffix not in {".py", ".cpp", ".hpp", ".ts", ".tsx"}:
                    continue
                if path in allowed:
                    continue
                text = path.read_text(encoding="utf-8")
                for token in removed:
                    assert token not in text, f"{token} still present in {path}"

    def test_v6_solver_fallback_contract_validation(self):
        """Fallback zero-after threshold must be >= fallback hold threshold."""
        with pytest.raises(ValidationError):
            ControllerContractsParams(
                solver_fallback_hold_s=1.0,
                solver_fallback_zero_after_s=0.5,
            )

    def test_v6_sections_present_in_default_config(self):
        """Default AppConfig should include V6 section models for app_config_v3."""
        config = SimulationConfig.create_default()
        app_cfg = config.app_config
        assert app_cfg.reference_scheduler.speed_policy == "min_non_hold_segment_speed"
        assert app_cfg.mpc_core.solver_backend == "OSQP"
        assert app_cfg.actuator_policy.enable_thruster_hysteresis is True
        assert app_cfg.controller_contracts.hold_duration_s == pytest.approx(
            DEFAULT_PATH_HOLD_END_S
        )

    def test_v6_actuator_policy_validation(self):
        """V6 actuator-policy on-threshold must exceed off-threshold."""
        with pytest.raises(ValidationError):
            ActuatorPolicyParams(
                thruster_hysteresis_on=0.01, thruster_hysteresis_off=0.01
            )
        with pytest.raises(ValidationError):
            ActuatorPolicyParams(
                thruster_hysteresis_on=0.005, thruster_hysteresis_off=0.007
            )

    def test_v6_controller_contract_validation(self):
        """V6 recover-exit threshold must remain <= recover-enter threshold."""
        with pytest.raises(ValidationError):
            ControllerContractsParams(
                recover_enter_error_m=0.10,
                recover_exit_error_m=0.20,
            )

    def test_path_hold_end_defaults_when_missing(self):
        """Missing hold fields should use the global mission hold default."""
        mission_state = ConfigIO._dict_to_mission_state({})
        assert mission_state.path_hold_end == pytest.approx(DEFAULT_PATH_HOLD_END_S)

    def test_path_hold_end_preserves_explicit_zero(self):
        """Explicit 0.0 hold should not be overwritten by defaults."""
        mission_state = ConfigIO._dict_to_mission_state({"path_hold_end": 0.0})
        assert mission_state.path_hold_end == pytest.approx(0.0)

        mission_state_nested = ConfigIO._dict_to_mission_state(
            {"trajectory": {"hold_end": 0.0}}
        )
        assert mission_state_nested.path_hold_end == pytest.approx(0.0)

    def test_path_hold_end_preserves_explicit_nonzero(self):
        """Explicit hold values in legacy fields should be preserved."""
        mission_state = ConfigIO._dict_to_mission_state({"trajectory_hold_end": 5.0})
        assert mission_state.path_hold_end == pytest.approx(5.0)

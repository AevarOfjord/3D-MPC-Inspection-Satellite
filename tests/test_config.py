"""
Configuration Tests.

Tests configuration validation, presets, and model integrity.
Consolidates `test_config_validation.py` and `test_presets.py`.
"""

import pytest
from pydantic import ValidationError
from satellite_control.config import physics as physics_cfg
from satellite_control.config.models import SatellitePhysicalParams
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.config.presets import load_preset, list_presets


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

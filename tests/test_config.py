"""
Unit tests for config.py module.

Tests configuration validation, parameter access, and configuration methods.

V4.0.0: Tests updated to use SimulationConfig where possible.
Some tests still use SatelliteConfig for deprecated API compatibility.
"""

import numpy as np
import pytest

# V4.0.0: Prefer SimulationConfig, but keep SatelliteConfig for deprecated API tests
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.config.validator import ConfigValidator


class TestSatelliteConfigValidation:
    """Test configuration validation methods (V4.0.0: uses SimulationConfig)."""

    def test_validate_parameters_success(self):
        """Test that default parameters pass validation."""
        # V4.0.0: Use SimulationConfig and ConfigValidator instead of SatelliteConfig
        config = SimulationConfig.create_default()
        # Should not raise any exceptions
        ConfigValidator.validate_and_raise(config.app_config)

    def test_physical_parameters_are_positive(self):
        """Test that physical parameters are positive values."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        physics = config.app_config.physics

        assert physics.total_mass > 0, "Mass must be positive"
        assert physics.moment_of_inertia > 0, "Inertia must be positive"
        assert physics.satellite_size > 0, "Size must be positive"

    def test_thruster_forces_are_positive(self):
        """Test that all thruster forces are positive."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        physics = config.app_config.physics

        for thruster_id, force in physics.thruster_forces.items():
            assert force > 0, f"Thruster {thruster_id} force must be positive"

    def test_mpc_horizons_are_valid(self):
        """Test that MPC horizons are valid."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        mpc = config.app_config.mpc

        assert mpc.prediction_horizon > 0
        assert mpc.control_horizon > 0
        assert mpc.control_horizon <= mpc.prediction_horizon

    def test_cost_weights_are_positive(self):
        """Test that MPC cost weights are positive."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        mpc = config.app_config.mpc

        assert mpc.Q_contour >= 0
        assert mpc.Q_progress >= 0
        assert mpc.Q_smooth >= 0
        assert mpc.q_angular_velocity >= 0
        assert mpc.r_thrust >= 0
        assert mpc.r_rw_torque >= 0
        # Note: r_switch is not in MPCParams (was in old config)


class TestSatelliteConfigConstants:
    """Test configuration constants and boundaries (V4.0.0: uses SimulationConfig)."""

    def test_simulation_dt_is_reasonable(self):
        """Test that simulation timestep is reasonable."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        dt = config.app_config.simulation.dt
        assert 0.001 <= dt <= 0.1

    def test_control_dt_is_reasonable(self):
        """Test that control timestep is reasonable."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        control_dt = config.app_config.simulation.control_dt
        assert 0.01 <= control_dt <= 1.0

    def test_control_dt_is_multiple_of_simulation_dt(self):
        """Test that control DT is a multiple of simulation DT."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        dt = config.app_config.simulation.dt
        control_dt = config.app_config.simulation.control_dt
        ratio = control_dt / dt
        assert abs(ratio - round(ratio)) < 1e-6, (
            "Control DT should be multiple of simulation DT"
        )

    def test_solver_time_limit_less_than_control_dt(self):
        """Test that solver has time budget within control interval."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        solver_time_limit = config.app_config.mpc.solver_time_limit
        control_dt = config.app_config.simulation.control_dt
        assert solver_time_limit < control_dt

    def test_path_speed_is_reasonable(self):
        """Test that path speed is within expected bounds."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        path_speed = config.app_config.mpc.path_speed
        assert 0.0 < path_speed <= 1.0


class TestSatelliteConfigThrusterGeometry:
    """Test thruster positions and directions (V4.0.0: uses SimulationConfig)."""

    def test_thruster_positions_are_valid(self):
        """Test that thruster positions are within satellite bounds."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        physics = config.app_config.physics
        half_size = physics.satellite_size / 2

        for thruster_id, pos in physics.thruster_positions.items():
            x, y = pos[0], pos[1]
            assert abs(x) <= half_size * 1.1, (
                f"Thruster {thruster_id} x position out of bounds"
            )
            assert abs(y) <= half_size * 1.1, (
                f"Thruster {thruster_id} y position out of bounds"
            )

    def test_thruster_directions_are_unit_vectors(self):
        """Test that thruster directions are unit vectors."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        physics = config.app_config.physics

        for thruster_id, direction in physics.thruster_directions.items():
            dx, dy, dz = direction[0], direction[1], direction[2]
            magnitude = np.sqrt(dx**2 + dy**2 + dz**2)
            assert abs(magnitude - 1.0) < 1e-6, (
                f"Thruster {thruster_id} direction not unit vector"
            )

    def test_thruster_ids_are_consistent(self):
        """Test that thruster IDs are consistent across dictionaries."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()
        physics = config.app_config.physics

        force_ids = set(physics.thruster_forces.keys())
        position_ids = set(physics.thruster_positions.keys())
        direction_ids = set(physics.thruster_directions.keys())

        assert force_ids == position_ids == direction_ids
        assert len(force_ids) == 8


class TestSatelliteConfigIntegration:
    """Integration tests for configuration system (V4.0.0: uses SimulationConfig)."""

    def test_config_supports_simulation_initialization(self):
        """Test that config provides all needed params for simulation."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config = SimulationConfig.create_default()

        # Should be able to create simulation with these params
        assert config.app_config.physics is not None
        assert config.app_config.mpc is not None
        assert config.app_config.simulation is not None

    def test_config_consistency_after_multiple_gets(self):
        """Test that config returns consistent values across multiple calls."""
        # V4.0.0: Use SimulationConfig instead of SatelliteConfig
        config1 = SimulationConfig.create_default()
        config2 = SimulationConfig.create_default()

        assert (
            config1.app_config.physics.total_mass
            == config2.app_config.physics.total_mass
        )
        assert (
            config1.app_config.physics.moment_of_inertia
            == config2.app_config.physics.moment_of_inertia
        )


class TestConfigValidator:
    """Test comprehensive configuration validator."""

    def test_validator_validates_default_config(self):
        """Test that default configuration passes validation."""
        app_config = SimulationConfig.create_default().app_config
        validator = ConfigValidator()
        issues = validator.validate_all(app_config)

        # Default config should be valid
        assert len(issues) == 0, f"Default config has validation issues: {issues}"

    def test_validator_detects_invalid_mass(self):
        """Test that validator detects invalid mass."""
        from src.satellite_control.config.models import (
            AppConfig,
            MPCParams,
            SatellitePhysicalParams,
            SimulationParams,
        )

        # Create config with invalid mass
        # Pydantic V2 raises ValidationError on instantiation
        import pytest
        from pydantic import ValidationError

        with pytest.raises(ValidationError) as excinfo:
            AppConfig(
                physics=SatellitePhysicalParams(
                    total_mass=-1.0,  # Invalid: negative mass
                    moment_of_inertia=0.312,
                    satellite_size=0.29,
                    com_offset=(0.0, 0.0, 0.0),
                    thruster_positions={1: (0.1, 0.1, 0.0)},
                    thruster_directions={1: (1.0, 0.0, 0.0)},
                    thruster_forces={1: 0.45},
                ),
                mpc=MPCParams(
                    prediction_horizon=15,
                    control_horizon=12,
                    dt=0.06,
                    solver_time_limit=0.05,
                    solver_type="OSQP",
                    Q_contour=1000.0,
                    Q_progress=100.0,
                    Q_smooth=10.0,
                    q_angular_velocity=1500.0,
                    r_thrust=1.0,
                    r_rw_torque=0.1,
                    path_speed=0.15,
                    thruster_type="PWM",
                ),
                simulation=SimulationParams(
                    dt=0.005,
                    max_duration=500.0,
                    headless=True,
                    window_width=700,
                    window_height=600,
                ),
            )

        # Verify specific errors are in the raised exception
        errors = str(excinfo.value)
        assert "total_mass" in errors
        assert "thruster_positions" in errors

    def test_validator_detects_invalid_mpc_horizon(self):
        """Test that validator detects invalid MPC horizon."""
        from src.satellite_control.config.models import AppConfig

        app_config = SimulationConfig.create_default().app_config

        # Create invalid config with control horizon > prediction horizon
        invalid_config = AppConfig(
            physics=app_config.physics,
            mpc=app_config.mpc.model_copy(
                update={"control_horizon": 20, "prediction_horizon": 15}
            ),
            simulation=app_config.simulation,
        )

        validator = ConfigValidator()
        issues = validator.validate_all(invalid_config)

        # Should detect invalid horizon relationship
        assert len(issues) > 0, "Validator should detect invalid horizon"
        assert any("horizon" in issue.lower() for issue in issues), (
            "Should report horizon issue"
        )


# Mark all tests in this file as unit tests
pytestmark = pytest.mark.unit

"""
State Validation Tests.

Tests state vector validity checks, bounds enforcement, and trajectory continuity.
Trims down `test_simulation_state_validator.py`.
"""

import numpy as np
import pytest

from controller.configs.constants import Constants
from controller.shared.python.simulation.state_validator import (
    create_state_validator_from_config,
)


class TestStateValidation:
    """Tests for SimulationStateValidator."""

    @pytest.fixture
    def validator(self, fresh_config):
        return create_state_validator_from_config(app_config=fresh_config.app_config)

    def test_nan_detection(self, validator):
        """Validator should reject NaNs."""
        state = np.zeros(13)
        state[0] = np.nan
        with pytest.raises(ValueError, match="non-finite"):
            validator.validate_state_format(state)

    def test_inf_detection(self, validator):
        """Validator should reject Infs."""
        state = np.zeros(13)
        state[0] = np.inf
        with pytest.raises(ValueError, match="non-finite"):
            validator.validate_state_format(state)

    def test_shape_validation(self, validator):
        """Validator should enforce correct array shape."""
        # Too short
        with pytest.raises(ValueError, match="shape"):
            validator.validate_state_format(np.zeros(10))
        # Too long
        with pytest.raises(ValueError, match="shape"):
            validator.validate_state_format(np.zeros(20))
        # Correct (13 or 16 depending on RW)
        assert validator.validate_state_format(np.zeros(13)) is True

    def test_position_bounds(self, validator):
        """Validator should check position limits."""
        state = np.zeros(13)
        # Assuming default limit is < 100m
        state[0] = 1000.0
        is_valid, msg = validator.check_position_bounds(state[:3])
        assert is_valid is False
        assert msg is not None and "exceeds bounds" in msg

    def test_velocity_bounds(self, validator):
        """Validator should check velocity limits."""
        state = np.zeros(13)
        state[7] = 100.0  # Excessive velocity
        is_valid, msg = validator.check_velocity_bounds(state[7:10])
        assert is_valid is False
        assert msg is not None and "exceeds max" in msg

    def test_trajectory_continuity(self, validator):
        """Test continuity check between steps."""
        state_t0 = np.zeros(13)
        state_t1_good = state_t0.copy()
        state_t1_good[0] += 0.01  # Small step

        state_t1_bad = state_t0.copy()
        state_t1_bad[0] += 10.0  # Huge jump (teleportation)

        history_good = [state_t0, state_t1_good]
        valid_good, err_good = validator.validate_state_trajectory(
            history_good, check_continuity=True, max_position_jump=1.0
        )
        assert valid_good is True

        history_bad = [state_t0, state_t1_bad]
        valid_bad, err_bad = validator.validate_state_trajectory(
            history_bad, check_continuity=True, max_position_jump=1.0
        )
        assert valid_bad is False
        assert any("position jump" in e for e in err_bad)

    def test_default_tolerance_contract_values(self, validator):
        """Default validator tolerances should match the termination contract."""
        assert validator.position_tolerance == pytest.approx(0.1)
        assert validator.angle_tolerance == pytest.approx(Constants.ANGLE_TOLERANCE)
        assert validator.velocity_tolerance == pytest.approx(0.05)
        assert validator.angular_velocity_tolerance == pytest.approx(
            Constants.ANGULAR_VELOCITY_TOLERANCE
        )

    def test_check_within_tolerances_is_inclusive_at_boundary(self, validator):
        current = np.zeros(13, dtype=float)
        reference = np.zeros(13, dtype=float)
        current[3] = 1.0
        reference[3] = 1.0
        current[0] = float(validator.position_tolerance)
        current[7] = float(validator.velocity_tolerance)
        current[10] = float(validator.angular_velocity_tolerance)
        checks = validator.check_within_tolerances(current, reference)
        assert checks["position"] is True
        assert checks["velocity"] is True
        assert checks["angular_velocity"] is True

    def test_hold_hysteresis_thresholds_allow_small_exit_band(self, validator):
        current = np.zeros(13, dtype=float)
        reference = np.zeros(13, dtype=float)
        current[3] = 1.0
        reference[3] = 1.0
        current[0] = float(validator.position_hold_exit_tolerance)
        enter_checks = validator.check_within_tolerances(
            current, reference, hysteresis_mode="enter"
        )
        hold_checks = validator.check_within_tolerances(
            current, reference, hysteresis_mode="hold"
        )
        assert enter_checks["position"] is False
        assert hold_checks["position"] is True

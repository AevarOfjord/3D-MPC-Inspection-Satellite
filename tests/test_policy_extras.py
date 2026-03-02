"""
Unit tests for ActuatorPolicy hysteresis and PointingGuardrail breach/clear timing.

These cover code paths not exercised by the existing test_runtime_policy.py suite.
"""

import numpy as np
import pytest

from controller.shared.python.runtime.policy import ActuatorPolicy, PointingGuardrail


class TestActuatorPolicyHysteresis:
    """Tests for ActuatorPolicy._apply_hysteresis() via .apply()."""

    @pytest.fixture
    def policy(self):
        # Explicit on/off thresholds for easy reasoning in tests
        return ActuatorPolicy(
            enable_thruster_hysteresis=True,
            thruster_hysteresis_on=0.015,
            thruster_hysteresis_off=0.007,
        )

    def _zeros(self, n=6):
        return np.zeros(n, dtype=np.float64)

    def test_actuator_turns_on_above_on_threshold(self, policy):
        """Thruster with previous=0 turns ON when command exceeds on-threshold."""
        previous = self._zeros()
        command = self._zeros()
        command[0] = 0.02  # > 0.015 on-threshold

        result = policy.apply(command, previous, mode="TRACK", endpoint_error_m=None)

        assert result[0] == pytest.approx(0.02), (
            "Should pass through command when turning on"
        )
        np.testing.assert_array_equal(result[1:], 0.0)

    def test_actuator_stays_on_between_thresholds(self, policy):
        """Thruster stays ON when command drops between off- and on-thresholds."""
        previous = self._zeros()
        previous[0] = 0.02  # was active (>= off-threshold 0.007)
        command = self._zeros()
        command[0] = 0.010  # between off=0.007 and on=0.015

        result = policy.apply(command, previous, mode="TRACK", endpoint_error_m=None)

        assert result[0] == pytest.approx(0.010), "Should remain on (hysteresis hold)"

    def test_actuator_turns_off_below_off_threshold(self, policy):
        """Thruster turns OFF when command drops below off-threshold."""
        previous = self._zeros()
        previous[0] = 0.02  # was active
        command = self._zeros()
        command[0] = 0.005  # < off-threshold 0.007

        result = policy.apply(command, previous, mode="TRACK", endpoint_error_m=None)

        assert result[0] == pytest.approx(0.0), "Should turn off below off-threshold"

    def test_complete_mode_returns_all_zeros(self, policy):
        """In COMPLETE mode all thruster outputs are zero regardless of command."""
        previous = np.ones(6, dtype=np.float64)
        command = np.ones(6, dtype=np.float64)

        result = policy.apply(command, previous, mode="COMPLETE", endpoint_error_m=None)

        np.testing.assert_array_equal(result, np.zeros(6))


class TestPointingGuardrailHysteresis:
    """Tests for PointingGuardrail breach/clear hold-time logic."""

    @pytest.fixture
    def guardrail(self):
        return PointingGuardrail(
            enabled=True,
            z_error_deg_max=4.0,
            x_error_deg_max=6.0,
            breach_hold_s=0.30,
            clear_hold_s=0.80,
        )

    def test_guardrail_not_breached_before_hold_duration(self, guardrail):
        """Error above threshold for less than breach_hold_s does not latch breach."""
        # Error starts at t=0
        guardrail.update(sim_time_s=0.0, x_error_deg=None, z_error_deg=5.0)
        # Just under hold duration (0.29 < 0.30)
        status = guardrail.update(sim_time_s=0.29, x_error_deg=None, z_error_deg=5.0)

        assert not status.breached, (
            "Should not be breached before hold duration elapses"
        )

    def test_guardrail_breached_after_hold_duration(self, guardrail):
        """Error above threshold sustained for >= breach_hold_s latches breach."""
        guardrail.update(sim_time_s=0.0, x_error_deg=None, z_error_deg=5.0)
        status = guardrail.update(sim_time_s=0.31, x_error_deg=None, z_error_deg=5.0)

        assert status.breached, "Should be breached after breach_hold_s elapses"
        assert status.last_reason == "z_axis_error"

    def test_guardrail_clears_after_clear_hold(self, guardrail):
        """Once breached, guardrail clears only after clear_hold_s of clean readings."""
        # Establish breach
        guardrail.update(sim_time_s=0.0, x_error_deg=None, z_error_deg=5.0)
        guardrail.update(sim_time_s=0.31, x_error_deg=None, z_error_deg=5.0)
        assert guardrail.status.breached

        # Error clears at t=1.0 — clear_hold_s=0.80 so must hold until t=1.80
        guardrail.update(sim_time_s=1.0, x_error_deg=None, z_error_deg=0.0)
        still_breached = guardrail.update(
            sim_time_s=1.79, x_error_deg=None, z_error_deg=0.0
        )
        assert still_breached.breached, (
            "Should remain breached before clear hold elapses"
        )

        cleared = guardrail.update(sim_time_s=1.81, x_error_deg=None, z_error_deg=0.0)
        assert not cleared.breached, "Should clear after clear_hold_s elapses"

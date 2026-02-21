"""
Property-Based Tests for Satellite Control System.

Uses Hypothesis to generate edge cases and test invariants for math and state.
Retains high-value tests from previous `test_property_based.py`.
"""

import numpy as np
import pytest

try:
    from hypothesis import given, settings
    from hypothesis import strategies as st

    HYPOTHESIS_AVAILABLE = True
except ImportError:
    HYPOTHESIS_AVAILABLE = False

    # Keep module importable when hypothesis is missing so skipif can apply.
    def _identity_decorator(*args, **kwargs):
        def _wrap(fn):
            return fn

        return _wrap

    class _StrategiesStub:
        def __getattr__(self, _name):
            def _stub(*args, **kwargs):
                return None

            return _stub

    given = _identity_decorator
    settings = _identity_decorator
    st = _StrategiesStub()


@pytest.mark.skipif(not HYPOTHESIS_AVAILABLE, reason="hypothesis not installed")
class TestMathInvariants:
    """Property-based tests for mathematical invariants."""

    @given(theta=st.floats(-100.0, 100.0))
    @settings(max_examples=50)
    def test_angle_normalization_range(self, theta):
        """Normalized angle is always within [-pi, pi]."""
        from satellite_control.utils.navigation_utils import normalize_angle

        norm = normalize_angle(theta)
        assert -np.pi <= norm <= np.pi

    @given(
        angle1=st.floats(-10.0, 10.0),
        angle2=st.floats(-10.0, 10.0),
    )
    @settings(max_examples=50)
    def test_angle_difference_symmetry(self, angle1, angle2):
        """Angle difference is antisymmetric (diff(a,b) == -diff(b,a))."""
        from satellite_control.utils.navigation_utils import angle_difference

        d1 = angle_difference(angle1, angle2)
        d2 = angle_difference(angle2, angle1)
        # Handle wrap around edge cases where diff is exactly pi/-pi
        if abs(abs(d1) - np.pi) < 1e-9:
            # If d1 is pi, d2 could be pi or -pi
            assert abs(abs(d2) - np.pi) < 1e-9
        else:
            assert np.isclose(d1, -d2, atol=1e-9)

    @given(
        roll=st.floats(-np.pi, np.pi),
        pitch=st.floats(-np.pi / 2, np.pi / 2),
        yaw=st.floats(-np.pi, np.pi),
    )
    @settings(max_examples=50)
    def test_quaternion_normalization(self, roll, pitch, yaw):
        """Quaternions from Euler angles are always unit length."""
        from satellite_control.utils.orientation_utils import euler_xyz_to_quat_wxyz

        q = euler_xyz_to_quat_wxyz((roll, pitch, yaw))
        assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-9)


@pytest.mark.skipif(not HYPOTHESIS_AVAILABLE, reason="hypothesis not installed")
class TestStateInvariants:
    """Property-based tests for state vector validity."""

    @given(
        x=st.floats(-10.0, 10.0),
        y=st.floats(-10.0, 10.0),
        z=st.floats(-10.0, 10.0),
    )
    @settings(max_examples=20)
    def test_state_validator_completeness(self, x, y, z):
        """Validator should handle any finite float input without crashing."""
        from satellite_control.config.simulation_config import SimulationConfig
        from satellite_control.utils.simulation_state_validator import (
            create_state_validator_from_config,
        )

        cfg = SimulationConfig.create_default()
        # Fix: pass app_config correctly
        validator = create_state_validator_from_config(app_config=cfg.app_config)

        state = np.zeros(13)
        state[0:3] = [x, y, z]
        state[3] = 1.0  # Valid quat

        # Should return bool, not raise
        # check_safety_bounds does not exist, use check_all_bounds which returns (bool, list)
        valid, errors = validator.check_all_bounds(state)
        assert isinstance(valid, bool)
        assert isinstance(errors, list)

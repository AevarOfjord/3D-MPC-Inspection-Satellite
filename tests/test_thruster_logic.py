"""
Thruster Logic Tests.

Tests thruster management, PWM vs continuous modes, and realistic physics artifacts
(valve delays, impulse bits). Consolidates `test_thruster_manager.py` and `test_thruster_toggle.py`.
"""

import numpy as np
import pytest
from satellite_control.core.thruster_manager import ThrusterManager


class TestThrusterLogic:
    """Tests for ThrusterManager logic."""

    @pytest.fixture
    def manager(self):
        return ThrusterManager(num_thrusters=6)

    def test_continuous_mode_passthrough(self, manager):
        """In continuous mode, commands should pass through directly."""
        cmd = np.array([0.1, 0.5, 0.9, 0.0, 0.0, 0.0])
        # Force continuous mode
        manager.thruster_type = "CON"
        manager.use_realistic_physics = False

        manager.set_thruster_pattern(cmd, simulation_time=0.0)
        # Must call process to update output
        manager.process_command_queue(
            simulation_time=0.0,
            control_update_interval=0.1,
            last_control_update=0.0,
            sim_dt=0.01,
        )

        output = manager.get_actual_output()
        np.testing.assert_allclose(output, cmd)

    def test_pwm_duty_cycle(self, manager):
        """Test PWM duty cycle conversion."""
        manager.thruster_type = "PWM"
        control_interval = 0.1  # 10Hz

        # 50% duty cycle
        cmd = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Start of cycle (t=0)
        manager.set_thruster_pattern(cmd, simulation_time=0.0)
        manager.process_command_queue(0.0, control_interval, 0.0, 0.001)
        assert manager.get_actual_output()[0] == 1.0

        # Mid cycle (t=0.04 < 0.05) - still ON
        manager.process_command_queue(0.04, control_interval, 0.0, 0.001)
        assert manager.get_actual_output()[0] == 1.0

        # End of cycle (t=0.06 > 0.05) - should be OFF
        manager.process_command_queue(0.06, control_interval, 0.0, 0.001)
        assert manager.get_actual_output()[0] == 0.0

    def test_valve_delay(self, manager):
        """Test simulated valve delay."""
        manager.use_realistic_physics = True
        manager.VALVE_DELAY = 0.01  # 10ms delay
        manager.thruster_type = "PWM"  # Must be PWM for valve logic to apply

        cmd = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Command set at t=0
        manager.set_thruster_pattern(cmd, simulation_time=0.0)

        # Process at t=0.005 (5ms < 10ms delay) -> Should be 0
        manager.process_command_queue(0.005, 0.1, 0.0, 0.001)
        assert manager.get_actual_output()[0] == 0.0

        # Process output at t=0.015 (15ms > 10ms delay) -> Should be > 0 (ramp up starts)
        manager.process_command_queue(0.015, 0.1, 0.0, 0.001)
        # If ramp up is 0, it should be 1.0 immediately
        if manager.THRUST_RAMPUP_TIME == 0:
            assert manager.get_actual_output()[0] == 1.0
        else:
            assert manager.get_actual_output()[0] > 0.0

    def test_ramp_up_dynamics(self, manager):
        """Test thrust ramp-up over time."""
        manager.use_realistic_physics = True
        manager.thruster_type = "PWM"

        manager.THRUST_RAMPUP_TIME = 0.05  # 50ms
        manager.VALVE_DELAY = 0.0

        cmd = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Full on
        manager.set_thruster_pattern(cmd, simulation_time=0.0)

        # Force open command time to be 0
        manager.thruster_open_command_time[0] = 0.0

        # At t=25ms (50% of ramp), thrust should be approx 0.5
        manager.process_command_queue(0.025, 0.1, 0.0, 0.001)
        out_mid = manager.get_actual_output()[0]
        assert 0.4 < out_mid < 0.6

        # At t=100ms (full ramp), thrust should be 1.0
        manager.process_command_queue(0.1, 0.1, 0.0, 0.001)
        out_full = manager.get_actual_output()[0]
        assert out_full == pytest.approx(1.0, abs=0.01)

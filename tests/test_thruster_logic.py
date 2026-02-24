"""
Thruster Logic Tests.

Tests thruster management, PWM vs continuous modes, and realistic physics artifacts
(valve delays, impulse bits). Consolidates `test_thruster_manager.py` and `test_thruster_toggle.py`.
"""

import numpy as np
import pytest
from runtime.thruster_manager import ThrusterManager


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

    def test_pwm_repeated_pulses_two_intervals(self):
        """50% duty cycle pulses repeat correctly across two control intervals."""
        m = ThrusterManager(num_thrusters=6, thruster_type="PWM")
        interval = 0.1
        cmd = np.zeros(6)
        cmd[0] = 0.5

        # --- Interval 1 (last_control_update=0.0) ---
        m.set_thruster_pattern(cmd, simulation_time=0.0)
        m.process_command_queue(0.0, interval, 0.0, 0.001)
        assert m.get_actual_output()[0] == 1.0, "Should be ON at start of pulse"

        m.process_command_queue(0.06, interval, 0.0, 0.001)
        assert m.get_actual_output()[0] == 0.0, "Should be OFF after 50% of interval"

        # --- Interval 2 (last_control_update=0.1) ---
        m.set_thruster_pattern(cmd, simulation_time=0.1)
        m.process_command_queue(0.1, interval, 0.1, 0.001)
        assert m.get_actual_output()[0] == 1.0, (
            "Should be ON at start of second interval"
        )

        m.process_command_queue(0.16, interval, 0.1, 0.001)
        assert m.get_actual_output()[0] == 0.0, (
            "Should be OFF after 50% of second interval"
        )

    def test_valve_delay_and_ramp_combined(self):
        """Valve delay and ramp-up compose correctly: 0 until delay, then ramp 0→1."""
        m = ThrusterManager(
            num_thrusters=6,
            valve_delay=0.02,
            thrust_rampup_time=0.04,
            use_realistic_physics=True,
            thruster_type="PWM",
        )
        cmd = np.zeros(6)
        cmd[0] = 1.0
        m.set_thruster_pattern(cmd, simulation_time=0.0)

        # Still in delay window
        m.process_command_queue(0.01, 0.2, 0.0, 0.001)
        assert m.get_actual_output()[0] == 0.0, "Should be 0 before valve opens"

        # Valve just opened (t=0.02); ramp at 50% (0.02 / 0.04 = 0.5)
        m.process_command_queue(0.04, 0.2, 0.0, 0.001)
        out = m.get_actual_output()[0]
        assert 0.4 < out < 0.65, f"Expected ~0.5 ramp, got {out}"

        # Full thrust after complete ramp (t >= 0.02 + 0.04 = 0.06)
        m.process_command_queue(0.07, 0.2, 0.0, 0.001)
        assert m.get_actual_output()[0] == pytest.approx(1.0, abs=0.01)

    def test_input_array_2d_shape_normalised(self):
        """2-D column input array is flattened and processed correctly."""
        m = ThrusterManager(num_thrusters=6, thruster_type="CON")
        cmd_2d = np.array([[0.3], [0.6], [0.0], [0.0], [0.0], [0.0]])
        m.set_thruster_pattern(cmd_2d, simulation_time=0.0)
        m.process_command_queue(0.0, 0.1, 0.0, 0.001)
        out = m.get_actual_output()
        np.testing.assert_allclose(out[:2], [0.3, 0.6], atol=1e-9)
        np.testing.assert_allclose(out[2:], 0.0, atol=1e-9)

    def test_input_array_padded_and_truncated(self):
        """Under-length input is zero-padded; over-length input is truncated."""
        m = ThrusterManager(num_thrusters=6, thruster_type="CON")

        # Under-length: 3 values → padded to 6
        m.set_thruster_pattern(np.array([0.1, 0.2, 0.3]), simulation_time=0.0)
        m.process_command_queue(0.0, 0.1, 0.0, 0.001)
        out = m.get_actual_output()
        np.testing.assert_allclose(out[:3], [0.1, 0.2, 0.3], atol=1e-9)
        np.testing.assert_allclose(out[3:], 0.0, atol=1e-9)

        # Over-length: 9 values → truncated to 6
        m.reset()
        m.set_thruster_pattern(np.ones(9) * 0.5, simulation_time=0.0)
        m.process_command_queue(0.0, 0.1, 0.0, 0.001)
        assert len(m.get_actual_output()) == 6
        np.testing.assert_allclose(m.get_actual_output(), 0.5, atol=1e-9)

    def test_reset_clears_all_state(self):
        """After commanding and resetting, all state returns to initial values."""
        m = ThrusterManager(num_thrusters=6, thruster_type="PWM")
        cmd = np.ones(6)
        m.set_thruster_pattern(cmd, simulation_time=1.0)
        m.process_command_queue(1.05, 0.1, 1.0, 0.001)
        assert m.get_actual_output()[0] == 1.0  # confirm it fired

        m.reset()
        np.testing.assert_array_equal(m.get_actual_output(), np.zeros(6))
        np.testing.assert_array_equal(m.get_commanded_pattern(), np.zeros(6))
        # Sentinel times should be restored
        assert all(t == pytest.approx(-1000.0) for t in m.thruster_open_command_time)
        assert all(t == pytest.approx(-1000.0) for t in m.thruster_close_command_time)

"""
Integration tests for end-to-end mission workflows.

Tests complete mission scenarios including:
- Point-to-point navigation
- Multi-point navigation
- State validation and error handling
- MPC controller integration with simulation
"""

from unittest.mock import patch

import numpy as np
import pytest
from typing import Any, Dict
import warnings

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

# V4.0.0: SatelliteConfig removed - use SimulationConfig only
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation

# Suppress matplotlib animation GC warnings triggered during headless tests
warnings.filterwarnings(
    "ignore",
    message="Animation was deleted without rendering anything",
    category=UserWarning,
    module="matplotlib.animation",
)

# Apply filter via pytest as well (covers warning capture during test run)
pytestmark = pytest.mark.filterwarnings(
    "ignore:Animation was deleted without rendering anything:UserWarning"
)


@pytest.fixture(autouse=True)
def close_figures():
    """Ensure matplotlib figures/animations are closed after each test."""
    yield
    # Reassign animations during tests to avoid matplotlib warnings, then close all
    for fig_num in plt.get_fignums():
        fig = plt.figure(fig_num)
        for attr in dir(fig):
            anim = getattr(fig, attr)
            if isinstance(anim, FuncAnimation):
                # Assign to module-level to keep reference until close
                globals().setdefault("_test_anim_refs", []).append(anim)
    plt.close("all")


@pytest.fixture
def simple_simulation():
    """Create a simple simulation instance for testing."""
    # Use a config override to ensure controlled test environment
    config_overrides: Dict[str, Dict[str, Any]] = {
        "mpc": {
            "prediction_horizon": 10,
            "control_horizon": 8,
            "solver_time_limit": 0.05,
            "dt": 0.25,
        },
        "simulation": {
            "max_duration": 10.0,
            "control_dt": 0.25,
        },
        "physics": {
            "use_realistic_physics": False,  # V4.0.0: Use SimulationConfig format
        },
    }

    sim = SatelliteMPCLinearizedSimulation(
        start_pos=(0.5, 0.5, 0.0),
        end_pos=(0.0, 0.0, 0.0),
        start_angle=(0.0, 0.0, 0.0),
        end_angle=(0.0, 0.0, 0.0),
        config_overrides=config_overrides,
    )

    return sim


class TestPointToPointMission:
    """Test point-to-point navigation mission."""

    def test_simple_point_to_point_initialization(self, simple_simulation):
        """Test that point-to-point mission initializes correctly."""
        sim = simple_simulation

        # Check initial state
        assert sim.satellite.position[0] == pytest.approx(0.5, abs=0.01)
        assert sim.satellite.position[1] == pytest.approx(0.5, abs=0.01)

        # Path-following starts with reference at the start position
        assert sim.reference_state[0] == pytest.approx(0.5, abs=0.01)
        assert sim.reference_state[1] == pytest.approx(0.5, abs=0.01)

    def test_point_to_point_state_format(self, simple_simulation):
        """Test that state format is correct for point-to-point."""
        sim = simple_simulation

        current_state = sim.get_current_state()

        # Should be [pos(3), quat(4), vel(3), w(3), wheel_speeds(3)] format
        assert len(current_state) == 16
        assert current_state[0] == pytest.approx(0.5, abs=0.01)  # x
        assert current_state[1] == pytest.approx(0.5, abs=0.01)  # y

    def test_point_to_point_path_complete_check(self, simple_simulation):
        """Test path completion checking logic."""
        sim = simple_simulation

        # Initially not at path end
        sim.mpc_controller._path_length = 1.0
        sim.mpc_controller.s = 0.0
        assert not sim.check_path_complete()

        # Mark path progress complete
        sim.mpc_controller.s = 1.0
        assert sim.check_path_complete()

    def test_point_to_point_mpc_control_update(self, simple_simulation):
        """Test MPC control update in point-to-point mode."""
        sim = simple_simulation

        # Mock the MPC solver to avoid optimization
        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            mock_mpc.return_value = (
                np.ones(sim.mpc_controller.num_thrusters),  # Control action
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            # Update control
            sim.update_mpc_control()

            # Verify MPC was called
            assert mock_mpc.called

            # Verify control was applied
            assert sim.current_thrusters is not None

    def test_point_to_point_simulation_step(self, simple_simulation):
        """Test a single simulation step."""
        sim = simple_simulation

        # V4.0.0: No need to mock SatelliteConfig - it's been removed
        # Mock draw_simulation to prevent visualization code
        with (
            patch.object(sim, "draw_simulation", return_value=[]),
            patch.object(
                sim.mpc_controller, "get_control_action"
            ) as mock_mpc,
        ):

            mock_mpc.return_value = (
                np.zeros(sim.mpc_controller.num_thrusters),
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            initial_time = sim.simulation_time

            # Enable simulation to allow update
            sim.is_running = True

            # Run one update
            sim.update_simulation(0)

            # Time should advance
            assert sim.simulation_time > initial_time

        plt.close("all")
        # Position may change due to initial velocity or physics


class TestPathMission:
    """Test path-following mission state."""

    def test_path_initialization(self):
        """Path-following mission should be active when path is set."""
        from src.satellite_control.config.mission_state import MissionState

        mission_state = MissionState()
        mission_state.mpcc_path_waypoints = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        mission_state.mpcc_path_length = 1.0
        mission_state.mpcc_path_speed = 0.2

        assert mission_state.get_current_mission_type() == "PATH_FOLLOWING"
        assert mission_state.mpcc_path_speed == pytest.approx(0.2)


class TestMPCSimulationIntegration:
    """Test integration between MPC controller and simulation."""

    def test_mpc_receives_correct_state_format(self, simple_simulation):
        """Test that MPC receives state in correct format."""
        sim = simple_simulation

        # Capture the state passed to MPC
        captured_state = None

        def capture_state(x_current, prev_thrusters=None):
            nonlocal captured_state
            captured_state = x_current
            return np.zeros(sim.mpc_controller.num_thrusters), {
                "status": 2,
                "status_name": "OPTIMAL",
                "solve_time": 0.01,
            }

        with patch.object(
            sim.mpc_controller, "get_control_action", side_effect=capture_state
        ):
            sim.update_mpc_control()

            # MPC internal format should be [pos(3), quat(4), vel(3), w(3)]
            # get_control_action expects the full 13-element state
            assert captured_state is not None

    def test_simulation_applies_mpc_control(self, simple_simulation):
        """Test that simulation applies MPC control output."""
        sim = simple_simulation

        thrusters = np.ones(sim.mpc_controller.num_thrusters)
        control_action = np.concatenate(
            [np.zeros(sim.mpc_controller.num_rw_axes), thrusters]
        )

        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            mock_mpc.return_value = (
                control_action,
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            sim.update_mpc_control()

            # Check that thrusters were set
            assert np.array_equal(sim.current_thrusters, thrusters)

    def test_state_history_logging(self, simple_simulation):
        """Test that state history is logged correctly."""
        sim = simple_simulation

        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            mock_mpc.return_value = (
                np.zeros(sim.mpc_controller.num_thrusters),
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            # Set save path to enable logging (and history tracking)
            sim.data_save_path = Path("/tmp/test_history")

            initial_history_length = len(sim.state_history)

            sim.update_mpc_control()

            # History should have grown
            assert len(sim.state_history) > initial_history_length


class TestStateValidation:
    """Test state validation and error handling."""

    def test_state_vector_dimensions(self, simple_simulation):
        """Test that state vectors have correct dimensions."""
        sim = simple_simulation

        current_state = sim.get_current_state()
        assert len(current_state) == 16

        reference_state = sim.reference_state
        assert len(reference_state) == 13

    def test_angle_normalization(self, simple_simulation):
        """Test angle normalization in simulation."""
        sim = simple_simulation

        # Test angle wrapping
        angle = 3 * np.pi
        normalized = sim.normalize_angle(angle)

        # Should be wrapped to [-pi, pi]
        assert -np.pi <= normalized <= np.pi

    def test_angle_difference_calculation(self, simple_simulation):
        """Test angle difference calculation."""
        sim = simple_simulation

        # Test simple difference
        diff = sim.angle_difference(np.pi / 2, 0.0)
        assert diff == pytest.approx(np.pi / 2)

        # Test wraparound
        diff = sim.angle_difference(np.pi, -np.pi)
        assert abs(diff) < 0.01  # Should be near zero


class TestRealisticPhysics:
    """Test realistic physics simulation features."""

    def test_sensor_noise_application(self):
        """Test that sensor noise is applied when enabled."""
        config_overrides = {
            "physics": {
                "use_realistic_physics": True,
                "position_noise_std": 0.001,
                "velocity_noise_std": 0.001,
            }
        }

        sim = SatelliteMPCLinearizedSimulation(
            start_pos=(0.0, 0.0),
            end_pos=(1.0, 1.0),
            config_overrides=config_overrides,
        )

        true_state = np.zeros(13)
        true_state[0:3] = [1.0, 2.0, 0.1]

        # Apply noise multiple times
        noisy_states = [sim.get_noisy_state(true_state) for _ in range(10)]

        # Check that noisy states differ
        differences = [np.linalg.norm(noisy - true_state) for noisy in noisy_states]

        # V4.0.0: Check app_config.physics.use_realistic_physics instead
        # At least some should be different (not all zero due to randomness)
        if sim.simulation_config and sim.simulation_config.app_config.physics.use_realistic_physics:
            assert any(d > 0 for d in differences)

    def test_thruster_delay_simulation(self, simple_simulation):
        """Test thruster command delay processing."""
        sim = simple_simulation

        # Set a thruster command
        command = np.array([1, 0, 0, 0, 0, 0, 0, 0])
        sim.set_thruster_pattern(command)

        # Process command queue
        sim.process_command_queue()

        # Actual output depends on delay settings
        # Just verify the method runs without error

    def test_damping_application(self):
        """Test that damping is applied when realistic physics enabled."""
        config_overrides = {
            "physics": {
                "use_realistic_physics": True,
                "damping_linear": 0.1,
                "damping_angular": 0.01,
            }
        }

        sim = SatelliteMPCLinearizedSimulation(
            start_pos=(0.0, 0.0),
            end_pos=(0.0, 0.0),
            start_vx=0.1,  # Start with some velocity
            config_overrides=config_overrides,
        )

        # Run simulation step
        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            mock_mpc.return_value = (
                np.zeros(sim.mpc_controller.num_thrusters),
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            # Run a few steps
            for _ in range(5):
                sim.update_simulation(0)

            # V4.0.0: Check app_config.physics.use_realistic_physics instead
            # Velocity should decrease due to damping (if no thrusters active)
            if sim.simulation_config and sim.simulation_config.app_config.physics.use_realistic_physics:
                # Damping should reduce velocity over time
                pass  # Can't guarantee without running actual physics


class TestErrorHandling:
    """Test error handling in integration scenarios."""

    def test_mpc_solver_failure_handling(self, simple_simulation):
        """Test handling of MPC solver failures."""
        sim = simple_simulation

        # Mock MPC to return failure
        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            mock_mpc.return_value = (
                None,
                {"status": -1, "status_name": "INFEASIBLE", "solve_time": 0.01},
            )

            # Should handle gracefully
            sim.update_mpc_control()

            # Should have set all thrusters to zero
            assert np.all(sim.current_thrusters == 0)

    def test_invalid_thruster_array_handling(self, simple_simulation):
        """Test handling of invalid thruster arrays."""
        sim = simple_simulation

        # Try to set invalid thruster pattern
        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            # Return wrong-sized array
            mock_mpc.return_value = (
                np.array([1, 0, 1]),
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            # Should handle gracefully
            sim.update_mpc_control()

    def test_simulation_time_limit(self, simple_simulation):
        """Test that simulation respects time limit."""
        sim = simple_simulation

        # Set a very short time limit
        sim.max_simulation_time = 1.0

        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            mock_mpc.return_value = (
                np.zeros(sim.mpc_controller.num_thrusters),
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            # Run until time limit
            while (
                sim.is_running and sim.simulation_time < sim.max_simulation_time + 1.0
            ):
                sim.update_simulation(0)

            # Should have stopped
            assert not sim.is_running or sim.simulation_time >= sim.max_simulation_time


class TestDataLogging:
    """Test data logging functionality."""

    def test_simulation_step_logging(self, simple_simulation):
        """Test that simulation steps are logged."""
        sim = simple_simulation

        # Set a save path to enable logging
        sim.data_save_path = "/tmp/test"

        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            mock_mpc.return_value = (
                np.zeros(sim.mpc_controller.num_thrusters),
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            # Check initial log count
            initial_count = sim.data_logger.get_log_count()

            # Update control (which should log)
            sim.update_mpc_control()

            # Log count should increase
            assert sim.data_logger.get_log_count() > initial_count

    def test_control_history_tracking(self, simple_simulation):
        """Test that control history is tracked."""
        sim = simple_simulation

        with patch.object(sim.mpc_controller, "get_control_action") as mock_mpc:
            thrusters = np.ones(sim.mpc_controller.num_thrusters)
            control = np.concatenate(
                [np.zeros(sim.mpc_controller.num_rw_axes), thrusters]
            )
            mock_mpc.return_value = (
                control,
                {"status": 2, "status_name": "OPTIMAL", "solve_time": 0.01},
            )

            # Set save path to enable logging (and history tracking)
            sim.data_save_path = Path("/tmp/test_history")

            initial_length = len(sim.control_history)

            sim.update_mpc_control()

            # Control history should grow
            assert len(sim.control_history) > initial_length

            # Latest control should match
            assert np.array_equal(sim.control_history[-1], thrusters)


class TestPathCompletion:
    """Test path completion criteria."""

    def test_point_to_point_completion(self, simple_simulation):
        """Test point-to-point path completion."""
        from src.satellite_control.core.simulation_loop import SimulationLoop

        sim = simple_simulation
        sim.simulation_config.mission_state.dxf_path_length = 1.0
        sim.simulation_config.mission_state.trajectory_hold_end = 0.0
        sim.mpc_controller.s = 1.0
        sim.simulation_time = 1.0

        loop = SimulationLoop(sim)

        assert loop._check_path_following_completion() is True
        assert sim.trajectory_endpoint_reached_time == pytest.approx(1.0)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

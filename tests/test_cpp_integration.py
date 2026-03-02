"""
C++ Simulation Engine Integration Tests.

Validates that the C++ simulation backend (and Python bindings) works correctly.
Consolidates `simulation/test_cpp_sim.py`, `integration/test_orbital.py`, and `integration/test_rw_control.py`.
"""

import numpy as np
import pytest

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.control_common.mpc_controller import MPCController
from controller.shared.python.simulation.cpp_backend import CppSatelliteSimulator
from controller.shared.python.simulation.engine import SatelliteMPCLinearizedSimulation


class TestCPPEngine:
    """Tests for the C++ simulation engine."""

    def test_cpp_mpc_params_fallback_fields(self):
        """C++ MPCV2Params binding should expose bounded fallback policy fields."""
        cpp_mpc = pytest.importorskip("cpp._cpp_mpc")
        params = cpp_mpc.MPCV2Params()

        params.solver_fallback_hold_s = 0.4
        params.solver_fallback_decay_s = 0.5
        params.solver_fallback_zero_after_s = 1.1

        assert params.solver_fallback_hold_s == pytest.approx(0.4)
        assert params.solver_fallback_decay_s == pytest.approx(0.5)
        assert params.solver_fallback_zero_after_s == pytest.approx(1.1)

    def test_engine_initialization(self):
        """Test that C++ engine is initialized when configured."""
        config = SimulationConfig.create_with_overrides({"physics": {"engine": "cpp"}})
        sim = SatelliteMPCLinearizedSimulation(simulation_config=config)

        assert isinstance(sim.satellite, CppSatelliteSimulator)
        assert sim.satellite.engine is not None

    def test_physics_stepping(self):
        """Test that C++ engine advances state correctly."""
        config = SimulationConfig.create_with_overrides({"physics": {"engine": "cpp"}})
        # Use CppSatelliteSimulator directly to isolate from Wrapper logic
        sim = CppSatelliteSimulator(app_config=config.app_config)

        # Explicitly set velocity to ensure movement
        sim.velocity = np.array([0.1, 0.0, 0.0], dtype=np.float64)

        initial_x = sim.position[0]
        dt = sim.dt

        # Verify velocity was set correctly
        assert sim.velocity[0] == pytest.approx(0.1)

        # Step for 0.1s
        steps = int(0.1 / dt)
        if steps < 1:
            steps = 1

        for _ in range(steps):
            sim.update_physics(dt)

        final_x = sim.position[0]
        final_v = sim.velocity[0]

        # Should have moved: x = x0 + v * t = 0 + 0.1 * (steps * dt)
        expected_dist = 0.1 * steps * dt

        if not (final_x > initial_x):
            pytest.fail(
                f"Satellite did not move. InitX={initial_x}, FinalX={final_x}, "
                f"InitV=0.1, FinalV={final_v}, dt={dt}, steps={steps}"
            )

        assert np.isclose(final_x, expected_dist, rtol=0.1)


class TestReactionWheelControl:
    """Tests for Reaction Wheel integration."""

    @pytest.mark.slow
    def test_closed_loop_rw_control(self):
        """Run a short closed-loop test with RW control."""
        # Setup config with RW enabled
        config = SimulationConfig.create_default()
        config.app_config.simulation.dt = 0.01

        controller = MPCController(config.app_config)
        sim = CppSatelliteSimulator(app_config=config.app_config)

        # Set a path for the MPCC controller
        controller.set_path([(0, 0, 0), (10, 0, 0)])

        # Run loop
        duration = 0.1  # Short duration just to check integration
        t = 0.0
        dt = 0.01
        control_steps = 0
        successful_solves = 0

        while t < duration:
            # Build state (16 elements — controller handles augmentation)
            state = np.concatenate(
                [
                    sim.position,
                    sim.quaternion,
                    sim.velocity,
                    sim.angular_velocity,
                    sim.wheel_speeds,
                ]
            )

            # Control
            u, info = controller.get_control_action(state)
            if u is None:
                pytest.fail(
                    "Controller returned None control action during closed-loop run"
                )

            rw_cmds, thruster_cmds = controller.split_control(u)
            control_steps += 1
            if info.get("status") == 1:
                successful_solves += 1

            # Actuate
            limits = np.array([0.06, 0.06, 0.06])  # Mock limits
            sim.set_reaction_wheel_torque(rw_cmds * limits)
            sim.apply_force(list(thruster_cmds))
            sim.update_physics(dt)
            t += dt

        assert control_steps > 0, "Closed-loop test never executed a control step"

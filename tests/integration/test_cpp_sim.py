"""
Integration test for C++ Simulation Engine.
Verifies that the simulation can run using the _cpp_sim backend.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

from src.satellite_control.config import SimulationConfig
from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation
from src.satellite_control.core.cpp_satellite import CppSatelliteSimulator


def test_cpp_engine_initialization():
    """Test that C++ engine is initialized when configured."""
    # Create config with C++ engine override
    sim_config = SimulationConfig.create_with_overrides({"physics": {"engine": "cpp"}})

    # Initialize simulation
    # Note: cfg argument is deprecated/legacy used for hydration but we pass simulation_config directly
    sim = SatelliteMPCLinearizedSimulation(
        cfg=None,
        simulation_config=sim_config,
    )

    # Check satellite type
    assert isinstance(sim.satellite, CppSatelliteSimulator), (
        "Failed to initialize CppSatelliteSimulator"
    )
    assert sim.satellite.engine is not None, "C++ Engine implementation is None"


def test_cpp_engine_stepping():
    """Test that C++ engine advances state."""
    sim_config = SimulationConfig.create_with_overrides({"physics": {"engine": "cpp"}})

    sim = SatelliteMPCLinearizedSimulation(
        cfg=None,
        simulation_config=sim_config,
        start_pos=(1.0, 0.0, 0.0),  # Will be overridden by init params if not careful?
        # SatelliteMPCLinearizedSimulation uses legacy kwargs for start_pos overrides
        # but delegates to SimulationInitializer which uses them.
        start_vx=0.1,
    )

    # Run a few steps manually
    sim.is_running = True
    sim.simulation_time = 0.0

    initial_x = sim.satellite.position[0]

    # Run update loop for 0.1s
    # In batch mode logic (simulation_loop), it calls update_physics
    dt = sim.satellite.dt
    steps = int(0.1 / dt)

    for _ in range(steps):
        sim.process_command_queue()
        sim.satellite.update_physics(dt)
        sim.simulation_time += dt
        sim.log_physics_step()

    final_x = sim.satellite.position[0]

    # Verify movement (x should increase due to vx)
    # x approx 1.0 + 0.1 * 0.1 = 1.01
    assert final_x > initial_x, "Satellite did not move in X direction"
    assert np.isclose(final_x, 1.0 + 0.1 * 0.1, rtol=0.1), (
        f"Movement incorrect: {final_x} vs {1.0 + 0.1 * 0.1}"
    )

    # Verify quaternion normalization
    q = sim.satellite.quaternion
    assert np.isclose(np.linalg.norm(q), 1.0), "Quaternion not normalized"


if __name__ == "__main__":
    sys.exit(pytest.main(["-v", __file__]))

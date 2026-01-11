#!/usr/bin/env python3
"""
Verification script for Single Thruster + RW mode.
"""
import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation
from src.satellite_control.config.simulation_config import SimulationConfig


def main():
    print("Initializing Single Thruster Simulation Verification...")

    # Create configuration override for One Thruster mode
    config_overrides = {
        "actuator": {"mode": "one_thruster_rw"},
        "simulation": {"max_duration": 5.0},  # Short run to verify startup
    }

    try:
        # Initialize simulation configuration with overrides
        sim_config = SimulationConfig.create_with_overrides(config_overrides)

        # Initialize simulation
        sim = SatelliteMPCLinearizedSimulation(
            start_pos=(0.0, 0.0, 0.0),
            target_pos=(1.0, 0.0, 0.0),  # Target 1m forward
            simulation_config=sim_config,
        )

        print("Simulation initialized successfully.")
        print(f"Actuator Mode: {sim.simulation_config.app_config.actuator.mode}")
        print(f"Model Path: {sim.simulation_config.app_config.actuator.model_path}")

        # Run brief simulation (headless)
        print("Running simulation (headless)...")
        sim.run_simulation(show_animation=False)
        print("Simulation completed successfully.")

    except Exception as e:
        print(f"FAILED: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

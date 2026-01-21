"""
Simulation Test Modes - Launchpad for various simulation scenarios.
"""

import argparse

# V4.0.0: SatelliteConfig removed - use SimulationConfig only
from src.satellite_control.core.simulation import (
    SatelliteMPCLinearizedSimulation,
)


def main():
    """Main entry point for simulation execution."""
    parser = argparse.ArgumentParser(description="Satellite Simulation")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Run in auto mode with default parameters (skip prompts)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Override max simulation time in seconds",
    )
    parser.add_argument(
        "--no-anim",
        action="store_true",
        help="Disable animation",
    )

    args = parser.parse_args()

    # V3.0.0: Create SimulationConfig instead of mutating SatelliteConfig
    from src.satellite_control.config.simulation_config import SimulationConfig
    
    simulation_config = None
    
    print("Running in path-only mode with default parameters...")
    simulation_config = SimulationConfig.create_default()

    # Create and run simulation
    print("\nInitializing Simulation...")
    
    # Handle duration override (v3.0.0: should be passed via config_overrides)
    config_overrides = None
    if args.duration:
        if simulation_config is None:
            simulation_config = SimulationConfig.create_default()
        # Create a new config with updated max_duration
        from copy import deepcopy
        updated_simulation = deepcopy(simulation_config.app_config.simulation)
        updated_simulation.max_duration = args.duration
        updated_app_config = deepcopy(simulation_config.app_config)
        updated_app_config.simulation = updated_simulation
        simulation_config = SimulationConfig(
            app_config=updated_app_config,
            mission_state=simulation_config.mission_state
        )

    sim = SatelliteMPCLinearizedSimulation(simulation_config=simulation_config)

    print("Starting Simulation...")
    try:
        sim.run_simulation(show_animation=not args.no_anim)
    except KeyboardInterrupt:
        print("\nSimulation stopping...")
    finally:
        sim.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Simple Path Motion Test

Tests basic MPC functionality by moving from (0,0,0) to (1,0,0) along a path.

Usage:
    ./.venv311/bin/python3 scripts/run_simple_motion_test.py
"""

import numpy as np

# Project imports
from src.satellite_control.config.defaults import create_default_app_config
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.config.mission_state import create_mission_state
from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation


def run_simple_motion_test():
    """Run a simple point-to-point motion test."""

    print("=" * 60)
    print("SIMPLE MOTION TEST: (0,0,0) -> (1,0,0)")
    print("=" * 60)

    # Create default config (path-only)
    app_config = create_default_app_config()

    # Simulation settings
    app_config.simulation.max_duration = 30.0  # 30 seconds
    app_config.simulation.headless = True

    print(f"\nConfiguration:")
    print(f"  Start: (0, 0, 0) m")
    print(f"  End: (1, 0, 0) m")
    print(f"  Duration: {app_config.simulation.max_duration} s")

    # Create simulation config
    sim_config = SimulationConfig(
        app_config=app_config, mission_state=create_mission_state()
    )

    # Starting and end positions
    start_pos = (0.0, 0.0, 0.0)
    end_pos = (1.0, 0.0, 0.0)

    print(f"\nInitializing simulation...")

    try:
        sim = SatelliteMPCLinearizedSimulation(
            cfg=app_config,
            simulation_config=sim_config,
            start_pos=start_pos,
            end_pos=end_pos,
            start_angle=(0, 0, 0),
            end_angle=(0, 0, 0),
        )

        print("Running simulation...")

        # Collect trajectory data
        positions = []
        times = []

        # Initialize
        sim.is_running = True
        max_time = app_config.simulation.max_duration
        sample_interval = 0.5
        last_sample_time = -sample_interval

        # Record initial state
        state = sim.get_current_state()
        positions.append(state[:3].copy())
        times.append(sim.simulation_time)

        step_count = 0
        while sim.is_running and sim.simulation_time < max_time:
            sim.step()
            step_count += 1

            # Sample periodically
            if sim.simulation_time - last_sample_time >= sample_interval:
                state = sim.get_current_state()
                positions.append(state[:3].copy())
                times.append(sim.simulation_time)
                last_sample_time = sim.simulation_time

                # Print progress
                pos = state[:3]
                dist_to_end = np.linalg.norm(pos - np.array(end_pos))
                print(
                    f"  t={sim.simulation_time:.1f}s: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), dist={dist_to_end:.3f}m"
                )

                # Early exit if reached end
                if dist_to_end < 0.05:
                    print(f"\n✅ END REACHED at t={sim.simulation_time:.1f}s!")
                    break

        print(f"\nSimulation completed at t={sim.simulation_time:.2f}s")

        # Final state
        final_state = sim.get_current_state()
        final_pos = final_state[:3]
        final_dist = np.linalg.norm(final_pos - np.array(end_pos))

        print(f"\n" + "=" * 40)
        print("RESULTS")
        print("=" * 40)
        print(
            f"Final Position: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f}) m"
        )
        print(f"End Position: {end_pos}")
        print(f"Distance to End: {final_dist:.4f} m")

        if final_dist < 0.1:
            print("\n✅ TEST PASSED - Satellite reached end!")
            success = True
        else:
            print(
                "\n⚠️ TEST INCOMPLETE - Satellite did not reach end within time limit"
            )
            success = False

        sim.close()
        return success

    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback

        traceback.print_exc()
        return False


if __name__ == "__main__":
    run_simple_motion_test()

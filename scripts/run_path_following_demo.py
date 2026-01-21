#!/usr/bin/env python3
"""
Path-Following MPC Demo Script

Demonstrates the MPCC path-following mode by running a short simulation
where the satellite follows a circular path in the Hill frame.

Usage:
    ./.venv311/bin/python3 scripts/run_path_following_demo.py
"""

import numpy as np
import matplotlib.pyplot as plt

# Project imports
from src.satellite_control.config.defaults import create_default_app_config
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.config.mission_state import create_mission_state
from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation


def run_path_following_demo():
    """Run a path-following simulation demo."""

    print("=" * 60)
    print("PATH-FOLLOWING MPC DEMO")
    print("=" * 60)

    # Create configuration with path-following enabled
    app_config = create_default_app_config()

    # Enable path following mode in MPC params
    app_config.mpc.mode_path_following = True
    app_config.mpc.path_radius = 5.0  # 5 meter radius circle
    app_config.mpc.path_speed = 0.05  # 0.05 rad/s (~2 min for full circle)

    # Simulation settings
    app_config.simulation.max_duration = 30.0  # 30 second simulation for demo
    app_config.simulation.headless = True

    print(f"\nConfiguration:")
    print(f"  Path Radius: {app_config.mpc.path_radius} m")
    print(f"  Path Speed: {app_config.mpc.path_speed} rad/s")
    print(f"  Duration: {app_config.simulation.max_duration} s")

    # Calculate expected arc length
    expected_arc = app_config.mpc.path_speed * app_config.simulation.max_duration
    expected_arc_deg = np.degrees(expected_arc)
    print(f"  Expected arc traversal: {expected_arc:.2f} rad ({expected_arc_deg:.1f}°)")

    # Create simulation config with mission state
    sim_config = SimulationConfig(
        app_config=app_config, mission_state=create_mission_state()
    )

    # Starting position: on the circle at s=0 (x=R, y=0, z=0)
    start_pos = (app_config.mpc.path_radius, 0.0, 0.0)

    print(f"\nStarting position: {start_pos}")
    print("\nInitializing simulation...")

    try:
        sim = SatelliteMPCLinearizedSimulation(
            cfg=app_config,
            simulation_config=sim_config,
            start_pos=start_pos,
            start_angle=(0, 0, 0),
        )

        print("Running simulation...")

        # Collect trajectory data
        positions = []
        times = []

        # Initialize simulation for stepping
        sim.is_running = True
        max_time = app_config.simulation.max_duration
        sample_interval = 0.5  # Sample every 0.5s
        last_sample_time = -sample_interval

        # Record initial state
        state = sim.get_current_state()
        positions.append(state[:3].copy())
        times.append(sim.simulation_time)

        step_count = 0
        while sim.is_running and sim.simulation_time < max_time:
            # Step the simulation
            sim.step()
            step_count += 1

            # Collect position periodically
            if sim.simulation_time - last_sample_time >= sample_interval:
                state = sim.get_current_state()
                positions.append(state[:3].copy())
                times.append(sim.simulation_time)
                last_sample_time = sim.simulation_time

            # Progress indicator every 5 seconds
            if step_count % 1000 == 0:
                print(f"  t = {sim.simulation_time:.1f}s")

        print(f"\nSimulation completed at t={sim.simulation_time:.2f}s")
        print(f"Collected {len(positions)} trajectory points")

        # Convert to numpy for analysis
        positions = np.array(positions)
        times = np.array(times)

        if len(positions) < 2:
            print("\n⚠️ Not enough data points collected. Check simulation setup.")
            return False

        # Analyze results
        print("\n" + "=" * 40)
        print("TRAJECTORY ANALYSIS")
        print("=" * 40)

        # Calculate radii
        radii = np.sqrt(positions[:, 0] ** 2 + positions[:, 1] ** 2)
        mean_radius = np.mean(radii)
        radius_std = np.std(radii)

        print(
            f"Mean radius: {mean_radius:.3f} m (target: {app_config.mpc.path_radius} m)"
        )
        print(f"Radius std dev: {radius_std:.4f} m")
        print(
            f"Max deviation from circle: {np.max(np.abs(radii - app_config.mpc.path_radius)):.4f} m"
        )

        # Calculate angular progress
        angles = np.arctan2(positions[:, 1], positions[:, 0])
        # Unwrap to handle 2π crossings
        angles_unwrap = np.unwrap(angles)
        total_angle = angles_unwrap[-1] - angles_unwrap[0]

        print(
            f"Total angle traversed: {total_angle:.3f} rad ({np.degrees(total_angle):.1f}°)"
        )
        print(f"Expected: {expected_arc:.3f} rad ({expected_arc_deg:.1f}°)")

        # Plot results
        fig = plt.figure(figsize=(12, 5))

        # 2D XY trajectory
        ax1 = fig.add_subplot(121)
        ax1.plot(
            positions[:, 0], positions[:, 1], "b-", linewidth=2, label="Trajectory"
        )
        ax1.scatter(
            positions[0, 0],
            positions[0, 1],
            c="green",
            s=100,
            marker="o",
            label="Start",
            zorder=5,
        )
        ax1.scatter(
            positions[-1, 0],
            positions[-1, 1],
            c="red",
            s=100,
            marker="x",
            label="End",
            zorder=5,
        )

        # Draw reference circle
        theta = np.linspace(0, 2 * np.pi, 100)
        ax1.plot(
            app_config.mpc.path_radius * np.cos(theta),
            app_config.mpc.path_radius * np.sin(theta),
            "k--",
            alpha=0.5,
            label="Reference Circle",
        )

        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_title("Path Following Trajectory (XY Plane)")
        ax1.legend()
        ax1.axis("equal")
        ax1.grid(True, alpha=0.3)

        # Radius vs Time
        ax2 = fig.add_subplot(122)
        ax2.plot(times, radii, "b-", linewidth=2)
        ax2.axhline(
            y=app_config.mpc.path_radius,
            color="k",
            linestyle="--",
            alpha=0.5,
            label="Target Radius",
        )
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Radius (m)")
        ax2.set_title("Radius Tracking Over Time")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save figure
        output_path = "Data/path_following_demo.png"
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")

        # Don't show for headless
        # plt.show()

        sim.close()

        print("\n✅ Path-following demo completed successfully!")
        return True

    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")
        import traceback

        traceback.print_exc()
        return False


if __name__ == "__main__":
    run_path_following_demo()

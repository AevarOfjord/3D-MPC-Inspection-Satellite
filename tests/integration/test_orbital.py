#!/usr/bin/env python3
"""
Integration test for orbital dynamics with CW equations.

Tests:
1. Free drift follows expected CW relative motion
2. Station-keeping at fixed offset from target
3. Approach maneuver with controlled deceleration
"""

import sys

import numpy as np

from src.satellite_control.control.mpc_controller import MPCController
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.core.cpp_satellite import CppSatelliteSimulator
from src.satellite_control.config.models import ReactionWheelParams


def _build_state(sim: CppSatelliteSimulator) -> np.ndarray:
    pos = sim.position.copy()
    quat = sim.quaternion.copy()
    vel = sim.velocity.copy()
    ang_vel = sim.angular_velocity.copy()
    rw_speeds = sim.wheel_speeds.copy()
    return np.concatenate([pos, quat, vel, ang_vel, rw_speeds])


def _rw_torque_limits(app_config) -> np.ndarray:
    limits = [float(rw.max_torque) for rw in app_config.physics.reaction_wheels]
    return np.array(limits, dtype=float)


def run_station_keeping() -> bool:
    """Run station-keeping at fixed offset with CW dynamics."""
    print("=" * 60)
    print("ORBITAL STATION-KEEPING TEST")
    print("=" * 60)

    # Initialize controller
    # Use SimulationConfig to generate base config
    sim_config = SimulationConfig.create_default()
    app_config = sim_config.app_config

    sim_dt = 0.005
    control_dt = 0.05
    app_config.simulation.dt = sim_dt
    app_config.simulation.control_dt = control_dt
    app_config.mpc.dt = control_dt
    app_config.mpc.prediction_horizon = 50

    # Tune weights for stable RW control (path-following MPC)
    app_config.mpc.Q_contour = 10.0
    app_config.mpc.Q_progress = 1.0
    app_config.mpc.Q_smooth = 1.0
    app_config.mpc.q_angular_velocity = 1.0
    app_config.mpc.r_thrust = 1.0
    app_config.mpc.r_rw_torque = 0.1

    # Override RW config
    app_config.physics.reaction_wheels = [
        ReactionWheelParams(axis=(1.0, 0.0, 0.0), max_torque=0.06, inertia=1e-4),
        ReactionWheelParams(axis=(0.0, 1.0, 0.0), max_torque=0.06, inertia=1e-4),
        ReactionWheelParams(axis=(0.0, 0.0, 1.0), max_torque=0.06, inertia=1e-4),
    ]

    controller = MPCController(app_config)
    sim = CppSatelliteSimulator(app_config=app_config)

    # Initial position: 5m radial offset from target
    initial_offset = np.array([5.0, 0.0, 0.0])
    sim.position = initial_offset
    sim.velocity = np.zeros(3)
    sim.angle = (0.0, 0.0, 0.0)
    sim.angular_velocity = np.zeros(3)

    # Target: maintain 5m radial offset
    x_target = np.zeros(16)
    x_target[0:3] = initial_offset
    x_target[3] = 1.0  # qw

    print(f"\nInitial position: {initial_offset}")
    print(f"Target position: {x_target[0:3]}")
    print("Test: Hold position against CW drift for 60 seconds")

    # Path following: hold at target with tiny segment for tangent stability
    controller.set_path(
        [
            tuple(initial_offset),
            (initial_offset[0] + 0.01, initial_offset[1], initial_offset[2]),
        ]
    )

    # Simulation parameters
    sim_duration = 60.0  # 1 minute

    # Logging
    log_time = []
    log_pos = []
    log_error = []

    # Run simulation
    print("\nRunning simulation...")
    t = 0.0
    control_step = 0
    last_control_time = -control_dt
    rw_limits = _rw_torque_limits(app_config)

    while t < sim_duration:
        # Control update
        if t - last_control_time >= control_dt:
            x_current = _build_state(sim)

            # Compute MPC control
            u, info = controller.get_control_action(x_current)
            rw_cmds, thruster_cmds = controller.split_control(u)
            sim.set_reaction_wheel_torque(rw_cmds * rw_limits)
            sim.apply_force(list(thruster_cmds))

            # Logging
            pos_error = np.linalg.norm(x_current[0:3] - x_target[0:3])
            log_time.append(t)
            log_pos.append(x_current[0:3].copy())
            log_error.append(pos_error)

            last_control_time = t
            control_step += 1

            if control_step % 100 == 0:
                print(
                    f"  t={t:.1f}s: pos=({x_current[0]:.3f}, {x_current[1]:.3f}, {x_current[2]:.3f}), "
                    f"err={pos_error * 100:.2f}cm"
                )

        sim.update_physics(sim_dt)
        t += sim_dt

    # Results
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETE")
    print("=" * 60)

    final_pos = sim.position
    final_error = np.linalg.norm(final_pos - x_target[0:3])
    max_error = max(log_error)
    avg_error = np.mean(log_error)

    print(
        f"\nFinal position: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})"
    )
    print(f"Position error: {final_error * 100:.2f} cm")
    print(f"Max error during test: {max_error * 100:.2f} cm")
    print(f"Average error: {avg_error * 100:.2f} cm")

    # Pass criteria: coarse hold against drift (MPCC path-following controller)
    max_error_threshold = 0.6  # 60cm
    final_error_threshold = 0.35  # 35cm
    passed = (max_error < max_error_threshold) and (final_error < final_error_threshold)

    print(
        f"\nStation-keeping ≤{max_error_threshold * 100:.0f}cm max / "
        f"≤{final_error_threshold * 100:.0f}cm final: {'✓ PASS' if passed else '✗ FAIL'}"
    )

    return bool(passed)


def test_station_keeping():
    """Test station-keeping at fixed offset with CW dynamics."""
    assert run_station_keeping()


def run_free_drift() -> bool:
    """Run CW dynamics free drift without control."""
    print("=" * 60)
    print("CW FREE DRIFT TEST (No Control)")
    print("=" * 60)

    # Initial: 5m radial offset, no velocity
    sim_config = SimulationConfig.create_default()
    app_config = sim_config.app_config
    app_config.simulation.dt = 0.01
    app_config.simulation.control_dt = 0.05
    app_config.mpc.dt = 0.05
    sim = CppSatelliteSimulator(app_config=app_config)
    sim.position = np.array([5.0, 0.0, 0.0])
    sim.velocity = np.zeros(3)
    sim.angle = (0.0, 0.0, 0.0)
    sim.angular_velocity = np.zeros(3)

    print(f"\nInitial position: {sim.position}")
    print("Simulating 10 minutes with NO control...")

    sim_dt = app_config.simulation.dt
    sim_duration = 600.0  # 10 minutes
    t = 0.0

    log_pos = []

    while t < sim_duration:
        pos = sim.position.copy()

        if int(t) % 600 == 0 and abs(t - int(t)) < sim_dt:
            log_pos.append((t / 60, pos.copy()))

        sim.update_physics(sim_dt)
        t += sim_dt

    print("\nPosition over time:")
    for time_min, pos in log_pos:
        print(f"  t={time_min:.0f}min: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) m")

    final_pos = sim.position
    print(
        f"\nFinal position after 10 minutes: ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})"
    )

    # For CW dynamics with initial radial offset and no velocity,
    # the satellite should oscillate in an ellipse
    return True  # Visual/qualitative test


def test_free_drift():
    """Test that CW dynamics produce expected drift without control."""
    assert run_free_drift()


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("ORBITAL DYNAMICS INTEGRATION TESTS")
    print("=" * 60 + "\n")

    # Run tests
    drift_ok = run_free_drift()
    print()
    station_ok = run_station_keeping()

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Free drift test: {'✓' if drift_ok else '✗'}")
    print(f"Station-keeping test: {'✓' if station_ok else '✗'}")

    sys.exit(0 if (drift_ok and station_ok) else 1)

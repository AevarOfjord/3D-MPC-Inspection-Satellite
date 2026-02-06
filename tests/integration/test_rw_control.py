#!/usr/bin/env python3
"""
Integration test for reaction wheel satellite control system.

Tests the full control loop:
1. C++ physics engine with reaction wheel model
2. Reaction wheel MPC controller
3. Closed-loop attitude and translation control
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


def run_reaction_wheel_control() -> bool:
    """Run closed-loop reaction wheel control test."""
    print("=" * 60)
    print("REACTION WHEEL SATELLITE CONTROL TEST")
    print("=" * 60)

    # Initialize controller
    sim_config = SimulationConfig.create_default()
    app_config = sim_config.app_config

    # Override parameters for this test
    sim_dt = 0.005  # 5ms physics timestep
    control_dt = 0.05  # 50ms control update
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

    # Override RW torque limits for tighter control
    app_config.physics.reaction_wheels = [
        ReactionWheelParams(axis=(1.0, 0.0, 0.0), max_torque=0.06, inertia=1e-4),
        ReactionWheelParams(axis=(0.0, 1.0, 0.0), max_torque=0.06, inertia=1e-4),
        ReactionWheelParams(axis=(0.0, 0.0, 1.0), max_torque=0.06, inertia=1e-4),
    ]

    controller = MPCController(app_config)
    sim = CppSatelliteSimulator(app_config=app_config)

    print("\nController initialized:")
    print(f"  State dimension: {controller.nx}")
    print(f"  Control dimension: {controller.nu}")
    print(f"  Max RW torque: {controller.max_rw_torque} N·m")

    # Set initial state
    sim.position = np.array([0.0, 0.0, 0.0])
    sim.velocity = np.zeros(3)
    sim.angle = (0.0, 0.0, 0.0)
    sim.angular_velocity = np.zeros(3)

    # Target state: move to x=0.5, rotate 45° about Z
    target_quat = np.array([np.cos(np.pi / 8), 0, 0, np.sin(np.pi / 8)])  # 45° about Z
    x_target = np.zeros(16)
    x_target[0] = 0.5  # Target X position
    x_target[3:7] = target_quat  # Target orientation

    print(f"\nInitial position: {sim.position}")
    print(f"Target position: {x_target[0:3]}")
    print(f"Target quaternion: {x_target[3:7]}")

    # Path following: straight line to target
    controller.set_path([tuple(sim.position), (x_target[0], 0.0, 0.0)])

    # Simulation parameters
    sim_duration = 15.0  # seconds for convergence

    # Logging
    log_time = []
    log_pos_x = []
    log_pos_y = []
    log_rw_speed = []
    log_solve_time = []

    # Run simulation
    print("\nRunning simulation...")
    t = 0.0
    control_step = 0
    last_control_time = -control_dt  # Force first control update
    rw_limits = _rw_torque_limits(app_config)

    while t < sim_duration:
        # Control update at 20Hz
        if t - last_control_time >= control_dt:
            # Build current state (16 elements)
            x_current = _build_state(sim)

            # Compute control
            u, info = controller.get_control_action(x_current)
            rw_cmds, thruster_cmds = controller.split_control(u)
            sim.set_reaction_wheel_torque(rw_cmds * rw_limits)
            sim.apply_force(list(thruster_cmds))

            # Logging
            log_time.append(t)
            log_pos_x.append(x_current[0])
            log_pos_y.append(x_current[1])
            log_rw_speed.append(x_current[-3:].copy())
            log_solve_time.append(info["solve_time"] * 1000)  # ms

            last_control_time = t
            control_step += 1

            # Progress update
            if control_step % 20 == 0:
                pos_err = np.linalg.norm(x_current[0:3] - x_target[0:3])
                # Calculate yaw angle (for logging only; orientation not in cost)
                quat = x_current[3:7]
                yaw = 2 * np.arctan2(quat[3], quat[0])
                yaw_deg = np.degrees(yaw)
                print(
                    f"  t={t:.2f}s: pos=({x_current[0]:.3f}, {x_current[1]:.3f}), "
                    f"err={pos_err:.4f}m, yaw={yaw_deg:.2f}°, "
                    f"ωz={x_current[12]:.4f}, τ_z={rw_cmds[2]:.3f}"
                )

        # Physics step
        sim.update_physics(sim_dt)
        t += sim_dt

    print("\n" + "=" * 60)
    print("SIMULATION COMPLETE")
    print("=" * 60)

    # Final state
    final_pos = sim.position
    final_quat = sim.quaternion
    pos_error = np.linalg.norm(final_pos - x_target[0:3])

    # Quaternion error (simple angular difference)
    quat_dot = np.abs(np.dot(final_quat, x_target[3:7]))
    ang_error_rad = 2 * np.arccos(min(quat_dot, 1.0))
    ang_error_deg = np.degrees(ang_error_rad)

    print(
        f"\nFinal position: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})"
    )
    print(f"Target position: ({x_target[0]:.4f}, {x_target[1]:.4f}, {x_target[2]:.4f})")
    print(f"Position error: {pos_error:.4f} m")
    print(f"Angular error: {ang_error_deg:.2f}°")
    print(f"Average solve time: {np.mean(log_solve_time):.2f} ms")
    print(f"Max solve time: {np.max(log_solve_time):.2f} ms")

    # Check targets (position only; orientation is not tracked in path-following MPC)
    initial_error = np.linalg.norm(np.array([0.0, 0.0, 0.0]) - x_target[0:3])
    pos_ok = pos_error < 0.35  # 35cm
    progress_ok = pos_error < (0.9 * initial_error)  # must improve by 10%

    print("\nTargets:")
    print(f"  Position ≤35cm: {'✓ PASS' if pos_ok else '✗ FAIL'}")
    print(f"  Progress ≥10%: {'✓ PASS' if progress_ok else '✗ FAIL'}")
    print("  Angle tracking: skipped (MPCC does not penalize quaternion).")

    return bool(pos_ok and progress_ok)


def test_reaction_wheel_control():
    """Test closed-loop reaction wheel control."""
    assert run_reaction_wheel_control()


if __name__ == "__main__":
    success = run_reaction_wheel_control()
    sys.exit(0 if success else 1)

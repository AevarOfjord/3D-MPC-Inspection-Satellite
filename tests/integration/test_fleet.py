#!/usr/bin/env python3
"""
Multi-satellite fleet coordination test.

Tests:
1. Formation keeping - 3 inspectors maintain 120° spacing
2. Collision avoidance - no inter-inspector collisions
3. Keep-out zone - all inspectors stay >2m from target
"""

import sys

import numpy as np

from src.satellite_control.control.mpc_controller import MPCController
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.core.cpp_satellite import CppSatelliteSimulator
from src.satellite_control.config.models import ReactionWheelParams
from src.satellite_control.fleet.fleet_manager import create_fleet_manager


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


def run_fleet_formation() -> bool:
    """Run fleet formation keeping and return success."""
    print("=" * 60)
    print("FLEET FORMATION KEEPING TEST")
    print("=" * 60)

    # Create fleet manager
    fleet = create_fleet_manager(num_inspectors=3, formation_radius=5.0)
    print("\nFormation targets:")
    for i in range(3):
        target = fleet.get_formation_target(i)
        print(f"  Inspector {i}: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})")

    # Create controllers (one per inspector)
    sim_config = SimulationConfig.create_default()
    app_config = sim_config.app_config

    sim_dt = 0.005
    control_dt = 0.05
    app_config.simulation.dt = sim_dt
    app_config.simulation.control_dt = control_dt
    app_config.mpc.dt = control_dt
    app_config.mpc.prediction_horizon = 30

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

    controllers = {i: MPCController(app_config) for i in range(3)}
    sims = {i: CppSatelliteSimulator(app_config=app_config) for i in range(3)}

    # Simulation parameters
    sim_duration = 60.0  # 1 minute

    # Logging
    log_separations = []
    log_to_target = []
    log_errors = {0: [], 1: [], 2: []}

    print(f"\nSimulating {sim_duration}s of formation keeping...")

    t = 0.0
    control_step = 0
    last_control_time = -control_dt
    rw_limits = _rw_torque_limits(app_config)

    # Initialize inspectors at formation targets
    for insp_id, sim in sims.items():
        target = fleet.get_formation_target(insp_id)
        sim.position = target.copy()
        sim.velocity = np.zeros(3)
        sim.angle = (0.0, 0.0, 0.0)
        sim.angular_velocity = np.zeros(3)
        controllers[insp_id].set_path(
            [tuple(target), (target[0] + 0.01, target[1], target[2])]
        )

    while t < sim_duration:
        if t - last_control_time >= control_dt:
            # Process each inspector
            for insp_id in range(3):
                sim = sims[insp_id]
                x_current = _build_state(sim)
                pos = x_current[0:3]
                quat = x_current[3:7]
                vel = x_current[7:10]
                angvel = x_current[10:13]
                rw_speeds = x_current[13:16]

                fleet.update_inspector(insp_id, pos, vel, quat, angvel, rw_speeds)

                target_pos = fleet.get_formation_target(insp_id)
                controller = controllers[insp_id]
                u, info = controller.get_control_action(x_current)
                rw_cmds, thruster_cmds = controller.split_control(u)
                sim.set_reaction_wheel_torque(rw_cmds * rw_limits)
                sim.apply_force(list(thruster_cmds))

                error = np.linalg.norm(pos - target_pos)
                log_errors[insp_id].append(error)

            # Check separations
            separations = fleet.get_min_separations()
            log_separations.append(separations["inter_inspector"])
            log_to_target.append(separations["to_target"])

            last_control_time = t
            control_step += 1

            if control_step % 50 == 0:
                avg_err = np.mean([log_errors[i][-1] for i in range(3)])
                print(
                    f"  t={t:.1f}s: avg_err={avg_err * 100:.1f}cm, "
                    f"min_sep={separations['inter_inspector']:.2f}m, "
                    f"min_to_target={separations['to_target']:.2f}m"
                )

        for sim in sims.values():
            sim.update_physics(sim_dt)
        t += sim_dt

    # Results
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETE")
    print("=" * 60)

    # Calculate metrics
    final_errors = [log_errors[i][-1] for i in range(3)]
    avg_final_error = np.mean(final_errors)
    max_final_error = max(final_errors)

    # Filter out initial zeros (before all inspectors registered)
    valid_separations = [s for s in log_separations if s > 0]
    valid_to_target = [s for s in log_to_target if s > 0]
    min_separation = min(valid_separations) if valid_separations else 0
    min_to_target = min(valid_to_target) if valid_to_target else 0

    print("\nFormation Errors:")
    for i in range(3):
        print(f"  Inspector {i}: {final_errors[i] * 100:.2f} cm")
    print(f"  Average: {avg_final_error * 100:.2f} cm")

    print("\nSafety Margins:")
    print(f"  Min inter-inspector separation: {min_separation:.2f} m (target: >1m)")
    print(f"  Min distance to target: {min_to_target:.2f} m (target: >2m)")

    # Pass criteria
    formation_ok = max_final_error < 0.50  # 50cm
    separation_ok = min_separation > 1.0  # 1m
    keepout_ok = min_to_target > 2.0  # 2m

    print("\nResults:")
    print(f"  Formation ±50cm: {'✓ PASS' if formation_ok else '✗ FAIL'}")
    print(f"  Inter-inspector >1m: {'✓ PASS' if separation_ok else '✗ FAIL'}")
    print(f"  Target keep-out >2m: {'✓ PASS' if keepout_ok else '✗ FAIL'}")

    return bool(formation_ok and separation_ok and keepout_ok)


def test_fleet_formation():
    """Test fleet formation keeping with 3 inspectors."""
    assert run_fleet_formation()


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("MULTI-SATELLITE FLEET COORDINATION TESTS")
    print("=" * 60 + "\n")

    success = run_fleet_formation()

    print("\n" + "=" * 60)
    print(f"OVERALL: {'✓ PASS' if success else '✗ FAIL'}")
    print("=" * 60)

    sys.exit(0 if success else 1)

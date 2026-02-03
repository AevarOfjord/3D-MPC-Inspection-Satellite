"""
MPCC Arc Path Demo

Runs a closed-loop MPCC test following a helical arc:
start (2, 0, 0) -> end (0, 2, 2).

This is a standalone demo script (not collected by pytest).
"""

from __future__ import annotations

import sys
from typing import List, Tuple

import numpy as np

from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.control.mpc_controller import MPCController
from src.satellite_control.core.cpp_satellite import CppSatelliteSimulator


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


def build_helical_arc_path(
    radius: float = 2.0, z_end: float = 2.0, num_points: int = 200
) -> List[Tuple[float, float, float]]:
    """
    Build a simple arc path:
    x = r cos(t), y = r sin(t), z = linear ramp, t in [0, pi/2]
    """
    t = np.linspace(0.0, 0.5 * np.pi, num_points)
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    z = z_end * (t / (0.5 * np.pi))
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def run_mpcc_arc_demo() -> bool:
    print("=" * 60)
    print("MPCC ARC PATH DEMO")
    print("=" * 60)

    # Config + controller
    sim_config = SimulationConfig.create_default()
    app_config = sim_config.app_config

    sim_dt = 0.01
    control_dt = 0.05
    app_config.simulation.dt = sim_dt
    app_config.simulation.control_dt = control_dt
    app_config.mpc.dt = control_dt
    app_config.mpc.prediction_horizon = 60

    # Weights for path following
    app_config.mpc.Q_contour = 50.0
    app_config.mpc.Q_progress = 5.0
    app_config.mpc.Q_smooth = 1.0
    app_config.mpc.q_angular_velocity = 0.5
    app_config.mpc.Q_attitude = 50.0
    app_config.mpc.r_thrust = 0.2
    app_config.mpc.r_rw_torque = 0.1
    app_config.mpc.path_speed = 0.2

    controller = MPCController(app_config)
    sim = CppSatelliteSimulator(app_config=app_config)

    # Path definition
    path = build_helical_arc_path(radius=2.0, z_end=2.0, num_points=220)
    controller.set_path(path)

    start = np.array(path[0], dtype=float)
    end = np.array(path[-1], dtype=float)

    # Initial state
    sim.position = start.copy()
    sim.velocity = np.zeros(3)
    sim.angle = (0.0, 0.0, 0.0)
    sim.angular_velocity = np.zeros(3)

    print(f"\nStart: {start}")
    print(f"End:   {end}")
    print(f"Path length: {controller._path_length:.3f} m")

    # Simulation loop
    sim_duration = 25.0
    t = 0.0
    last_control_time = -control_dt
    rw_limits = _rw_torque_limits(app_config)

    track_errors = []
    s_progress = []

    while t < sim_duration:
        if t - last_control_time >= control_dt:
            x_current = _build_state(sim)
            u, info = controller.get_control_action(x_current)

            rw_cmds, thruster_cmds = controller.split_control(u)
            if rw_limits.size > 0:
                sim.set_reaction_wheel_torque(rw_cmds * rw_limits)
            sim.apply_force(list(thruster_cmds))

            # Reference tracking error at current s
            ref_pos, _ = controller.get_path_reference()
            track_err = float(np.linalg.norm(x_current[0:3] - ref_pos))
            track_errors.append(track_err)
            s_progress.append(info.get("path_s", controller.s))

            if int(t / 1.0) != int((t - control_dt) / 1.0):
                print(
                    f"  t={t:5.2f}s | s={controller.s:5.2f}m | "
                    f"track_err={track_err:6.3f}m | "
                    f"pos=({x_current[0]:.2f}, {x_current[1]:.2f}, {x_current[2]:.2f})"
                )

            last_control_time = t

        sim.update_physics(sim_dt)
        t += sim_dt

    final_pos = sim.position.copy()
    final_err = float(np.linalg.norm(final_pos - end))
    metrics = controller.get_path_progress(final_pos)
    progress = metrics.get("progress", 0.0)
    endpoint_error = metrics.get("endpoint_error", final_err)
    max_track_err = max(track_errors) if track_errors else 0.0
    mean_track_err = float(np.mean(track_errors)) if track_errors else 0.0

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"Final position: ({final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f})")
    print(f"Final error: {final_err:.3f} m (endpoint {endpoint_error:.3f} m)")
    print(f"Progress: {progress*100:.1f}% of path length")
    print(f"Track error: mean={mean_track_err:.3f} m, max={max_track_err:.3f} m")

    pos_ok = endpoint_error < 0.5
    progress_ok = progress > 0.85

    print("\nChecks:")
    print(f"  Final error < 0.5 m: {'✓ PASS' if pos_ok else '✗ FAIL'}")
    print(f"  Progress > 85%:      {'✓ PASS' if progress_ok else '✗ FAIL'}")

    return bool(pos_ok and progress_ok)


if __name__ == "__main__":
    success = run_mpcc_arc_demo()
    sys.exit(0 if success else 1)

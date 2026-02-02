#!/usr/bin/env python3
"""
Mission System Integration Test

Tests the mission executor with different mission templates.
"""

from pathlib import Path
import sys
import logging

import numpy as np

ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.satellite_control.mission import (
    Mission,
    create_flyby_mission,
    create_circumnavigation_mission,
    create_station_keeping_mission,
)
from src.satellite_control.control.mpc_controller import MPCController
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.core.cpp_satellite import CppSatelliteSimulator
from src.satellite_control.config.models import ReactionWheelParams

# Set up logging
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


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


def _run_mission_segment(mission: Mission, waypoint_idx: int = 0, duration: float = 10.0) -> float:
    sim_config = SimulationConfig.create_default()
    app_config = sim_config.app_config

    sim_dt = 0.005
    control_dt = 0.05
    app_config.simulation.dt = sim_dt
    app_config.simulation.control_dt = control_dt
    app_config.mpc.dt = control_dt
    app_config.mpc.prediction_horizon = 40

    # Tune weights for stable path following
    app_config.mpc.Q_contour = 10.0
    app_config.mpc.Q_progress = 1.0
    app_config.mpc.Q_smooth = 1.0
    app_config.mpc.q_angular_velocity = 1.0
    app_config.mpc.r_thrust = 1.0
    app_config.mpc.r_rw_torque = 0.1

    app_config.physics.reaction_wheels = [
        ReactionWheelParams(axis=(1.0, 0.0, 0.0), max_torque=0.06, inertia=1e-4),
        ReactionWheelParams(axis=(0.0, 1.0, 0.0), max_torque=0.06, inertia=1e-4),
        ReactionWheelParams(axis=(0.0, 0.0, 1.0), max_torque=0.06, inertia=1e-4),
    ]

    controller = MPCController(app_config)
    sim = CppSatelliteSimulator(app_config=app_config)

    sim.position = mission.start_position.copy()
    sim.velocity = np.zeros(3)
    sim.angle = (0.0, 0.0, 0.0)
    sim.angular_velocity = np.zeros(3)

    waypoints = mission.get_all_waypoints()
    if not waypoints:
        return 0.0

    target = waypoints[waypoint_idx].position
    controller.set_path([tuple(sim.position), tuple(target)])

    rw_limits = _rw_torque_limits(app_config)

    t = 0.0
    last_control_time = -control_dt
    initial_error = np.linalg.norm(sim.position - target)

    while t < duration:
        if t - last_control_time >= control_dt:
            x_current = _build_state(sim)
            u, info = controller.get_control_action(x_current)
            rw_cmds, thruster_cmds = controller.split_control(u)
            sim.set_reaction_wheel_torque(rw_cmds * rw_limits)
            sim.apply_force(list(thruster_cmds))
            last_control_time = t

        sim.update_physics(sim_dt)
        t += sim_dt

    final_error = np.linalg.norm(sim.position - target)
    print(f"  Segment error: {initial_error:.3f} -> {final_error:.3f} m")
    return final_error


def run_flyby_mission() -> bool:
    """Run flyby mission execution and return success."""
    print("=" * 60)
    print("FLYBY MISSION TEST")
    print("=" * 60)

    mission = create_flyby_mission(
        start_distance=6.0,
        pass_distance=4.0,
        approach_speed=0.15,  # Faster for testing
    )

    print(f"\nMission: {mission.name}")
    print(f"Type: {mission.mission_type.value}")
    print(f"Phases: {len(mission.phases)}")
    print(f"Total waypoints: {mission.total_waypoints}")

    for phase in mission.phases:
        print(f"\n  Phase: {phase.name}")
        for i, wp in enumerate(phase.waypoints):
            print(f"    {i+1}. {wp.name}: {wp.position}")

    print("\nExecuting mission segment...")
    final_error = _run_mission_segment(mission, waypoint_idx=0, duration=10.0)
    success = final_error < 0.5

    print(f"\nResult:")
    print(f"  Success: {success}")
    print(f"  Final error: {final_error * 100:.2f}cm")

    return bool(success)


def test_flyby_mission():
    """Test flyby mission execution."""
    assert run_flyby_mission()


def run_circumnavigation_mission() -> bool:
    """Run circumnavigation mission and return success."""
    print("\n" + "=" * 60)
    print("CIRCUMNAVIGATION MISSION TEST")
    print("=" * 60)

    mission = create_circumnavigation_mission(
        orbit_radius=4.0,
        num_points=4,  # 4 points for faster test
        hold_time=2.0,  # Short holds
    )

    print(f"\nMission: {mission.name}")
    print(f"Waypoints: {mission.total_waypoints}")

    print("\nExecuting mission segment...")
    final_error = _run_mission_segment(mission, waypoint_idx=0, duration=12.0)
    success = final_error < 0.75

    print(f"\nResult:")
    print(f"  Success: {success}")
    print(f"  Final error: {final_error * 100:.2f}cm")

    return bool(success)


def test_circumnavigation_mission():
    """Test circumnavigation mission."""
    assert run_circumnavigation_mission()


def run_station_keeping_mission() -> bool:
    """Run station-keeping mission and return success."""
    print("\n" + "=" * 60)
    print("STATION-KEEPING MISSION TEST")
    print("=" * 60)

    mission = create_station_keeping_mission(
        position=np.array([5.0, 0.0, 0.0]),
        duration=30.0,  # 30 second hold
    )

    print(f"\nMission: {mission.name}")
    print(f"Hold position: {mission.phases[0].waypoints[0].position}")
    print(f"Duration: {mission.phases[0].waypoints[0].hold_time}s")

    print("\nExecuting mission segment...")
    final_error = _run_mission_segment(mission, waypoint_idx=0, duration=10.0)
    success = final_error < 0.5

    print(f"\nResult:")
    print(f"  Success: {success}")
    print(f"  Final error: {final_error * 100:.2f}cm")

    return bool(success)


def test_station_keeping_mission():
    """Test station-keeping mission."""
    assert run_station_keeping_mission()


def run_mission_save_load() -> bool:
    """Run mission serialization test and return success."""
    print("\n" + "=" * 60)
    print("MISSION SAVE/LOAD TEST")
    print("=" * 60)

    import tempfile

    # Create mission
    mission = create_flyby_mission()

    # Save to temp file
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        mission.save(Path(f.name))
        print(f"Saved to: {f.name}")

        # Load back
        loaded = Mission.load(Path(f.name))

        print(f"Original: {mission.name}, {mission.total_waypoints} waypoints")
        print(f"Loaded: {loaded.name}, {loaded.total_waypoints} waypoints")

        # Verify
        success = mission.name == loaded.name and mission.total_waypoints == loaded.total_waypoints

        print(f"Match: {'✓ PASS' if success else '✗ FAIL'}")

        return bool(success)


def test_mission_save_load():
    """Test mission serialization."""
    assert run_mission_save_load()


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("MISSION SYSTEM INTEGRATION TESTS")
    print("=" * 60 + "\n")

    # Run tests
    save_load_ok = run_mission_save_load()
    station_ok = run_station_keeping_mission()
    flyby_ok = run_flyby_mission()

    # Skip circumnavigation for now (takes longer)
    # circum_ok = run_circumnavigation_mission()

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Save/Load: {'✓' if save_load_ok else '✗'}")
    print(f"Station-keeping: {'✓' if station_ok else '✗'}")
    print(f"Flyby: {'✓' if flyby_ok else '✗'}")

    all_pass = save_load_ok and station_ok and flyby_ok
    print(f"\nOVERALL: {'✓ PASS' if all_pass else '✗ FAIL'}")

    sys.exit(0 if all_pass else 1)

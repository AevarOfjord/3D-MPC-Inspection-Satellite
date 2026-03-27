"""Compare hybrid vs acados control outputs on the same state/path."""

import json
import logging

import numpy as np

logging.basicConfig(level=logging.WARNING)

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller
from controller.shared.python.mission.runtime_loader import (
    compile_unified_mission_runtime,
    parse_unified_mission_payload,
)

mission_json = json.load(open("missions/1Turn.json"))
sim_cfg_base = SimulationConfig.create_default()
mission_def = parse_unified_mission_payload(mission_json)
mission_runtime = compile_unified_mission_runtime(
    mission_def, simulation_config=sim_cfg_base
)
sim_cfg_base = mission_runtime.simulation_config

print(
    f"Path: {len(mission_runtime.path)} pts, length={mission_runtime.path_length:.3f}m"
)
print(f"Start pos: {mission_runtime.start_pos}")
print(f"Path speed from config: {sim_cfg_base.app_config.mpc.path_speed} m/s")
print(
    f"Path speed min/max: {sim_cfg_base.app_config.mpc.path_speed_min}/{sim_cfg_base.app_config.mpc.path_speed_max}"
)
print(f"Thruster forces (N): {sim_cfg_base.app_config.physics.thruster_forces}")
print(f"Satellite mass (kg): {sim_cfg_base.app_config.physics.total_mass}")

# State at start
x0 = np.zeros(16)
x0[0:3] = (
    list(mission_runtime.start_pos) if mission_runtime.start_pos else [2.3, 0, 2.0]
)
x0[3] = 1.0  # identity quaternion

# Build path points
waypoints = [(p[0], p[1], p[2]) for p in mission_runtime.path]

print(f"\n{'=' * 70}")
print("Control output comparison (first 5 steps with path set):")
print(f"{'=' * 70}")

for profile in ["hybrid", "nmpc", "acados_rti"]:
    cfg = SimulationConfig.create_with_overrides(
        {"mpc_core": {"controller_profile": profile}}, base_config=sim_cfg_base
    )
    ctrl = create_controller(cfg.app_config)
    ctrl.set_path(waypoints)
    ctrl.set_runtime_mode("TRACK")

    print(f"\nProfile: {profile}")
    print(f"  num_thrusters={ctrl.num_thrusters}, num_rw_axes={ctrl.num_rw_axes}")
    print(f"  path_speed={ctrl.path_speed}, path_length={ctrl._path_length:.3f}m")

    x = x0.copy()
    for i in range(3):
        u, info = ctrl.get_control_action(x)
        thr = u[ctrl.num_rw_axes :]  # thruster commands
        rw = u[: ctrl.num_rw_axes]  # RW torques

        # Compute physical forces
        phys_forces = [
            thr[j] * float(ctrl.thruster_forces[j]) for j in range(ctrl.num_thrusters)
        ]

        print(
            f"  Step {i}: s={ctrl.s:.4f}/{ctrl._path_length:.3f}m "
            f"| thr=[{','.join(f'{t:.3f}' for t in thr[:6])}] "
            f"| F_phys_N={sum(abs(f) for f in phys_forces):.4f}N total "
            f"| success={info.get('solver_success', '?')}"
        )

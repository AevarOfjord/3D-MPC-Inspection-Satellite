"""Diagnostic script comparing acados vs hybrid path-following."""

import json

import numpy as np

from controller.configs.simulation_config import SimulationConfig
from controller.factory import create_controller
from controller.shared.python.mission.runtime_loader import (
    parse_unified_mission_payload,
)
from controller.shared.python.mission.unified_compiler import (
    compile_unified_mission_runtime,
)

mission = json.load(open("missions/1Turn.json"))
runtime = parse_unified_mission_payload(mission)
compiled = compile_unified_mission_runtime(runtime)

waypoints = [(p[0], p[1], p[2]) for p in compiled.path_waypoints]
print(f"Path: {len(waypoints)} waypoints, total length will be computed by controller")

for profile in ["hybrid", "acados_rti", "acados_sqp"]:
    print(f"\n{'=' * 60}")
    print(f"Profile: {profile}")
    cfg = SimulationConfig.create_with_overrides(
        {
            "mpc": {"prediction_horizon": 20, "control_horizon": 20},
            "mpc_core": {"controller_profile": profile},
            "simulation": {"max_duration": 5.0, "headless": True},
        }
    )
    ctrl = create_controller(cfg.app_config)
    ctrl.set_path(waypoints)
    print(f"  path_length: {ctrl._path_length:.3f}m, path_set: {ctrl._path_set}")

    x0 = np.zeros(16)
    x0[0:3] = [2.3, 0.0, 2.0]  # start pos
    x0[3] = 1.0  # quaternion identity

    ctrl.set_runtime_mode("TRACK")
    for i in range(10):
        u, info = ctrl.get_control_action(x0)
        print(
            f"  step {i:2d}: rw_torque={u[:3]}, "
            f"thr_sum={u[3:].sum():.4f}, "
            f"s={ctrl.s:.4f}/{ctrl._path_length:.3f}, "
            f"success={info['solver_success']}, "
            f"iters={info.get('acados_iterations', info.get('sqp_iterations', '?'))}"
        )
        # Update state roughly by integrating position with a tiny thrust
        x0[0:3] += u[3:9:2].sum() * 0.05 * np.array([1, 0, 0])  # crude

print("\nDone.")

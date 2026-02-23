"""Diagnostic: Verify s advances over multiple MPC calls."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src", "python"))
import numpy as np
from config.defaults import create_default_app_config

cfg = create_default_app_config()

from control.mpc_controller import MPCController

ctrl = MPCController(cfg)

# Load path
import json

with open("missions/Circle_Test.json") as f:
    mission = json.load(f)
path = None
for key in ("path", "waypoints"):
    if key in mission:
        path = mission[key]
        break
if path is None:
    path = [[10, 0, 0], [15, 5, 0], [20, 10, 0], [25, 5, 0], [30, 0, 0]]
ctrl.set_path(path)
ctrl.set_runtime_mode("TRACK")

# Start near the path origin at rest
x = np.zeros(16, dtype=np.float64)
x[0] = 10.0  # px - at first waypoint
x[3] = 1.0  # qw - identity quaternion

print(f"Path total length: {ctrl._cpp.path_length:.2f} m")
print(
    f"{'Step':>5} {'s':>8} {'v_s':>8} {'pos_x':>8} {'pos_y':>8} {'pos_z':>8} {'thr':>20} {'status':>8}"
)
print("-" * 85)

for step in range(40):
    u_phys, info = ctrl.get_control_action(x)
    rw, thr = ctrl.split_control(u_phys)
    s = ctrl.s

    # Extract v_s from raw control (last element)
    active = [f"{i + 1}:{t:.2f}" for i, t in enumerate(thr) if abs(t) > 0.01]

    if step % 5 == 0 or step < 5:
        print(
            f"{step:5d} {s:8.3f} {'':>8} {x[0]:8.3f} {x[1]:8.3f} {x[2]:8.3f} {str(active):>20} {info['status_name']:>8}"
        )

    # Simple fake propagation: apply thrust effect on velocity, advance position
    dt = 0.05
    # Approximate: each thruster gives F=0.45N on m=10kg -> a=0.045 m/s²
    from scipy.spatial.transform import Rotation

    R = Rotation.from_quat([x[4], x[5], x[6], x[3]]).as_matrix()
    thr_dirs = [[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]]
    accel = np.zeros(3)
    for i in range(6):
        accel += R @ np.array(thr_dirs[i]) * 0.45 * thr[i] / 10.0

    x[7:10] += accel * dt
    x[0:3] += x[7:10] * dt
    # Keep quaternion identity for simplicity
    x[3] = 1.0
    x[4:7] = 0.0

print(f"\nFinal s = {ctrl.s:.4f}")
print(f"s advanced: {'YES' if ctrl.s > 0.01 else 'NO - BUG!'}")

"""Diagnostic: Check SETTLE mode behavior at s≈19.20 with 154mm error."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src", "python"))
import numpy as np
from config.defaults import create_default_app_config

cfg = create_default_app_config()

from control.mpc_controller import MPCController

ctrl = MPCController(cfg)

# Create a simple circular path with ~19.22m length
import math

path_points = []
R = 3.06  # radius gives ~19.22m circumference
N_pts = 100
for i in range(N_pts + 1):
    theta = 2.0 * math.pi * i / N_pts
    px = 10.0 + R * math.cos(theta) - R  # starts at ~[10,0,0]
    py = R * math.sin(theta)
    pz = 0.5 * math.sin(theta)  # small z variation
    path_points.append((px, py, pz))
ctrl.set_path(path_points)

# Simulate reaching near end of path
ctrl.set_runtime_mode("TRACK")

# First, advance s to ~19.20 by calling get_control_action several times
# with the satellite near the path end

# Get path endpoint
path_len = ctrl._cpp.path_length
print(f"Path length: {path_len:.2f} m")

# State near the end of path at yaw≈86.6°, similar to user's telemetry
x = np.zeros(16, dtype=np.float64)
x[0] = 8.749  # px (user: 8749mm)
x[1] = -0.016  # py
x[2] = 0.490  # pz
# Quaternion for yaw=86.6°
yaw_rad = np.radians(86.6)
cy, sy = np.cos(yaw_rad / 2), np.sin(yaw_rad / 2)
x[3] = cy  # qw
x[6] = sy  # qz
qn = np.linalg.norm(x[3:7])
x[3:7] /= qn
# Very low velocity (settled)
x[7:10] = [0.001, 0.0, 0.0]
x[10:13] = [0.0, 0.0, 0.0]

# First call in TRACK mode to set up internal state
print("\\n=== TRACK mode ===")
u_phys, info = ctrl.get_control_action(x)
# Manually advance internal s to near end
for _ in range(50):
    u_phys, info = ctrl.get_control_action(x)
rw, thr = ctrl.split_control(u_phys)
s = ctrl.s
print(f"s={s:.3f}, status={info['status_name']}")
print(f"thr: {[f'{t:.3f}' for t in thr]}")
active = [(i + 1, t) for i, t in enumerate(thr) if abs(t) > 0.01]
print(f"Active: {active}")

# Switch to SETTLE mode
ctrl.set_runtime_mode("SETTLE")
print("\n=== SETTLE mode ===")
u_phys, info = ctrl.get_control_action(x)
rw, thr = ctrl.split_control(u_phys)
s = ctrl.s
print(f"s={s:.3f}, status={info['status_name']}")
print(f"thr: {[f'{t:.3f}' for t in thr]}")
active = [(i + 1, t) for i, t in enumerate(thr) if abs(t) > 0.01]
print(f"Active: {active}")

# Repeat a few times to see convergence
for step in range(10):
    u_phys, info = ctrl.get_control_action(x)
    rw, thr = ctrl.split_control(u_phys)
    s = ctrl.s
    if step % 3 == 0:
        active = [(i + 1, f"{t:.3f}") for i, t in enumerate(thr) if abs(t) > 0.005]
        print(
            f"  step {step}: s={s:.3f}, active={active}, iter={info.get('iterations', 0)}"
        )

# Check path reference at s=19.20
s_test = 19.20
p_ref = ctrl._cpp.path_point(s_test)
print(f"\nPath ref at s={s_test}: {p_ref}")
print(f"Satellite pos: {x[:3]}")
pos_err = x[:3] - p_ref
print(f"Position error: {pos_err} ({np.linalg.norm(pos_err) * 1000:.1f}mm)")

# Check endpoint
p_end = ctrl._cpp.path_point(path_len)
endpoint_err = np.linalg.norm(x[:3] - p_end)
print(f"Endpoint: {p_end}")
print(f"Endpoint error: {endpoint_err * 1000:.1f}mm")

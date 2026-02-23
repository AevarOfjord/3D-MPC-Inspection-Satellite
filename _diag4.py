"""Diagnostic: Check B matrix thruster effects at ~180° yaw orientation."""

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
ctrl.set_runtime_mode("RECOVER")

# State matching telemetry: pos=[15.7, 1.5, -1.6], yaw=-178°, vel~[0.9,0,0]
x = np.zeros(16, dtype=np.float64)
x[0] = 15.7  # px
x[1] = 1.5  # py
x[2] = -1.6  # pz

# Quaternion for yaw=-178°, roll≈0, pitch≈-1.5°
yaw_rad = np.radians(-178.0)
pitch_rad = np.radians(-1.5)
roll_rad = np.radians(0.1)
cy, sy = np.cos(yaw_rad / 2), np.sin(yaw_rad / 2)
cp, sp = np.cos(pitch_rad / 2), np.sin(pitch_rad / 2)
cr, sr = np.cos(roll_rad / 2), np.sin(roll_rad / 2)
x[3] = cr * cp * cy + sr * sp * sy
x[4] = sr * cp * cy - cr * sp * sy
x[5] = cr * sp * cy + sr * cp * sy
x[6] = cr * cp * sy - sr * sp * cy
qn = np.linalg.norm(x[3:7])
x[3:7] /= qn

# Linear velocity ~0.9 m/s in +X
x[7] = 0.9
x[8] = 0.02
x[9] = -0.05
# Small angular velocity
x[10] = 0.01
x[11] = 0.01
x[12] = 0.01

print(f"State: pos={x[:3]}")
print(f"       quat={x[3:7]}")
print(f"       vel={x[7:10]}")
print(f"       omega={x[10:13]}")

# Check quaternion rotation: what does body -X become in world?
from scipy.spatial.transform import Rotation

R = Rotation.from_quat([x[4], x[5], x[6], x[3]]).as_matrix()  # scipy uses [x,y,z,w]
print("\nRotation matrix (body→world):")
print(R)
print(f"\nBody -X (thr 1) in world: {R @ np.array([-1, 0, 0])}")
print(f"Body +X (thr 2) in world: {R @ np.array([+1, 0, 0])}")
print(f"Body -Y (thr 3) in world: {R @ np.array([0, -1, 0])}")
print(f"Body +Y (thr 4) in world: {R @ np.array([0, +1, 0])}")
print(f"Body -Z (thr 5) in world: {R @ np.array([0, 0, -1])}")
print(f"Body +Z (thr 6) in world: {R @ np.array([0, 0, +1])}")

# Check CasADi B matrix at this state
x_aug = np.append(x, 0.0)  # add s=0
u_zero = np.zeros(10, dtype=np.float64)
p = ctrl._casadi_params
dt = ctrl._dt

result = ctrl._f_and_jacs(x_aug, u_zero, p, dt)
x_next = np.array(result[0]).ravel()
A_k = np.array(result[1])
B_k = np.array(result[2])

print("\n--- B matrix: velocity rows (7-9) for each thruster ---")
print(f"{'Thr':>4} {'B_vx':>10} {'B_vy':>10} {'B_vz':>10}  World Force Direction")
for j in range(6):
    bv = B_k[7:10, 3 + j]  # velocity effect of thruster j
    direction = bv / (np.linalg.norm(bv) + 1e-15)
    print(
        f"  {j + 1}   {bv[0]:10.6f} {bv[1]:10.6f} {bv[2]:10.6f}  [{direction[0]:+.3f}, {direction[1]:+.3f}, {direction[2]:+.3f}]"
    )

print("\n--- B matrix: angular vel rows (10-12) for each RW ---")
for j in range(3):
    bw = B_k[10:13, j]
    print(f"  RW{j + 1} {bw[0]:10.6f} {bw[1]:10.6f} {bw[2]:10.6f}")

# Path reference at s=0
print("\n--- Position error and needed correction ---")
try:
    ref = np.array([path[0][0], path[0][1], path[0][2]], dtype=float)
except (IndexError, TypeError):
    ref = np.array([10.0, 0.0, 0.0])
print(f"Path ref at s=0: {ref}")
pos_err = x[:3] - ref
print(f"Position error: {pos_err}")
print(f"Need force direction: {-pos_err / np.linalg.norm(pos_err)}")

# What the optimal thruster selection should be
print("\n--- Expected optimal thrusters ---")
needed_dir = -pos_err / np.linalg.norm(pos_err)
for j in range(6):
    bv = B_k[7:10, 3 + j]
    direction = bv / (np.linalg.norm(bv) + 1e-15)
    alignment = np.dot(direction, needed_dir)
    print(
        f"  Thr {j + 1}: alignment = {alignment:+.4f} {'<-- USE' if alignment > 0.3 else ''}"
    )

# Run MPC solve
print("\n--- MPC Solve ---")
u_phys, info = ctrl.get_control_action(x)
rw, thr = ctrl.split_control(u_phys)
print(f"status: {info['status']} ({info['status_name']})")
print(f"solver_status: {info['solver_status']}")
print(f"iterations: {info['iterations']}")
print(f"objective: {info['objective_value']}")
print(f"solve_time: {info['solve_time'] * 1000:.1f}ms")
print(f"rw:  {rw}")
print(f"thr: {thr}")

# Verify: which thrusters did the MPC select?
active = [(i + 1, t) for i, t in enumerate(thr) if abs(t) > 0.01]
print(f"Active thrusters: {active}")

# Check what force these produce in world frame
thr_world_force = np.zeros(3)
for j in range(6):
    thr_world_force += (
        B_k[7:10, 3 + j] * thr[j] / dt
    )  # B is discrete, divide by dt for force-like
print(f"Net world velocity change from thrusters: {B_k[7:10, 3:9] @ thr}")
print(f"Toward or away from ref? dot={np.dot(B_k[7:10, 3:9] @ thr, -pos_err):.6f}")

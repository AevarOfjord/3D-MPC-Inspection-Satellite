"""Diagnostic: Trace MPC control pipeline with actual runtime state."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src", "python"))

import numpy as np
from config.defaults import create_default_app_config

cfg = create_default_app_config()

from control.mpc_controller import MPCController

ctrl = MPCController(cfg)

# Check key attributes
print(f"rw_torque_limits: {ctrl.rw_torque_limits}")
print(f"max_rw_torque:    {ctrl.max_rw_torque}")
print(f"_f_and_jacs set:  {ctrl._f_and_jacs is not None}")

# Set path matching user's 19.22m path (circle-like test?)
# From missions/Circle_Test.json potentially
try:
    import json

    with open("missions/Circle_Test.json") as f:
        mission = json.load(f)
    if "path" in mission:
        path = mission["path"]
    elif "waypoints" in mission:
        path = mission["waypoints"]
    else:
        # Try finding path key
        print(f"Mission keys: {list(mission.keys())}")
        path = None
except Exception as e:
    print(f"Couldn't load mission: {e}")
    path = None

if path is None:
    # Use a 19.22m path approximation
    path = [[10, 0, 0], [15, 5, 0], [20, 10, 0], [25, 5, 0], [30, 0, 0]]

ctrl.set_path(path)
print(f"Path length: {ctrl._path_length:.2f}m")

# Set RECOVER mode like the actual run
ctrl.set_runtime_mode("RECOVER")

# Build state from user's telemetry:
# Pos: [10.982, 0.086, 0.036]
# Yaw: 110.5°, Roll: -47°, Pitch: 5.7°
# Ang Vel Err: 524.9°/s → satellite has ~9.16 rad/s angular velocity
x = np.zeros(16, dtype=np.float64)
x[0] = 10.982
x[1] = 0.086
x[2] = 0.036

# Euler to quaternion (ZYX convention)
yaw_rad = np.radians(110.5)
roll_rad = np.radians(-47.0)
pitch_rad = np.radians(5.7)
cy, sy = np.cos(yaw_rad / 2), np.sin(yaw_rad / 2)
cp, sp = np.cos(pitch_rad / 2), np.sin(pitch_rad / 2)
cr, sr = np.cos(roll_rad / 2), np.sin(roll_rad / 2)
# qw, qx, qy, qz
x[3] = cr * cp * cy + sr * sp * sy
x[4] = sr * cp * cy - cr * sp * sy
x[5] = cr * sp * cy + sr * cp * sy
x[6] = cr * cp * sy - sr * sp * cy
# Normalize
qn = np.linalg.norm(x[3:7])
x[3:7] /= qn

# Angular velocity ~ 9.16 rad/s total (mostly yaw)
x[10] = 0.5  # wx
x[11] = 0.3  # wy
x[12] = 9.0  # wz (mostly yaw spin)
# Linear velocity
x[7] = 0.2  # vx

print(f"\nState: pos={x[:3]}")
print(f"       quat={x[3:7]}")
print(f"       vel={x[7:10]}")
print(f"       omega={x[10:13]}")
print(
    f"       omega_norm={np.linalg.norm(x[10:13]):.2f} rad/s = {np.degrees(np.linalg.norm(x[10:13])):.1f} deg/s"
)

# Check CasADi linearization directly
print("\n--- CasADi Linearization Check ---")
N = ctrl._cpp.prediction_horizon
p = ctrl._casadi_params

for k in range(min(3, N)):
    x_k = np.array(ctrl._cpp.get_stage_state(k), dtype=float)
    u_k = np.array(ctrl._cpp.get_stage_control(k), dtype=float)

    if np.linalg.norm(x_k[:3]) < 1e-6:
        x_k = np.array(np.append(x, 0.0), dtype=float)  # add s=0
        print(f"  Stage {k}: using x_current (stage state was zero)")

    try:
        result = ctrl._f_and_jacs(x_k, u_k, p, ctrl._dt)
        x_next = np.array(result[0]).ravel()
        A_k = np.array(result[1])
        B_k = np.array(result[2])

        has_nan = not (
            np.all(np.isfinite(A_k))
            and np.all(np.isfinite(B_k))
            and np.all(np.isfinite(x_next))
        )
        print(
            f"  Stage {k}: A finite={np.all(np.isfinite(A_k))}, "
            f"B finite={np.all(np.isfinite(B_k))}, "
            f"|B|={np.linalg.norm(B_k):.4f}, "
            f"NaN/Inf={has_nan}"
        )

        if has_nan:
            print(f"    A NaN count: {np.count_nonzero(~np.isfinite(A_k))}")
            print(f"    B NaN count: {np.count_nonzero(~np.isfinite(B_k))}")
            print(f"    x_next NaN: {np.count_nonzero(~np.isfinite(x_next))}")

        # Check B matrix structure - are thruster columns non-zero?
        B_thr = B_k[:, 3:9]  # thruster columns
        B_rw = B_k[:, 0:3]  # RW columns
        print(
            f"    B_rw norms: [{np.linalg.norm(B_rw[:, 0]):.6f}, "
            f"{np.linalg.norm(B_rw[:, 1]):.6f}, "
            f"{np.linalg.norm(B_rw[:, 2]):.6f}]"
        )
        print(
            f"    B_thr norms: [{', '.join(f'{np.linalg.norm(B_thr[:, j]):.6f}' for j in range(6))}]"
        )
    except Exception as e:
        print(f"  Stage {k}: CasADi EXCEPTION: {e}")

# Now try MPC solve
print("\n--- MPC Solve ---")
u_phys, info = ctrl.get_control_action(x)
rw, thr = ctrl.split_control(u_phys)

print(f"status: {info['status']} ({info['status_name']})")
print(f"solver_status: {info['solver_status']}")
print(f"iterations: {info['iterations']}")
print(f"objective: {info['objective_value']}")
print(f"solve_time: {info['solve_time'] * 1000:.1f}ms")
print(f"rw (norm): {rw}")
print(f"thrusters: {thr}")
print(f"path_s: {info.get('path_s')}")
print(f"path_error: {info.get('path_error')}")

# Verify 2nd and 3rd solves
for i in range(2):
    u2, info2 = ctrl.get_control_action(x)
    rw2, thr2 = ctrl.split_control(u2)
    print(
        f"\nSolve {i + 2}: status={info2['status']}, solver_status={info2['solver_status']}, "
        f"iter={info2['iterations']}"
    )
    print(f"  rw={rw2}")
    print(f"  thr={thr2}")

# Also try with ZERO angular velocity - should produce non-zero controls
print("\n\n--- Low Angular Velocity Test ---")
ctrl2 = MPCController(cfg)
ctrl2.set_path(path)
ctrl2.set_runtime_mode("RECOVER")

x_calm = x.copy()
x_calm[10:13] = 0.0  # zero angular velocity
u_calm, info_calm = ctrl2.get_control_action(x_calm)
rw_calm, thr_calm = ctrl2.split_control(u_calm)
print(f"status: {info_calm['status']} ({info_calm['status_name']})")
print(f"rw: {rw_calm}")
print(f"thr: {thr_calm}")

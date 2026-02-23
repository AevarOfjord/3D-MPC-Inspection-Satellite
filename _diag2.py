"""Diagnostic: Run one MPC solve and dump raw outputs."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src", "python"))

import numpy as np

# Build config
from config.defaults import create_default_app_config

cfg = create_default_app_config()

# Build controller
from control.mpc_controller import MPCController

ctrl = MPCController(cfg)

# Check attributes
print(f"num_rw_axes:     {ctrl.num_rw_axes}")
print(f"num_thrusters:   {ctrl.num_thrusters}")
print(f"rw_torque_limits:{ctrl.rw_torque_limits}")
print(f"max_rw_torque:   {ctrl.max_rw_torque}")
print(f"nx={ctrl.nx}, nu={ctrl.nu}")
print(f"N={ctrl.N}, dt={ctrl._dt}")

# Set up a simple path
path = [
    [10, 0, 0],
    [15, 5, 0],
    [20, 0, 0],
]
ctrl.set_path(path)
print(f"\nPath set, path_length={ctrl._path_length:.2f}m")

# State: satellite at [10.98, 0.09, 0.04] with yaw~153°, big angular velocity
x = np.zeros(16, dtype=np.float64)
x[0] = 10.98  # x position
x[1] = 0.09  # y position
x[2] = 0.04  # z position
# Quaternion for ~153° yaw (qw, qx, qy, qz)
yaw_rad = np.radians(153)
x[3] = np.cos(yaw_rad / 2)  # qw
x[4] = 0.0
x[5] = 0.0
x[6] = np.sin(yaw_rad / 2)  # qz
# small velocity
x[7] = 0.2  # vx

print(f"\nState: pos={x[:3]}, quat={x[3:7]}, vel={x[7:10]}")

# Run solver
u_phys, info = ctrl.get_control_action(x)
print("\n--- Solver Result ---")
print(f"status:          {info.get('status')} ({info.get('status_name')})")
print(f"solver_status:   {info.get('solver_status')}")
print(f"solver_fallback: {info.get('solver_fallback')}")
print(f"solver_success:  {info.get('solver_success')}")
print(f"solve_time:      {info.get('solve_time', 0) * 1000:.1f}ms")
print(f"iterations:      {info.get('iterations')}")
print(f"objective:       {info.get('objective_value')}")
print(f"timeout:         {info.get('timeout')}")
print(f"path_s:          {info.get('path_s')}")
print(f"path_error:      {info.get('path_error')}")

rw, thr = ctrl.split_control(u_phys)
print("\n--- Control Output ---")
print(f"u_phys shape:  {u_phys.shape}")
print(f"u_phys:        {u_phys}")
print(f"rw (norm):     {rw}")
print(f"thrusters:     {thr}")
print(f"rw_physical:   {[rw[i] * ctrl.rw_torque_limits[i] for i in range(len(rw))]}")

# Check what CasADi linearization produced
print("\n--- CasADi Dynamics ---")
print(f"_dynamics:     {ctrl._dynamics}")
print(f"_f_and_jacs:   {'set' if ctrl._f_and_jacs else 'None'}")

# Second solve (after warm start)
u_phys2, info2 = ctrl.get_control_action(x)
rw2, thr2 = ctrl.split_control(u_phys2)
print("\n--- Second Solve ---")
print(f"status:       {info2.get('status')} ({info2.get('status_name')})")
print(f"solver_st:    {info2.get('solver_status')}")
print(f"rw (norm):    {rw2}")
print(f"thrusters:    {thr2}")

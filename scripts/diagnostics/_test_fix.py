"""Test acados_rti on 1Turn mission after the omega-weight fix."""

import json
import logging
import os
import sys

sys.path.insert(0, ".")
os.environ.setdefault("SATELLITE_HEADLESS", "1")
logging.disable(logging.WARNING)

import numpy as np

# Load mission
with open("missions/1Turn.json") as f:
    mission_json = json.load(f)

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.mission.runtime_loader import (
    compile_unified_mission_runtime,
    parse_unified_mission_payload,
)

cfg = SimulationConfig.create_default()
app = cfg.app_config

print("Loading acados_rti (should hit cache — no recompile)...", flush=True)
import time

t0 = time.perf_counter()

from controller.acados_rti.python.controller import AcadosRtiController

ctrl = AcadosRtiController(app)

t1 = time.perf_counter()
print(f"  Loaded in {t1 - t0:.1f}s", flush=True)

Q_angvel = ctrl.Q_angvel
N = ctrl.N
print(f"  Q_angvel = {Q_angvel}")
print(f"  Stage omega weight (code computes {Q_angvel / 100:.1f}, was {Q_angvel:.1f})")
print(
    f"  Terminal omega weight (code computes {Q_angvel * N / 200:.1f}, was {Q_angvel * N:.1f})"
)
print(f"  Terminal att weight: {ctrl.Q_attitude:.1f}")
print(
    f"  Improvement ratio terminal att/omega: {ctrl.Q_attitude / (Q_angvel * N / 200):.1f}x (was {ctrl.Q_attitude / (Q_angvel * N):.2f}x)"
)

# Run simulation
print("\nSetting up path...", flush=True)
payload = parse_unified_mission_payload(mission_json)
runtime = compile_unified_mission_runtime(payload, simulation_config=cfg)

path_pts = runtime.path  # list[tuple[float,float,float]]
print(f"  Path: {len(path_pts)} points")

ctrl.set_runtime_mode("TRACK")
ctrl.set_path(path_pts)

# Initial state: 17-dim [pos(3), q(4), vel(3), omega(3), wr(3), s(1)]
x0 = np.array(
    [
        float(path_pts[0][0]),
        float(path_pts[0][1]),
        float(path_pts[0][2]),  # pos
        1.0,
        0.0,
        0.0,
        0.0,  # quat (identity)
        0.0,
        0.0,
        0.0,  # vel
        0.0,
        0.0,
        0.0,  # omega
        0.0,
        0.0,
        0.0,  # rw speeds
        0.0,  # s
    ]
)

print("\nRunning 60 steps (3s) of control loop:", flush=True)
x = x0.copy()
dt = ctrl._dt

max_att_err = 0.0
max_pos_err = 0.0
rw_saturated_steps = 0

for step in range(60):
    u, info = ctrl.get_control_action(x)
    tau_rw = u[: ctrl.num_rw_axes]

    # Very simple attitude integration (just track omega for diagnosis)
    # x[10:13] = omega, x[13:16] = rw speeds
    omega = x[10:13]
    wr = x[13:16]

    # Compute q_ref for current step
    refs = ctrl._build_reference_trajectory(x)
    q_ref = refs[6:10, 0]
    q_curr = x[3:7]
    dot = abs(np.dot(q_curr, q_ref))
    att_err_deg = 2 * np.degrees(np.arccos(min(1.0, dot)))

    pos_err = np.linalg.norm(x[0:3] - refs[0:3, 0])
    max_att_err = max(max_att_err, att_err_deg)
    max_pos_err = max(max_pos_err, pos_err)

    rw_norm = np.max(np.abs(tau_rw))
    if rw_norm > 0.075:  # close to delta_u_max = 0.08
        rw_saturated_steps += 1

    if step < 5 or step % 10 == 0:
        omega_dps = np.degrees(np.linalg.norm(omega))
        print(
            f"  t={step * dt:.2f}s: att_err={att_err_deg:.1f}°  omega={omega_dps:.2f}°/s  "
            f"tau_rw={tau_rw[0]:.4f},{tau_rw[1]:.4f},{tau_rw[2]:.4f}"
        )

    # Physics step (simplified: just propagate omega from torques)
    # In full sim this would use RK4, here just Euler for diagnosis
    # tau_rw normalized: actual torque = u * max_torque
    max_torques = ctrl.rw_torque_limits
    I_body = ctrl.moment_of_inertia  # diagonal [Ixx, Iyy, Izz]
    alpha = np.array([tau_rw[i] * max_torques[i] / I_body[i] for i in range(3)])

    x = x.copy()
    x[10:13] = omega + alpha * dt  # omega update
    x[13:16] = wr + tau_rw * dt  # rw speed update (simplified)
    # Quaternion integration
    omg_new = x[10:13]
    omg_mag = np.linalg.norm(omg_new)
    if omg_mag > 1e-10:
        angle = omg_mag * dt
        axis = omg_new / omg_mag
        s_half = np.sin(angle / 2)
        dq = np.array(
            [np.cos(angle / 2), s_half * axis[0], s_half * axis[1], s_half * axis[2]]
        )
        # q_new = dq * q_curr (quaternion multiply, scalar-first wxyz)
        w1, x1, y1, z1 = q_curr
        w2, x2, y2, z2 = dq
        q_new = np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )
        x[3:7] = q_new / np.linalg.norm(q_new)

print("\nSummary after 60 steps:")
print(f"  Max attitude error: {max_att_err:.2f}°")
print(f"  Max position error: {max_pos_err:.3f}m")
print(f"  Steps with |tau_rw| > 0.075 (near rate-limit): {rw_saturated_steps}/60")

if max_att_err < 15.0 and rw_saturated_steps < 10:
    print(
        "\n✓ FIX APPEARS TO WORK — attitude error is bounded, no persistent RW saturation"
    )
elif max_att_err < 30.0:
    print("\n~ PARTIAL IMPROVEMENT — better than before but still needs tuning")
else:
    print("\n✗ STILL FAILING — need further investigation")

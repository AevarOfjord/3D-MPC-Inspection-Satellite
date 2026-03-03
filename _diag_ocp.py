"""Diagnostic: what does the acados q_ref horizon look like, and what does
the NMPC do with the same input? Answers *why* acados pre-rotates."""

import json
import math
import os
import sys

sys.path.insert(0, ".")
os.environ.setdefault("SATELLITE_HEADLESS", "1")

import numpy as np

# ── Load mission path ──────────────────────────────────────────────────────
with open("missions/1Turn.json") as f:
    mission = json.load(f)

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.mission.runtime_loader import (
    parse_unified_mission_payload,
)
from controller.shared.python.mission.unified_compiler import (
    compile_unified_mission_runtime,
)

cfg = SimulationConfig.create_default()
app = cfg.app_config

import logging

logging.disable(logging.CRITICAL)

try:
    payload = parse_unified_mission_payload(mission)
    runtime = compile_unified_mission_runtime(payload, app)
    path = [(p[0], p[1], p[2]) for p in runtime.path_points]
    print(f"Path: {len(path)} points")
except Exception as e:
    print(f"Could not load mission path: {e}")
    path = []

# ── Instantiate acados_rti controller (uses compiled .dylib if cached) ─────
print("Loading acados_rti controller (may take a moment if recompiling)...")
from controller.acados_rti.python.controller import AcadosRtiController

ctrl = AcadosRtiController(app)

if path:
    ctrl.set_runtime_mode("TRACK")
    ctrl.set_path(path)

# ── Build reference trajectory from initial state ──────────────────────────
# Use the state from the simulation start
x0 = np.array(
    [
        2.3,
        0.0,
        2.0,
        1.0,
        0.0,
        0.0,
        0.0,  # pos(m), quat wxyz
        0.0,
        0.0,
        0.0,  # vel
        0.0,
        0.0,
        0.0,  # omega
        0.0,
        0.0,
        0.0,  # rw speeds
        0.0,
    ]
)  # path s

refs = ctrl._build_reference_trajectory(x0)  # (10, N+1)

print(f"\nHorizon reference trajectory (N={ctrl.N}, dt={ctrl._dt}s):")
print(f"{'k':>4}  {'s_k (m)':>8}  {'q_ref(wxyz)':>35}  {'ang vs k0 (°)':>14}")
q0 = refs[6:10, 0]
for k in [0, 5, 10, 20, 30, 40, 50]:
    sk = ctrl.s + k * ctrl._dt * ctrl.path_speed
    qk = refs[6:10, k]
    # Angle between q0 and qk
    dq_w = q0[0] * qk[0] + q0[1] * qk[1] + q0[2] * qk[2] + q0[3] * qk[3]
    angle = 2 * math.degrees(math.acos(min(1.0, abs(dq_w))))
    print(
        f"{k:>4}  {sk:>8.3f}  {qk[0]:>7.4f} {qk[1]:>7.4f} {qk[2]:>7.4f} {qk[3]:>7.4f}  {angle:>14.2f}°"
    )

print("\nPath tangents k=0 and k=50:")
print(f"  t_ref[k=0]:  {refs[3:6, 0]}")
print(f"  t_ref[k=50]: {refs[3:6, 50]}")
dot_tangent = float(np.dot(refs[3:6, 0], refs[3:6, 50]))
dot_tangent = max(-1.0, min(1.0, dot_tangent))
tangent_change_deg = math.degrees(math.acos(dot_tangent))
print(f"  tangent change angle: {tangent_change_deg:.2f}°")
print(
    f"\nThis is the attitude rotation the OCP is trying to achieve in {ctrl.N * ctrl._dt:.1f}s"
)

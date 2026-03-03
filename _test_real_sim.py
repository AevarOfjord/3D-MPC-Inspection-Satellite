"""
Run acados_rti through the real SatelliteMPCLinearizedSimulation engine
and report per-step telemetry to diagnose the fix.
"""

import json
import logging
import os
import sys
import time

sys.path.insert(0, ".")
os.environ.setdefault("SATELLITE_HEADLESS", "1")
logging.disable(logging.WARNING)

import numpy as np

# Load 1Turn mission
with open("missions/1Turn.json") as f:
    mission_json = json.load(f)

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.mission.runtime_loader import (
    compile_unified_mission_runtime,
    parse_unified_mission_payload,
)
from controller.shared.python.simulation.engine import SatelliteMPCLinearizedSimulation

cfg = SimulationConfig.create_default()

# Override to use acados_rti
import dataclasses

app = cfg.app_config
mpc_core = dataclasses.replace(app.mpc_core, controller_profile="acados_rti")
app2 = dataclasses.replace(app, mpc_core=mpc_core)

payload = parse_unified_mission_payload(mission_json)
runtime = compile_unified_mission_runtime(payload, simulation_config=cfg)

print(f"Path: {len(runtime.path)} points, {runtime.path_length:.1f}m")
print("Building simulation engine with acados_rti controller...")
t0 = time.perf_counter()

sim = SatelliteMPCLinearizedSimulation(
    mission_config=runtime,
    app_config=app2,
)
t1 = time.perf_counter()
print(f"  Engine ready in {t1 - t0:.1f}s")

# Step-by-step simulation with detailed logging
print("\nRunning 100 steps (5s):")
print(
    f"{'t':>5}  {'att_err':>8}  {'pos_err':>8}  {'|omega|':>8}  {'tau_rw':>30}  {'status':>12}"
)

max_att = 0.0
max_pos = 0.0
rw_sat_count = 0

for step in range(100):
    result = sim.step()
    t = result.get("t", step * 0.05)
    att_err = result.get("attitude_error_deg", float("nan"))
    pos_err = result.get("position_error_m", float("nan"))
    state = result.get("state", np.zeros(17))
    ctrl_u = result.get("control_u", np.zeros(10))
    status = result.get("status", "?")

    omega = state[10:13] if len(state) > 12 else np.zeros(3)
    omega_dps = np.degrees(np.linalg.norm(omega))

    tau_rw = ctrl_u[:3]
    rw_max = np.max(np.abs(tau_rw))

    max_att = max(max_att, abs(att_err) if not np.isnan(att_err) else 0)
    max_pos = max(max_pos, abs(pos_err) if not np.isnan(pos_err) else 0)
    if rw_max > 0.075:
        rw_sat_count += 1

    if step < 10 or step % 10 == 0 or att_err > 30:
        rw_str = f"{tau_rw[0]:+.3f},{tau_rw[1]:+.3f},{tau_rw[2]:+.3f}"
        print(
            f"{t:>5.2f}  {att_err:>8.2f}°  {pos_err:>8.3f}m  {omega_dps:>7.2f}°/s  {rw_str:>30}  {status:>12}"
        )

    if status in ("COMPLETE", "FAILED", "ERROR"):
        print(f"\nStopped at step {step}: {status}")
        break

print(f"\n{'=' * 70}")
print(f"Max attitude error: {max_att:.2f}°")
print(f"Max position error: {max_pos:.3f}m")
print(f"RW near-saturation steps: {rw_sat_count}/100")

if max_att < 15.0:
    print("\n✓ FIX WORKING — attitude stays bounded")
elif max_att < 45.0:
    print("\n~ PARTIAL — better but still drifting")
else:
    print("\n✗ STILL FAILING")

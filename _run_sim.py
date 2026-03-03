"""Run a short simulation with a given profile and the 1Turn.json mission."""

import json
import logging
import sys

import numpy as np

logging.basicConfig(level=logging.WARNING)

profile = sys.argv[1] if len(sys.argv) > 1 else "hybrid"
max_time = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.mission.runtime_loader import (
    compile_unified_mission_runtime,
    parse_unified_mission_payload,
)
from controller.shared.python.simulation.engine import SatelliteMPCLinearizedSimulation
from controller.shared.python.simulation.loop import SimulationLoop

mission_json = json.load(open("missions/1Turn.json"))
simulation_config = SimulationConfig.create_default()
mission_def = parse_unified_mission_payload(mission_json)
mission_runtime = compile_unified_mission_runtime(
    mission_def, simulation_config=simulation_config
)
simulation_config = mission_runtime.simulation_config

# Apply profile and duration overrides
simulation_config = SimulationConfig.create_with_overrides(
    {
        "mpc_core": {"controller_profile": profile},
        "simulation": {"max_duration": max_time, "headless": True},
    },
    base_config=simulation_config,
)

sim = SatelliteMPCLinearizedSimulation(
    start_pos=mission_runtime.start_pos,
    end_pos=mission_runtime.end_pos,
    simulation_config=simulation_config,
)
loop = SimulationLoop(sim)

print(f"Profile: {profile}")
print(
    f"Path: {len(mission_runtime.path)} pts, length={mission_runtime.path_length:.3f}m"
)
print(f"Running for {max_time}s...")

try:
    loop.run(show_animation=False)
except Exception as e:
    print(f"ERROR: {e}")
    import traceback

    traceback.print_exc()

print(f"Final sim time: {sim.simulation_time:.2f}s")
print(f"Final pos: {sim.current_state[:3]}")
ctrl = sim.mpc_controller
print(
    f"Path s: {ctrl.s:.3f} / {ctrl._path_length:.3f} ({100 * ctrl.s / max(ctrl._path_length, 1e-9):.1f}%)"
)
mode = getattr(sim, "mode_state", None)
print(f"Final mode: {getattr(mode, 'current_mode', 'N/A')}")
if hasattr(ctrl, "solve_times") and ctrl.solve_times:
    times = list(ctrl.solve_times)
    print(
        f"Solve times: mean={np.mean(times) * 1000:.1f}ms max={np.max(times) * 1000:.1f}ms over {len(times)} steps"
    )

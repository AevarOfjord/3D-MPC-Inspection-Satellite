import time

import numpy as np

from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.control import MPCController
from scipy.spatial.transform import Rotation


def test_mpc_perf():
    # Setup - use 3D tuples
    start_pos = (1.0, 1.0, 0.0)
    start_euler = (0.0, 0.0, 0.0)
    end_pos = (0.0, 0.0, 0.0)

    # Init Controller
    sim_config = SimulationConfig.create_default()
    controller = MPCController(sim_config.app_config)
    controller.set_path([start_pos, end_pos])

    # Current State (13 elements): [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
    # Convert Euler to Quat (w, x, y, z)
    q_start = Rotation.from_euler("xyz", start_euler, degrees=False).as_quat()
    # Scipy returns [x, y, z, w], we want [w, x, y, z]
    q_start_mj = [q_start[3], q_start[0], q_start[1], q_start[2]]

    x_current = np.zeros(16)
    x_current[0:3] = np.array(start_pos, dtype=float)
    x_current[3:7] = np.array(q_start_mj, dtype=float)

    # Warm up
    print("Warming up...")
    controller.get_control_action(x_current)

    print("Testing repeated solves on path-following controller...")

    # Loop
    times = []
    for i in range(20):
        t0 = time.perf_counter()
        # Pass trajectory
        controller.get_control_action(x_current)
        dt = time.perf_counter() - t0
        times.append(dt)
        print(f"Step {i}: {dt*1000:.3f} ms")

    avg = np.mean(times)
    print(f"Average Solve Time: {avg*1000:.3f} ms")


if __name__ == "__main__":
    test_mpc_perf()

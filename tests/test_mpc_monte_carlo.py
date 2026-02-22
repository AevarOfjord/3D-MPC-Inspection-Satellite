"""
Monte Carlo regression harness for MPC robustness and timing.
"""

import numpy as np
import pytest
from config.simulation_config import SimulationConfig
from control.mpc_controller import MPCController
from core.cpp_satellite import CppSatelliteSimulator


@pytest.mark.slow
def test_mpc_monte_carlo_regression_contracts():
    """
    Randomized closed-loop campaign with timing and robustness gates.

    Gates are intentionally practical (not razor-thin) so the test is stable
    across CI hardware variance while still catching regressions.
    """
    cfg = SimulationConfig.create_default()
    cfg.app_config.mpc.robustness_mode = "tube"
    cfg.app_config.mpc.constraint_tightening_scale = 0.05
    cfg.app_config.mpc.tube_feedback_gain_scale = 0.15
    cfg.app_config.mpc.tube_feedback_max_correction = 0.2

    controller = MPCController(cfg.app_config)
    dt = float(cfg.app_config.mpc.dt)

    seeds = list(range(8))
    steps_per_seed = 24
    solve_times: list[float] = []
    success_count = 0
    total_count = 0

    for seed in seeds:
        rng = np.random.default_rng(seed)
        sim = CppSatelliteSimulator(app_config=cfg.app_config)

        sim.position = rng.normal(0.0, 0.25, size=3).astype(np.float64)
        sim.velocity = rng.normal(0.0, 0.04, size=3).astype(np.float64)
        sim.angular_velocity = rng.normal(0.0, 0.03, size=3).astype(np.float64)

        for _ in range(steps_per_seed):
            state = np.concatenate(
                [
                    sim.position,
                    sim.quaternion,
                    sim.velocity,
                    sim.angular_velocity,
                    sim.wheel_speeds,
                ]
            )
            u, info = controller.get_control_action(state)
            rw_cmd, thruster_cmd = controller.split_control(u)

            limits = np.array([0.06, 0.06, 0.06], dtype=np.float64)
            sim.set_reaction_wheel_torque(rw_cmd * limits[: rw_cmd.size])
            sim.apply_force(list(thruster_cmd))
            sim.update_physics(dt)

            solve_t = float(info.get("solve_time", 0.0))
            solve_times.append(solve_t)
            total_count += 1
            if int(info.get("status", -1)) == 1:
                success_count += 1

            assert np.isfinite(sim.position).all()
            assert np.isfinite(sim.velocity).all()
            assert np.isfinite(sim.angular_velocity).all()
            assert np.isfinite(sim.quaternion).all()

    assert total_count > 0
    success_rate = success_count / total_count
    p95 = float(np.percentile(np.array(solve_times, dtype=float), 95))

    # Practical contracts for broad hardware reproducibility.
    assert success_rate >= 0.90
    assert p95 <= max(2.0 * dt, 0.08)

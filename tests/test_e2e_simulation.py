"""
End-to-End Simulation Tests.

Runs valid headless simulations to verify the entire stack holds together.
Consolidates `e2e/test_simulation_runner.py`.
"""

import json
import os

import pytest
from config.simulation_config import SimulationConfig
from simulation.artifact_paths import artifact_path
from simulation.engine import SatelliteMPCLinearizedSimulation
from simulation.loop import SimulationLoop


@pytest.mark.slow
class TestE2ESimulation:
    """End-to-End simulation tests."""

    @pytest.mark.skipif(
        os.getenv("RUN_E2E_SIM_TESTS") != "1",
        reason="Set RUN_E2E_SIM_TESTS=1 to run E2E simulation tests",
    )
    def test_simulation_run_completion(self, tmp_path):
        """Run a full simulation headless and verify output."""
        # Short duration for test
        config = SimulationConfig.create_with_overrides(
            {
                "simulation": {
                    "max_duration": 0.5,
                    "headless": True,
                }
            }
        )

        sim = SatelliteMPCLinearizedSimulation(
            start_pos=(1.0, 1.0, 0.0), end_pos=(0.0, 0.0, 0.0), simulation_config=config
        )

        # Set save path using tmp_path
        sim.data_save_path = tmp_path / "test_sim_data"

        loop = SimulationLoop(sim)

        # Explicitly set max time to ensure termination
        sim.max_simulation_time = 0.5

        try:
            output_path = loop.run(show_animation=False)
        except Exception as e:
            pytest.fail(f"Simulation crashed: {e}")

        # Verify it actually ran
        # Sim might stop early due to path completion or other reasons
        assert sim.simulation_time > 0.0
        assert output_path is not None
        assert output_path.exists()

        # Verify core simulation artifacts were created.
        assert artifact_path(output_path, "physics_data.csv").exists()
        assert artifact_path(output_path, "control_data.csv").exists()
        metrics_path = artifact_path(output_path, "performance_metrics.json")
        assert metrics_path.exists()

        metrics = json.loads(metrics_path.read_text())
        contract = metrics.get("mpc_timing_contract", {})
        assert "target_mean_ms" in contract
        assert "hard_max_ms" in contract
        assert "pass" in contract

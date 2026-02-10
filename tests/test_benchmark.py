"""
Performance Benchmarks and Regression Tests.

Benchmarks MPC solve times and core loops to detect performance regressions.
Consolidates `test_benchmark.py` and `benchmarks/test_physics_benchmarks.py`.
"""

import time
import numpy as np
import pytest
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.control.mpc_controller import MPCController

try:
    import pytest_benchmark

    BENCHMARK_AVAILABLE = True
except ImportError:
    BENCHMARK_AVAILABLE = False


@pytest.mark.skipif(not BENCHMARK_AVAILABLE, reason="pytest-benchmark not installed")
class TestBenchmarks:
    """Performance benchmarks."""

    @pytest.fixture
    def controller(self):
        config = SimulationConfig.create_default()
        return MPCController(config.app_config)

    @pytest.fixture
    def state_vector(self):
        state = np.zeros(16)
        state[0:3] = [1.0, 0.5, -0.2]
        state[3:7] = [1.0, 0.0, 0.0, 0.0]
        return state

    def test_mpc_solve_time(self, benchmark, controller, state_vector):
        """Benchmark MPC solver execution."""

        def run_solve():
            controller.get_control_action(state_vector)

        benchmark(run_solve)


class TestPerformanceRegressions:
    """Tests for performance regressions (without external benchmark tool)."""

    def test_mpc_solve_speed_check(self):
        """Ensure solver is reasonably fast (<50ms)."""
        config = SimulationConfig.create_default()
        controller = MPCController(config.app_config)

        state = np.zeros(16)
        state[0:3] = [1.0, 0.5, -0.2]
        state[3:7] = [1.0, 0.0, 0.0, 0.0]

        # Warmup
        controller.get_control_action(state)

        times = []
        for _ in range(20):
            start = time.perf_counter()
            controller.get_control_action(state)
            end = time.perf_counter()
            times.append(end - start)

        p95 = np.percentile(times, 95)

        # Threshold: 50ms is generous for OSQP (usually 5-10ms)
        assert p95 < 0.05, f"MPC solver too slow! P95: {p95 * 1000:.2f}ms"

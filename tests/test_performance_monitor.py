"""Tests for MPC timing contract tracking in performance monitoring."""

import json

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.runtime.performance_monitor import PerformanceMonitor


def test_default_simulation_timing_contract_config() -> None:
    """Default config exposes the expected MPC timing contract thresholds."""
    cfg = SimulationConfig.create_default()
    sim = cfg.app_config.simulation
    assert sim.mpc_target_mean_solve_time_ms == 5.0
    assert sim.mpc_hard_max_solve_time_ms == 35.0
    assert sim.enforce_mpc_timing_contract is False


def test_mpc_timing_contract_metrics_and_breach_count() -> None:
    """Contract metadata should report failures and hard-limit breaches."""
    monitor = PerformanceMonitor()
    monitor.set_mpc_timing_contract(target_mean_ms=5.0, hard_max_ms=35.0, enforce=False)

    monitor.record_mpc_solve(0.002)  # 2ms
    monitor.record_mpc_solve(0.060)  # 60ms -> hard-limit breach

    metrics = monitor.get_metrics()
    contract = metrics.to_dict()["mpc_timing_contract"]

    assert contract["hard_limit_breaches"] == 1
    assert contract["mean_pass"] is False
    assert contract["max_pass"] is False
    assert contract["pass"] is False


def test_enforced_mpc_timing_contract_signals_failure() -> None:
    """Strict contract mode should request run failure when violated."""
    monitor = PerformanceMonitor()
    monitor.set_mpc_timing_contract(target_mean_ms=5.0, hard_max_ms=35.0, enforce=True)

    monitor.record_mpc_solve(0.060)  # 60ms -> hard-limit breach

    assert monitor.timing_contract_violated() is True
    assert monitor.should_fail_on_timing_contract() is True


def test_metrics_json_export_is_complete_and_serializable(tmp_path) -> None:
    """Metrics export should produce complete JSON with native bool/float values."""
    monitor = PerformanceMonitor()
    monitor.set_mpc_timing_contract(target_mean_ms=5.0, hard_max_ms=35.0, enforce=False)
    monitor.record_mpc_solve(0.002)
    monitor.record_mpc_solve(0.060)

    out = tmp_path / "performance_metrics.json"
    monitor.export_metrics(out)

    payload = json.loads(out.read_text())
    contract = payload["mpc_timing_contract"]
    assert isinstance(contract["mean_pass"], bool)
    assert isinstance(contract["max_pass"], bool)
    assert isinstance(contract["pass"], bool)
    assert payload["mpc_mean_solve_time_ms"] > 0.0
    assert payload["mpc_max_solve_time_ms"] >= payload["mpc_mean_solve_time_ms"]

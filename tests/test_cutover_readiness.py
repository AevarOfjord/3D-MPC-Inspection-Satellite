"""Tests for V6 cutover readiness evaluation."""

from satellite_control.benchmarks.cutover_readiness import (
    evaluate_cutover_readiness,
)


def _scenario(name: str, *, passed: bool = True, timing_pass: bool = True) -> dict:
    contracts = {
        "mean_solve_ms": {"pass": timing_pass},
        "max_solve_ms": {"pass": timing_pass},
        "hard_limit_breaches": {"pass": timing_pass},
    }
    return {"name": name, "passed": passed, "contracts": contracts}


def test_cutover_readiness_passes_with_required_history() -> None:
    history = []
    for idx in range(10):
        history.append(
            {
                "schema_version": "mpc_quality_suite_v6",
                "generated_at": f"2026-02-2{idx}T00:00:00+00:00",
                "full": False,
                "passed": True,
                "scenarios": [_scenario("auto_short")],
            }
        )

    history.append(
        {
            "schema_version": "mpc_quality_suite_v6",
            "generated_at": "2026-03-01T00:00:00+00:00",
            "full": True,
            "passed": True,
            "scenarios": [
                _scenario("auto_short"),
                _scenario("planner_m4"),
                _scenario("planner_2m"),
                _scenario("stress_manual_scurve"),
                _scenario("long_completion_tier"),
            ],
        }
    )

    report = evaluate_cutover_readiness(
        suite_history=history,
        min_fast_consecutive=10,
        min_full_passes=1,
        schema_migration_ok=True,
    )

    assert report.ready is True
    assert all(item.passed for item in report.checks)


def test_cutover_readiness_fails_when_long_tier_or_timing_breach() -> None:
    history = []
    for idx in range(10):
        history.append(
            {
                "schema_version": "mpc_quality_suite_v6",
                "generated_at": f"2026-04-2{idx}T00:00:00+00:00",
                "full": False,
                "passed": True,
                "scenarios": [_scenario("auto_short")],
            }
        )

    history.append(
        {
            "schema_version": "mpc_quality_suite_v6",
            "generated_at": "2026-05-01T00:00:00+00:00",
            "full": True,
            "passed": False,
            "scenarios": [
                _scenario("auto_short"),
                _scenario("planner_m4"),
                _scenario("planner_2m", timing_pass=False),
                _scenario("stress_manual_scurve"),
                _scenario("long_completion_tier", passed=False),
            ],
        }
    )

    report = evaluate_cutover_readiness(
        suite_history=history,
        min_fast_consecutive=10,
        min_full_passes=1,
        schema_migration_ok=True,
    )

    assert report.ready is False
    check_map = {item.name: item for item in report.checks}
    assert check_map["full_suite_regression_gate"].passed is False
    assert check_map["long_completion_tier_gate"].passed is False
    assert check_map["timing_sla_gate"].passed is False

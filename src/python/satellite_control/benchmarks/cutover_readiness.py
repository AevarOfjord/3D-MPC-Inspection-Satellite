"""V6 default-cutover readiness checks based on quality-suite history."""

from __future__ import annotations

import json
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

EXPECTED_FULL_SCENARIOS = {
    "auto_short",
    "planner_m4",
    "planner_2m",
    "stress_manual_scurve",
    "long_completion_tier",
}
TIMING_CONTRACT_KEYS = ("mean_solve_ms", "max_solve_ms", "hard_limit_breaches")


@dataclass
class CutoverCheckResult:
    """Single cutover gate result."""

    name: str
    passed: bool
    details: str


@dataclass
class CutoverReadinessReport:
    """Serializable summary for V6 default-cutover readiness."""

    schema_version: str
    generated_at: str
    ready: bool
    checks: list[CutoverCheckResult]
    summary: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["checks"] = [asdict(item) for item in self.checks]
        return payload


def _now_iso() -> str:
    return datetime.now(UTC).isoformat()


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, int | float):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return False


def _sort_key(entry: dict[str, Any]) -> str:
    value = str(entry.get("generated_at", "")).strip()
    return value


def _normalize_suite_entry(raw: dict[str, Any]) -> dict[str, Any]:
    scenarios_raw = raw.get("scenarios", [])
    scenarios: list[dict[str, Any]] = []
    if isinstance(scenarios_raw, list):
        for item in scenarios_raw:
            if isinstance(item, dict):
                scenarios.append(item)
    return {
        "schema_version": str(raw.get("schema_version", "")).strip(),
        "generated_at": str(raw.get("generated_at", "")).strip(),
        "full": _coerce_bool(raw.get("full", False)),
        "passed": _coerce_bool(raw.get("passed", False)),
        "scenarios": scenarios,
    }


def load_suite_summaries(paths: Sequence[Path]) -> list[dict[str, Any]]:
    """Load quality-suite summaries from JSON files."""
    entries: list[dict[str, Any]] = []
    for path in paths:
        raw = _read_json(path)
        if isinstance(raw, dict):
            entries.append(_normalize_suite_entry(raw))
        elif isinstance(raw, list):
            for item in raw:
                if isinstance(item, dict):
                    entries.append(_normalize_suite_entry(item))
    return entries


def load_suite_history(path: Path) -> list[dict[str, Any]]:
    """Load suite history from JSON array or JSONL file."""
    if not path.exists():
        return []
    try:
        raw = _read_json(path)
    except Exception:
        raw = None
    if isinstance(raw, list):
        return [_normalize_suite_entry(item) for item in raw if isinstance(item, dict)]
    if isinstance(raw, dict):
        return [_normalize_suite_entry(raw)]

    entries: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            item = json.loads(line)
        except Exception:
            continue
        if isinstance(item, dict):
            entries.append(_normalize_suite_entry(item))
    return entries


def _iter_consecutive_passes(entries: Iterable[dict[str, Any]]) -> int:
    count = 0
    for item in reversed(list(entries)):
        if not _coerce_bool(item.get("passed", False)):
            break
        count += 1
    return count


def _scenario_index(entry: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for item in entry.get("scenarios", []):
        name = str(item.get("name", "")).strip()
        if name:
            out[name] = item
    return out


def _validate_full_scenarios(entry: dict[str, Any]) -> tuple[bool, str]:
    idx = _scenario_index(entry)
    available = set(idx.keys())
    missing = sorted(EXPECTED_FULL_SCENARIOS - available)
    if missing:
        return False, f"missing scenarios: {', '.join(missing)}"
    failed = [
        name
        for name in sorted(EXPECTED_FULL_SCENARIOS)
        if not _coerce_bool(idx[name].get("passed"))
    ]
    if failed:
        return False, f"failed scenarios: {', '.join(failed)}"
    return True, "all required full-scenarios passed"


def _validate_timing_contract(entry: dict[str, Any]) -> tuple[bool, str]:
    timing_failures: list[str] = []
    for scenario in entry.get("scenarios", []):
        scenario_name = str(scenario.get("name", "")).strip() or "<unnamed>"
        contracts = scenario.get("contracts", {})
        if not isinstance(contracts, dict):
            continue
        for metric_name in TIMING_CONTRACT_KEYS:
            metric = contracts.get(metric_name)
            if not isinstance(metric, dict):
                continue
            if not _coerce_bool(metric.get("pass", False)):
                timing_failures.append(scenario_name + ":" + metric_name)
    if timing_failures:
        return False, "timing breaches: " + ", ".join(timing_failures)
    return True, "timing contracts passed"


def evaluate_cutover_readiness(
    *,
    suite_history: Sequence[dict[str, Any]],
    min_fast_consecutive: int = 10,
    min_full_passes: int = 1,
    require_schema_migration_check: bool = True,
    schema_migration_ok: bool | None = None,
) -> CutoverReadinessReport:
    """Evaluate V6 default-cutover acceptance gates."""
    normalized = [
        _normalize_suite_entry(item) for item in suite_history if isinstance(item, dict)
    ]
    sorted_entries = sorted(normalized, key=_sort_key)

    fast_runs = [
        item for item in sorted_entries if not _coerce_bool(item.get("full", False))
    ]
    full_runs = [
        item for item in sorted_entries if _coerce_bool(item.get("full", False))
    ]

    fast_consecutive = _iter_consecutive_passes(fast_runs)
    fast_ok = fast_consecutive >= int(max(1, min_fast_consecutive))
    check_fast = CutoverCheckResult(
        name="fast_consecutive_passes",
        passed=fast_ok,
        details=f"{fast_consecutive}/{min_fast_consecutive} consecutive fast runs passed",
    )

    full_ok = False
    full_details = "no full runs found"
    if len(full_runs) >= int(max(1, min_full_passes)):
        recent_full = full_runs[-int(max(1, min_full_passes)) :]
        recent_pass = all(_coerce_bool(item.get("passed")) for item in recent_full)
        if recent_pass:
            scenario_checks = [_validate_full_scenarios(item) for item in recent_full]
            if all(item[0] for item in scenario_checks):
                full_ok = True
                full_details = f"{len(recent_full)}/{min_full_passes} recent full runs passed with required scenarios"
            else:
                first_fail = next(msg for ok, msg in scenario_checks if not ok)
                full_details = first_fail
        else:
            full_details = "one or more recent full runs failed"
    check_full = CutoverCheckResult(
        name="full_suite_regression_gate",
        passed=full_ok,
        details=full_details,
    )

    latest_full = full_runs[-1] if full_runs else None
    long_ok = False
    long_details = "latest full run unavailable"
    if latest_full is not None:
        idx = _scenario_index(latest_full)
        scenario = idx.get("long_completion_tier")
        if scenario is None:
            long_details = "long_completion_tier missing in latest full run"
        else:
            long_ok = _coerce_bool(scenario.get("passed", False))
            long_details = (
                "long_completion_tier passed"
                if long_ok
                else "long_completion_tier failed in latest full run"
            )
    check_long = CutoverCheckResult(
        name="long_completion_tier_gate",
        passed=long_ok,
        details=long_details,
    )

    timing_ok = False
    timing_details = "latest full run unavailable"
    if latest_full is not None:
        timing_ok, timing_details = _validate_timing_contract(latest_full)
    check_timing = CutoverCheckResult(
        name="timing_sla_gate",
        passed=timing_ok,
        details=timing_details,
    )

    if require_schema_migration_check:
        migration_ok = bool(schema_migration_ok is True)
        if schema_migration_ok is None:
            migration_details = "schema migration result not supplied"
        else:
            migration_details = (
                "schema migration gate passed"
                if migration_ok
                else "schema migration gate failed"
            )
    else:
        migration_ok = True
        migration_details = "schema migration check skipped"
    check_migration = CutoverCheckResult(
        name="schema_migration_gate",
        passed=migration_ok,
        details=migration_details,
    )

    checks = [check_fast, check_full, check_long, check_timing, check_migration]
    ready = all(item.passed for item in checks)

    summary = {
        "total_runs": len(sorted_entries),
        "fast_runs": len(fast_runs),
        "full_runs": len(full_runs),
        "latest_generated_at": _sort_key(sorted_entries[-1])
        if sorted_entries
        else None,
        "min_fast_consecutive": int(max(1, min_fast_consecutive)),
        "min_full_passes": int(max(1, min_full_passes)),
    }

    return CutoverReadinessReport(
        schema_version="v6_cutover_readiness_v1",
        generated_at=_now_iso(),
        ready=ready,
        checks=checks,
        summary=summary,
    )

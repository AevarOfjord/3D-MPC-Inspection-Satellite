#!/usr/bin/env python3
"""Evaluate V6 default-cutover readiness from quality-suite history."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from satellite_control.benchmarks.cutover_readiness import (
    evaluate_cutover_readiness,
    load_suite_history,
    load_suite_summaries,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check V6 cutover readiness gates from quality-suite outputs"
    )
    parser.add_argument(
        "--suite-summary",
        action="append",
        default=[],
        help="Path to suite output JSON from scripts/run_mpc_quality_suite.py --output",
    )
    parser.add_argument(
        "--history",
        default=None,
        help="Optional JSON/JSONL history file with prior suite results",
    )
    parser.add_argument(
        "--min-fast-consecutive",
        type=int,
        default=10,
        help="Required consecutive passing fast-suite runs (default: 10)",
    )
    parser.add_argument(
        "--min-full-passes",
        type=int,
        default=1,
        help="Required passing full-suite runs without regression (default: 1)",
    )
    parser.add_argument(
        "--schema-migration-ok",
        action="store_true",
        help="Mark schema-migration gate as passed for this check run",
    )
    parser.add_argument(
        "--skip-schema-migration-check",
        action="store_true",
        help="Skip schema-migration gate for this check run",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional path to write readiness report JSON",
    )
    parser.add_argument(
        "--fail-on-not-ready",
        action="store_true",
        help="Return non-zero if any gate is not ready",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    history_entries = []
    if args.history:
        history_entries.extend(load_suite_history(Path(args.history)))
    if args.suite_summary:
        suite_paths = [Path(item) for item in args.suite_summary]
        history_entries.extend(load_suite_summaries(suite_paths))

    report = evaluate_cutover_readiness(
        suite_history=history_entries,
        min_fast_consecutive=args.min_fast_consecutive,
        min_full_passes=args.min_full_passes,
        require_schema_migration_check=not args.skip_schema_migration_check,
        schema_migration_ok=(True if args.schema_migration_ok else None),
    )

    print("V6 Cutover Readiness:", "READY" if report.ready else "NOT_READY")
    for check in report.checks:
        status = "PASS" if check.passed else "FAIL"
        print(f"[{status}] {check.name}: {check.details}")

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
        print(f"Wrote readiness report: {out_path}")

    return 0 if report.ready or not args.fail_on_not_ready else 1


if __name__ == "__main__":
    raise SystemExit(main())

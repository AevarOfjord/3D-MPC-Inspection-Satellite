#!/usr/bin/env python3
"""Run V5 MPC quality contract scenarios."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from satellite_control.benchmarks.mpc_quality import run_mpc_quality_suite


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run MPC quality contract suite")
    parser.add_argument(
        "--full",
        action="store_true",
        help="Run full suite (planner missions + stress scenarios)",
    )
    parser.add_argument(
        "--fail-on-breach",
        action="store_true",
        help="Return non-zero when any contract breaches",
    )
    parser.add_argument(
        "--python-executable",
        default=None,
        help="Python interpreter used to launch simulation scenarios (default: current)",
    )
    parser.add_argument(
        "--keep-temp-files",
        action="store_true",
        help="Keep temporary generated scenario files (debug only)",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional path for suite summary JSON output",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        suite = run_mpc_quality_suite(
            full=args.full,
            fail_on_breach=args.fail_on_breach,
            python_executable=args.python_executable,
            keep_temp_files=args.keep_temp_files,
        )
    except RuntimeError as exc:
        print(str(exc))
        return 1

    for scenario in suite.scenarios:
        status = "PASS" if scenario.passed else "FAIL"
        print(f"[{status}] {scenario.name} run_dir={scenario.run_dir or 'n/a'}")
        if scenario.breaches:
            for breach in scenario.breaches:
                print(f"  - {breach}")

    print("\nSuite result:", "PASS" if suite.passed else "FAIL")

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(suite.to_dict(), indent=2), encoding="utf-8")
        print(f"Wrote suite summary: {output_path}")

    return 0 if suite.passed or not args.fail_on_breach else 1


if __name__ == "__main__":
    raise SystemExit(main())

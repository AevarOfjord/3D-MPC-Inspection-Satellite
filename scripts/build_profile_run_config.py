#!/usr/bin/env python3
"""Build a runtime config that enables persisted per-profile sweep winners."""

from __future__ import annotations

import argparse
from pathlib import Path

from controller.shared.python.control_common.profile_run_config import (
    write_profile_sim_config,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Write a runtime config that activates per-profile parameter files."
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        required=True,
        help="Path to the baseline config JSON file.",
    )
    parser.add_argument(
        "--profile",
        required=True,
        help="Controller profile to activate.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Path to write the generated config JSON.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    write_profile_sim_config(
        base_config_path=args.base_config,
        profile=args.profile,
        output_path=args.output,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

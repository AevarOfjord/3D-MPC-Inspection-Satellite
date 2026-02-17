#!/usr/bin/env python3
"""Migrate legacy unified mission JSON payloads to schema_version=2."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from satellite_control.dashboard.mission_v2_service import ensure_v2_payload


def _discover_json_files(inputs: list[str], recursive: bool) -> list[Path]:
    discovered: list[Path] = []
    for raw in inputs:
        path = Path(raw).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Input does not exist: {path}")
        if path.is_file():
            if path.suffix.lower() == ".json":
                discovered.append(path)
            continue
        pattern = "**/*.json" if recursive else "*.json"
        discovered.extend(sorted(path.glob(pattern)))
    unique: dict[Path, None] = {}
    for item in discovered:
        unique[item.resolve()] = None
    return sorted(unique.keys())


def _read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object at root: {path}")
    return payload


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _migrate_file(
    source: Path,
    *,
    output_dir: Path | None,
    in_place: bool,
    overwrite: bool,
    dry_run: bool,
) -> dict[str, Any]:
    legacy_payload = _read_json(source)
    migrated = ensure_v2_payload(legacy_payload, name_hint=source.stem)
    target = source if in_place else (output_dir / source.name)

    if not in_place and target.resolve() == source.resolve():
        raise ValueError(f"Refusing to overwrite source path without --in-place: {source}")
    if target.exists() and target.resolve() != source.resolve() and not overwrite:
        raise FileExistsError(f"Output already exists: {target}")
    if dry_run:
        return {
            "source": str(source),
            "target": str(target),
            "mission_id": migrated.mission_id,
            "schema_version": migrated.schema_version,
            "status": "dry_run",
        }

    _write_json(target, migrated.model_dump(mode="json"))
    return {
        "source": str(source),
        "target": str(target),
        "mission_id": migrated.mission_id,
        "schema_version": migrated.schema_version,
        "status": "migrated",
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Migrate legacy mission JSON files to UnifiedMissionV2 schema."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        help="JSON files or directories containing legacy mission files.",
    )
    parser.add_argument(
        "--output-dir",
        default="missions_v2_migrated",
        help="Output directory for migrated files (ignored with --in-place).",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Search input directories recursively for *.json files.",
    )
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Overwrite source files with migrated V2 payloads.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Allow overwriting files in --output-dir.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and report migrations without writing files.",
    )
    parser.add_argument(
        "--report-json",
        default=None,
        help="Optional path to write migration report JSON.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    output_dir = None if args.in_place else Path(args.output_dir).expanduser()
    report: dict[str, Any] = {
        "total_files": 0,
        "migrated": 0,
        "failed": 0,
        "results": [],
    }

    try:
        files = _discover_json_files(args.inputs, recursive=bool(args.recursive))
    except Exception as exc:
        print(f"Error discovering inputs: {exc}", file=sys.stderr)
        return 2

    report["total_files"] = len(files)
    if not files:
        print("No JSON files found to migrate.")
        return 0

    for source in files:
        try:
            result = _migrate_file(
                source,
                output_dir=output_dir,
                in_place=bool(args.in_place),
                overwrite=bool(args.overwrite),
                dry_run=bool(args.dry_run),
            )
            report["migrated"] += 1
            report["results"].append(result)
            print(f"[OK] {result['source']} -> {result['target']}")
        except Exception as exc:
            report["failed"] += 1
            report["results"].append(
                {
                    "source": str(source),
                    "status": "failed",
                    "error": str(exc),
                }
            )
            print(f"[FAIL] {source}: {exc}", file=sys.stderr)

    if args.report_json:
        report_path = Path(args.report_json).expanduser()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"Report written to {report_path}")

    print(
        "Migration summary: "
        f"total={report['total_files']} "
        f"migrated={report['migrated']} "
        f"failed={report['failed']}"
    )
    return 1 if report["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())

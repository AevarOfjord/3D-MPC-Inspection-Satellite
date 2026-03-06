#!/usr/bin/env python3
"""Validate thesis fairness compatibility across six controller runs."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from collections import defaultdict
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

from controller.configs.paths import SIMULATION_DATA_ROOT
from controller.registry import SUPPORTED_CONTROLLER_PROFILES
from controller.shared.python.simulation.artifact_paths import (
    artifact_path,
    resolve_existing_artifact_path,
)

CANONICAL_PROFILES: tuple[str, ...] = tuple(SUPPORTED_CONTROLLER_PROFILES)
REPORT_SCHEMA_VERSION = "fairness_report_v1"
MISSION_TOLERANCE_M = 1e-6
EXCLUDED_RUN_DIRS = {"comparisons", "fairness_reports", "fairness_batches"}


@dataclass
class RunEvidence:
    run_id: str
    run_dir: str
    controller_profile: str | None
    shared_params_hash: str | None
    effective_params_hash: str | None
    override_diff: dict[str, Any] | None
    override_diff_empty: bool
    shared_parameters: bool | None
    profile_parameter_file: str | None
    profile_parameter_file_applied: bool
    mission_name: str | None
    mission_path: str | None
    mission_tokens: list[str]
    path_length_m: float | None
    path_waypoint_count: int | None
    git_commit: str | None
    platform: str | None
    python_version: str | None
    profile_overrides_empty: bool
    deterministic_random_disturbances_disabled: bool
    deterministic_noise_disabled: bool
    artifact_paths: dict[str, str]
    errors: list[str]


def _now_iso() -> str:
    return datetime.now(UTC).isoformat()


def _now_stamp() -> str:
    return datetime.now(UTC).strftime("%Y%m%d_%H%M%S")


def _slug(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    cleaned = cleaned.strip("_")
    return cleaned or "fairness"


def _coalesce_text(*values: Any) -> str | None:
    for value in values:
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return None


def _to_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _artifact_for_run(run_dir: Path, name: str) -> tuple[Path, dict[str, Any]]:
    path = resolve_existing_artifact_path(run_dir, name) or artifact_path(run_dir, name)
    return path, _read_json(path)


def _profile_from_run_dir_name(run_dir: Path) -> str | None:
    parts = run_dir.name.split("__")
    if len(parts) < 3:
        return None
    token = parts[1].strip()
    if token in CANONICAL_PROFILES:
        return token
    candidate = f"cpp_{token}"
    if candidate in CANONICAL_PROFILES:
        return candidate
    return None


def _mission_suffix_token(run_dir: Path) -> str | None:
    parts = run_dir.name.split("__")
    if len(parts) < 3:
        return None
    suffix = parts[-1].strip()
    return suffix or None


def _dedupe_preserve_order(values: list[str]) -> list[str]:
    out: list[str] = []
    seen: set[str] = set()
    for value in values:
        key = value.lower()
        if key in seen:
            continue
        seen.add(key)
        out.append(value)
    return out


def _mission_tokens(
    *,
    run_dir: Path,
    mission_name: str | None,
    mission_path: str | None,
    mission_metadata: dict[str, Any],
) -> list[str]:
    tokens: list[str] = []
    if mission_name:
        tokens.append(mission_name)
    if mission_path:
        tokens.append(mission_path)
        tokens.append(Path(mission_path).stem)
    for key in ("mission_name", "name", "mission_type"):
        value = mission_metadata.get(key)
        if isinstance(value, str) and value.strip():
            tokens.append(value.strip())
    suffix = _mission_suffix_token(run_dir)
    if suffix:
        tokens.append(suffix)
    return _dedupe_preserve_order(tokens)


def _extract_profile_override_empty_state(
    reproducibility_manifest: dict[str, Any],
) -> tuple[bool, list[str], bool, bool, bool | None, str | None]:
    issues: list[str] = []
    configuration = reproducibility_manifest.get("configuration")
    if not isinstance(configuration, dict):
        return (
            False,
            ["missing configuration in reproducibility_manifest"],
            False,
            False,
            None,
            None,
        )
    app_config = configuration.get("app_config")
    if not isinstance(app_config, dict):
        return (
            False,
            ["missing app_config in reproducibility_manifest"],
            False,
            False,
            None,
            None,
        )

    shared_parameters_raw = configuration.get("shared_parameters")
    shared_parameters = (
        bool(shared_parameters_raw)
        if isinstance(shared_parameters_raw, bool | int)
        else None
    )
    profile_parameter_file = _coalesce_text(configuration.get("profile_parameter_file"))

    overrides = app_config.get("mpc_profile_overrides")
    if not isinstance(overrides, dict):
        return (
            False,
            ["missing mpc_profile_overrides in reproducibility_manifest"],
            False,
            False,
            shared_parameters,
            profile_parameter_file,
        )

    for profile in CANONICAL_PROFILES:
        entry = overrides.get(profile)
        if not isinstance(entry, dict):
            issues.append(f"mpc_profile_overrides.{profile} missing")
            continue
        base_overrides = entry.get("base_overrides")
        profile_specific = entry.get("profile_specific")
        if not isinstance(base_overrides, dict) or base_overrides:
            issues.append(f"mpc_profile_overrides.{profile}.base_overrides not empty")
        if not isinstance(profile_specific, dict) or profile_specific:
            issues.append(f"mpc_profile_overrides.{profile}.profile_specific not empty")

    physics = app_config.get("physics")
    deterministic_random_disturbances_disabled = False
    deterministic_noise_disabled = False
    if isinstance(physics, dict):
        random_disturbances_enabled = bool(
            physics.get("random_disturbances_enabled", True)
        )
        noise_terms = (
            _to_float(physics.get("position_noise_std")) or 0.0,
            _to_float(physics.get("velocity_noise_std")) or 0.0,
            _to_float(physics.get("angle_noise_std")) or 0.0,
            _to_float(physics.get("angular_velocity_noise_std")) or 0.0,
            _to_float(physics.get("thrust_force_noise_percent")) or 0.0,
            _to_float(physics.get("disturbance_force_std")) or 0.0,
            _to_float(physics.get("disturbance_torque_std")) or 0.0,
        )
        deterministic_random_disturbances_disabled = not random_disturbances_enabled
        deterministic_noise_disabled = all(abs(item) <= 1e-12 for item in noise_terms)

    return (
        not issues,
        issues,
        deterministic_random_disturbances_disabled,
        deterministic_noise_disabled,
        shared_parameters,
        profile_parameter_file,
    )


def _extract_run_evidence(run_dir: Path) -> RunEvidence:
    errors: list[str] = []

    run_status_path, run_status = _artifact_for_run(run_dir, "run_status.json")
    kpi_summary_path, kpi_summary = _artifact_for_run(run_dir, "kpi_summary.json")
    repro_path, reproducibility_manifest = _artifact_for_run(
        run_dir, "reproducibility_manifest.json"
    )
    mission_metadata_path, mission_metadata = _artifact_for_run(
        run_dir, "mission_metadata.json"
    )

    if not run_status:
        errors.append(f"missing_or_invalid_run_status:{run_status_path}")
    if not kpi_summary:
        errors.append(f"missing_or_invalid_kpi_summary:{kpi_summary_path}")
    if not reproducibility_manifest:
        errors.append(f"missing_or_invalid_reproducibility_manifest:{repro_path}")

    config = run_status.get("config")
    if not isinstance(config, dict):
        config = {}
        errors.append("run_status.config missing")

    controller_profile = _coalesce_text(
        config.get("controller_profile"),
        run_status.get("controller_profile"),
        _profile_from_run_dir_name(run_dir),
    )
    if controller_profile not in CANONICAL_PROFILES:
        errors.append(f"unsupported_or_missing_controller_profile:{controller_profile}")

    mission = run_status.get("mission")
    if not isinstance(mission, dict):
        mission = {}
    mission_name = _coalesce_text(
        mission.get("name"),
        mission_metadata.get("mission_name"),
        mission_metadata.get("name"),
    )
    mission_path = _coalesce_text(mission.get("path"))

    mission_tokens = _mission_tokens(
        run_dir=run_dir,
        mission_name=mission_name,
        mission_path=mission_path,
        mission_metadata=mission_metadata,
    )

    override_diff = config.get("override_diff")
    override_diff_empty = isinstance(override_diff, dict) and not override_diff

    shared_params_hash = _coalesce_text(config.get("shared_params_hash"))
    if not shared_params_hash:
        errors.append("shared_params_hash missing")
    effective_params_hash = _coalesce_text(config.get("effective_params_hash"))
    if not effective_params_hash:
        errors.append("effective_params_hash missing")

    shared_parameters_raw = config.get("shared_parameters")
    shared_parameters = (
        bool(shared_parameters_raw)
        if isinstance(shared_parameters_raw, bool | int)
        else None
    )
    if shared_parameters is None:
        errors.append("run_status.config.shared_parameters missing")
    profile_parameter_file = _coalesce_text(config.get("profile_parameter_file"))
    profile_parameter_file_applied = bool(config.get("profile_parameter_file_applied"))

    path_length_m = _to_float(kpi_summary.get("path_length_m"))
    if path_length_m is None:
        errors.append("kpi.path_length_m missing")
    path_waypoint_count = _to_int(kpi_summary.get("path_waypoint_count"))
    if path_waypoint_count is None:
        errors.append("kpi.path_waypoint_count missing")

    software = reproducibility_manifest.get("software")
    if not isinstance(software, dict):
        software = {}
        errors.append("reproducibility_manifest.software missing")

    git_info = software.get("git")
    if not isinstance(git_info, dict):
        git_info = {}
        errors.append("reproducibility_manifest.software.git missing")

    git_commit = _coalesce_text(git_info.get("commit"))
    platform = _coalesce_text(software.get("platform"))
    python_version = _coalesce_text(software.get("python_version"))

    (
        profile_overrides_empty,
        profile_override_issues,
        deterministic_random_disturbances_disabled,
        deterministic_noise_disabled,
        manifest_shared_parameters,
        manifest_profile_parameter_file,
    ) = _extract_profile_override_empty_state(reproducibility_manifest)
    errors.extend(profile_override_issues)
    if manifest_shared_parameters is None:
        errors.append(
            "reproducibility_manifest.configuration.shared_parameters missing"
        )
    elif (
        shared_parameters is not None
        and manifest_shared_parameters != shared_parameters
    ):
        errors.append("shared_parameters mismatch between run_status and manifest")
    if manifest_profile_parameter_file != profile_parameter_file:
        errors.append("profile_parameter_file mismatch between run_status and manifest")

    artifact_paths = {
        "run_status.json": str(run_status_path),
        "kpi_summary.json": str(kpi_summary_path),
        "reproducibility_manifest.json": str(repro_path),
        "mission_metadata.json": str(mission_metadata_path),
    }

    return RunEvidence(
        run_id=run_dir.name,
        run_dir=str(run_dir.resolve()),
        controller_profile=controller_profile,
        shared_params_hash=shared_params_hash,
        effective_params_hash=effective_params_hash,
        override_diff=override_diff if isinstance(override_diff, dict) else None,
        override_diff_empty=override_diff_empty,
        shared_parameters=shared_parameters,
        profile_parameter_file=profile_parameter_file,
        profile_parameter_file_applied=profile_parameter_file_applied,
        mission_name=mission_name,
        mission_path=mission_path,
        mission_tokens=mission_tokens,
        path_length_m=path_length_m,
        path_waypoint_count=path_waypoint_count,
        git_commit=git_commit,
        platform=platform,
        python_version=python_version,
        profile_overrides_empty=profile_overrides_empty,
        deterministic_random_disturbances_disabled=deterministic_random_disturbances_disabled,
        deterministic_noise_disabled=deterministic_noise_disabled,
        artifact_paths=artifact_paths,
        errors=errors,
    )


def _resolve_run_arg(token: str, runs_root: Path) -> Path:
    raw = Path(token).expanduser()
    if raw.exists() and raw.is_dir():
        return raw.resolve()
    candidate = (runs_root / token).resolve()
    if candidate.exists() and candidate.is_dir():
        return candidate
    raise FileNotFoundError(f"Run not found: {token}")


def _discover_run_dirs(runs_root: Path) -> list[Path]:
    if not runs_root.exists():
        return []
    candidates = [
        path
        for path in runs_root.iterdir()
        if path.is_dir() and path.name.lower() not in EXCLUDED_RUN_DIRS
    ]
    return sorted(candidates, key=lambda item: item.stat().st_mtime, reverse=True)


def _run_matches_mission(run_dir: Path, mission_query: str) -> tuple[bool, str | None]:
    query = mission_query.strip().lower()
    run_status_path, run_status = _artifact_for_run(run_dir, "run_status.json")
    if not run_status:
        _ = run_status_path
        return False, None
    mission_metadata_path, mission_metadata = _artifact_for_run(
        run_dir, "mission_metadata.json"
    )
    _ = mission_metadata_path

    mission = run_status.get("mission")
    if not isinstance(mission, dict):
        mission = {}
    tokens = _mission_tokens(
        run_dir=run_dir,
        mission_name=_coalesce_text(mission.get("name")),
        mission_path=_coalesce_text(mission.get("path")),
        mission_metadata=mission_metadata,
    )
    if any(token.lower() == query for token in tokens):
        return True, _coalesce_text(
            run_status.get("config", {}).get("controller_profile")
            if isinstance(run_status.get("config"), dict)
            else None,
            _profile_from_run_dir_name(run_dir),
        )
    return False, None


def _select_latest_runs_for_mission(mission_query: str, runs_root: Path) -> list[Path]:
    selected: dict[str, Path] = {}
    for run_dir in _discover_run_dirs(runs_root):
        matched, profile = _run_matches_mission(run_dir, mission_query)
        if not matched:
            continue
        if profile not in CANONICAL_PROFILES:
            continue
        if profile in selected:
            continue
        selected[profile] = run_dir
        if len(selected) == len(CANONICAL_PROFILES):
            break
    return [selected[profile] for profile in CANONICAL_PROFILES if profile in selected]


def _distinct_non_none(values: list[str | None]) -> list[str]:
    out: list[str] = []
    for value in values:
        if value is None:
            continue
        if value not in out:
            out.append(value)
    return out


def _validate_contract(evidence: list[RunEvidence]) -> list[str]:
    failures: list[str] = []

    if len(evidence) != len(CANONICAL_PROFILES):
        failures.append(f"expected {len(CANONICAL_PROFILES)} runs, got {len(evidence)}")

    for item in evidence:
        if item.errors:
            failures.append(f"{item.run_id}: " + "; ".join(item.errors))

    profile_to_runs: dict[str, list[str]] = defaultdict(list)
    for item in evidence:
        profile = item.controller_profile or "<missing_profile>"
        profile_to_runs[profile].append(item.run_id)

    missing_profiles = [
        profile for profile in CANONICAL_PROFILES if profile not in profile_to_runs
    ]
    if missing_profiles:
        failures.append("missing profiles: " + ", ".join(missing_profiles))

    duplicate_profiles = [
        profile
        for profile, run_ids in profile_to_runs.items()
        if profile in CANONICAL_PROFILES and len(run_ids) > 1
    ]
    if duplicate_profiles:
        details = ", ".join(
            f"{profile} -> {profile_to_runs[profile]}" for profile in duplicate_profiles
        )
        failures.append("duplicate profiles: " + details)

    unexpected_profiles = sorted(
        profile for profile in profile_to_runs if profile not in CANONICAL_PROFILES
    )
    if unexpected_profiles:
        failures.append("unexpected profiles: " + ", ".join(unexpected_profiles))

    shared_hashes = {
        item.shared_params_hash for item in evidence if item.shared_params_hash
    }
    if any(not item.shared_params_hash for item in evidence):
        failures.append("one or more runs missing shared_params_hash")
    if len(shared_hashes) > 1:
        failures.append(
            "shared_params_hash mismatch: " + ", ".join(sorted(shared_hashes))
        )

    for item in evidence:
        if item.shared_parameters is not True:
            failures.append(f"{item.run_id}: shared.parameters must be true")
        if not item.override_diff_empty:
            failures.append(
                f"{item.run_id}: override_diff not empty ({item.override_diff})"
            )
        if item.profile_parameter_file_applied or item.profile_parameter_file:
            failures.append(
                f"{item.run_id}: profile_parameter_file must not be applied "
                f"({item.profile_parameter_file})"
            )

    mission_names = _distinct_non_none([item.mission_name for item in evidence])
    mission_paths = _distinct_non_none([item.mission_path for item in evidence])
    if len(mission_names) > 1:
        failures.append("mission name mismatch: " + ", ".join(mission_names))
    if len(mission_paths) > 1:
        failures.append("mission path mismatch: " + ", ".join(mission_paths))
    if not mission_names and not mission_paths:
        failures.append("mission identity missing: mission name/path not found in runs")

    path_lengths = [
        item.path_length_m for item in evidence if item.path_length_m is not None
    ]
    if len(path_lengths) != len(evidence):
        failures.append("one or more runs missing kpi.path_length_m")
    elif max(path_lengths) - min(path_lengths) > MISSION_TOLERANCE_M:
        failures.append("path_length_m mismatch across runs")

    waypoint_counts = [
        item.path_waypoint_count
        for item in evidence
        if item.path_waypoint_count is not None
    ]
    if len(waypoint_counts) != len(evidence):
        failures.append("one or more runs missing kpi.path_waypoint_count")
    elif len(set(waypoint_counts)) > 1:
        failures.append("path_waypoint_count mismatch across runs")

    commits = _distinct_non_none([item.git_commit for item in evidence])
    platforms = _distinct_non_none([item.platform for item in evidence])
    python_versions = _distinct_non_none([item.python_version for item in evidence])
    if len(commits) != 1:
        failures.append("git commit mismatch across runs")
    if len(platforms) != 1:
        failures.append("platform mismatch across runs")
    if len(python_versions) != 1:
        failures.append("python_version mismatch across runs")

    for item in evidence:
        if not item.profile_overrides_empty:
            failures.append(f"{item.run_id}: mpc_profile_overrides not empty")
        if not item.deterministic_random_disturbances_disabled:
            failures.append(f"{item.run_id}: random_disturbances_enabled must be false")
        if not item.deterministic_noise_disabled:
            failures.append(f"{item.run_id}: noise/disturbance std values must be zero")

    return failures


def _default_output_path(
    *,
    runs_root: Path,
    mission_query: str | None,
) -> Path:
    reports_dir = runs_root / "fairness_reports"
    mission_token = _slug(mission_query) if mission_query else "explicit_runs"
    return reports_dir / f"{_now_stamp()}_{mission_token}_fairness_report.json"


def _build_report(
    *,
    mission_query: str | None,
    runs_root: Path,
    selected_runs: list[Path],
    evidence: list[RunEvidence],
    failures: list[str],
) -> dict[str, Any]:
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "generated_at": _now_iso(),
        "pass": not failures,
        "required_profiles": list(CANONICAL_PROFILES),
        "mission_query": mission_query,
        "runs_root": str(runs_root.resolve()),
        "selected_run_ids": [run.name for run in selected_runs],
        "selected_runs": [str(run.resolve()) for run in selected_runs],
        "summary": {
            "run_count": len(evidence),
            "expected_run_count": len(CANONICAL_PROFILES),
        },
        "fail_reasons": failures,
        "runs": [asdict(item) for item in evidence],
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate six-profile thesis fairness compatibility contract.",
    )
    parser.add_argument(
        "--run",
        action="append",
        default=[],
        help="Run directory path or run ID under --runs-root (repeatable).",
    )
    parser.add_argument(
        "--mission",
        default=None,
        help=(
            "Mission token (name/path stem). When provided, auto-selects "
            "latest run for each canonical profile."
        ),
    )
    parser.add_argument(
        "--runs-root",
        type=Path,
        default=SIMULATION_DATA_ROOT,
        help="Root directory containing simulation runs.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output JSON path for fairness report.",
    )
    parser.add_argument(
        "--print-report",
        action="store_true",
        help="Print full JSON report to stdout in addition to writing the file.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(sys.argv[1:] if argv is None else argv))
    runs_root = args.runs_root.resolve()

    if args.run and args.mission:
        print("Use either --run or --mission, not both.")
        return 2
    if not args.run and not args.mission:
        print("Provide --run (repeatable) or --mission.")
        return 2

    selected_runs: list[Path] = []
    if args.run:
        try:
            selected_runs = [_resolve_run_arg(token, runs_root) for token in args.run]
        except FileNotFoundError as exc:
            print(str(exc))
            return 2
    else:
        selected_runs = _select_latest_runs_for_mission(args.mission, runs_root)
        if not selected_runs:
            print(f"No runs found for mission query: {args.mission}")
            return 2

    evidence = [_extract_run_evidence(run_dir) for run_dir in selected_runs]
    failures = _validate_contract(evidence)

    output_path = (
        args.output.resolve()
        if args.output
        else _default_output_path(
            runs_root=runs_root,
            mission_query=args.mission,
        ).resolve()
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    report = _build_report(
        mission_query=args.mission,
        runs_root=runs_root,
        selected_runs=selected_runs,
        evidence=evidence,
        failures=failures,
    )
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    status = "PASS" if report["pass"] else "FAIL"
    print(f"[{status}] Fairness contract report: {output_path}")
    if failures:
        for item in failures:
            print(f"  - {item}")
    if args.print_report:
        print(json.dumps(report, indent=2))

    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

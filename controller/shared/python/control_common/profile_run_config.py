"""Helpers for runtime profile-config selection and sweep winner persistence."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

from controller.configs.paths import resolve_repo_path
from controller.registry import (
    normalize_controller_profile,
    rewrite_legacy_controller_profile,
    rewrite_profile_identifiers_in_payload,
)
from controller.shared.python.control_common.parameter_policy import (
    default_profile_parameter_files,
)


def normalize_profile_id(profile: str) -> str:
    """Return canonical controller profile id."""
    rewritten = rewrite_legacy_controller_profile(profile)
    candidate = rewritten if isinstance(rewritten, str) else profile
    return normalize_controller_profile(candidate)


def load_json_object(path: Path) -> dict[str, Any]:
    """Load a JSON object from disk."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object in {path}")
    rewrite_profile_identifiers_in_payload(payload)
    return payload


def build_profile_sim_overrides(
    base_overrides: dict[str, Any],
    profile: str,
) -> dict[str, Any]:
    """
    Enable per-profile parameter-file mode for a normal simulation run.

    The returned payload preserves the baseline config while forcing
    shared.parameters=false and selecting the active controller profile.
    """
    normalized_profile = normalize_profile_id(profile)
    payload = deepcopy(base_overrides)
    rewrite_profile_identifiers_in_payload(payload)

    shared = payload.get("shared")
    if not isinstance(shared, dict):
        shared = {}
        payload["shared"] = shared

    merged_profile_files = default_profile_parameter_files()
    existing_profile_files = shared.get("profile_parameter_files")
    if isinstance(existing_profile_files, dict):
        for key, value in existing_profile_files.items():
            if isinstance(key, str) and isinstance(value, str) and value.strip():
                merged_profile_files[normalize_profile_id(key)] = value.strip()

    shared["parameters"] = False
    shared["profile_parameter_files"] = merged_profile_files

    mpc_core = payload.get("mpc_core")
    if not isinstance(mpc_core, dict):
        mpc_core = {}
        payload["mpc_core"] = mpc_core
    mpc_core["controller_profile"] = normalized_profile

    return payload


def write_profile_sim_config(
    *,
    base_config_path: Path,
    profile: str,
    output_path: Path,
) -> Path:
    """Build and write a runtime config file that enables saved profile winners."""
    base_overrides = load_json_object(base_config_path)
    payload = build_profile_sim_overrides(base_overrides, profile)
    output_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return output_path


def persist_profile_sweep_winner(
    *,
    profile: str,
    prediction_horizon: int,
    control_horizon: int,
    dt: float,
    solver_time_limit: float,
    profile_file_path: Path | None = None,
    simulation_control_dt: float | None = None,
) -> Path:
    """Persist a sweep winner into a controller's profile parameter file."""
    normalized_profile = normalize_profile_id(profile)
    profile_files = default_profile_parameter_files()
    resolved_path = (
        resolve_repo_path(profile_files[normalized_profile])
        if profile_file_path is None
        else resolve_repo_path(profile_file_path)
    )

    if resolved_path.exists():
        payload = load_json_object(resolved_path)
    else:
        payload = {}

    mpc_core = payload.get("mpc_core")
    if not isinstance(mpc_core, dict):
        mpc_core = {}
        payload["mpc_core"] = mpc_core
    mpc_core["controller_profile"] = normalized_profile

    mpc = payload.get("mpc")
    if not isinstance(mpc, dict):
        mpc = {}
        payload["mpc"] = mpc
    mpc["prediction_horizon"] = int(prediction_horizon)
    mpc["control_horizon"] = int(control_horizon)
    mpc["dt"] = float(dt)
    mpc["solver_time_limit"] = float(solver_time_limit)

    simulation = payload.get("simulation")
    if not isinstance(simulation, dict):
        simulation = {}
        payload["simulation"] = simulation
    simulation["control_dt"] = float(
        dt if simulation_control_dt is None else simulation_control_dt
    )

    overrides_root = payload.get("mpc_profile_overrides")
    if not isinstance(overrides_root, dict):
        overrides_root = {}
        payload["mpc_profile_overrides"] = overrides_root

    profile_section = overrides_root.get(normalized_profile)
    if not isinstance(profile_section, dict):
        profile_section = {}
        overrides_root[normalized_profile] = profile_section

    base_overrides = profile_section.get("base_overrides")
    if not isinstance(base_overrides, dict):
        base_overrides = {}
        profile_section["base_overrides"] = base_overrides

    profile_specific = profile_section.get("profile_specific")
    if not isinstance(profile_specific, dict):
        profile_specific = {}
        profile_section["profile_specific"] = profile_specific

    base_overrides["prediction_horizon"] = int(prediction_horizon)
    base_overrides["control_horizon"] = int(control_horizon)
    base_overrides["dt"] = float(dt)
    base_overrides["solver_time_limit"] = float(solver_time_limit)

    resolved_path.parent.mkdir(parents=True, exist_ok=True)
    resolved_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return resolved_path

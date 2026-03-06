"""Shared-vs-profile parameter policy helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from controller.configs.paths import resolve_repo_path
from controller.registry import (
    normalize_controller_profile,
    rewrite_legacy_controller_profile,
    rewrite_profile_identifiers_in_payload,
)

DEFAULT_PROFILE_PARAMETER_FILES: dict[str, str] = {
    "cpp_linearized_rti_osqp": "controller/linear/profile_parameters.json",
    "cpp_hybrid_rti_osqp": "controller/hybrid/profile_parameters.json",
    "cpp_nonlinear_rti_osqp": "controller/nonlinear/profile_parameters.json",
    "cpp_nonlinear_fullnlp_ipopt": "controller/nmpc/profile_parameters.json",
    "cpp_nonlinear_rti_hpipm": "controller/acados_rti/profile_parameters.json",
    "cpp_nonlinear_sqp_hpipm": "controller/acados_sqp/profile_parameters.json",
}


def default_profile_parameter_files() -> dict[str, str]:
    """Return the canonical profile-parameter file map."""
    return dict(DEFAULT_PROFILE_PARAMETER_FILES)


def normalize_shared_parameters_payload(payload: dict[str, Any]) -> None:
    """Normalize dotted shared.parameters aliases to canonical nested payload."""
    if not isinstance(payload, dict):
        return
    if "shared.parameters" in payload:
        alias_value = payload.pop("shared.parameters")
        shared_section = payload.get("shared")
        if not isinstance(shared_section, dict):
            shared_section = {}
            payload["shared"] = shared_section
        if "parameters" not in shared_section:
            shared_section["parameters"] = alias_value


def deep_merge_dict(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge overlay into base."""
    merged = dict(base)
    for key, value in overlay.items():
        existing = merged.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merged[key] = deep_merge_dict(existing, value)
        else:
            merged[key] = value
    return merged


def resolve_profile_from_overrides(
    config_overrides: dict[str, Any],
    default_profile: str,
) -> str:
    """Resolve canonical profile from overrides or fallback default."""
    profile_raw = default_profile
    mpc_core = config_overrides.get("mpc_core")
    if isinstance(mpc_core, dict):
        candidate = mpc_core.get("controller_profile")
        if isinstance(candidate, str):
            profile_raw = candidate
    rewritten = rewrite_legacy_controller_profile(profile_raw)
    return normalize_controller_profile(
        rewritten if isinstance(rewritten, str) else profile_raw
    )


def apply_profile_parameter_file_if_needed(
    *,
    config_overrides: dict[str, Any],
    default_profile: str,
) -> tuple[dict[str, Any], str | None, bool, str]:
    """
    Apply active-profile parameter file when shared.parameters is disabled.

    Returns:
        (merged_overrides, applied_profile_file_path, shared_parameters_enabled, profile)
    """
    normalize_shared_parameters_payload(config_overrides)
    rewrite_profile_identifiers_in_payload(config_overrides)

    profile = resolve_profile_from_overrides(config_overrides, default_profile)
    shared = config_overrides.get("shared")
    if not isinstance(shared, dict):
        return config_overrides, None, True, profile

    shared_parameters = bool(shared.get("parameters", True))
    if shared_parameters:
        return config_overrides, None, True, profile

    profile_files = shared.get("profile_parameter_files")
    if not isinstance(profile_files, dict) or not profile_files:
        return config_overrides, None, False, profile

    raw_profile_path = profile_files.get(profile)
    if not isinstance(raw_profile_path, str) or not raw_profile_path.strip():
        return config_overrides, None, False, profile

    profile_path = resolve_repo_path(Path(raw_profile_path.strip()))
    if not profile_path.exists():
        raise ValueError(
            f"Profile parameter file for {profile} not found: {profile_path}"
        )

    try:
        payload = json.loads(profile_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"Invalid JSON in profile parameter file {profile_path}: {exc}"
        ) from exc

    if not isinstance(payload, dict):
        raise ValueError(
            f"Profile parameter file must contain a JSON object: {profile_path}"
        )

    rewrite_profile_identifiers_in_payload(payload)
    normalize_shared_parameters_payload(payload)
    merged = deep_merge_dict(config_overrides, payload)

    merged_mpc_core = merged.get("mpc_core")
    if not isinstance(merged_mpc_core, dict):
        merged_mpc_core = {}
        merged["mpc_core"] = merged_mpc_core
    merged_mpc_core["controller_profile"] = profile
    return merged, str(profile_path), False, profile

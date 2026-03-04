"""Helpers for shared-baseline + per-profile MPC override resolution."""

from __future__ import annotations

import hashlib
import json
import logging
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from controller.configs.models import AppConfig

logger = logging.getLogger(__name__)

_SUPPORTED_PROFILES: tuple[str, ...] = (
    "cpp_hybrid_rti_osqp",
    "cpp_nonlinear_rti_osqp",
    "cpp_linearized_rti_osqp",
    "cpp_nonlinear_fullnlp_ipopt",
    "cpp_nonlinear_rti_hpipm",
    "cpp_nonlinear_sqp_hpipm",
)

_PROFILE_SPECIFIC_DEFAULTS: dict[str, dict[str, Any]] = {
    "cpp_hybrid_rti_osqp": {
        "allow_stale_stage_reuse": True,
    },
    "cpp_nonlinear_rti_osqp": {
        "strict_integrity": True,
    },
    "cpp_linearized_rti_osqp": {
        "freeze_refresh_interval_steps": 1,
    },
    "cpp_nonlinear_fullnlp_ipopt": {
        "ipopt_max_iter": 3000,
    },
    "cpp_nonlinear_rti_hpipm": {
        "acados_max_iter": 1,
        "acados_tol_stat": 1e-2,
        "acados_tol_eq": 1e-2,
        "acados_tol_ineq": 1e-2,
    },
    "cpp_nonlinear_sqp_hpipm": {
        "acados_max_iter": 50,
        "acados_tol_stat": 1e-2,
        "acados_tol_eq": 1e-2,
        "acados_tol_ineq": 1e-2,
    },
}

_PROFILE_SPECIFIC_ALLOWED_KEYS: dict[str, set[str]] = {
    "cpp_hybrid_rti_osqp": set(
        _PROFILE_SPECIFIC_DEFAULTS["cpp_hybrid_rti_osqp"].keys()
    ),
    "cpp_nonlinear_rti_osqp": set(
        _PROFILE_SPECIFIC_DEFAULTS["cpp_nonlinear_rti_osqp"].keys()
    ),
    "cpp_linearized_rti_osqp": set(
        _PROFILE_SPECIFIC_DEFAULTS["cpp_linearized_rti_osqp"].keys()
    ),
    "cpp_nonlinear_fullnlp_ipopt": set(
        _PROFILE_SPECIFIC_DEFAULTS["cpp_nonlinear_fullnlp_ipopt"].keys()
    ),
    "cpp_nonlinear_rti_hpipm": set(
        _PROFILE_SPECIFIC_DEFAULTS["cpp_nonlinear_rti_hpipm"].keys()
    ),
    "cpp_nonlinear_sqp_hpipm": set(
        _PROFILE_SPECIFIC_DEFAULTS["cpp_nonlinear_sqp_hpipm"].keys()
    ),
}


def _stable_payload_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _freeze_payload(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({k: _freeze_payload(v) for k, v in value.items()})
    if isinstance(value, list):
        return tuple(_freeze_payload(v) for v in value)
    return value


@dataclass(frozen=True)
class EffectiveMPCProfileContract:
    """Resolved immutable MPC contract for one controller profile."""

    profile: str
    shared_mpc: MappingProxyType[str, Any]
    effective_mpc: MappingProxyType[str, Any]
    profile_specific: MappingProxyType[str, Any]
    override_diff: MappingProxyType[str, Any]
    shared_signature: str
    effective_signature: str


def _normalize_profile(profile: str | None) -> str:
    if isinstance(profile, str) and profile in _SUPPORTED_PROFILES:
        return profile
    return "cpp_hybrid_rti_osqp"


def _extract_profile_override_payload(
    cfg: AppConfig, profile: str
) -> tuple[dict[str, Any], dict[str, Any]]:
    overrides_root = getattr(cfg, "mpc_profile_overrides", None)
    if overrides_root is None:
        return {}, dict(_PROFILE_SPECIFIC_DEFAULTS[profile])

    section = getattr(overrides_root, profile, None)
    if section is None:
        return {}, dict(_PROFILE_SPECIFIC_DEFAULTS[profile])

    base_overrides = dict(getattr(section, "base_overrides", {}) or {})
    profile_specific = dict(_PROFILE_SPECIFIC_DEFAULTS[profile])
    incoming_specific = dict(getattr(section, "profile_specific", {}) or {})
    allowed_keys = _PROFILE_SPECIFIC_ALLOWED_KEYS.get(profile, set())
    for key, value in incoming_specific.items():
        if key in allowed_keys:
            profile_specific[key] = value
        else:
            logger.warning(
                "Ignoring unsupported profile-specific override key '%s' for profile '%s'.",
                key,
                profile,
            )
    return base_overrides, profile_specific


def resolve_effective_mpc_profile_contract(
    cfg: AppConfig,
    profile: str | None,
) -> EffectiveMPCProfileContract:
    """Resolve immutable shared/effective profile contract from AppConfig."""
    normalized_profile = _normalize_profile(profile)
    shared_mpc = cfg.mpc.model_dump()
    base_overrides, profile_specific = _extract_profile_override_payload(
        cfg=cfg,
        profile=normalized_profile,
    )
    effective_mpc = dict(shared_mpc)
    effective_mpc.update(base_overrides)

    override_diff = {
        key: effective_mpc[key]
        for key in sorted(effective_mpc.keys())
        if key not in shared_mpc or shared_mpc.get(key) != effective_mpc[key]
    }

    shared_signature = _stable_payload_hash({"shared_mpc": shared_mpc})
    effective_signature = _stable_payload_hash(
        {
            "profile": normalized_profile,
            "effective_mpc": effective_mpc,
            "profile_specific": profile_specific,
        }
    )
    return EffectiveMPCProfileContract(
        profile=normalized_profile,
        shared_mpc=_freeze_payload(shared_mpc),
        effective_mpc=_freeze_payload(effective_mpc),
        profile_specific=_freeze_payload(profile_specific),
        override_diff=_freeze_payload(override_diff),
        shared_signature=shared_signature,
        effective_signature=effective_signature,
    )

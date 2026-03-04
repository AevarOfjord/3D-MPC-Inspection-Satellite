"""Controller profile registry and compatibility helpers."""

from __future__ import annotations

from typing import Final

ControllerProfile = str

HYBRID_PROFILE: Final[str] = "cpp_hybrid_rti_osqp"
NONLINEAR_PROFILE: Final[str] = "cpp_nonlinear_rti_osqp"
LINEAR_PROFILE: Final[str] = "cpp_linearized_rti_osqp"
NMPC_PROFILE: Final[str] = "cpp_nonlinear_fullnlp_ipopt"
ACADOS_RTI_PROFILE: Final[str] = "cpp_nonlinear_rti_hpipm"
ACADOS_SQP_PROFILE: Final[str] = "cpp_nonlinear_sqp_hpipm"

SUPPORTED_CONTROLLER_PROFILES: Final[tuple[str, ...]] = (
    LINEAR_PROFILE,
    HYBRID_PROFILE,
    NONLINEAR_PROFILE,
    NMPC_PROFILE,
    ACADOS_RTI_PROFILE,
    ACADOS_SQP_PROFILE,
)

LEGACY_PROFILE_REWRITE_MAP: Final[dict[str, str]] = {
    "hybrid": HYBRID_PROFILE,
    "nonlinear": NONLINEAR_PROFILE,
    "linear": LINEAR_PROFILE,
    "nmpc": NMPC_PROFILE,
    "acados_rti": ACADOS_RTI_PROFILE,
    "acados_sqp": ACADOS_SQP_PROFILE,
}


def rewrite_legacy_controller_profile(profile: str | None) -> str | None:
    """Rewrite legacy profile IDs to canonical IDs."""
    if not isinstance(profile, str):
        return profile
    normalized = profile.strip().lower()
    return LEGACY_PROFILE_REWRITE_MAP.get(normalized, profile)


def rewrite_profile_identifiers_in_payload(payload: dict[str, object]) -> None:
    """
    Rewrite legacy controller profile identifiers in a mutable config payload.

    Applies to:
      - app_config.mpc_core.controller_profile
      - mpc_core.controller_profile
      - app_config.mpc_profile_overrides keys
      - mpc_profile_overrides keys
    """
    if not isinstance(payload, dict):
        return

    def _rewrite_profile_field(maybe_section: object) -> None:
        if not isinstance(maybe_section, dict):
            return
        value = maybe_section.get("controller_profile")
        rewritten = rewrite_legacy_controller_profile(
            value if isinstance(value, str) else None
        )
        if isinstance(rewritten, str):
            maybe_section["controller_profile"] = rewritten

    def _rewrite_overrides_keys(maybe_section: object) -> None:
        if not isinstance(maybe_section, dict):
            return
        for old_key, new_key in LEGACY_PROFILE_REWRITE_MAP.items():
            if old_key in maybe_section and new_key not in maybe_section:
                maybe_section[new_key] = maybe_section.pop(old_key)

    _rewrite_profile_field(payload.get("mpc_core"))
    _rewrite_overrides_keys(payload.get("mpc_profile_overrides"))

    app_config = payload.get("app_config")
    if isinstance(app_config, dict):
        _rewrite_profile_field(app_config.get("mpc_core"))
        _rewrite_overrides_keys(app_config.get("mpc_profile_overrides"))


def normalize_controller_profile(
    profile: str | None,
) -> str:
    """Return a supported canonical profile, or raise for unsupported input."""
    if profile is None:
        return HYBRID_PROFILE
    if isinstance(profile, str):
        normalized = profile.strip()
        if not normalized:
            return HYBRID_PROFILE
        if normalized in SUPPORTED_CONTROLLER_PROFILES:
            return normalized
    supported = ", ".join(SUPPORTED_CONTROLLER_PROFILES)
    raise ValueError(
        f"Unsupported controller profile {profile!r}. Supported canonical profiles: {supported}"
    )

"""Controller profile registry and compatibility helpers."""

from __future__ import annotations

from typing import Final

ControllerProfile = str

HYBRID_PROFILE: Final[str] = "hybrid"
NONLINEAR_PROFILE: Final[str] = "nonlinear"
LINEAR_PROFILE: Final[str] = "linear"

SUPPORTED_CONTROLLER_PROFILES: Final[tuple[str, ...]] = (
    HYBRID_PROFILE,
    NONLINEAR_PROFILE,
    LINEAR_PROFILE,
)


def normalize_controller_profile(
    profile: str | None,
) -> str:
    """Return a supported profile with a safe default."""
    if isinstance(profile, str) and profile in SUPPORTED_CONTROLLER_PROFILES:
        return profile
    return HYBRID_PROFILE

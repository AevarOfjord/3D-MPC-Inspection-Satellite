"""Interactive controller-profile picker used by `make sim`."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SUPPORTED_CONTROLLER_PROFILES: tuple[str, ...] = (
    "cpp_linearized_rti_osqp",
    "cpp_hybrid_rti_osqp",
    "cpp_nonlinear_rti_osqp",
    "cpp_nonlinear_fullnlp_ipopt",
    "cpp_nonlinear_rti_hpipm",
    "cpp_nonlinear_sqp_hpipm",
)

LEGACY_PROFILE_REWRITE_MAP: dict[str, str] = {
    "hybrid": "cpp_hybrid_rti_osqp",
    "nonlinear": "cpp_nonlinear_rti_osqp",
    "linear": "cpp_linearized_rti_osqp",
    "nmpc": "cpp_nonlinear_fullnlp_ipopt",
    "acados_rti": "cpp_nonlinear_rti_hpipm",
    "acados_sqp": "cpp_nonlinear_sqp_hpipm",
}

# Keep compatibility default even though display order is linear -> nonlinear.
DEFAULT_PROFILE = "cpp_hybrid_rti_osqp"

_PROFILE_LABELS: dict[str, str] = {
    "cpp_hybrid_rti_osqp": "Hybrid RTI + OSQP",
    "cpp_nonlinear_rti_osqp": "Nonlinear RTI + OSQP",
    "cpp_linearized_rti_osqp": "Linearized RTI + OSQP",
    "cpp_nonlinear_fullnlp_ipopt": "Nonlinear Full NLP + IPOPT",
    "cpp_nonlinear_rti_hpipm": "Nonlinear RTI + HPIPM",
    "cpp_nonlinear_sqp_hpipm": "Nonlinear SQP + HPIPM",
}


def _normalize_choice(raw: str | None) -> str:
    if raw is None:
        return DEFAULT_PROFILE
    candidate = raw.strip()
    if not candidate:
        return DEFAULT_PROFILE
    rewritten = LEGACY_PROFILE_REWRITE_MAP.get(candidate.lower(), candidate)
    if rewritten in SUPPORTED_CONTROLLER_PROFILES:
        return rewritten
    return DEFAULT_PROFILE


def prompt_controller_profile() -> str:
    """Prompt user for controller profile and return canonical profile id."""
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        return DEFAULT_PROFILE

    try:
        import questionary
    except ImportError:
        questionary = None

    if questionary is not None:
        try:
            select_style = questionary.Style(
                [
                    ("qmark", ""),
                    ("question", "bold"),
                    ("pointer", "fg:#ffffff bg:#005f87 bold"),
                    ("highlighted", "fg:#ffffff bg:#005f87 bold"),
                ]
            )
            choices = [
                questionary.Choice(
                    title=f"{profile} [{_PROFILE_LABELS.get(profile, profile)}]",
                    value=profile,
                )
                for profile in SUPPORTED_CONTROLLER_PROFILES
            ]
            selected = questionary.select(
                "Select controller profile:",
                choices=choices,
                qmark="",
                style=select_style,
            ).ask()
            return _normalize_choice(selected)
        except Exception:
            # Fall through to numeric prompt.
            pass

    print("questionary unavailable, using numbered selection.")
    for idx, profile in enumerate(SUPPORTED_CONTROLLER_PROFILES, start=1):
        label = _PROFILE_LABELS.get(profile, profile)
        print(f"  {idx}. {profile} [{label}]")
    raw = input("Select controller profile number (blank for default 1): ").strip()
    if not raw:
        return DEFAULT_PROFILE
    if raw.isdigit():
        selected_idx = int(raw)
        if 1 <= selected_idx <= len(SUPPORTED_CONTROLLER_PROFILES):
            return SUPPORTED_CONTROLLER_PROFILES[selected_idx - 1]
    return _normalize_choice(raw)


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="python -m controller.shared.python.simulation.profile_prompt",
        description="Interactive controller-profile picker",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Path to write the selected canonical profile ID.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(list(sys.argv[1:] if argv is None else argv))
    profile = prompt_controller_profile()
    args.output.write_text(profile + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Repository path contracts and legacy compatibility helpers."""

from __future__ import annotations

import os
from pathlib import Path


def _resolve_project_root() -> Path:
    override = os.environ.get("SATELLITE_CONTROL_ROOT", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    return Path(__file__).resolve().parents[3]


PROJECT_ROOT = _resolve_project_root()

SRC_ROOT = PROJECT_ROOT / "src"
SRC_PYTHON_ROOT = SRC_ROOT / "python"
SRC_CPP_ROOT = SRC_ROOT / "cpp"

UI_ROOT = PROJECT_ROOT / "ui"
UI_DIST_DIR = UI_ROOT / "dist"
UI_PUBLIC_MODEL_FILES_ROOT = UI_ROOT / "public" / "model_files"

DATA_ROOT = PROJECT_ROOT / "data"
ASSETS_ROOT = DATA_ROOT / "assets"
ASSET_MODEL_FILES_ROOT = ASSETS_ROOT / "model_files"
ASSET_PATHS_ROOT = ASSETS_ROOT / "paths"
ASSET_SCAN_PROJECTS_ROOT = ASSETS_ROOT / "scan_projects"
SIMULATION_DATA_ROOT = DATA_ROOT / "simulation_data"
DASHBOARD_DATA_ROOT = DATA_ROOT / "dashboard"

MISSIONS_DIR = PROJECT_ROOT / "missions"
SCRIPTS_DIR = PROJECT_ROOT / "scripts"

LEGACY_DATA_ROOT = PROJECT_ROOT / "Data"
LEGACY_SIMULATION_DATA_ROOT = LEGACY_DATA_ROOT / "Simulation"
LEGACY_DASHBOARD_DATA_ROOT = LEGACY_DATA_ROOT / "Dashboard"
LEGACY_ASSETS_ROOT = PROJECT_ROOT / "assets"
LEGACY_ASSET_MODEL_FILES_ROOT = LEGACY_ASSETS_ROOT / "model_files"
LEGACY_ASSET_PATHS_ROOT = LEGACY_ASSETS_ROOT / "paths"
LEGACY_ASSET_SCAN_PROJECTS_ROOT = LEGACY_ASSETS_ROOT / "scan_projects"


def canonicalize_repo_relative(path_value: str | Path) -> Path:
    """Normalize legacy top-level path aliases to canonical repo-relative paths."""
    raw = str(path_value).replace("\\", "/").strip()
    while raw.startswith("./"):
        raw = raw[2:]
    if raw == "" or raw == ".":
        return Path(".")
    if raw == "Data/Simulation":
        return Path("data/simulation_data")
    if raw.startswith("Data/Simulation/"):
        return Path("data/simulation_data") / raw[len("Data/Simulation/") :]
    if raw == "Data/Dashboard":
        return Path("data/dashboard")
    if raw.startswith("Data/Dashboard/"):
        return Path("data/dashboard") / raw[len("Data/Dashboard/") :]
    if raw == "Data/Linearized":
        return Path("data/linearized")
    if raw.startswith("Data/Linearized/"):
        return Path("data/linearized") / raw[len("Data/Linearized/") :]
    if raw == "Data/Thruster_Data":
        return Path("data/thruster_data")
    if raw.startswith("Data/Thruster_Data/"):
        return Path("data/thruster_data") / raw[len("Data/Thruster_Data/") :]
    if raw == "Data":
        return Path("data")
    if raw.startswith("Data/"):
        return Path("data") / raw[len("Data/") :]
    if raw == "assets":
        return Path("data/assets")
    if raw.startswith("assets/"):
        return Path("data/assets") / raw[len("assets/") :]
    return Path(raw)


def normalize_repo_relative_str(path_value: str | Path) -> str:
    """Return a canonical repo-relative path string when possible."""
    path = Path(path_value)
    if path.is_absolute():
        try:
            path = path.resolve().relative_to(PROJECT_ROOT)
        except ValueError:
            return str(path)
    return canonicalize_repo_relative(path).as_posix()


def resolve_repo_path(path_value: str | Path, *, prefer_canonical: bool = True) -> Path:
    """
    Resolve path values while supporting legacy top-level aliases.

    Relative paths are interpreted from PROJECT_ROOT and may resolve to either
    canonical or legacy locations depending on what exists on disk.
    """
    candidate = Path(path_value)
    if candidate.is_absolute():
        return candidate.resolve()

    legacy = PROJECT_ROOT / candidate
    canonical = PROJECT_ROOT / canonicalize_repo_relative(candidate)

    if canonical.exists():
        return canonical.resolve()
    if legacy.exists():
        return legacy.resolve()
    return canonical.resolve() if prefer_canonical else legacy.resolve()


def ensure_runtime_data_dirs() -> None:
    """Create canonical runtime data directories."""
    for path in (DATA_ROOT, ASSETS_ROOT, SIMULATION_DATA_ROOT, DASHBOARD_DATA_ROOT):
        path.mkdir(parents=True, exist_ok=True)

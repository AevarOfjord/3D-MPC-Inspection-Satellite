"""Canonical artifact layout and compatibility helpers for simulation runs."""

from pathlib import Path

DATA_ROOT = Path("Data")
PLOTS_ROOT = Path("Plots")

DATA_TIMESERIES_DIR = DATA_ROOT / "01_timeseries"
DATA_METADATA_DIR = DATA_ROOT / "02_metadata"
DATA_DIAGNOSTICS_DIR = DATA_ROOT / "03_diagnostics"
DATA_MANIFESTS_DIR = DATA_ROOT / "04_manifests"
DATA_NOTES_DIR = DATA_ROOT / "05_notes"
DATA_MEDIA_DIR = DATA_ROOT / "06_media"

ARTIFACT_RELATIVE_PATHS: dict[str, Path] = {
    "control_data.csv": DATA_TIMESERIES_DIR / "control_data.csv",
    "physics_data.csv": DATA_TIMESERIES_DIR / "physics_data.csv",
    "mpc_step_stats.csv": DATA_TIMESERIES_DIR / "mpc_step_stats.csv",
    "simulation_terminal_log.csv": DATA_TIMESERIES_DIR / "simulation_terminal_log.csv",
    "physics_terminal_log.csv": DATA_TIMESERIES_DIR / "physics_terminal_log.csv",
    "mission_metadata.json": DATA_METADATA_DIR / "mission_metadata.json",
    "reproducibility_manifest.json": (
        DATA_METADATA_DIR / "reproducibility_manifest.json"
    ),
    "performance_metrics.json": DATA_METADATA_DIR / "performance_metrics.json",
    "run_status.json": DATA_METADATA_DIR / "run_status.json",
    "kpi_summary.json": DATA_DIAGNOSTICS_DIR / "kpi_summary.json",
    "constraint_violations.json": DATA_DIAGNOSTICS_DIR / "constraint_violations.json",
    "controller_health.json": DATA_DIAGNOSTICS_DIR / "controller_health.json",
    "compare_signature.json": DATA_DIAGNOSTICS_DIR / "compare_signature.json",
    "event_timeline.jsonl": DATA_DIAGNOSTICS_DIR / "event_timeline.jsonl",
    "mode_timeline.csv": DATA_DIAGNOSTICS_DIR / "mode_timeline.csv",
    "completion_gate_trace.csv": DATA_DIAGNOSTICS_DIR / "completion_gate_trace.csv",
    "plots_index.json": DATA_MANIFESTS_DIR / "plots_index.json",
    "media_metadata.json": DATA_MANIFESTS_DIR / "media_metadata.json",
    "artifacts_manifest.json": DATA_MANIFESTS_DIR / "artifacts_manifest.json",
    "checksums.sha256": DATA_MANIFESTS_DIR / "checksums.sha256",
    "mission_summary.txt": DATA_NOTES_DIR / "mission_summary.txt",
    "run_notes.md": DATA_NOTES_DIR / "run_notes.md",
    "Simulation_3D_Render.mp4": DATA_MEDIA_DIR / "Simulation_3D_Render.mp4",
    "Simulation_3D_Render.gif": DATA_MEDIA_DIR / "Simulation_3D_Render.gif",
    "Simulation_3D_Render.webm": DATA_MEDIA_DIR / "Simulation_3D_Render.webm",
}


def artifact_relative_path(name: str) -> Path:
    """Return canonical relative path for an artifact filename."""
    return ARTIFACT_RELATIVE_PATHS.get(name, Path(name))


def artifact_path(run_dir: Path, name: str) -> Path:
    """Return canonical full path for an artifact filename."""
    return run_dir / artifact_relative_path(name)


def artifact_candidates(run_dir: Path, name: str) -> tuple[Path, ...]:
    """
    Return preferred + legacy candidate paths for reading.

    Preferred path is new Data/ layout; legacy path is run root.
    """
    preferred = artifact_path(run_dir, name)
    legacy = run_dir / name
    if preferred == legacy:
        return (preferred,)
    return (preferred, legacy)


def resolve_existing_artifact_path(run_dir: Path, name: str) -> Path | None:
    """Return first existing candidate path or None."""
    for candidate in artifact_candidates(run_dir, name):
        if candidate.exists():
            return candidate
    return None


def ensure_artifact_directories(run_dir: Path) -> None:
    """Create canonical Data/ and Plots/ subdirectories for a run."""
    data_dirs = {DATA_ROOT}
    for rel_path in ARTIFACT_RELATIVE_PATHS.values():
        if rel_path.parts and rel_path.parts[0] == DATA_ROOT.name:
            data_dirs.add(rel_path.parent)

    for rel_dir in sorted(data_dirs):
        (run_dir / rel_dir).mkdir(parents=True, exist_ok=True)
    (run_dir / PLOTS_ROOT).mkdir(parents=True, exist_ok=True)

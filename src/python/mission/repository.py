"""
Mission repository helpers.

Provides a single place for mission file discovery, resolution, and persistence
for unified mission storage (`missions_unified`).
"""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from config.paths import MISSIONS_DIR

MissionSource = Literal["local"]


@dataclass(frozen=True)
class MissionEntry:
    """Mission file metadata used by CLI/UI selection flows."""

    name: str
    path: Path
    source: MissionSource


SOURCE_DIRS: dict[str, Path] = {
    "local": MISSIONS_DIR,
}

DEFAULT_SOURCE_PRIORITY: tuple[str, ...] = ("local",)


def sanitize_mission_name(name: str) -> str:
    """Return a filesystem-safe mission name (without extension)."""
    safe = "".join(c for c in name if c.isalnum() or c in ("-", "_")).strip()
    return safe


def with_json_extension(name: str) -> str:
    """Normalize mission filename by ensuring `.json` suffix."""
    return name if name.endswith(".json") else f"{name}.json"


def get_source_dir(source: MissionSource) -> Path:
    """Return on-disk directory for a mission source."""
    return SOURCE_DIRS[source]


def list_mission_entries(
    source_priority: Iterable[MissionSource] = DEFAULT_SOURCE_PRIORITY,
) -> list[MissionEntry]:
    """
    List all mission files with source tags.

    Duplicates are preserved when the same filename exists in multiple sources.
    """
    entries: list[MissionEntry] = []
    for source in source_priority:
        missions_dir = get_source_dir(source)
        if not missions_dir.exists():
            continue
        for mission_file in sorted(missions_dir.glob("*.json")):
            entries.append(
                MissionEntry(
                    name=mission_file.name,
                    path=mission_file,
                    source=source,
                )
            )
    return entries


def list_mission_names(
    source_priority: Iterable[MissionSource] = DEFAULT_SOURCE_PRIORITY,
) -> list[str]:
    """List unique mission filenames across sources."""
    names = {entry.name for entry in list_mission_entries(source_priority)}
    return sorted(names)


def resolve_mission_file(
    mission_name: str,
    source_priority: Iterable[MissionSource] = DEFAULT_SOURCE_PRIORITY,
) -> Path:
    """
    Resolve a mission filename across sources by priority.

    Raises:
        FileNotFoundError: If mission does not exist in any selected source.
    """
    candidate_name = with_json_extension(mission_name)
    for source in source_priority:
        mission_file = get_source_dir(source) / candidate_name
        if mission_file.exists():
            return mission_file
    raise FileNotFoundError(f"Mission not found: {mission_name}")


def load_mission_json(
    mission_name: str,
    source_priority: Iterable[MissionSource] = DEFAULT_SOURCE_PRIORITY,
) -> dict:
    """Load mission JSON payload using repository resolution."""
    mission_file = resolve_mission_file(mission_name, source_priority=source_priority)
    return json.loads(mission_file.read_text())


def save_mission_json(
    name: str,
    payload: dict,
    source: MissionSource = "local",
) -> Path:
    """
    Save mission JSON payload to a source directory.

    Raises:
        ValueError: If sanitized name is empty.
    """
    safe_name = sanitize_mission_name(name)
    if not safe_name:
        raise ValueError("Invalid mission name")
    missions_dir = get_source_dir(source)
    missions_dir.mkdir(parents=True, exist_ok=True)
    mission_file = missions_dir / f"{safe_name}.json"
    mission_file.write_text(json.dumps(payload, indent=2))
    return mission_file

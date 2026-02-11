"""
Path asset storage helpers.

Stores reusable path definitions (typically generated from OBJ scans) so they
can be selected later in the web UI and used by missions.
"""

from __future__ import annotations

import json
import re
from collections.abc import Iterable
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


PATH_ASSET_DIR = _repo_root() / "assets" / "paths"


def _ensure_dir() -> None:
    PATH_ASSET_DIR.mkdir(parents=True, exist_ok=True)


def _safe_id(name: str) -> str:
    raw = name.strip()
    if not raw:
        raw = "path_asset"
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", raw).strip("_")
    return safe or "path_asset"


def _compute_path_length(path: Iterable[Iterable[float]]) -> float:
    points = np.array(list(path), dtype=float)
    if points.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(points[1:] - points[:-1], axis=1)))


def _asset_path(asset_id: str) -> Path:
    safe_id = _safe_id(asset_id)
    return PATH_ASSET_DIR / f"{safe_id}.json"


def _normalize_open_path(path: list[Any], is_open: bool) -> list[Any]:
    if not is_open or len(path) <= 2:
        return path
    try:
        pts = [np.array(p, dtype=float) for p in path]
    except Exception:
        return path
    while len(pts) > 2 and np.linalg.norm(pts[-1] - pts[0]) <= 1e-6:
        pts.pop()
    return [list(map(float, p)) for p in pts]


def save_path_asset(data: dict[str, Any]) -> dict[str, Any]:
    """Persist a path asset to disk and return the stored payload."""
    _ensure_dir()

    name = str(data.get("name") or "").strip()
    if not name:
        raise ValueError("Path asset name is required")

    asset_id = str(data.get("id") or _safe_id(name))
    obj_path = str(data.get("obj_path") or "")
    path = data.get("path") or []
    if not isinstance(path, list) or not path:
        raise ValueError("Path asset requires a non-empty 'path' list")

    open_path = bool(data.get("open", True))
    path = _normalize_open_path(path, open_path)

    now_iso = datetime.now(UTC).isoformat()
    path_length = _compute_path_length(path)

    payload: dict[str, Any] = {
        "id": asset_id,
        "name": name,
        "obj_path": obj_path,
        "path": path,
        "open": open_path,
        "relative_to_obj": bool(data.get("relative_to_obj", True)),
        "notes": data.get("notes"),
        "points": int(len(path)),
        "path_length": float(path_length),
        "created_at": data.get("created_at") or now_iso,
        "updated_at": now_iso,
    }

    asset_path = _asset_path(asset_id)
    asset_path.write_text(json.dumps(payload, indent=2))
    return payload


def load_path_asset(asset_id: str) -> dict[str, Any]:
    """Load a path asset by id (filename stem)."""
    _ensure_dir()
    asset_path = _asset_path(asset_id)
    if asset_path.exists():
        return json.loads(asset_path.read_text())

    # Fallback: scan for matching name/id in files
    for candidate in PATH_ASSET_DIR.rglob("*.json"):
        try:
            data = json.loads(candidate.read_text())
        except Exception:
            continue
        if data.get("id") == asset_id or data.get("name") == asset_id:
            return data
    raise FileNotFoundError(f"Path asset not found: {asset_id}")


def list_path_assets() -> list[dict[str, Any]]:
    """Return summary metadata for all saved path assets."""
    _ensure_dir()
    assets: list[dict[str, Any]] = []
    for asset_file in sorted(PATH_ASSET_DIR.rglob("*.json")):
        try:
            data = json.loads(asset_file.read_text())
        except Exception:
            continue
        assets.append(
            {
                "id": data.get("id", asset_file.stem),
                "name": data.get("name", asset_file.stem),
                "obj_path": data.get("obj_path", ""),
                "points": int(data.get("points") or len(data.get("path") or [])),
                "path_length": float(data.get("path_length") or 0.0),
                "open": bool(data.get("open", True)),
                "relative_to_obj": bool(data.get("relative_to_obj", True)),
                "created_at": data.get("created_at"),
                "updated_at": data.get("updated_at"),
            }
        )
    return assets

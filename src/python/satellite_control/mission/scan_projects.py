"""Scan project storage and compile helpers."""

from __future__ import annotations

import json
import math
import re
from collections import defaultdict
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

from satellite_control.mission.mesh_scan import compute_mesh_bounds, load_obj_vertices

try:
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover - scipy is expected but keep fallback robust
    cKDTree = None  # type: ignore[assignment]


AXIS_TO_FRAME: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {
    "X": (
        np.array([1.0, 0.0, 0.0], dtype=float),
        np.array([0.0, 1.0, 0.0], dtype=float),
        np.array([0.0, 0.0, 1.0], dtype=float),
    ),
    "Y": (
        np.array([0.0, 1.0, 0.0], dtype=float),
        np.array([1.0, 0.0, 0.0], dtype=float),
        np.array([0.0, 0.0, 1.0], dtype=float),
    ),
    "Z": (
        np.array([0.0, 0.0, 1.0], dtype=float),
        np.array([1.0, 0.0, 0.0], dtype=float),
        np.array([0.0, 1.0, 0.0], dtype=float),
    ),
}


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


SCAN_PROJECT_DIR = _repo_root() / "assets" / "scan_projects"


def _ensure_dir() -> None:
    SCAN_PROJECT_DIR.mkdir(parents=True, exist_ok=True)


def _safe_id(name: str) -> str:
    raw = name.strip() or "scan_project"
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", raw).strip("_")
    return safe or "scan_project"


def _project_path(project_id: str) -> Path:
    return SCAN_PROJECT_DIR / f"{_safe_id(project_id)}.json"


def _compute_path_length(path: list[list[float]]) -> float:
    if len(path) < 2:
        return 0.0
    arr = np.asarray(path, dtype=float)
    return float(np.sum(np.linalg.norm(arr[1:] - arr[:-1], axis=1)))


def save_scan_project(data: dict[str, Any]) -> dict[str, Any]:
    _ensure_dir()

    name = str(data.get("name") or "").strip()
    if not name:
        raise ValueError("Scan project name is required")

    project_id = str(data.get("id") or _safe_id(name))
    scans = data.get("scans") or []
    if not isinstance(scans, list) or len(scans) == 0:
        raise ValueError("Scan project requires at least one scan")

    connectors = data.get("connectors") or []
    if not isinstance(connectors, list):
        raise ValueError("connectors must be a list")

    now_iso = datetime.now(UTC).isoformat()
    payload: dict[str, Any] = {
        "schema_version": int(data.get("schema_version") or 1),
        "id": project_id,
        "name": name,
        "obj_path": str(data.get("obj_path") or ""),
        "scans": scans,
        "connectors": connectors,
        "created_at": data.get("created_at") or now_iso,
        "updated_at": now_iso,
    }

    path = _project_path(project_id)
    path.write_text(json.dumps(payload, indent=2))
    return payload


def load_scan_project(project_id: str) -> dict[str, Any]:
    _ensure_dir()
    path = _project_path(project_id)
    if path.exists():
        return json.loads(path.read_text())

    # Fallback by id/name in file contents.
    for candidate in SCAN_PROJECT_DIR.rglob("*.json"):
        try:
            data = json.loads(candidate.read_text())
        except Exception:
            continue
        if data.get("id") == project_id or data.get("name") == project_id:
            return data

    raise FileNotFoundError(f"Scan project not found: {project_id}")


def list_scan_projects() -> list[dict[str, Any]]:
    _ensure_dir()
    out: list[dict[str, Any]] = []
    for project_file in sorted(SCAN_PROJECT_DIR.rglob("*.json")):
        try:
            data = json.loads(project_file.read_text())
        except Exception:
            continue
        out.append(
            {
                "id": data.get("id", project_file.stem),
                "name": data.get("name", project_file.stem),
                "obj_path": data.get("obj_path", ""),
                "scans": int(len(data.get("scans") or [])),
                "connectors": int(len(data.get("connectors") or [])),
                "created_at": data.get("created_at"),
                "updated_at": data.get("updated_at"),
            }
        )
    return out


def _to_vec3(value: Any, fallback: tuple[float, float, float]) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 3:
        return np.array(fallback, dtype=float)
    try:
        return np.array(
            [float(value[0]), float(value[1]), float(value[2])], dtype=float
        )
    except Exception:
        return np.array(fallback, dtype=float)


def _resample_polyline(path: list[list[float]], target_points: int) -> list[list[float]]:
    if len(path) < 2 or target_points <= len(path):
        return [[float(p[0]), float(p[1]), float(p[2])] for p in path]

    arr = np.asarray(path, dtype=float)
    seg = np.linalg.norm(arr[1:] - arr[:-1], axis=1)
    total = float(np.sum(seg))
    if total < 1e-9:
        return [[float(arr[0, 0]), float(arr[0, 1]), float(arr[0, 2])]] * target_points

    cum = np.concatenate(([0.0], np.cumsum(seg)))
    targets = np.linspace(0.0, total, target_points)
    x = np.interp(targets, cum, arr[:, 0])
    y = np.interp(targets, cum, arr[:, 1])
    z = np.interp(targets, cum, arr[:, 2])
    return [[float(xi), float(yi), float(zi)] for xi, yi, zi in zip(x, y, z)]


def _interpolate_key_level(levels: list[dict[str, Any]], t: float) -> dict[str, float]:
    if not levels:
        return {
            "radius_x": 1.0,
            "radius_y": 1.0,
            "rotation_deg": 0.0,
            "offset_x": 0.0,
            "offset_y": 0.0,
        }

    sorted_levels = sorted(levels, key=lambda x: float(x.get("t", 0.0)))
    if t <= float(sorted_levels[0].get("t", 0.0)):
        lv = sorted_levels[0]
        off = lv.get("center_offset") or [0.0, 0.0]
        return {
            "radius_x": max(0.01, float(lv.get("radius_x", 1.0))),
            "radius_y": max(0.01, float(lv.get("radius_y", 1.0))),
            "rotation_deg": float(lv.get("rotation_deg", 0.0)),
            "offset_x": float(off[0]),
            "offset_y": float(off[1]),
        }
    if t >= float(sorted_levels[-1].get("t", 1.0)):
        lv = sorted_levels[-1]
        off = lv.get("center_offset") or [0.0, 0.0]
        return {
            "radius_x": max(0.01, float(lv.get("radius_x", 1.0))),
            "radius_y": max(0.01, float(lv.get("radius_y", 1.0))),
            "rotation_deg": float(lv.get("rotation_deg", 0.0)),
            "offset_x": float(off[0]),
            "offset_y": float(off[1]),
        }

    for i in range(1, len(sorted_levels)):
        prev = sorted_levels[i - 1]
        nxt = sorted_levels[i]
        t0 = float(prev.get("t", 0.0))
        t1 = float(nxt.get("t", 1.0))
        if t1 <= t0:
            continue
        if t <= t1:
            alpha = (t - t0) / (t1 - t0)
            prev_off = prev.get("center_offset") or [0.0, 0.0]
            nxt_off = nxt.get("center_offset") or [0.0, 0.0]
            return {
                "radius_x": max(
                    0.01,
                    float(prev.get("radius_x", 1.0))
                    + alpha
                    * (float(nxt.get("radius_x", 1.0)) - float(prev.get("radius_x", 1.0))),
                ),
                "radius_y": max(
                    0.01,
                    float(prev.get("radius_y", 1.0))
                    + alpha
                    * (float(nxt.get("radius_y", 1.0)) - float(prev.get("radius_y", 1.0))),
                ),
                "rotation_deg": float(prev.get("rotation_deg", 0.0))
                + alpha
                * (
                    float(nxt.get("rotation_deg", 0.0))
                    - float(prev.get("rotation_deg", 0.0))
                ),
                "offset_x": float(prev_off[0])
                + alpha * (float(nxt_off[0]) - float(prev_off[0])),
                "offset_y": float(prev_off[1])
                + alpha * (float(nxt_off[1]) - float(prev_off[1])),
            }

    lv = sorted_levels[-1]
    off = lv.get("center_offset") or [0.0, 0.0]
    return {
        "radius_x": max(0.01, float(lv.get("radius_x", 1.0))),
        "radius_y": max(0.01, float(lv.get("radius_y", 1.0))),
        "rotation_deg": float(lv.get("rotation_deg", 0.0)),
        "offset_x": float(off[0]),
        "offset_y": float(off[1]),
    }


def _generate_scan_path(
    scan: dict[str, Any],
    model_center: np.ndarray,
    quality: str,
) -> tuple[list[list[float]], list[list[float]]]:
    axis = str(scan.get("axis", "Z")).upper().strip()
    normal, u_axis, v_axis = AXIS_TO_FRAME.get(axis, AXIS_TO_FRAME["Z"])

    plane_a = _to_vec3(scan.get("plane_a"), (0.0, 0.0, -0.5))
    plane_b = _to_vec3(scan.get("plane_b"), (0.0, 0.0, 0.5))

    d_a = float(np.dot(plane_a - model_center, normal))
    d_b = float(np.dot(plane_b - model_center, normal))
    center_a = model_center + normal * d_a
    center_b = model_center + normal * d_b

    scan_span = float(max(abs(d_b - d_a), 1e-6))
    spacing_raw = scan.get("level_spacing_m")
    if spacing_raw is None:
        spacing_raw = scan.get("level_spacing")
    try:
        level_spacing_m = float(spacing_raw) if spacing_raw is not None else float("nan")
    except Exception:
        level_spacing_m = float("nan")

    if math.isfinite(level_spacing_m) and level_spacing_m > 0.0:
        turns = max(1.0, scan_span / level_spacing_m)
    else:
        # Backward compatibility for older saved projects.
        turns = max(1.0, float(scan.get("turns", 6.0)))
    points_per_turn = max(4, int(scan.get("coarse_points_per_turn", 4)))
    coarse_points = max(8, int(math.ceil(turns * points_per_turn)) + 1)

    key_levels = scan.get("key_levels") or []

    def _point_at_t(t: float) -> list[float]:
        center = center_a + (center_b - center_a) * t
        shape = _interpolate_key_level(key_levels, t)

        rot = math.radians(shape["rotation_deg"])
        ang = 2.0 * math.pi * turns * t
        ex = shape["radius_x"] * math.cos(ang)
        ey = shape["radius_y"] * math.sin(ang)

        # Rotate the ellipse in the local plane.
        local_x = ex * math.cos(rot) - ey * math.sin(rot)
        local_y = ex * math.sin(rot) + ey * math.cos(rot)

        world = (
            center
            + u_axis * (shape["offset_x"] + local_x)
            + v_axis * (shape["offset_y"] + local_y)
        )
        return [float(world[0]), float(world[1]), float(world[2])]

    coarse_path: list[list[float]] = []
    for idx in range(coarse_points):
        t = idx / float(max(1, coarse_points - 1))
        coarse_path.append(_point_at_t(t))

    densify_multiplier = max(1, int(scan.get("densify_multiplier", 8)))
    if quality == "preview":
        densify_multiplier = max(1, int(round(densify_multiplier * 0.5)))

    dense_points = max(len(coarse_path), (len(coarse_path) - 1) * densify_multiplier + 1)
    dense_path: list[list[float]] = []
    for idx in range(dense_points):
        t = idx / float(max(1, dense_points - 1))
        dense_path.append(_point_at_t(t))
    return coarse_path, dense_path


def _auto_connector_controls(
    start: np.ndarray,
    end: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    vec = end - start
    dist = float(np.linalg.norm(vec))
    if dist < 1e-9:
        return start.copy(), end.copy()
    dir_vec = vec / dist
    up = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(up, dir_vec))) > 0.9:
        up = np.array([0.0, 1.0, 0.0], dtype=float)
    side = np.cross(dir_vec, up)
    side_norm = float(np.linalg.norm(side))
    if side_norm < 1e-9:
        side = np.array([0.0, 1.0, 0.0], dtype=float)
    else:
        side = side / side_norm
    bulge = min(max(dist * 0.25, 0.15), 2.0)
    c1 = start + vec * 0.33 + side * bulge
    c2 = start + vec * 0.66 + side * bulge
    return c1, c2


def _sample_cubic_bezier(
    start: np.ndarray,
    control1: np.ndarray,
    control2: np.ndarray,
    end: np.ndarray,
    samples: int,
) -> list[list[float]]:
    n = max(4, samples)
    out: list[list[float]] = []
    for i in range(n + 1):
        t = i / float(n)
        u = 1.0 - t
        p = (
            (u * u * u) * start
            + (3.0 * u * u * t) * control1
            + (3.0 * u * t * t) * control2
            + (t * t * t) * end
        )
        out.append([float(p[0]), float(p[1]), float(p[2])])
    return out


def _collisions_for_path(
    path: list[list[float]],
    tree: Any,
    threshold: float,
) -> tuple[float | None, int, list[float] | None]:
    if tree is None or len(path) == 0:
        return None, 0, None

    arr = np.asarray(path, dtype=float)
    if cKDTree is not None and isinstance(tree, cKDTree):
        dists, _ = tree.query(arr, k=1)
        dist_arr = np.asarray(dists, dtype=float)
    else:
        # Fallback: brute-force nearest distance.
        points = np.asarray(tree, dtype=float)
        dist_arr = np.empty((arr.shape[0],), dtype=float)
        for i, p in enumerate(arr):
            dist_arr[i] = float(np.min(np.linalg.norm(points - p, axis=1)))

    min_clearance = float(np.min(dist_arr)) if dist_arr.size else None
    collisions = int(np.sum(dist_arr < threshold)) if dist_arr.size else 0
    return min_clearance, collisions, [float(v) for v in dist_arr.tolist()]


def compile_scan_project(
    project: dict[str, Any],
    quality: str = "preview",
    include_collision: bool = True,
    collision_threshold_m: float = 0.05,
) -> dict[str, Any]:
    scans = project.get("scans") or []
    connectors = project.get("connectors") or []
    obj_path = str(project.get("obj_path") or "")
    if not obj_path:
        raise ValueError("project.obj_path is required")
    if not scans:
        raise ValueError("project must include at least one scan")

    vertices = load_obj_vertices(obj_path)
    _, _, model_center, _ = compute_mesh_bounds(vertices)

    tree: Any = None
    if include_collision:
        if cKDTree is not None:
            tree = cKDTree(vertices)
        else:
            tree = vertices

    scan_ids = [str(scan.get("id")) for scan in scans]
    if len(set(scan_ids)) != len(scan_ids):
        raise ValueError("scan IDs must be unique")

    scans_by_id = {str(scan["id"]): scan for scan in scans}
    compiled_scan_paths: dict[str, dict[str, Any]] = {}
    for scan in scans:
        sid = str(scan["id"])
        coarse, dense = _generate_scan_path(scan, model_center, quality=quality)
        min_clearance, collision_count, clearance = _collisions_for_path(
            dense, tree, threshold=collision_threshold_m
        )
        compiled_scan_paths[sid] = {
            "scan_id": sid,
            "coarse_path": coarse,
            "path": dense,
            "points": len(dense),
            "path_length": _compute_path_length(dense),
            "min_clearance_m": min_clearance,
            "collision_points_count": collision_count,
            "clearance_per_point": clearance,
            "speed_max": float(scan.get("speed_max", 0.2)),
        }

    incoming: dict[str, list[dict[str, Any]]] = defaultdict(list)
    outgoing: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for connector in connectors:
        from_id = str(connector.get("from_scan_id") or "")
        to_id = str(connector.get("to_scan_id") or "")
        if from_id not in scans_by_id or to_id not in scans_by_id:
            raise ValueError(f"connector {connector.get('id')} references unknown scan")
        if from_id == to_id:
            raise ValueError(f"connector {connector.get('id')} must connect two different scans")
        outgoing[from_id].append(connector)
        incoming[to_id].append(connector)

    enforce_linear_chain = str(quality).lower() == "final"

    if len(scans) > 1:
        if enforce_linear_chain and not connectors:
            raise ValueError("multi-scan project requires connectors")
        if enforce_linear_chain and len(connectors) != len(scans) - 1:
            raise ValueError("connectors must form a single linear chain")

        for sid in scan_ids:
            if enforce_linear_chain and (
                len(incoming[sid]) > 1 or len(outgoing[sid]) > 1
            ):
                raise ValueError("only linear scan chains are supported in this version")

        starts = [sid for sid in scan_ids if len(incoming[sid]) == 0]
        if enforce_linear_chain and len(starts) != 1:
            raise ValueError("scan connectors must define exactly one chain start")

        if enforce_linear_chain:
            order = []
            current = starts[0]
            visited: set[str] = set()
            while True:
                if current in visited:
                    raise ValueError("scan connectors contain a cycle")
                visited.add(current)
                order.append(current)
                out = outgoing[current]
                if not out:
                    break
                current = str(out[0]["to_scan_id"])

            if len(order) != len(scans):
                raise ValueError("scan connectors do not connect all scans")
        else:
            # Preview mode: allow disconnected scans so users can author and inspect
            # independent scan paths before adding connectors.
            order = scan_ids
    else:
        order = [scan_ids[0]]

    reverse_map: dict[str, bool] = {}
    for sid in order:
        if not enforce_linear_chain:
            reverse_map[sid] = False
            continue
        rev_from_in: bool | None = None
        rev_from_out: bool | None = None
        if incoming[sid]:
            rev_from_in = str(incoming[sid][0].get("to_endpoint", "start")) == "end"
        if outgoing[sid]:
            rev_from_out = str(outgoing[sid][0].get("from_endpoint", "end")) == "start"
        if rev_from_in is not None and rev_from_out is not None and rev_from_in != rev_from_out:
            raise ValueError(
                f"scan '{sid}' has conflicting connector endpoint orientation"
            )
        reverse_map[sid] = (
            rev_from_in
            if rev_from_in is not None
            else (rev_from_out if rev_from_out is not None else False)
        )

    oriented_scan_paths: dict[str, list[list[float]]] = {}
    for sid in order:
        path = compiled_scan_paths[sid]["path"]
        oriented_scan_paths[sid] = list(reversed(path)) if reverse_map[sid] else path

    combined_path: list[list[float]] = []
    scan_diagnostics: list[dict[str, Any]] = []
    connector_diagnostics: list[dict[str, Any]] = []
    endpoints: dict[str, dict[str, list[float]]] = {}

    for idx, sid in enumerate(order):
        path = oriented_scan_paths[sid]
        if idx == 0:
            combined_path.extend(path)
        else:
            if (
                combined_path
                and np.linalg.norm(np.asarray(combined_path[-1]) - np.asarray(path[0])) <= 1e-9
            ):
                combined_path.extend(path[1:])
            else:
                combined_path.extend(path)

        endpoints[sid] = {"start": list(path[0]), "end": list(path[-1])}

        scan_diag = compiled_scan_paths[sid]
        scan_diagnostics.append(
            {
                "id": sid,
                "kind": "scan",
                "points": int(scan_diag["points"]),
                "path_length": float(scan_diag["path_length"]),
                "path": path,
                "min_clearance_m": scan_diag["min_clearance_m"],
                "collision_points_count": int(scan_diag["collision_points_count"]),
                "clearance_per_point": scan_diag["clearance_per_point"],
            }
        )

        if idx >= len(order) - 1:
            continue

        if not outgoing[sid]:
            continue
        connector = outgoing[sid][0]
        conn_id = str(connector.get("id") or f"conn_{idx + 1}")
        next_sid = order[idx + 1]
        start = np.asarray(path[-1], dtype=float)
        end = np.asarray(oriented_scan_paths[next_sid][0], dtype=float)

        control1 = connector.get("control1")
        control2 = connector.get("control2")
        if control1 is None or control2 is None:
            auto_c1, auto_c2 = _auto_connector_controls(start, end)
            c1 = auto_c1
            c2 = auto_c2
        else:
            c1 = _to_vec3(control1, tuple(_auto_connector_controls(start, end)[0]))
            c2 = _to_vec3(control2, tuple(_auto_connector_controls(start, end)[1]))

        samples = max(4, int(connector.get("samples", 24)))
        if quality == "preview":
            samples = max(6, samples // 2)

        connector_path = _sample_cubic_bezier(start, c1, c2, end, samples=samples)
        if combined_path:
            combined_path.extend(connector_path[1:])
        else:
            combined_path.extend(connector_path)

        min_clearance, collision_count, clearance = _collisions_for_path(
            connector_path, tree, threshold=collision_threshold_m
        )
        connector_diagnostics.append(
            {
                "id": conn_id,
                "kind": "connector",
                "points": int(len(connector_path)),
                "path_length": float(_compute_path_length(connector_path)),
                "path": connector_path,
                "min_clearance_m": min_clearance,
                "collision_points_count": int(collision_count),
                "clearance_per_point": clearance,
            }
        )

    total_length = _compute_path_length(combined_path)
    avg_speed = np.mean(
        [max(1e-3, float(scans_by_id[sid].get("speed_max", 0.2))) for sid in order]
    )
    estimated_duration = float(total_length / max(1e-3, float(avg_speed)))

    global_min_clearance, global_collisions, combined_clearance = _collisions_for_path(
        combined_path, tree, threshold=collision_threshold_m
    )
    warnings: list[str] = []
    if include_collision and global_collisions > 0:
        warnings.append(
            f"{global_collisions} points are below {collision_threshold_m:.3f}m clearance"
        )

    return {
        "status": "success",
        "combined_path": combined_path,
        "path_length": float(total_length),
        "estimated_duration": estimated_duration,
        "points": len(combined_path),
        "endpoints": endpoints,
        "scan_paths": scan_diagnostics,
        "connector_paths": connector_diagnostics,
        "diagnostics": {
            "min_clearance_m": global_min_clearance,
            "collision_points_count": int(global_collisions),
            "clearance_threshold_m": float(collision_threshold_m),
            "combined_clearance_per_point": combined_clearance,
            "warnings": warnings,
        },
    }

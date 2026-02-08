"""
Unified mission compiler.

Converts unified mission segments into a single MPCC path suitable for the
existing MPC path-following pipeline.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np

from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.mission.mesh_scan import (
    compute_scan_sampling,
    load_obj_vertices,
)
from satellite_control.mission.path_assets import load_path_asset
from satellite_control.mission.path_following import build_point_to_point_path
from satellite_control.mission.unified_mission import (
    MissionDefinition,
    MissionObstacle,
    SegmentType,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _resolve_target_obj_path(target_id: str) -> Path | None:
    if not target_id:
        return None
    upper = target_id.upper()
    root = _repo_root()
    if "ISS" in upper:
        return root / "assets" / "models" / "ISS" / "ISS.obj"
    if "STARLINK" in upper:
        return root / "assets" / "models" / "Starlink" / "starlink.obj"
    return None


def _axis_to_scan_axis(axis: str) -> str:
    axis = axis.upper()
    if "X" in axis:
        return "X"
    if "Y" in axis:
        return "Y"
    return "Z"


def _parse_axis(axis: str) -> tuple[str, int]:
    axis = axis.strip().upper()
    sign = -1 if axis.startswith("-") else 1
    letter = axis[-1] if axis and axis[-1] in ("X", "Y", "Z") else "Z"
    return letter, sign


def _axis_permutation(axis: str) -> tuple[list[int], list[int]]:
    if axis == "X":
        return [1, 2, 0], [2, 0, 1]
    if axis == "Y":
        return [2, 0, 1], [1, 2, 0]
    return [0, 1, 2], [0, 1, 2]


def _build_scan_path(
    target_pos: np.ndarray,
    obj_path: Path | None,
    scan: Any,
) -> list[tuple[float, float, float]]:
    """Generate a scan path around the target (spiral or circles)."""
    axis_letter, axis_sign = _parse_axis(scan.axis)
    perm, inv = _axis_permutation(axis_letter)

    center_orig = np.zeros(3, dtype=float)
    center_perm = np.zeros(3, dtype=float)
    radius_xy = 0.5
    height = 1.0

    if obj_path and obj_path.exists():
        vertices = load_obj_vertices(str(obj_path))
        if vertices.size > 0:
            min_bounds = vertices.min(axis=0)
            max_bounds = vertices.max(axis=0)
            center_orig = (min_bounds + max_bounds) / 2.0
            center_perm = center_orig[perm]
            verts_perm = vertices[:, perm]
            min_p = verts_perm.min(axis=0)
            max_p = verts_perm.max(axis=0)
            height = float(max_p[2] - min_p[2])
            offsets = verts_perm[:, :2] - center_perm[:2]
            if offsets.size > 0:
                radius_xy = float(np.max(np.linalg.norm(offsets, axis=1)))

    ring_step, points_per_ring = compute_scan_sampling(
        radius=radius_xy,
        standoff=scan.standoff,
        fov_deg=scan.fov_deg,
        overlap=scan.overlap,
    )
    overlap = float(min(max(scan.overlap, 0.0), 0.9))
    fov_rad = math.radians(float(max(scan.fov_deg, 1.0)))
    footprint = 2.0 * max(scan.standoff, 1e-3) * math.tan(0.5 * fov_rad)
    auto_pitch = footprint * (1.0 - overlap)
    pitch = float(scan.pitch) if scan.pitch and scan.pitch > 0 else float(auto_pitch)
    if not math.isfinite(pitch) or pitch <= 0:
        pitch = 0.5

    radius = radius_xy + scan.standoff
    direction_mult = -1.0 if str(scan.direction).upper() == "CW" else 1.0

    pattern = getattr(scan, "pattern", "spiral")
    path: list[tuple[float, float, float]] = []

    if pattern == "circles":
        # Stacked rings
        num_levels = max(1, int(math.ceil(height / pitch))) if height > 0 else 1
        # Center levels around the object center
        total_scan_height = (num_levels - 1) * pitch
        z_start = -0.5 * total_scan_height

        points_per_ring_int = max(360, int(points_per_ring))

        for lvl in range(num_levels):
            z_rel = z_start + lvl * pitch
            if axis_sign < 0:
                z_rel = -z_rel

            # Generate ring
            ring_points = []
            for i in range(points_per_ring_int + 1):
                angle = 2.0 * math.pi * (i / points_per_ring_int)
                angle *= direction_mult

                x = center_perm[0] + radius * math.cos(angle)
                y = center_perm[1] + radius * math.sin(angle)
                z = center_perm[2] + z_rel

                perm_point = np.array([x, y, z], dtype=float)
                orig_point = perm_point[inv]
                world_point = orig_point + (target_pos - center_orig)
                ring_points.append(tuple(map(float, world_point)))

            path.extend(ring_points)

            # Add safe transition to next ring if not last
            if lvl < num_levels - 1:
                # Simply connect end of this ring to start of next (calculated above)
                # But actually, the start of next ring is just (radius, 0, next_z) logic
                # To make it smooth, we could just let MPCC handle it, or add an intermediate waypoint
                pass

    else:
        # Spiral (Default)
        revolutions = int(scan.revolutions) if scan.revolutions else 1
        if height > 1e-6:
            turns_needed = int(math.ceil(height / pitch))
            revolutions = max(revolutions, turns_needed, 1)
        revolutions = max(revolutions, 1)

        total_height = pitch * revolutions
        z_start = -0.5 * total_height

        points_per_rev = max(360, int(points_per_ring))
        total_points = max(3, int(points_per_rev * revolutions))

        for i in range(total_points + 1):
            frac = i / total_points
            angle = 2.0 * math.pi * revolutions * frac
            angle *= direction_mult

            z_rel = z_start + total_height * frac
            if axis_sign < 0:
                z_rel = -z_rel

            x = center_perm[0] + radius * math.cos(angle)
            y = center_perm[1] + radius * math.sin(angle)
            z = center_perm[2] + z_rel

            perm_point = np.array([x, y, z], dtype=float)
            orig_point = perm_point[inv]
            world_point = orig_point + (target_pos - center_orig)
            path.append(tuple(map(float, world_point)))

    return path


def _build_asset_path(
    asset_id: str, target_pos: np.ndarray
) -> tuple[list[tuple[float, float, float]], bool]:
    """Load a prebuilt path asset and optionally offset to target."""
    try:
        asset = load_path_asset(asset_id)
    except Exception:
        return [], False

    raw_path = asset.get("path") or []
    if not raw_path:
        return [], False

    relative = bool(asset.get("relative_to_obj", True))
    if relative and target_pos is not None:
        offset = np.array(target_pos, dtype=float)
        return [
            tuple(map(float, np.array(p, dtype=float) + offset)) for p in raw_path
        ], True
    return [tuple(map(float, p)) for p in raw_path], False


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q = [w, x, y, z]."""
    w, x, y, z = q
    q_vec = np.array([x, y, z], dtype=float)
    uv = np.cross(q_vec, v)
    uuv = np.cross(q_vec, uv)
    return v + 2.0 * (w * uv + uuv)


def _apply_target_orientation(
    path: list[tuple[float, float, float]],
    target_pos: np.ndarray,
    target_orientation: Sequence[float] | None,
) -> list[tuple[float, float, float]]:
    if not target_orientation:
        return path
    q = np.array(target_orientation, dtype=float)
    if q.shape[0] != 4:
        return path
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        return path
    q = q / norm
    rotated: list[tuple[float, float, float]] = []
    for p in path:
        vec = np.array(p, dtype=float) - target_pos
        vec_rot = _quat_rotate(q, vec)
        rotated.append(tuple(map(float, target_pos + vec_rot)))
    return rotated


def _compute_path_length(path: list[tuple[float, float, float]]) -> float:
    if len(path) < 2:
        return 0.0
    arr = np.array(path, dtype=float)
    return float(np.sum(np.linalg.norm(arr[1:] - arr[:-1], axis=1)))


def _build_segment_path(
    start: np.ndarray,
    end: np.ndarray,
    obstacles: Sequence[MissionObstacle],
    step_size: float,
    margin: float,
) -> list[tuple[float, float, float]]:
    return build_point_to_point_path(
        waypoints=[tuple(map(float, start)), tuple(map(float, end))],
        obstacles=obstacles,
        step_size=step_size,
        safety_margin=margin,
    )


def _normalize_frame(frame: Any) -> str:
    if hasattr(frame, "value"):
        return str(frame.value).upper()
    return str(frame).upper()


def _mission_uses_lvlh(mission: MissionDefinition) -> bool:
    if _normalize_frame(getattr(mission.start_pose, "frame", "ECI")) == "LVLH":
        return True
    for segment in mission.segments:
        if segment.type == SegmentType.TRANSFER:
            frame = getattr(segment.end_pose, "frame", "ECI")
            if _normalize_frame(frame) == "LVLH":
                return True
        if segment.type == SegmentType.SCAN:
            frame = getattr(segment.scan, "frame", "ECI")
            if _normalize_frame(frame) == "LVLH":
                return True
    return False


def _resolve_reference_origin(mission: MissionDefinition) -> np.ndarray:
    target_id = getattr(mission, "start_target_id", None)
    if target_id:
        for segment in mission.segments:
            if (
                segment.type == SegmentType.SCAN
                and segment.target_id == target_id
                and segment.target_pose
            ):
                return np.array(segment.target_pose.position, dtype=float)
    for segment in mission.segments:
        if segment.type == SegmentType.SCAN and segment.target_pose:
            return np.array(segment.target_pose.position, dtype=float)
    return np.zeros(3, dtype=float)


def _convert_position(
    position: Sequence[float],
    source_frame: str,
    target_frame: str,
    origin: np.ndarray,
) -> np.ndarray:
    src = _normalize_frame(source_frame)
    dst = _normalize_frame(target_frame)
    pos = np.array(position, dtype=float)
    if src == dst:
        return pos
    if src == "ECI" and dst == "LVLH":
        return pos - origin
    if src == "LVLH" and dst == "ECI":
        return pos + origin
    return pos


def compile_unified_mission_path(
    mission: MissionDefinition,
    sim_config: SimulationConfig,
    output_frame: str | None = None,
) -> tuple[list[tuple[float, float, float]], float, float]:
    """
    Convert a unified mission into a single MPCC path.

    Returns:
        path, path_length, path_speed
    """
    if not mission.segments:
        start = tuple(mission.start_pose.position)
        return [start], 0.0, float(sim_config.app_config.mpc.path_speed)

    frame_mode = (
        output_frame.upper()
        if output_frame is not None
        else ("LVLH" if _mission_uses_lvlh(mission) else "ECI")
    )
    origin = _resolve_reference_origin(mission)

    start_pos = _convert_position(
        mission.start_pose.position,
        getattr(mission.start_pose, "frame", "ECI"),
        frame_mode,
        origin,
    )

    path: list[tuple[float, float, float]] = [tuple(start_pos)]
    current = np.array(path[-1], dtype=float)
    obstacles = mission.obstacles
    margin = float(sim_config.app_config.mpc.obstacle_margin)

    # Choose a conservative path speed based on constraints
    speed_candidates = []
    for segment in mission.segments:
        if segment.constraints and segment.constraints.speed_max:
            speed_candidates.append(float(segment.constraints.speed_max))
    path_speed = (
        min(speed_candidates)
        if speed_candidates
        else float(sim_config.app_config.mpc.path_speed)
    )

    for segment in mission.segments:
        if segment.type == SegmentType.TRANSFER:
            end = _convert_position(
                segment.end_pose.position,
                getattr(segment.end_pose, "frame", "ECI"),
                frame_mode,
                origin,
            )
            dist = np.linalg.norm(end - current)
            # Dynamic step size: clamp between 0.1m and 100m, target ~1000 points per segment if long
            step_size = max(0.1, min(100.0, dist / 1000.0))

            seg_path = _build_segment_path(
                start=current,
                end=end,
                obstacles=obstacles,
                step_size=step_size,
                margin=margin,
            )
            if seg_path:
                path.extend(seg_path[1:])
            current = end

        elif segment.type == SegmentType.SCAN:
            scan = segment.scan
            scan_frame = _normalize_frame(getattr(scan, "frame", "ECI"))

            target_pos = np.zeros(3, dtype=float)
            if scan_frame == "ECI":
                target_pos = (
                    np.array(segment.target_pose.position, dtype=float)
                    if segment.target_pose
                    else np.zeros(3, dtype=float)
                )
                if frame_mode == "LVLH":
                    target_pos = target_pos - origin
            elif scan_frame == "LVLH":
                # LVLH scan is defined around the local origin.
                target_pos = np.zeros(3, dtype=float)

            target_orientation = None
            if (
                frame_mode == "ECI"
                and scan_frame == "ECI"
                and segment.target_pose
                and segment.target_pose.orientation is not None
            ):
                target_orientation = list(segment.target_pose.orientation)
            obj_path = _resolve_target_obj_path(segment.target_id)

            scan_path: list[tuple[float, float, float]] = []
            apply_orientation = False
            asset_id = getattr(segment, "path_asset", None)
            if asset_id:
                scan_path, apply_orientation = _build_asset_path(asset_id, target_pos)
            if not scan_path:
                scan_path = _build_scan_path(
                    target_pos=target_pos,
                    obj_path=obj_path,
                    scan=scan,
                )
                apply_orientation = True

            if scan_path and apply_orientation and target_orientation:
                scan_path = _apply_target_orientation(
                    scan_path,
                    target_pos=target_pos,
                    target_orientation=target_orientation,
                )

            if scan_path and scan_frame == "LVLH" and frame_mode == "ECI":
                scan_path = [
                    tuple(map(float, np.array(p, dtype=float) + origin))
                    for p in scan_path
                ]

            if scan_path:
                start_p = current
                end_p = np.array(scan_path[0], dtype=float)
                dist_conn = np.linalg.norm(end_p - start_p)
                step_size_conn = max(0.1, min(100.0, dist_conn / 1000.0))

                connect = _build_segment_path(
                    start=start_p,
                    end=end_p,
                    obstacles=obstacles,
                    step_size=step_size_conn,
                    margin=margin,
                )
                if connect:
                    path.extend(connect[1:])
                path.extend(scan_path[1:])
                current = np.array(scan_path[-1], dtype=float)

        elif segment.type == SegmentType.HOLD:
            # Hold by repeating current position
            path.append(tuple(current))

    path_length = _compute_path_length(path)
    return path, path_length, path_speed

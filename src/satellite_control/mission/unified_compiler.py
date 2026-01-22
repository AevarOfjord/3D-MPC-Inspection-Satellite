"""
Unified mission compiler.

Converts unified mission segments into a single MPCC path suitable for the
existing MPC path-following pipeline.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple, Optional, Sequence

import numpy as np

from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.mission.mesh_scan import (
    build_cylinder_scan_trajectory,
    build_mesh_scan_trajectory,
    compute_mesh_bounds,
    compute_scan_sampling,
    load_obj_vertices,
)
from src.satellite_control.mission.path_following import build_point_to_point_path
from src.satellite_control.mission.unified_mission import MissionDefinition, SegmentType, MissionObstacle


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _resolve_target_obj_path(target_id: str) -> Optional[Path]:
    if not target_id:
        return None
    upper = target_id.upper()
    root = _repo_root()
    if "ISS" in upper:
        return root / "OBJ_files" / "ISS.obj"
    if "STARLINK" in upper:
        return root / "OBJ_files" / "starlink.obj"
    return None


def _axis_to_scan_axis(axis: str) -> str:
    axis = axis.upper()
    if "X" in axis:
        return "X"
    if "Y" in axis:
        return "Y"
    return "Z"


def _compute_path_length(path: List[Tuple[float, float, float]]) -> float:
    if len(path) < 2:
        return 0.0
    arr = np.array(path, dtype=float)
    return float(np.sum(np.linalg.norm(arr[1:] - arr[:-1], axis=1)))


def _distance_point_to_segment(
    point: np.ndarray, start: np.ndarray, end: np.ndarray
) -> float:
    segment = end - start
    seg_len_sq = float(np.dot(segment, segment))
    if seg_len_sq < 1e-12:
        return float(np.linalg.norm(point - start))
    t = float(np.dot(point - start, segment) / seg_len_sq)
    t = max(0.0, min(1.0, t))
    proj = start + t * segment
    return float(np.linalg.norm(point - proj))


def _segment_intersects_sphere(
    start: np.ndarray,
    end: np.ndarray,
    center: np.ndarray,
    radius: float,
    margin: float,
) -> bool:
    dist = _distance_point_to_segment(center, start, end)
    return dist <= float(radius + margin)


def _compute_detour_point(
    start: np.ndarray,
    end: np.ndarray,
    center: np.ndarray,
    radius: float,
    margin: float,
) -> np.ndarray:
    direction = end - start
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return center + np.array([radius + margin, 0.0, 0.0], dtype=float)
    unit_dir = direction / norm
    axis = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(unit_dir[2])) > 0.9:
        axis = np.array([0.0, 1.0, 0.0], dtype=float)
    perp = np.cross(unit_dir, axis)
    perp_norm = float(np.linalg.norm(perp))
    if perp_norm < 1e-9:
        perp = np.array([1.0, 0.0, 0.0], dtype=float)
    else:
        perp = perp / perp_norm
    return center + perp * (radius + margin)


def _build_segment_path(
    start: np.ndarray,
    end: np.ndarray,
    obstacles: Sequence[MissionObstacle],
    step_size: float,
    margin: float,
) -> List[Tuple[float, float, float]]:
    waypoints: List[Tuple[float, float, float]] = [
        tuple(map(float, start)),
        tuple(map(float, end)),
    ]
    for obstacle in obstacles:
        center = np.array(obstacle.position, dtype=float)
        if _segment_intersects_sphere(start, end, center, obstacle.radius, margin):
            detour = _compute_detour_point(start, end, center, obstacle.radius, margin)
            waypoints = [
                tuple(map(float, start)),
                tuple(map(float, detour)),
                tuple(map(float, end)),
            ]
            break

    return build_point_to_point_path(
        waypoints=waypoints,
        obstacles=None,
        step_size=step_size,
    )


def compile_unified_mission_path(
    mission: MissionDefinition,
    sim_config: SimulationConfig,
) -> Tuple[List[Tuple[float, float, float]], float, float]:
    """
    Convert a unified mission into a single MPCC path.

    Returns:
        path, path_length, path_speed
    """
    if not mission.segments:
        start = tuple(mission.start_pose.position)
        return [start], 0.0, float(sim_config.app_config.mpc.path_speed)

    path: List[Tuple[float, float, float]] = [tuple(mission.start_pose.position)]
    current = np.array(path[-1], dtype=float)
    obstacles = mission.obstacles
    margin = float(sim_config.app_config.mpc.obstacle_margin)

    # Choose a conservative path speed based on constraints
    speed_candidates = []
    for segment in mission.segments:
        if segment.constraints and segment.constraints.speed_max:
            speed_candidates.append(float(segment.constraints.speed_max))
    path_speed = min(speed_candidates) if speed_candidates else float(
        sim_config.app_config.mpc.path_speed
    )

    for segment in mission.segments:
        if segment.type == SegmentType.TRANSFER:
            end = np.array(segment.end_pose.position, dtype=float)
            seg_path = _build_segment_path(
                start=current,
                end=end,
                obstacles=obstacles,
                step_size=0.1,
                margin=margin,
            )
            if seg_path:
                path.extend(seg_path[1:])
            current = end

        elif segment.type == SegmentType.SCAN:
            scan = segment.scan
            target_pos = (
                np.array(segment.target_pose.position, dtype=float)
                if segment.target_pose
                else np.zeros(3, dtype=float)
            )
            obj_path = _resolve_target_obj_path(segment.target_id)

            v_max = (
                float(segment.constraints.speed_max)
                if segment.constraints and segment.constraints.speed_max
                else float(sim_config.app_config.mpc.path_speed)
            )
            v_min = max(0.05, v_max * 0.25)
            lateral_accel = float(sim_config.mission_state.mesh_scan_lateral_accel)
            dt = float(sim_config.app_config.mpc.dt)
            scan_axis = _axis_to_scan_axis(scan.axis)

            scan_path: List[Tuple[float, float, float]] = []

            if obj_path and obj_path.exists():
                vertices = load_obj_vertices(str(obj_path))
                min_bounds, max_bounds, center, radius_xy = compute_mesh_bounds(vertices)
                height = float(max_bounds[2] - min_bounds[2])
                ring_step, points_per_ring = compute_scan_sampling(
                    radius=radius_xy,
                    standoff=scan.standoff,
                    fov_deg=scan.fov_deg,
                    overlap=scan.overlap,
                )
                if scan.pitch and scan.pitch > 0:
                    ring_step = float(scan.pitch)
                levels = max(1, int(max(height, 1e-3) / max(ring_step, 1e-3)))
                scan_path, _, _ = build_mesh_scan_trajectory(
                    obj_path=str(obj_path),
                    standoff=scan.standoff,
                    levels=levels,
                    points_per_circle=points_per_ring,
                    v_max=v_max,
                    v_min=v_min,
                    lateral_accel=lateral_accel,
                    dt=dt,
                    z_margin=0.0,
                    scan_axis=scan_axis,
                    build_trajectory=False,
                )
                offset = target_pos - center
                scan_path = [
                    tuple(map(float, np.array(p, dtype=float) + offset))
                    for p in scan_path
                ]
            else:
                # Fallback cylinder scan around target position
                radius = 0.5
                height = 1.0
                scan_path, _, _ = build_cylinder_scan_trajectory(
                    center=target_pos,
                    rotation_xyz=(0.0, 0.0, 0.0),
                    radius=radius,
                    height=height,
                    standoff=scan.standoff,
                    fov_deg=scan.fov_deg,
                    overlap=scan.overlap,
                    v_max=v_max,
                    v_min=v_min,
                    lateral_accel=lateral_accel,
                    dt=dt,
                    ring_shape="circle",
                    hold_start=0.0,
                    hold_end=0.0,
                    build_trajectory=False,
                )

            if scan_path:
                connect = _build_segment_path(
                    start=current,
                    end=np.array(scan_path[0], dtype=float),
                    obstacles=obstacles,
                    step_size=0.1,
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

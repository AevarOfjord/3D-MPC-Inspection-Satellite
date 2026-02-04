"""
Point-to-Point Path Mission Utilities

Builds a continuous path through waypoints and converts it into a
time-parameterized trajectory for MPC tracking.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from src.satellite_control.config.constants import Constants
from src.satellite_control.mission.trajectory_utils import (
    apply_hold_segments,
    build_time_parameterized_trajectory,
    compute_curvature,
    compute_speed_profile,
)


def _interpolate_segment(
    start: np.ndarray, end: np.ndarray, step_size: float
) -> List[Tuple[float, float, float]]:
    distance = float(np.linalg.norm(end - start))
    if distance < 1e-9:
        return [tuple(map(float, end))]
    steps = max(2, int(np.ceil(distance / max(step_size, 1e-3))))
    points = [
        start + (end - start) * (i / steps) for i in range(1, steps + 1)
    ]
    return [tuple(map(float, p)) for p in points]


def _normalize_obstacles(
    obstacles: Optional[Sequence[object]],
) -> List[Tuple[np.ndarray, float]]:
    normalized: List[Tuple[np.ndarray, float]] = []
    if not obstacles:
        return normalized
    for obs in obstacles:
        if obs is None:
            continue
        if hasattr(obs, "position") and hasattr(obs, "radius"):
            center = np.array(getattr(obs, "position"), dtype=float)
            radius = float(getattr(obs, "radius"))
        elif isinstance(obs, dict):
            center = np.array(obs.get("position", [0.0, 0.0, 0.0]), dtype=float)
            radius = float(obs.get("radius", 0.0))
        else:
            try:
                obs_tuple = tuple(obs)
            except TypeError:
                continue
            if len(obs_tuple) < 4:
                continue
            center = np.array(obs_tuple[:3], dtype=float)
            radius = float(obs_tuple[3])

        if center.shape[0] < 3:
            center = np.pad(center, (0, 3 - center.shape[0]), "constant")
        elif center.shape[0] > 3:
            center = center[:3]
        if radius > 0.0:
            normalized.append((center, radius))
    return normalized


def _effective_obstacle_radius(
    radius: float,
    safety_margin: float,
    turning_margin: float,
) -> float:
    return float(max(radius + safety_margin + turning_margin, 1e-6))


def _segment_distance_and_t(
    start: np.ndarray, end: np.ndarray, point: np.ndarray
) -> Tuple[float, float]:
    seg = end - start
    seg_len_sq = float(np.dot(seg, seg))
    if seg_len_sq < 1e-12:
        return float(np.linalg.norm(point - start)), 0.0
    t = float(np.dot(point - start, seg) / seg_len_sq)
    t = min(max(t, 0.0), 1.0)
    closest = start + t * seg
    return float(np.linalg.norm(point - closest)), t


def _find_first_blocking_obstacle(
    start: np.ndarray,
    end: np.ndarray,
    obstacles: List[Tuple[np.ndarray, float]],
    safety_margin: float,
    turning_margin: float,
) -> Optional[Tuple[np.ndarray, float, float]]:
    best_t = 1.0
    best: Optional[Tuple[np.ndarray, float, float]] = None
    for center, radius in obstacles:
        r_eff = _effective_obstacle_radius(radius, safety_margin, turning_margin)
        dist, t = _segment_distance_and_t(start, end, center)
        if dist <= r_eff and t <= best_t:
            best_t = t
            best = (center, radius, r_eff)
    return best


def _compute_tangent_angles(point: np.ndarray, radius: float) -> Optional[Tuple[float, float]]:
    d = float(np.linalg.norm(point))
    if d <= radius:
        return None
    theta = float(np.arctan2(point[1], point[0]))
    ratio = max(min(radius / d, 1.0), -1.0)
    phi = float(np.arccos(ratio))
    return theta - phi, theta + phi  # cw, ccw


def _build_arc_detour(
    start: np.ndarray,
    end: np.ndarray,
    center: np.ndarray,
    radius: float,
    step_size: float,
    safety_margin: float,
    turning_margin: float,
) -> List[Tuple[float, float, float]]:
    seg = end - start
    seg_len = float(np.linalg.norm(seg))
    if seg_len < 1e-9:
        return [tuple(map(float, start))]

    r_eff = _effective_obstacle_radius(radius, safety_margin, turning_margin)

    e1 = seg / seg_len
    to_center = center - start
    proj = float(np.dot(to_center, e1))
    closest = start + proj * e1
    perp = center - closest
    perp_norm = float(np.linalg.norm(perp))
    if perp_norm < 1e-9:
        ref = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(e1, ref))) > 0.9:
            ref = np.array([0.0, 1.0, 0.0])
        perp = np.cross(e1, ref)
        perp_norm = float(np.linalg.norm(perp))
        if perp_norm < 1e-9:
            perp = np.array([1.0, 0.0, 0.0])
            perp_norm = 1.0
    e2 = perp / perp_norm

    s2 = np.array([np.dot(start - center, e1), np.dot(start - center, e2)], dtype=float)
    e2d = np.array([np.dot(end - center, e1), np.dot(end - center, e2)], dtype=float)

    s_angles = _compute_tangent_angles(s2, r_eff)
    e_angles = _compute_tangent_angles(e2d, r_eff)
    if s_angles is None or e_angles is None:
        return [tuple(map(float, start))] + _interpolate_segment(start, end, step_size)

    s_cw, s_ccw = s_angles
    e_cw, e_ccw = e_angles

    def _arc_length(theta_start: float, theta_end: float, direction: str) -> float:
        if direction == "cw":
            delta = (theta_start - theta_end) % (2.0 * np.pi)
        else:
            delta = (theta_end - theta_start) % (2.0 * np.pi)
        return float(r_eff * delta)

    def _total_length(theta_s: float, theta_e: float, direction: str) -> float:
        ts = np.array([r_eff * np.cos(theta_s), r_eff * np.sin(theta_s)], dtype=float)
        te = np.array([r_eff * np.cos(theta_e), r_eff * np.sin(theta_e)], dtype=float)
        line_s = float(np.linalg.norm(s2 - ts))
        line_e = float(np.linalg.norm(e2d - te))
        return line_s + line_e + _arc_length(theta_s, theta_e, direction)

    cw_len = _total_length(s_cw, e_cw, "cw")
    ccw_len = _total_length(s_ccw, e_ccw, "ccw")

    if cw_len <= ccw_len:
        theta_s = s_cw
        theta_e = e_cw
        direction = "cw"
    else:
        theta_s = s_ccw
        theta_e = e_ccw
        direction = "ccw"

    ts = np.array([r_eff * np.cos(theta_s), r_eff * np.sin(theta_s)], dtype=float)
    te = np.array([r_eff * np.cos(theta_e), r_eff * np.sin(theta_e)], dtype=float)

    ts_3d = center + e1 * ts[0] + e2 * ts[1]
    te_3d = center + e1 * te[0] + e2 * te[1]

    arc_len = _arc_length(theta_s, theta_e, direction)
    arc_steps = max(2, int(np.ceil(arc_len / max(step_size, 1e-3))))
    if direction == "cw":
        angles = np.linspace(theta_s, theta_s - arc_len / r_eff, arc_steps)
    else:
        angles = np.linspace(theta_s, theta_s + arc_len / r_eff, arc_steps)

    arc_points = [
        tuple(
            map(
                float,
                center
                + e1 * (r_eff * np.cos(a))
                + e2 * (r_eff * np.sin(a)),
            )
        )
        for a in angles
    ]

    path: List[Tuple[float, float, float]] = [tuple(map(float, start))]
    path.extend(_interpolate_segment(start, ts_3d, step_size))
    if arc_points:
        if path and np.linalg.norm(np.array(path[-1]) - np.array(arc_points[0])) < 1e-6:
            path.extend(arc_points[1:])
        else:
            path.extend(arc_points)
    path.extend(_interpolate_segment(te_3d, end, step_size))
    return path


def _densify_path(
    path: List[Tuple[float, float, float]],
    step_size: float,
) -> List[Tuple[float, float, float]]:
    if len(path) < 2:
        return path
    dense: List[Tuple[float, float, float]] = [path[0]]
    threshold = float(step_size) * 1.05
    for i in range(len(path) - 1):
        seg_start = np.array(path[i], dtype=float)
        seg_end = np.array(path[i + 1], dtype=float)
        seg_len = float(np.linalg.norm(seg_end - seg_start))
        if seg_len <= threshold:
            dense.append(tuple(map(float, seg_end)))
        else:
            dense.extend(_interpolate_segment(seg_start, seg_end, step_size))
    return dense


def build_point_to_point_path(
    waypoints: Sequence[Iterable[float]],
    obstacles: Optional[Sequence[Tuple[float, float, float, float]]] = None,
    step_size: float = 0.1,
    rrt_step: float = 0.5,
    rrt_max_iter: int = 800,
    safety_margin: Optional[float] = None,
    turning_margin: Optional[float] = None,
    max_passes: int = 6,
) -> List[Tuple[float, float, float]]:
    """Generate a path through waypoints with optional spherical obstacle detours."""
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required.")

    points = []
    for p in waypoints:
        arr = np.array(p, dtype=float)
        if arr.shape == (2,):
            arr = np.pad(arr, (0, 1), "constant")
        if arr.shape[0] > 3:
            arr = arr[:3]
        points.append(arr)
    path: List[Tuple[float, float, float]] = [tuple(map(float, points[0]))]

    safety_margin = (
        float(Constants.OBSTACLE_SAFETY_MARGIN)
        if safety_margin is None
        else float(safety_margin)
    )
    turning_margin = (
        float(Constants.OBSTACLE_TURNING_MARGIN)
        if turning_margin is None
        else float(turning_margin)
    )
    normalized_obstacles = _normalize_obstacles(obstacles)

    for idx in range(len(points) - 1):
        start = points[idx]
        end = points[idx + 1]
        if not normalized_obstacles:
            segment_points = _interpolate_segment(start, end, step_size)
            path.extend(segment_points)
            continue

        segment_path: List[Tuple[float, float, float]] = [
            tuple(map(float, start)),
            tuple(map(float, end)),
        ]
        passes = 0
        while passes < max_passes:
            updated = False
            new_path: List[Tuple[float, float, float]] = [segment_path[0]]
            for i in range(len(segment_path) - 1):
                seg_start = np.array(segment_path[i], dtype=float)
                seg_end = np.array(segment_path[i + 1], dtype=float)
                hit = _find_first_blocking_obstacle(
                    seg_start,
                    seg_end,
                    normalized_obstacles,
                    safety_margin,
                    turning_margin,
                )
                if hit is None:
                    new_path.append(tuple(map(float, seg_end)))
                    continue

                updated = True
                center, radius, _ = hit
                detour_points = _build_arc_detour(
                    seg_start,
                    seg_end,
                    center,
                    radius,
                    step_size,
                    safety_margin,
                    turning_margin,
                )
                if len(detour_points) <= 2:
                    new_path.append(tuple(map(float, seg_end)))
                else:
                    if (
                        new_path
                        and np.linalg.norm(
                            np.array(new_path[-1]) - np.array(detour_points[0])
                        )
                        < 1e-6
                    ):
                        new_path.extend(detour_points[1:])
                    else:
                        new_path.extend(detour_points)
            segment_path = new_path
            passes += 1
            if not updated:
                break

        segment_path = _densify_path(segment_path, step_size)

        path.extend(segment_path[1:])

    return path


def build_point_to_point_trajectory(
    waypoints: Sequence[Iterable[float]],
    obstacles: Optional[Sequence[Tuple[float, float, float, float]]],
    v_max: float,
    v_min: float,
    lateral_accel: float,
    dt: float,
    hold_start: float,
    hold_end: float,
    step_size: float = 0.1,
) -> Tuple[List[Tuple[float, float, float]], np.ndarray, float]:
    """Build path and time-parameterized trajectory for point-to-point mission."""
    path = build_point_to_point_path(
        waypoints=waypoints,
        obstacles=obstacles,
        step_size=step_size,
    )
    path_arr = np.array(path, dtype=float)
    curvature = compute_curvature(path_arr)
    speeds = compute_speed_profile(curvature, v_max, v_min, lateral_accel)
    trajectory, total_time = build_time_parameterized_trajectory(path_arr, speeds, dt)
    trajectory, total_time = apply_hold_segments(
        trajectory, dt=dt, hold_start=hold_start, hold_end=hold_end
    )
    return path, trajectory, total_time

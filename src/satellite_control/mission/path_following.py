"""
Point-to-Point Path Mission Utilities

Builds a continuous path through waypoints and converts it into a
time-parameterized trajectory for MPC tracking.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

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


def build_point_to_point_path(
    waypoints: Sequence[Iterable[float]],
    obstacles: Optional[Sequence[Tuple[float, float, float, float]]] = None,
    step_size: float = 0.1,
    rrt_step: float = 0.5,
    rrt_max_iter: int = 800,
) -> List[Tuple[float, float, float]]:
    """Generate a straight-line path through waypoints (no path planning)."""
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

    for idx in range(len(points) - 1):
        start = points[idx]
        end = points[idx + 1]
        segment_points = _interpolate_segment(start, end, step_size)

        path.extend(segment_points)

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

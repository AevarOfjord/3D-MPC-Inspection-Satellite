"""
Point-to-point path mission utilities.

Builds a continuous path through waypoints and converts it into a
time-parameterized trajectory for MPC tracking.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence

import numpy as np

from controller.shared.python.mission.trajectory_utils import (
    apply_hold_segments,
    build_time_parameterized_trajectory,
    compute_curvature,
    compute_speed_profile,
)


def _interpolate_segment(
    start: np.ndarray, end: np.ndarray, step_size: float
) -> list[tuple[float, float, float]]:
    distance = float(np.linalg.norm(end - start))
    if distance < 1e-9:
        return [tuple(map(float, end))]
    steps = max(2, int(np.ceil(distance / max(step_size, 1e-3))))
    points = [start + (end - start) * (i / steps) for i in range(1, steps + 1)]
    return [tuple(map(float, p)) for p in points]


def _to_xyz(point: Iterable[float]) -> np.ndarray:
    arr = np.array(point, dtype=float)
    if arr.shape == (2,):
        arr = np.pad(arr, (0, 1), "constant")
    if arr.shape[0] > 3:
        arr = arr[:3]
    return arr


def build_point_to_point_path(
    waypoints: Sequence[Iterable[float]],
    step_size: float = 0.1,
) -> list[tuple[float, float, float]]:
    """Generate a dense polyline path through waypoints."""
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required.")

    points = [_to_xyz(p) for p in waypoints]
    path: list[tuple[float, float, float]] = [tuple(map(float, points[0]))]
    for idx in range(len(points) - 1):
        segment = _interpolate_segment(points[idx], points[idx + 1], step_size)
        path.extend(segment)
    return path


def build_point_to_point_trajectory(
    waypoints: Sequence[Iterable[float]],
    v_max: float,
    v_min: float,
    lateral_accel: float,
    dt: float,
    hold_start: float,
    hold_end: float,
    step_size: float = 0.1,
) -> tuple[list[tuple[float, float, float]], np.ndarray, float]:
    """
    Build path and time-parameterized trajectory for point-to-point mission.
    """
    path = build_point_to_point_path(waypoints=waypoints, step_size=step_size)
    path_arr = np.array(path, dtype=float)
    curvature = compute_curvature(path_arr)
    speeds = compute_speed_profile(curvature, v_max, v_min, lateral_accel)
    trajectory, total_time = build_time_parameterized_trajectory(path_arr, speeds, dt)
    trajectory, total_time = apply_hold_segments(
        trajectory, dt=dt, hold_start=hold_start, hold_end=hold_end
    )
    return path, trajectory, total_time

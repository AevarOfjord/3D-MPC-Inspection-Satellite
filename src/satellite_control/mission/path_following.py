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
from src.satellite_control.planning.rrt_star import Obstacle, RRTStarPlanner


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
    """Generate a path through waypoints with optional RRT* obstacle avoidance."""
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required.")

    points = [np.array(p, dtype=float) for p in waypoints]
    path: List[Tuple[float, float, float]] = [tuple(map(float, points[0]))]

    has_obstacles = bool(obstacles)
    rrt_obstacles: List[Obstacle] = []
    if has_obstacles:
        for obs in obstacles or []:
            rrt_obstacles.append(Obstacle(position=np.array(obs[:3], dtype=float), radius=float(obs[3])))

    bounds_min = np.min(points, axis=0)
    bounds_max = np.max(points, axis=0)
    if has_obstacles:
        obs_positions = np.array([o.position for o in rrt_obstacles], dtype=float)
        if obs_positions.size:
            bounds_min = np.minimum(bounds_min, obs_positions.min(axis=0))
            bounds_max = np.maximum(bounds_max, obs_positions.max(axis=0))
    margin = 2.0
    bounds_min = bounds_min - margin
    bounds_max = bounds_max + margin

    planner = RRTStarPlanner(
        bounds_min=tuple(bounds_min),
        bounds_max=tuple(bounds_max),
        step_size=rrt_step,
        max_iter=rrt_max_iter,
        search_radius=1.5,
    )

    for idx in range(len(points) - 1):
        start = points[idx]
        end = points[idx + 1]
        segment_points: List[Tuple[float, float, float]]

        if has_obstacles:
            rrt_path = planner.plan(start, end, rrt_obstacles)
            if not rrt_path:
                segment_points = _interpolate_segment(start, end, step_size)
            else:
                segment_points = []
                prev = start
                for node in rrt_path:
                    node_arr = np.array(node, dtype=float)
                    segment_points.extend(_interpolate_segment(prev, node_arr, step_size))
                    prev = node_arr
        else:
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

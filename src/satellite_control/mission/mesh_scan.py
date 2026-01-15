"""
Mesh Scan Path Generator

Loads OBJ meshes and generates a cylindrical ring scan path with
curvature-based speed profiling for MPC trajectory tracking.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

import numpy as np


def load_obj_vertices(obj_path: str) -> np.ndarray:
    """Load vertex positions from an OBJ file."""
    vertices: List[List[float]] = []
    with open(obj_path, "r") as handle:
        for raw_line in handle:
            if not raw_line.startswith("v "):
                continue
            parts = raw_line.strip().split()
            if len(parts) < 4:
                continue
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            vertices.append([x, y, z])

    if not vertices:
        raise ValueError(f"No vertices found in OBJ: {obj_path}")

    return np.array(vertices, dtype=float)


def compute_mesh_bounds(vertices: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Return (min_bounds, max_bounds, center, max_xy_radius)."""
    min_bounds = vertices.min(axis=0)
    max_bounds = vertices.max(axis=0)
    center = (min_bounds + max_bounds) / 2.0
    offsets = vertices[:, :2] - center[:2]
    radius_xy = float(np.max(np.linalg.norm(offsets, axis=1)))
    return min_bounds, max_bounds, center, radius_xy


def generate_cylindrical_scan_path(
    center: Iterable[float],
    radius: float,
    z_levels: Iterable[float],
    points_per_circle: int,
) -> List[Tuple[float, float, float]]:
    """Generate stacked circular rings with vertical transitions."""
    points_per_circle = max(int(points_per_circle), 6)
    cx, cy, _cz = center
    z_vals = list(z_levels)
    if not z_vals:
        raise ValueError("z_levels must contain at least one level")

    path: List[Tuple[float, float, float]] = []
    for idx, z in enumerate(z_vals):
        for i in range(points_per_circle + 1):
            angle = 2.0 * np.pi * (i / points_per_circle)
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            path.append((float(x), float(y), float(z)))

        if idx < len(z_vals) - 1:
            next_z = z_vals[idx + 1]
            path.append((float(cx + radius), float(cy), float(next_z)))

    return path


def compute_curvature(path: np.ndarray) -> np.ndarray:
    """Estimate discrete curvature at each path point."""
    if path.shape[0] < 3:
        return np.zeros(path.shape[0], dtype=float)

    curvature = np.zeros(path.shape[0], dtype=float)
    for i in range(1, path.shape[0] - 1):
        p0 = path[i - 1]
        p1 = path[i]
        p2 = path[i + 1]
        a = p1 - p0
        b = p2 - p1
        c = p2 - p0
        denom = np.linalg.norm(a) * np.linalg.norm(b) * np.linalg.norm(c)
        if denom < 1e-9:
            curvature[i] = 0.0
            continue
        curvature[i] = 2.0 * np.linalg.norm(np.cross(a, b)) / denom
    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]
    return curvature


def compute_speed_profile(
    curvature: np.ndarray,
    v_max: float,
    v_min: float,
    lateral_accel: float,
) -> np.ndarray:
    """Compute curvature-limited speeds."""
    v_max = float(max(v_max, v_min))
    v_min = float(max(v_min, 0.0))
    lateral_accel = float(max(lateral_accel, 1e-6))
    speeds = np.zeros_like(curvature, dtype=float)
    for i, kappa in enumerate(curvature):
        if kappa < 1e-6:
            speeds[i] = v_max
        else:
            speeds[i] = min(v_max, np.sqrt(lateral_accel / kappa))
        speeds[i] = max(speeds[i], v_min)
    return speeds


def build_time_parameterized_trajectory(
    path: np.ndarray,
    speeds: np.ndarray,
    dt: float,
) -> Tuple[np.ndarray, float]:
    """Convert path + speed profile into [t, x, y, z, vx, vy, vz] samples."""
    if path.shape[0] < 2:
        return np.zeros((0, 7), dtype=float), 0.0

    dt = float(dt) if dt and dt > 0 else 0.05
    segment_vectors = path[1:] - path[:-1]
    segment_lengths = np.linalg.norm(segment_vectors, axis=1)
    segment_speeds = np.minimum(speeds[:-1], speeds[1:])
    segment_times = np.zeros_like(segment_lengths)
    for i, length in enumerate(segment_lengths):
        if length < 1e-8 or segment_speeds[i] < 1e-6:
            segment_times[i] = 0.0
        else:
            segment_times[i] = length / segment_speeds[i]

    segment_end_times = np.cumsum(segment_times)
    total_time = float(segment_end_times[-1]) if segment_end_times.size else 0.0
    if total_time <= 0.0:
        return np.zeros((0, 7), dtype=float), 0.0

    sample_times = np.arange(0.0, total_time + dt * 0.5, dt)
    trajectory = np.zeros((sample_times.size, 7), dtype=float)
    for i, t in enumerate(sample_times):
        seg_idx = int(np.searchsorted(segment_end_times, t, side="right"))
        seg_idx = min(seg_idx, len(segment_lengths) - 1)
        seg_start_time = segment_end_times[seg_idx - 1] if seg_idx > 0 else 0.0
        seg_time = segment_times[seg_idx]
        if seg_time <= 1e-9:
            position = path[seg_idx].copy()
            velocity = np.zeros(3, dtype=float)
        else:
            alpha = (t - seg_start_time) / seg_time
            alpha = min(max(alpha, 0.0), 1.0)
            position = path[seg_idx] + alpha * segment_vectors[seg_idx]
            direction = (
                segment_vectors[seg_idx] / segment_lengths[seg_idx]
                if segment_lengths[seg_idx] > 1e-9
                else np.zeros(3, dtype=float)
            )
            velocity = direction * segment_speeds[seg_idx]

        trajectory[i, 0] = t
        trajectory[i, 1:4] = position
        trajectory[i, 4:7] = velocity

    trajectory[-1, 4:7] = 0.0
    return trajectory, total_time


def build_mesh_scan_trajectory(
    obj_path: str,
    standoff: float,
    levels: int,
    points_per_circle: int,
    v_max: float,
    v_min: float,
    lateral_accel: float,
    dt: float,
    z_margin: float = 0.0,
) -> Tuple[List[Tuple[float, float, float]], np.ndarray, float]:
    """Generate scan path and time-parameterized trajectory."""
    vertices = load_obj_vertices(obj_path)
    min_bounds, max_bounds, center, radius_xy = compute_mesh_bounds(vertices)

    levels = max(int(levels), 1)
    z_min = float(min_bounds[2] + z_margin)
    z_max = float(max_bounds[2] - z_margin)
    if levels == 1:
        z_levels = [0.5 * (z_min + z_max)]
    else:
        z_levels = list(np.linspace(z_min, z_max, levels))

    scan_radius = float(radius_xy + max(standoff, 0.0))
    path = generate_cylindrical_scan_path(
        center=center,
        radius=scan_radius,
        z_levels=z_levels,
        points_per_circle=points_per_circle,
    )
    path_arr = np.array(path, dtype=float)
    curvature = compute_curvature(path_arr)
    speeds = compute_speed_profile(curvature, v_max, v_min, lateral_accel)
    trajectory, total_time = build_time_parameterized_trajectory(path_arr, speeds, dt)
    path_length = float(np.sum(np.linalg.norm(path_arr[1:] - path_arr[:-1], axis=1)))
    return path, trajectory, path_length

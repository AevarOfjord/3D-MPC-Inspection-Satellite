"""
Mesh Scan Path Generator

Loads OBJ meshes and generates a cylindrical ring scan path with
curvature-based speed profiling for MPC trajectory tracking.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

import numpy as np

from src.satellite_control.mission.trajectory_utils import (
    apply_hold_segments,
    build_time_parameterized_trajectory,
    compute_curvature,
    compute_speed_profile,
)

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


def _generate_circle_ring(
    center: Iterable[float],
    radius: float,
    z: float,
    points_per_ring: int,
) -> List[Tuple[float, float, float]]:
    points_per_ring = max(int(points_per_ring), 6)
    cx, cy, _cz = center
    ring: List[Tuple[float, float, float]] = []
    for i in range(points_per_ring + 1):
        angle = 2.0 * np.pi * (i / points_per_ring)
        x = cx + radius * np.cos(angle)
        y = cy + radius * np.sin(angle)
        ring.append((float(x), float(y), float(z)))
    return ring


def _generate_square_ring(
    center: Iterable[float],
    radius: float,
    z: float,
    points_per_ring: int,
) -> List[Tuple[float, float, float]]:
    """Generate a square ring around the center with side length 2*radius."""
    points_per_ring = max(int(points_per_ring), 4)
    points_per_side = max(2, int(np.ceil(points_per_ring / 4)))
    cx, cy, _cz = center
    r = float(radius)

    corners = np.array(
        [
            (cx + r, cy + r),
            (cx - r, cy + r),
            (cx - r, cy - r),
            (cx + r, cy - r),
            (cx + r, cy + r),
        ],
        dtype=float,
    )

    ring: List[Tuple[float, float, float]] = []
    for i in range(4):
        start = corners[i]
        end = corners[i + 1]
        for step in range(points_per_side):
            t = step / points_per_side
            xy = start + t * (end - start)
            ring.append((float(xy[0]), float(xy[1]), float(z)))
    ring.append((float(corners[-1][0]), float(corners[-1][1]), float(z)))
    return ring


def generate_cylindrical_scan_path(
    center: Iterable[float],
    radius: float,
    z_levels: Iterable[float],
    points_per_ring: int,
    ring_shape: str = "circle",
) -> List[Tuple[float, float, float]]:
    """Generate stacked rings (circle/square) with vertical transitions."""
    cx, cy, _cz = center
    z_vals = list(z_levels)
    if not z_vals:
        raise ValueError("z_levels must contain at least one level")

    shape = ring_shape.lower().strip()
    path: List[Tuple[float, float, float]] = []
    for idx, z in enumerate(z_vals):
        if shape == "square":
            ring = _generate_square_ring(center, radius, z, points_per_ring)
        else:
            ring = _generate_circle_ring(center, radius, z, points_per_ring)
        path.extend(ring)

        if idx < len(z_vals) - 1:
            next_z = z_vals[idx + 1]
            path.append((float(cx + radius), float(cy), float(next_z)))

    return path


def compute_scan_sampling(
    radius: float,
    standoff: float,
    fov_deg: float,
    overlap: float = 0.85,
) -> Tuple[float, int]:
    """Compute ring spacing and points-per-ring using FOV and standoff."""
    fov_rad = np.deg2rad(float(max(fov_deg, 1.0)))
    overlap = float(min(max(overlap, 0.1), 1.0))

    coverage = 2.0 * max(standoff, 1e-3) * np.tan(0.5 * fov_rad)
    ring_step = max(coverage * overlap, 0.01)

    arc_coverage = 2.0 * max(radius + standoff, 1e-3) * np.tan(0.5 * fov_rad)
    circumference = 2.0 * np.pi * max(radius + standoff, 1e-3)
    points_per_ring = max(12, int(np.ceil(circumference / max(arc_coverage * overlap, 1e-3))))
    return ring_step, points_per_ring


def _apply_pose(
    points: np.ndarray,
    position: Iterable[float],
    rotation_xyz: Iterable[float],
) -> np.ndarray:
    """Rotate + translate points by Euler XYZ rotation and position."""
    from scipy.spatial.transform import Rotation

    rot = Rotation.from_euler("xyz", rotation_xyz, degrees=False)
    rotated = rot.apply(points)
    pos = np.array(position, dtype=float)
    return rotated + pos


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


def build_cylinder_scan_trajectory(
    center: Iterable[float],
    rotation_xyz: Iterable[float],
    radius: float,
    height: float,
    standoff: float,
    fov_deg: float,
    overlap: float,
    v_max: float,
    v_min: float,
    lateral_accel: float,
    dt: float,
    ring_shape: str = "circle",
    hold_start: float = 0.0,
    hold_end: float = 0.0,
) -> Tuple[List[Tuple[float, float, float]], np.ndarray, float]:
    """Generate scan path and trajectory for a cylinder."""
    ring_step, points_per_ring = compute_scan_sampling(
        radius=radius,
        standoff=standoff,
        fov_deg=fov_deg,
        overlap=overlap,
    )

    total_height = float(max(height, 1e-3))
    z_min = -0.5 * total_height
    z_max = 0.5 * total_height
    z_levels = list(np.arange(z_min, z_max + ring_step * 0.5, ring_step))
    if len(z_levels) == 1 and total_height > 0:
        z_levels = [z_min, z_max]

    scan_radius = float(radius + max(standoff, 0.0))
    raw_path = generate_cylindrical_scan_path(
        center=(0.0, 0.0, 0.0),
        radius=scan_radius,
        z_levels=z_levels,
        points_per_ring=points_per_ring,
        ring_shape=ring_shape,
    )

    raw_path_arr = np.array(raw_path, dtype=float)
    path_arr = _apply_pose(raw_path_arr, position=center, rotation_xyz=rotation_xyz)

    curvature = compute_curvature(path_arr)
    speeds = compute_speed_profile(curvature, v_max, v_min, lateral_accel)
    trajectory, total_time = build_time_parameterized_trajectory(path_arr, speeds, dt)
    trajectory, total_time = apply_hold_segments(
        trajectory, dt=dt, hold_start=hold_start, hold_end=hold_end
    )
    path_length = float(np.sum(np.linalg.norm(path_arr[1:] - path_arr[:-1], axis=1)))
    return [tuple(map(float, p)) for p in path_arr], trajectory, path_length

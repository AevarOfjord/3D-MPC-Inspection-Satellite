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


def compute_mesh_bounds(
    vertices: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Return (min_bounds, max_bounds, center, max_xy_radius)."""
    min_bounds = vertices.min(axis=0)
    max_bounds = vertices.max(axis=0)
    center = (min_bounds + max_bounds) / 2.0
    offsets = vertices[:, :2] - center[:2]
    radius_xy = float(np.max(np.linalg.norm(offsets, axis=1)))
    return min_bounds, max_bounds, center, radius_xy


def compute_oriented_bounds(
    vertices: np.ndarray,
) -> Tuple[np.ndarray, float, float, float, np.ndarray, float, float]:
    """
    Compute Oriented Bounding Box (XY only) using PCA.
    Returns:
       center (3,),
       radius_x, radius_y (aligned half-extents),
       rotation_angle_rad,
       rotation_matrix (2x2),
       z_min, z_max
    """
    # 1. Z bounds (global Z is assumed up)
    z_min = float(vertices[:, 2].min())
    z_max = float(vertices[:, 2].max())

    # 2. XY PCA
    xy_points = vertices[:, :2]
    center_xy = np.mean(xy_points, axis=0)
    centered_xy = xy_points - center_xy

    # Covariance
    cov = np.cov(centered_xy, rowvar=False)

    # Eigendecomposition
    # If points are collinear or single point, cov might be singular.
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        # Fallback to axis aligned
        eigenvalues = np.array([1.0, 1.0])
        eigenvectors = np.eye(2)

    # Sort eigenvectors by eigenvalue (largest first = major axis)
    # np.linalg.eigh returns sorted ascending.
    order = eigenvalues.argsort()[::-1]
    eigenvectors = eigenvectors[:, order]
    # eigenvalues = eigenvalues[order]

    # Major axis (X') is eigenvectors[:, 0]
    # Minor axis (Y') is eigenvectors[:, 1]

    # 3. Project points onto PCA axes
    # Projection = centered_points . eigenvectors
    projected = np.dot(centered_xy, eigenvectors)

    min_p = projected.min(axis=0)
    max_p = projected.max(axis=0)

    # 4. Compute extents in PCA frame
    pca_center_offset = (min_p + max_p) / 2.0
    half_extents = (max_p - min_p) / 2.0

    rx = float(half_extents[0])
    ry = float(half_extents[1])

    # Refine center in world frame
    # World Center = Mean + (PCA_Center_Offset rotated back)
    # Rotated back = dot(pca_center_offset, eigenvectors.T)
    world_center_xy = center_xy + np.dot(pca_center_offset, eigenvectors.T)

    center_3d = np.array(
        [world_center_xy[0], world_center_xy[1], (z_min + z_max) / 2.0]
    )

    # Rotation angle of the major axis (col 0) w.r.t X axis
    # axis vector = eigenvectors[:, 0] = [x, y]
    angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])

    return center_3d, rx, ry, angle, eigenvectors, z_min, z_max


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


def _generate_ellipse_ring(
    center: Iterable[float],
    radius_x: float,
    radius_y: float,
    z: float,
    points_per_ring: int,
) -> List[Tuple[float, float, float]]:
    """Generate an elliptical ring."""
    points_per_ring = max(int(points_per_ring), 12)
    cx, cy, _cz = center
    ring: List[Tuple[float, float, float]] = []
    for i in range(points_per_ring + 1):
        # Use simple angle parameterization (not arc-length uniform, but sufficient)
        angle = 2.0 * np.pi * (i / points_per_ring)
        x = cx + radius_x * np.cos(angle)
        y = cy + radius_y * np.sin(angle)
        ring.append((float(x), float(y), float(z)))
    return ring


def _generate_rounded_rect_ring(
    center: Iterable[float],
    half_extent_x: float,
    half_extent_y: float,
    corner_radius: float,
    z: float,
    points_per_ring: int,
) -> List[Tuple[float, float, float]]:
    """
    Generate a rounded rectangle (offset box) ring.
    The shape defines a box of size 2*half_extent (inner) plus a margin of corner_radius.
    This effectively creates a path at constant distance 'corner_radius' from the inner box.
    """
    points_per_ring = max(int(points_per_ring), 16)
    cx, cy, _cz = center

    # Inner box corners (relative to center)
    x = half_extent_x
    y = half_extent_y
    r = corner_radius

    # 4 straight segments:
    # Right: (x+r, -y) to (x+r, y)  -> Length 2y
    # Top:   (x, y+r) to (-x, y+r)  -> Length 2x
    # Left:  (-x-r, y) to (-x-r, -y)-> Length 2y
    # Bottom:(-x, -y-r) to (x, -y-r)-> Length 2x

    # 4 corners (quarter circles):
    # Top-Right: center (x, y)
    # Top-Left:  center (-x, y)
    # Bot-Left:  center (-x, -y)
    # Bot-Right: center (x, -y)

    # Total perimeter approx
    perimeter = 4 * x + 4 * y + 2 * np.pi * r
    if perimeter < 1e-6:
        return [(cx, cy, z)] * points_per_ring

    # Distribute points
    # We'll just discretize by angle/segments for simplicity or uniform sampling?
    # Uniform sampling is better for constant speed.

    path = []

    # We will walk the perimeter.
    # Define segments as (type, length, start_val, end_val/center, etc)
    # Simplified approach: Parameterize by arc length t in [0, perimeter]

    # Corner centers
    c_tr = (x, y)
    c_tl = (-x, y)
    c_bl = (-x, -y)
    c_br = (x, -y)

    # Segment definitions (length, generator_func)

    def gen_line(p_start, p_end):
        return lambda t_norm: (
            p_start[0] + t_norm * (p_end[0] - p_start[0]),
            p_start[1] + t_norm * (p_end[1] - p_start[1]),
        )

    def gen_arc(center, angle_start, angle_end):
        return lambda t_norm: (
            center[0] + r * np.cos(angle_start + t_norm * (angle_end - angle_start)),
            center[1] + r * np.sin(angle_start + t_norm * (angle_end - angle_start)),
        )

    # 1. Right vertical (upwards)
    segments = []
    segments.append((2 * y, gen_line((x + r, -y), (x + r, y))))
    # 2. TR Corner (0 to 90 deg)
    segments.append((0.5 * np.pi * r, gen_arc(c_tr, 0.0, 0.5 * np.pi)))
    # 3. Top horizontal (leftwards)
    segments.append((2 * x, gen_line((x, y + r), (-x, y + r))))
    # 4. TL Corner (90 to 180 deg)
    segments.append((0.5 * np.pi * r, gen_arc(c_tl, 0.5 * np.pi, np.pi)))
    # 5. Left vertical (downwards)
    segments.append((2 * y, gen_line((-x - r, y), (-x - r, -y))))
    # 6. BL Corner (180 to 270 deg)
    segments.append((0.5 * np.pi * r, gen_arc(c_bl, np.pi, 1.5 * np.pi)))
    # 7. Bottom horizontal (rightwards)
    segments.append((2 * x, gen_line((-x, -y - r), (x, -y - r))))
    # 8. BR Corner (270 to 360 deg)
    segments.append((0.5 * np.pi * r, gen_arc(c_br, 1.5 * np.pi, 2.0 * np.pi)))

    total_len = sum(s[0] for s in segments)

    # Generate points
    seg_idx = 0
    seg_start_dist = 0.0

    for i in range(points_per_ring):  # i=0 to N-1. Note: last point connects to first.
        # Ensure we close the loop? usually we want N points, 0 to 360 equiv.
        target_dist = (i / points_per_ring) * total_len

        # Find segment
        while seg_idx < len(segments) and target_dist > (
            seg_start_dist + segments[seg_idx][0] + 1e-9
        ):
            seg_start_dist += segments[seg_idx][0]
            seg_idx += 1

        if seg_idx >= len(segments):
            seg_idx = len(segments) - 1

        seg_len, seg_func = segments[seg_idx]
        local_t = (target_dist - seg_start_dist) / max(seg_len, 1e-6)
        local_t = max(0.0, min(1.0, local_t))

        px, py = seg_func(local_t)
        path.append((float(cx + px), float(cy + py), float(z)))

    # Add closure point if needed or just rely on trajectory loop?
    # Usually we generate open loop of points for "points", but build_time_parameterized might expect wrap.
    # existing circle gen does: range(points_per_ring + 1). So it duplicates start at end.
    # Let's match that.

    # Add the first point again at the end
    path.append(path[0])

    return path


def generate_cylindrical_scan_path(
    center: Iterable[float],
    radius: float,  # Kept for backward compat, used as default if radii tuple not provided
    z_levels: Iterable[float],
    points_per_ring: int,
    ring_shape: str = "circle",
    radius_y: float = None,
    corner_radius: float = 0.0,
    inner_rx: float = 0.0,
    inner_ry: float = 0.0,
) -> List[Tuple[float, float, float]]:
    """Generate stacked rings (circle/square/ellipse) with vertical transitions."""
    cx, cy, _cz = center
    z_vals = list(z_levels)
    if not z_vals:
        raise ValueError("z_levels must contain at least one level")

    shape = ring_shape.lower().strip()
    path: List[Tuple[float, float, float]] = []

    # Radii setup
    rx = float(radius)
    ry = float(radius_y) if radius_y is not None else rx

    for idx, z in enumerate(z_vals):
        if shape == "square":
            ring = _generate_square_ring(center, rx, z, points_per_ring)
        elif shape == "ellipse":
            ring = _generate_ellipse_ring(center, rx, ry, z, points_per_ring)
        elif shape == "rounded_rect":
            ring = _generate_rounded_rect_ring(
                center, inner_rx, inner_ry, corner_radius, z, points_per_ring
            )
        else:
            ring = _generate_circle_ring(center, rx, z, points_per_ring)
        path.extend(ring)

        if idx < len(z_vals) - 1:
            next_z = z_vals[idx + 1]
            # Transition point: start of next ring (angle 0)
            # For ellipse/circle: (rx, 0) relative to center
            # For square: (rx, rx) corner? _generate_square_ring starts at top right?
            # Let's just use the first point of the next generated ring logic
            # Simpler: just connect last point of this ring to first of next.
            # But the logic below adds an explicit waypoint.
            # Let's maintain the "lifting" logic but adapted.

            if shape == "rounded_rect":
                transit_x, transit_y = (
                    cx + inner_rx + corner_radius,
                    cy - inner_ry,
                )  # Start point of generator
            elif shape == "square":
                transit_x, transit_y = cx + rx, cy + rx
            elif shape == "ellipse":
                transit_x, transit_y = cx + rx, cy  # Angle 0
            else:
                transit_x, transit_y = cx + rx, cy

            path.append((float(transit_x), float(transit_y), float(next_z)))

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
    points_per_ring = max(
        12, int(np.ceil(circumference / max(arc_coverage * overlap, 1e-3)))
    )
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
    scan_axis: str = "Z",
) -> Tuple[List[Tuple[float, float, float]], np.ndarray, float]:
    """Generate scan path and time-parameterized trajectory."""
    vertices = load_obj_vertices(obj_path)

    # Axis Permutation Logic
    # We map the requested scan "up" axis to Z for the algorithm, then map back.
    # Default Z: (x,y,z) -> (x,y,z)
    # X: (y,z,x) -> x is up.
    # Y: (z,x,y) -> y is up.

    axis = scan_axis.upper().strip()
    if axis == "X":
        # Permute: X->Z, Y->X, Z->Y ? No, we want X to be the stacking axis.
        # Let's say local frame is (u, v, w). w is stacking.
        # If we want X to be stacking, we map X->w.
        # vertices_local = vertices[:, [1, 2, 0]]  # (y, z, x)
        # So w=x. u=y, v=z.
        perm_order = [1, 2, 0]
        inv_perm_order = [2, 0, 1]
    elif axis == "Y":
        # Y is stacking. Map Y->w.
        # vertices_local = vertices[:, [2, 0, 1]] # (z, x, y)
        # w=y.
        perm_order = [2, 0, 1]
        inv_perm_order = [1, 2, 0]
    else:
        # Z is stacking.
        perm_order = [0, 1, 2]
        inv_perm_order = [0, 1, 2]

    vertices_local = vertices[:, perm_order]

    # Use PCA to find oriented bounds in local frame
    # Note: compute_oriented_bounds uses Z-up assumption for Z-bounds.
    # Since we permuted, the 'w' (local Z) is the scan axis. Good.
    center, model_rx, model_ry, angle, rot_matrix, z_min, z_max = (
        compute_oriented_bounds(vertices_local)
    )

    levels = max(int(levels), 1)
    z_min_adj = float(z_min + z_margin)
    z_max_adj = float(z_max - z_margin)

    if levels == 1:
        z_levels = [0.5 * (z_min_adj + z_max_adj)]
    else:
        z_levels = list(np.linspace(z_min_adj, z_max_adj, levels))

    # The compute_oriented_bounds returns half-extents, so model_rx/ry are already half-extents.
    # We need to re-extract min/max bounds from the PCA-aligned vertices to get the true half-extents
    # for the rounded rectangle, as model_rx/ry from compute_oriented_bounds might be based on a simplified
    # bounding box calculation.
    # For now, let's assume model_rx and model_ry are the half-extents along the PCA axes.

    # Use Rounded Rectangle (Stadium) for best fit
    # Defines an offset path at distance 'standoff' from the OBB

    # If the user wants a circle/ellipse explicitly, we might want to expose that choice later.
    # For now, "Smart" means tightest safe fit.

    aligned_path = generate_cylindrical_scan_path(
        center=(0.0, 0.0, 0.0),  # Generate in local PCA frame
        radius=model_rx
        + standoff,  # Fallback if someone used other args, or for circle/ellipse
        z_levels=z_levels,
        points_per_ring=points_per_circle,
        ring_shape="rounded_rect",
        inner_rx=model_rx,
        inner_ry=model_ry,
        corner_radius=max(standoff, 0.1),  # Ensure at least some curve
    )

    # Transform back to world
    aligned_arr = np.array(aligned_path, dtype=float)  # Nx3

    # 1. Rotate XY (in local frame) back to un-oriented local frame
    # aligned_arr[:, :2] is [u', v']
    uv_rotated = np.dot(aligned_arr[:, :2], rot_matrix.T)

    # 2. Add Center (in local frame)
    uv_shifted = uv_rotated + center[:2]

    # 3. Reconstruct Local Points (u, v, w)
    # w is aligned_arr[:, 2] (the stacking levels)
    local_points = np.column_stack((uv_shifted, aligned_arr[:, 2]))

    # 4. Inverse Permute to Global (x, y, z)
    path_arr = local_points[:, inv_perm_order]

    path = [tuple(p) for p in path_arr]

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

    # Override calculated step with fixed 0.5m step as requested
    ring_step = 0.5

    total_height = float(max(height, 1e-3))
    z_min = -0.5 * total_height
    z_max = 0.5 * total_height

    # Generate levels from bottom to top (inclusive of start, and capturing end if close)
    # Using a small epsilon to ensure z_max is included if it's a multiple
    epsilon = 1e-5
    z_levels = list(np.arange(z_min, z_max + epsilon, ring_step))

    # Ensure strictly within bounds if needed, but for scan usually we want to cover the extent.
    # If the last level is significantly past z_max, we might clip, but arange behaves well.
    # Verify we have at least one level
    if not z_levels:
        z_levels = [z_min]

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

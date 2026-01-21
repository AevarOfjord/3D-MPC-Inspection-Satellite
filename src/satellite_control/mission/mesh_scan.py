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


def load_obj_data(obj_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load vertex positions and face indices from an OBJ file."""
    vertices: List[List[float]] = []
    faces: List[List[int]] = []

    with open(obj_path, "r") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if line.startswith("v "):
                parts = line.split()
                try:
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                except (ValueError, IndexError):
                    continue
            elif line.startswith("f "):
                parts = line.split()
                # Handle f v1 v2 v3 or f v1/vt1 v2/vt2 ...
                face_idxs = []
                try:
                    for p in parts[1:]:
                        # OBJ is 1-indexed
                        idx = int(p.split("/")[0]) - 1
                        face_idxs.append(idx)
                    # Triangulate simple polygons (fan) if needed, or just take first 3 for tri
                    if len(face_idxs) >= 3:
                        # Simple fan triangulation for quads/polys
                        for i in range(len(face_idxs) - 2):
                            faces.append(
                                [face_idxs[0], face_idxs[i + 1], face_idxs[i + 2]]
                            )
                except (ValueError, IndexError):
                    continue

    if not vertices:
        raise ValueError(f"No vertices found in OBJ: {obj_path}")

    return np.array(vertices, dtype=float), np.array(faces, dtype=int)


def load_obj_vertices(obj_path: str) -> np.ndarray:
    """Backwards compatibility wrapper."""
    v, _ = load_obj_data(obj_path)
    return v


def slice_mesh_at_z(
    vertices: np.ndarray, faces: np.ndarray, z_level: float
) -> np.ndarray:
    """
    Compute the intersection points of the mesh triangles with a Z-plane.
    Returns an array of 2D points (x, y) on the slice.
    """
    if len(faces) == 0:
        return np.empty((0, 2))

    # vertices: (V, 3)
    # faces: (F, 3) indices

    # Get Z coordinates of all associated vertices
    # We want edges that cross z_level.
    # An edge (v1, v2) crosses if (z1 - z)*(z2 - z) <= 0
    # But usually < 0 to avoid duplicates if vertex is ON plane.

    # Vectorized approach:
    # 1. Get coords for all face vertices: (F, 3, 3)
    tris = vertices[faces]

    # 2. Check edges (0-1, 1-2, 2-0)
    # Define edges by indices [0,1], [1,2], [2,0]
    edges_idx = [[0, 1], [1, 2], [2, 0]]
    points = []

    for ea, eb in edges_idx:
        # vA and vB for all faces: Shape (F, 3)
        vA = tris[:, ea, :]
        vB = tris[:, eb, :]

        zA = vA[:, 2]
        zB = vB[:, 2]

        # Check crossing
        # (zA <= z < zB) or (zB <= z < zA)
        # Using strict limits to handle coplanar robustness
        cross_mask = ((zA <= z_level) & (zB > z_level)) | (
            (zB <= z_level) & (zA > z_level)
        )

        if np.any(cross_mask):
            # Interpolate
            # t = (z - zA) / (zB - zA)
            # P = A + t * (B - A)

            A_cross = vA[cross_mask]
            B_cross = vB[cross_mask]

            t = (z_level - A_cross[:, 2]) / (B_cross[:, 2] - A_cross[:, 2])
            # Expand t dimensions for broadcast: (N,) -> (N, 1)
            t = t[:, np.newaxis]

            P_cross = A_cross + t * (B_cross - A_cross)
            points.append(P_cross[:, :2])  # Keep XY

    if not points:
        return np.empty((0, 2))

    return np.vstack(points)


from scipy.spatial import ConvexHull


def offset_polygon(points: np.ndarray, distance: float) -> np.ndarray:
    """
    Offset a convex polygon (ordered CCW) outward by distance.
    Uses simple bisector offset (miter).
    """
    if len(points) < 3:
        return points

    # Calculate edge vectors
    # Roll -1 to get P_next - P_curr
    p_next = np.roll(points, -1, axis=0)
    p_prev = np.roll(points, 1, axis=0)

    v_in = points - p_prev
    v_out = p_next - points

    # Normalize
    len_in = np.linalg.norm(v_in, axis=1, keepdims=True) + 1e-9
    len_out = np.linalg.norm(v_out, axis=1, keepdims=True) + 1e-9

    n_in = v_in / len_in
    n_out = v_out / len_out

    # Tangent and Normal
    # Edge normal is (-y, x) for CCW
    # BUT we are offsetting *vertices*.
    # Vertex normal (bisector) vector direction
    # Average of adjacent edge normals?
    # Or just average of normalized edge vectors?
    # Bisector vector B = normalize(n_out - n_in)? No.

    # Let's use standard edge normals.
    # Edge i connects P_i to P_{i+1}. Normal N_i.
    # Point P_i is intersection of Line(P_{i-1}, N_{i-1}) and Line(P_i, N_i).

    # Edge normals (pointing right/out for CCW? No, usually left/in?)
    # If points are CCW, (x,y) -> (-y, x) is INWARD. (y, -x) is OUTWARD.
    # Check: P=(1,0), Q=(0,1). v=(-1, 1). Rot (-1, -1) is IN. (1, 1) is OUT.
    # Correct: (v_y, -v_x) is OUTWARD for CCW.

    norms_in = np.stack([n_in[:, 1], -n_in[:, 0]], axis=1)  # Normal of incoming edge
    norms_out = np.stack([n_out[:, 1], -n_out[:, 0]], axis=1)  # Normal of outgoing edge

    # Miter offset
    # Tangent vector at vertex T = normalize(n_in + n_out) ? No.
    # The vertex moves along the bisector.
    # Alpha = angle between edges.

    # Robust vector addition:
    # bisector = normalize(norms_in + norms_out)
    # This vector points outward.
    # Scale factor = distance / dot(bisector, norms_in)

    bisector = norms_in + norms_out
    b_len = np.linalg.norm(bisector, axis=1, keepdims=True) + 1e-9
    bisector /= b_len

    dot = np.sum(bisector * norms_in, axis=1, keepdims=True)
    # Limit sharp corners
    scale = distance / np.maximum(dot, 0.1)

    return points + bisector * scale


def resample_polygon(
    points: np.ndarray, num_points: int
) -> List[Tuple[float, float, float]]:
    """Resample a loop of points to fixed count equidistant points."""
    # Close loop efficiently
    pts = np.vstack([points, points[0]])

    # Access cumulative distance
    dists = np.linalg.norm(pts[1:] - pts[:-1], axis=1)
    cum_dist = np.concatenate(([0], np.cumsum(dists)))
    total_len = cum_dist[-1]

    if total_len < 1e-6:
        # Degenerate
        center = np.mean(points, axis=0)
        return [tuple(center)] * num_points

    # Interpolate
    # We want num_points from 0 to total_len (wrapping)
    # Generate N points. i=0 is start.
    target_dists = np.linspace(0, total_len, num_points, endpoint=False)

    # Numpy interp?
    # interp works on 1D. We map dist -> x, dist -> y, dist -> z
    new_x = np.interp(target_dists, cum_dist, pts[:, 0])
    new_y = np.interp(target_dists, cum_dist, pts[:, 1])
    # Z should be constant but let's interpolate to be safe
    if points.shape[1] > 2:
        new_z = np.interp(target_dists, cum_dist, pts[:, 2])
        return list(zip(new_x, new_y, new_z))
    else:
        return list(zip(new_x, new_y))


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
    build_trajectory: bool = True,
) -> Tuple[List[Tuple[float, float, float]], np.ndarray, float]:
    """Generate adaptive scan path using mesh slicing."""
    vertices, faces = load_obj_data(obj_path)

    # 1. Axis Permutation (Map Scan Axis to Z)
    axis = scan_axis.upper().strip()
    if axis == "X":
        # Map X->Z. (y,z,x)
        perm_order = [1, 2, 0]
        inv_perm_order = [2, 0, 1]
    elif axis == "Y":
        # Map Y->Z. (z,x,y)
        perm_order = [2, 0, 1]
        inv_perm_order = [1, 2, 0]
    else:
        # Z->Z
        perm_order = [0, 1, 2]
        inv_perm_order = [0, 1, 2]

    # vertices_perm: (N, 3) where Z is the scan axis
    vertices_perm = vertices[:, perm_order]

    # 2. PCA Alignment on XY plane of permuted vertices
    # We want to align the major axes of the object to X/Y for efficient bounding box fallback
    # AND to handle the slicing cleanly.
    xy_points = vertices_perm[:, :2]
    center_xy = np.mean(xy_points, axis=0)
    centered_xy = xy_points - center_xy

    # Compute Eigenvectors
    cov = np.cov(centered_xy, rowvar=False)
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        eigenvectors = np.eye(2)

    # Sort largest first
    order = eigenvalues.argsort()[::-1]
    eigenvectors = eigenvectors[
        :, order
    ]  # rotation matrix R st. Projected = Centered @ R

    # Project: local_aligned_xy = (xy - center) @ R
    vertices_aligned = np.zeros_like(vertices_perm)
    vertices_aligned[:, :2] = np.dot(centered_xy, eigenvectors)
    vertices_aligned[:, 2] = vertices_perm[:, 2]  # Z is unchanged

    # 3. Define Z-Levels
    z_min = vertices_aligned[:, 2].min()
    z_max = vertices_aligned[:, 2].max()

    levels = max(int(levels), 1)
    z_min_adj = float(z_min)
    z_max_adj = float(z_max)

    # Smart Level Placement (Center-Aligned)
    # Instead of scanning exactly at the edges (z_min, z_max) which are often problematic/degenerate,
    # we scan at the center of each "layer".
    # This effectively optimizes the placement to be in the "meat" of the section.
    height_aligned = z_max - z_min
    if height_aligned < 1e-6:
        z_levels = [z_min]
    else:
        # If levels is derived from spacing in previous logic, 'levels' is count.
        # Redistribute evenly.
        step = height_aligned / float(levels)
        # Generate levels at start + half_step, start + 1.5*step, etc.
        z_levels = [z_min + step * (0.5 + i) for i in range(levels)]

    # Bounding Box Fallback Params
    min_p = vertices_aligned.min(axis=0)
    max_p = vertices_aligned.max(axis=0)
    half_extents = (max_p - min_p) / 2.0
    model_rx = float(half_extents[0])
    model_ry = float(half_extents[1])

    path_aligned = []

    for i, z in enumerate(z_levels):
        # Slice
        slice_points = slice_mesh_at_z(vertices_aligned, faces, z)

        hull_ring = []
        is_fallback = False

        if len(slice_points) >= 3:
            try:
                # PCA on Slice to find best-fit Ellipse
                mean = np.mean(slice_points, axis=0)
                centered = slice_points - mean
                cov = np.cov(centered, rowvar=False)

                # Eigen decomp
                vals, vecs = np.linalg.eigh(cov)
                # Sort: vals[0] is smallest? eigh returns ascending.
                # So vecs[:, 1] is major axis.

                # Project onto axes to find extents (Bounding Box in PCA frame)
                projected = np.dot(centered, vecs)  # (N, 2)
                min_p_slice = projected.min(axis=0)
                max_p_slice = projected.max(axis=0)

                # Slicing is thin. We need to check the "Volume" around this Z level
                # to avoid missing features that start just above/below (like the main body).
                # Look at vertices within a Z-window.
                z_window = max(
                    standoff * 0.5, 0.1
                )  # Minimum 10cm window or half standoff

                # Filter vertices in aligned frame
                mask = (vertices_aligned[:, 2] >= z - z_window) & (
                    vertices_aligned[:, 2] <= z + z_window
                )
                if np.any(mask):
                    local_verts = vertices_aligned[mask]
                    # Project these onto the SAME PCA axes as the slice (using center/vecs from slice)
                    # Note: Using slice center might be slightly off if local_verts shift, but it's safe.
                    # Better: Just compute extents of local_verts relative to slice center.

                    lv_centered = (
                        local_verts[:, :2] - center_xy
                    )  # center_xy is GLOBAL aligned XY center
                    # Wait, 'center_xy' variable from line 647 is the mean of ALL vertices.
                    # 'mean' (line 700) is mean of SLICE.
                    # We should probably use the SLICE mean to center the local window to align them.

                    lv_centered_slice = (
                        local_verts[:, :2] - mean
                    )  # Center relative to slice
                    lv_proj = np.dot(lv_centered_slice, vecs)

                    min_p_vol = lv_proj.min(axis=0)
                    max_p_vol = lv_proj.max(axis=0)

                    # Update (expand) the bounding box to include the range volume
                    min_p_slice = np.minimum(min_p_slice, min_p_vol)
                    max_p_slice = np.maximum(max_p_slice, max_p_vol)

                extents = (max_p_slice - min_p_slice) / 2.0
                rx_local = extents[1]  # Major axis extent (corresponding to vecs[:,1])
                ry_local = extents[0]  # Minor axis extent

                # Ellipse Angle (vecs[:,1] is major axis)
                angle = np.arctan2(vecs[1, 1], vecs[0, 1])

                # Generate Canonical Ellipse
                # We want rings to be aligned, but adaptive orientation is okay for smooth transitions?
                # Using simple Angle parameterization
                # Note: rx corresponds to X axis in generate func, ry to Y.
                # Our Major Axis is aligned with 'angle'.

                # Checks
                rx_final = rx_local + standoff
                ry_final = ry_local + standoff

                # Corner Safety: Check if box corner (rx, ry) protrudes
                # Ellipse check: (rx/A)^2 + (ry/B)^2. If > 1, scale up.
                corner_val = (rx_local / rx_final) ** 2 + (ry_local / ry_final) ** 2
                if corner_val > 1.0:
                    scale = np.sqrt(corner_val)
                    rx_final *= scale
                    ry_final *= scale

                base_ring = _generate_ellipse_ring(
                    center=(0, 0, 0),
                    radius_x=rx_final,
                    radius_y=ry_final,
                    z=z,
                    points_per_ring=points_per_circle,
                )

                # Rotate and Translate
                # Rotation Matrix for 'angle'
                c, s = np.cos(angle), np.sin(angle)
                R = np.array([[c, -s], [s, c]])

                pts = np.array(base_ring)
                # pts[:, :2] is XY. Rot = XY @ R.T ?? No.
                # Canonical: Major axis along X.
                # We mapped 'rx_local' to X radius. 'angle' is angle of major axis.
                # So Rotate (x,y) by angle.
                rot_xy = np.dot(pts[:, :2], R.T)  # R.T because vectors are row? No.
                # v_rot = R @ v.  (N,2) -> (N,2). v_rot = v @ R.T

                final_xy = rot_xy + mean

                hull_ring = []
                for k in range(len(final_xy)):
                    hull_ring.append(
                        (float(final_xy[k, 0]), float(final_xy[k, 1]), float(z))
                    )

            except Exception as e:
                print(f"[Warn] Ellipse fit failed at Z={z:.2f}: {e}")
                is_fallback = True
        else:
            is_fallback = True

        if is_fallback:
            # Fallback to Global OBB Ellipse - With Diagonal Safety
            A_glob = model_rx + standoff
            B_glob = model_ry + standoff

            # Strict Diagonal Check for Fallback
            if model_rx > 1e-6 and model_ry > 1e-6:
                c_ang = np.arctan2(model_ry, model_rx)
                # Standoff Point at the corner
                ps_x = model_rx + standoff * np.cos(c_ang)
                ps_y = model_ry + standoff * np.sin(c_ang)

                # Check if inside standard ellipse
                cv = (ps_x / A_glob) ** 2 + (ps_y / B_glob) ** 2
                if cv > 1.0:
                    sf = np.sqrt(cv)
                    A_glob *= sf
                    B_glob *= sf

            hull_ring = _generate_ellipse_ring(
                center=(0, 0, 0),
                radius_x=A_glob,
                radius_y=B_glob,
                z=z,
                points_per_ring=points_per_circle,
            )

        # Safe Transition Logic
        # Ensure we move vertically at the MAXIMUM radius of the two levels
        # to avoid clipping corners of an expanding object.
        if i > 0 and len(path_aligned) > 0 and len(hull_ring) > 0:
            prev_pt = path_aligned[-1]  # (x, y, z_prev)
            curr_pt = hull_ring[0]  # (x, y, z_curr)

            # Calculate radii from center (0,0 in aligned frame)
            r_prev = np.sqrt(prev_pt[0] ** 2 + prev_pt[1] ** 2)
            r_curr = np.sqrt(curr_pt[0] ** 2 + curr_pt[1] ** 2)

            r_safe = max(r_prev, r_curr)

            # Create Transition Waypoints
            waypoints = []

            # W1: At prev Z, but expanded to safe radius
            if abs(r_safe - r_prev) > 1e-3:
                scale1 = r_safe / (r_prev + 1e-9)
                w1 = (prev_pt[0] * scale1, prev_pt[1] * scale1, prev_pt[2])
                waypoints.append(w1)

            # W2: At curr Z, but expanded to safe radius
            if abs(r_safe - r_curr) > 1e-3:
                scale2 = r_safe / (r_curr + 1e-9)
                w2 = (curr_pt[0] * scale2, curr_pt[1] * scale2, curr_pt[2])
                waypoints.append(w2)

            path_aligned.extend(waypoints)

        path_aligned.extend(hull_ring)

    # Add Safe Approach and Departure
    # Calculate global max radius to ensure we enter/exit from a truly safe distance
    # even if the first/last slices are narrow segments of a T-shape.
    global_safe_r = max(model_rx, model_ry) + standoff * 2.0

    if len(path_aligned) > 0:
        # Safe Approach
        start_pt = path_aligned[0]
        r_start = np.sqrt(start_pt[0] ** 2 + start_pt[1] ** 2)
        if r_start > 1e-6:
            # Scale out to at least global safe radius, or just buffer the current start
            # logic: If start is narrow, go out to global max. If start is wide, go out more.
            target_r = max(r_start + standoff, global_safe_r)
            scale_in = target_r / r_start

            entry_pt = (start_pt[0] * scale_in, start_pt[1] * scale_in, start_pt[2])
            path_aligned.insert(0, entry_pt)

        # Safe Departure
        end_pt = path_aligned[-1]
        r_end = np.sqrt(end_pt[0] ** 2 + end_pt[1] ** 2)
        if r_end > 1e-6:
            target_r_out = max(r_end + standoff, global_safe_r)
            scale_out = target_r_out / r_end

            exit_pt = (end_pt[0] * scale_out, end_pt[1] * scale_out, end_pt[2])
            path_aligned.append(exit_pt)

    # 4. Transform Back to World
    aligned_arr = np.array(path_aligned, dtype=float)

    # Reverse Step 2: Un-rotate XY
    # Centered = Projected @ R.T
    unrotated_xy = np.dot(aligned_arr[:, :2], eigenvectors.T)
    # Add Center
    permuted_xy = unrotated_xy + center_xy

    # Reconstruct Permuted (x', y', z')
    permuted_points = np.column_stack([permuted_xy, aligned_arr[:, 2]])

    # Reverse Step 1: Inverse Permutation
    world_arr = permuted_points[:, inv_perm_order]

    path = [tuple(p) for p in world_arr]

    path_length = (
        float(np.sum(np.linalg.norm(world_arr[1:] - world_arr[:-1], axis=1)))
        if len(world_arr) > 1
        else 0.0
    )
    if not build_trajectory:
        return path, np.empty((0, 7), dtype=float), path_length

    curvature = compute_curvature(world_arr)
    speeds = compute_speed_profile(curvature, v_max, v_min, lateral_accel)
    trajectory, total_time = build_time_parameterized_trajectory(world_arr, speeds, dt)
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
    build_trajectory: bool = True,
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

    path_length = float(np.sum(np.linalg.norm(path_arr[1:] - path_arr[:-1], axis=1)))
    if not build_trajectory:
        empty_traj = np.empty((0, 7), dtype=float)
        return [tuple(map(float, p)) for p in path_arr], empty_traj, path_length

    curvature = compute_curvature(path_arr)
    speeds = compute_speed_profile(curvature, v_max, v_min, lateral_accel)
    trajectory, total_time = build_time_parameterized_trajectory(path_arr, speeds, dt)
    trajectory, total_time = apply_hold_segments(
        trajectory, dt=dt, hold_start=hold_start, hold_end=hold_end
    )
    return [tuple(map(float, p)) for p in path_arr], trajectory, path_length

"""
Starlink Orbit Path Generator

Generates circular orbit trajectory around a Starlink satellite mesh with
camera-face orientation pointing at the target body.

Uses the same curvature-based speed profiling as cylinder scan mission.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from src.satellite_control.mission.mesh_scan import (
    load_obj_vertices,
    compute_mesh_bounds,
    _generate_circle_ring,
)
from src.satellite_control.mission.trajectory_utils import (
    apply_hold_segments,
    build_time_parameterized_trajectory,
    compute_curvature,
    compute_speed_profile,
)


def compute_look_at_orientation(
    position: np.ndarray,
    target: np.ndarray,
    camera_face: str = "-Y",
) -> Tuple[float, float, float]:
    """
    Compute Euler angles (XYZ) so that the specified camera face points at target.

    Args:
        position: Current satellite position [x, y, z]
        target: Target point to look at [x, y, z]
        camera_face: Which face has the camera ("-Y" or "+Y")

    Returns:
        Euler angles (roll, pitch, yaw) in radians
    """
    # Direction from satellite to target
    direction = target - position
    direction_norm = np.linalg.norm(direction)
    if direction_norm < 1e-6:
        return (0.0, 0.0, 0.0)
    direction = direction / direction_norm

    # Camera face axis in body frame
    if camera_face == "-Y":
        camera_axis = np.array([0.0, -1.0, 0.0])
    else:  # "+Y"
        camera_axis = np.array([0.0, 1.0, 0.0])

    # We want to rotate camera_axis to align with direction
    # Use Rodrigues' rotation formula via scipy

    # Cross product gives rotation axis
    cross = np.cross(camera_axis, direction)
    cross_norm = np.linalg.norm(cross)

    if cross_norm < 1e-6:
        # Vectors are parallel or anti-parallel
        dot = np.dot(camera_axis, direction)
        if dot > 0:
            # Already aligned
            return (0.0, 0.0, 0.0)
        else:
            # Opposite direction - rotate 180 degrees around X axis
            return (np.pi, 0.0, 0.0)

    # Normalize rotation axis
    axis = cross / cross_norm

    # Angle between vectors
    angle = np.arccos(np.clip(np.dot(camera_axis, direction), -1.0, 1.0))

    # Create rotation from axis-angle
    rot = Rotation.from_rotvec(axis * angle)

    # Convert to Euler XYZ
    euler = rot.as_euler("xyz", degrees=False)

    return (float(euler[0]), float(euler[1]), float(euler[2]))


def _apply_pose(
    points: np.ndarray,
    position: Iterable[float],
    rotation_xyz: Iterable[float],
) -> np.ndarray:
    """Rotate + translate points by Euler XYZ rotation and position."""
    rot = Rotation.from_euler("xyz", rotation_xyz, degrees=False)
    rotated = rot.apply(points)
    pos = np.array(position, dtype=float)
    return rotated + pos


def generate_orbit_path_with_orientations(
    center: Iterable[float],
    radius: float,
    z_levels: Iterable[float],
    points_per_ring: int,
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    """
    Generate circular orbit path with orientations pointing at center.

    Args:
        center: Center of the target object [x, y, z]
        radius: Orbit radius (standoff + object radius)
        z_levels: List of Z heights for each ring
        points_per_ring: Number of points per circular ring

    Returns:
        Tuple of (positions, orientations) where each orientation
        has the -Y face pointing toward center.
    """
    cx, cy, cz = center
    z_vals = list(z_levels)
    if not z_vals:
        raise ValueError("z_levels must contain at least one level")

    positions: List[Tuple[float, float, float]] = []
    orientations: List[Tuple[float, float, float]] = []

    for idx, z in enumerate(z_vals):
        # Generate circle ring
        ring = _generate_circle_ring(center, radius, z, points_per_ring)

        for pos in ring:
            positions.append(pos)

            # Compute orientation to point -Y at center
            pos_arr = np.array(pos)
            target_point = np.array([cx, cy, z])  # Look at center at same Z level
            euler = compute_look_at_orientation(pos_arr, target_point, camera_face="-Y")
            orientations.append(euler)

        # Transition to next ring Z level
        if idx < len(z_vals) - 1:
            next_z = z_vals[idx + 1]
            transition_pos = (float(cx + radius), float(cy), float(next_z))
            positions.append(transition_pos)

            # Orientation for transition point
            trans_arr = np.array(transition_pos)
            trans_target = np.array([cx, cy, next_z])
            euler = compute_look_at_orientation(
                trans_arr, trans_target, camera_face="-Y"
            )
            orientations.append(euler)

    return positions, orientations


def build_starlink_orbit_trajectory(
    obj_path: str,
    center: Iterable[float] = (0.0, 0.0, 0.0),
    rotation_xyz: Iterable[float] = (0.0, 0.0, 0.0),
    standoff: float = 0.5,
    z_step: float = 0.5,
    points_per_ring: int = 36,
    v_max: float = 0.1,
    v_min: float = 0.05,
    lateral_accel: float = 0.05,
    dt: float = 0.05,
    hold_start: float = 5.0,
    hold_end: float = 5.0,
    camera_face: str = "-Y",
) -> Tuple[
    List[Tuple[float, float, float]],
    np.ndarray,
    float,
    List[Tuple[float, float, float]],
]:
    """
    Generate orbit trajectory around Starlink satellite.

    Args:
        obj_path: Path to Starlink OBJ file
        center: Starlink center position in world coordinates [m]
        rotation_xyz: Starlink orientation (Euler XYZ) [rad]
        standoff: Distance from Starlink surface [m]
        z_step: Height increment between orbit rings [m]
        points_per_ring: Number of waypoints per circular orbit
        v_max: Maximum travel speed [m/s]
        v_min: Minimum speed at high curvature [m/s]
        lateral_accel: Lateral acceleration limit [m/s²]
        dt: Trajectory timestep [s]
        hold_start: Hold time at start [s]
        hold_end: Hold time at end [s]
        camera_face: Which face has camera ("-Y" or "+Y")

    Returns:
        Tuple of:
        - path: List of 3D waypoint positions
        - trajectory: Time-parameterized trajectory array
        - path_length: Total path length [m]
        - orientations: List of Euler angle orientations for each waypoint
    """
    # Load mesh and compute bounds
    vertices = load_obj_vertices(obj_path)
    min_bounds, max_bounds, mesh_center, radius_xy = compute_mesh_bounds(vertices)

    # Orbit radius = object radius + standoff
    orbit_radius = float(radius_xy + max(standoff, 0.0))

    # Z range for orbits
    z_min = float(min_bounds[2])
    z_max = float(max_bounds[2])

    # Generate Z levels from bottom to top
    epsilon = 1e-5
    z_levels_local = list(np.arange(z_min, z_max + epsilon, z_step))
    if not z_levels_local:
        z_levels_local = [0.5 * (z_min + z_max)]

    # Generate orbit path in local coordinates (centered at origin)
    local_positions, local_orientations = generate_orbit_path_with_orientations(
        center=(0.0, 0.0, 0.0),
        radius=orbit_radius,
        z_levels=z_levels_local,
        points_per_ring=points_per_ring,
    )

    # Transform to world coordinates
    local_arr = np.array(local_positions, dtype=float)
    world_arr = _apply_pose(local_arr, position=center, rotation_xyz=rotation_xyz)

    path = [tuple(map(float, p)) for p in world_arr]

    # Compute speed profile based on curvature
    curvature = compute_curvature(world_arr)
    speeds = compute_speed_profile(curvature, v_max, v_min, lateral_accel)

    # Build time-parameterized trajectory
    trajectory, total_time = build_time_parameterized_trajectory(world_arr, speeds, dt)
    trajectory, total_time = apply_hold_segments(
        trajectory, dt=dt, hold_start=hold_start, hold_end=hold_end
    )

    # Path length
    path_length = float(np.sum(np.linalg.norm(world_arr[1:] - world_arr[:-1], axis=1)))

    return path, trajectory, path_length, local_orientations


def get_starlink_bounds(obj_path: str) -> Tuple[float, float, float]:
    """
    Get the bounding dimensions of the Starlink model.

    Args:
        obj_path: Path to Starlink OBJ file

    Returns:
        Tuple of (max_xy_radius, z_height, z_center)
    """
    vertices = load_obj_vertices(obj_path)
    min_bounds, max_bounds, center, radius_xy = compute_mesh_bounds(vertices)

    z_height = float(max_bounds[2] - min_bounds[2])
    z_center = float(center[2])

    return (float(radius_xy), z_height, z_center)

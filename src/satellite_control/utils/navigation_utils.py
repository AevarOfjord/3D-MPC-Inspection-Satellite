"""
Navigation and Geometry Utilities for Satellite Control

Shared mathematical and geometric utility functions for satellite navigation.
Used by both real hardware and simulation controllers for consistent behavior.

Utility functions:
- Angle normalization and shortest-path difference calculation
- Point-to-line distance for path tracking
- Safe path generation with obstacle avoidance
- Linear and circular interpolation for trajectories
- Geometric calculations for waypoint navigation

Key features:
- Handles angle wrapping around ±π correctly
- Prevents 270° transition issues with shortest-path logic
- Circular obstacle avoidance for safe navigation
- Shared between simulation and hardware for consistency
"""

from typing import List, Tuple

import numpy as np


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.

    Args:
        angle: Angle in radians (any range)

    Returns:
        Normalized angle in [-pi, pi] range
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def angle_difference(target_angle: float, current_angle: float) -> float:
    """
    Calculate the shortest angular difference between target and current angles.
    This prevents the 270° transition issue by always taking the shortest path.

    Args:
        target_angle: Target orientation in radians
        current_angle: Current orientation in radians

    Returns:
        Angle difference in [-pi, pi] range, positive = CCW rotation needed
    """
    # Normalize both angles first
    target = normalize_angle(target_angle)
    current = normalize_angle(current_angle)

    # Calculate raw difference
    diff = target - current

    # Ensure we take the shortest path around the circle
    if diff > np.pi:
        diff -= 2 * np.pi
    elif diff < -np.pi:
        diff += 2 * np.pi

    return diff


def point_to_line_distance(
    point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray
) -> float:
    """
    Calculate the shortest distance from a point to a line segment (2D or 3D).

    Args:
        point: Point position as [x, y] or [x, y, z] numpy array
        line_start: Line segment start position as [x, y] or [x, y, z] numpy array
        line_end: Line segment end position as [x, y] or [x, y, z] numpy array

    Returns:
        Shortest distance from point to line segment in meters
    """
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_len = np.linalg.norm(line_vec)

    # Handle zero-length line segment
    if line_len < 1e-10:
        return float(np.linalg.norm(point_vec))

    line_unitvec = line_vec / line_len
    proj_length = np.dot(point_vec, line_unitvec)

    # Clamp to line segment
    proj_length = max(0, min(line_len, proj_length))
    proj_point = line_start + line_unitvec * proj_length

    return float(np.linalg.norm(point - proj_point))


def calculate_safe_path_to_waypoint(
    start_pos: np.ndarray,
    target_pos: np.ndarray,
    all_obstacles: List[Tuple[float, ...]],
    safety_radius: float,
) -> List[Tuple[float, float, float]]:
    """
    Calculate a safe path from start to target that avoids spherical obstacles.

    Uses a simple single-waypoint strategy: if the direct path crosses an obstacle,
    generates an intermediate waypoint that goes around it.

    Args:
        start_pos: Starting position as [x, y, z] (z optional)
        target_pos: Target position as [x, y, z] (z optional)
        all_obstacles: List of obstacles as (x, y, z, r) tuples
        safety_radius: Minimum distance to maintain from obstacle centers

    Returns:
        List of waypoints [(x, y, z), ...] from start to target
    """
    # Simple approach: If direct path crosses obstacles, generate intermediate waypoint.
    start = np.array(start_pos, dtype=float)
    target = np.array(target_pos, dtype=float)
    if start.shape[0] < 3:
        start = np.pad(start, (0, 3 - start.shape[0]), "constant")
    if target.shape[0] < 3:
        target = np.pad(target, (0, 3 - target.shape[0]), "constant")

    waypoints = [tuple(start)]

    blocking_obstacles = []
    for obstacle in all_obstacles:
        if len(obstacle) == 4:
            obs_x, obs_y, obs_z, obs_radius = obstacle
        elif len(obstacle) == 3:
            obs_x, obs_y, obs_radius = obstacle
            obs_z = 0.0
        else:
            continue
        obstacle_center = np.array([obs_x, obs_y, obs_z], dtype=float)
        distance_to_path = point_to_line_distance(obstacle_center, start, target)
        if distance_to_path < (safety_radius + obs_radius):
            blocking_obstacles.append((obstacle_center, obs_radius))

    if blocking_obstacles:
        # For simplicity, create a waypoint that goes around the closest
        # blocking obstacle
        closest_obstacle, obstacle_radius = min(
            blocking_obstacles,
            key=lambda obs: float(np.linalg.norm(obs[0] - start)),
        )

        path_vec = target - start
        path_len = np.linalg.norm(path_vec)
        if path_len < 1e-10:
            waypoints.append(tuple(target))
            return waypoints

        path_dir = path_vec / path_len
        to_obstacle = closest_obstacle - start
        projection = np.dot(to_obstacle, path_dir)
        projection = max(0.0, min(path_len, projection))
        closest_point = start + projection * path_dir

        offset_dir = closest_point - closest_obstacle
        offset_norm = np.linalg.norm(offset_dir)
        if offset_norm < 1e-10:
            ref = np.array([0.0, 0.0, 1.0])
            if abs(np.dot(path_dir, ref)) > 0.9:
                ref = np.array([0.0, 1.0, 0.0])
            offset_dir = np.cross(path_dir, ref)
            offset_norm = np.linalg.norm(offset_dir)
            if offset_norm < 1e-10:
                offset_dir = np.array([1.0, 0.0, 0.0])
                offset_norm = 1.0
        offset_dir = offset_dir / offset_norm

        from src.satellite_control.config.constants import Constants

        clearance = (
            obstacle_radius + safety_radius + Constants.OBSTACLE_AVOIDANCE_SAFETY_MARGIN
        )
        intermediate_waypoint = closest_obstacle + offset_dir * clearance

        waypoints.append(tuple(intermediate_waypoint))
        wp_x, wp_y, wp_z = intermediate_waypoint
        print(
            f" Generated intermediate waypoint to avoid obstacle: "
            f"({wp_x:.3f}, {wp_y:.3f}, {wp_z:.3f})"
        )

    waypoints.append(tuple(target))
    return waypoints

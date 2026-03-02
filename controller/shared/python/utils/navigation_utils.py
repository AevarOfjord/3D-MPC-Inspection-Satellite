"""
Navigation and Geometry Utilities for Satellite Control

Shared mathematical and geometric utility functions for satellite navigation.
Used by both real hardware and simulation controllers for consistent behavior.

Utility functions:
- Angle normalization and shortest-path difference calculation
- Point-to-line distance for path tracking
- Linear and circular interpolation for trajectories
- Geometric calculations for waypoint navigation

Key features:
- Handles angle wrapping around ±π correctly
- Prevents 270° transition issues with shortest-path logic
- Shared between simulation and hardware for consistency
"""

import math

import numpy as np

_TWO_PI = 2.0 * math.pi


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.

    Uses math.remainder (IEEE 754) for O(1) performance regardless of
    input magnitude, replacing the previous while-loop approach.

    Args:
        angle: Angle in radians (any range)

    Returns:
        Normalized angle in [-pi, pi] range
    """
    return math.remainder(angle, _TWO_PI)


def angle_difference(reference_angle: float, current_angle: float) -> float:
    """
    Calculate the shortest angular difference between reference and current angles.
    This prevents the 270° transition issue by always taking the shortest path.

    Uses a single math.remainder call, which directly computes the shortest-path
    difference in [-pi, pi].

    Args:
        reference_angle: Reference orientation in radians
        current_angle: Current orientation in radians

    Returns:
        Angle difference in [-pi, pi] range, positive = CCW rotation needed
    """
    return math.remainder(reference_angle - current_angle, _TWO_PI)


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

"""
Orientation Utilities

Helpers for converting between 3D Euler angles and quaternions, and
computing quaternion angle errors.
"""

from typing import Iterable, Optional

import numpy as np
from scipy.spatial.transform import Rotation


def euler_xyz_to_quat_wxyz(euler_xyz: Iterable[float]) -> np.ndarray:
    """
    Convert 3D Euler angles (roll, pitch, yaw) to quaternion [w, x, y, z].
    """
    euler = np.array(list(euler_xyz), dtype=float)
    if euler.shape != (3,):
        raise ValueError(f"Expected 3-element Euler angles, got shape {euler.shape}")

    quat_xyzw = Rotation.from_euler("xyz", euler, degrees=False).as_quat()
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=float)


def quat_wxyz_to_euler_xyz(quat_wxyz: Iterable[float]) -> np.ndarray:
    """
    Convert quaternion [w, x, y, z] to 3D Euler angles (roll, pitch, yaw).
    """
    quat = np.array(list(quat_wxyz), dtype=float)
    if quat.shape != (4,):
        raise ValueError(f"Expected 4-element quaternion, got shape {quat.shape}")

    quat_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]], dtype=float)
    return Rotation.from_quat(quat_xyzw).as_euler("xyz", degrees=False)


def quat_angle_error(q_target: Iterable[float], q_current: Iterable[float]) -> float:
    """
    Compute the shortest-angle error between two quaternions (radians).
    """
    q1 = np.array(list(q_target), dtype=float)
    q2 = np.array(list(q_current), dtype=float)
    if q1.shape != (4,) or q2.shape != (4,):
        raise ValueError("Quaternion inputs must be shape (4,)")

    dot = float(np.dot(q1, q2))
    dot = float(np.clip(np.abs(dot), -1.0, 1.0))
    return 2.0 * np.arccos(dot)


def quat_angle_error_symmetric(
    q_target: Iterable[float],
    q_current: Iterable[float],
    symmetry_quats: Optional[np.ndarray] = None,
) -> float:
    """
    Compute shortest-angle error between quaternions under symmetry rotations.

    Default symmetry assumes opposite faces are identical: 180° rotations about
    body X/Y/Z axes are considered equivalent.
    """
    q_t = np.array(list(q_target), dtype=float)
    q_c = np.array(list(q_current), dtype=float)
    if q_t.shape != (4,) or q_c.shape != (4,):
        raise ValueError("Quaternion inputs must be shape (4,)")

    if symmetry_quats is None:
        symmetry_quats = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],  # identity
                [0.0, 1.0, 0.0, 0.0],  # 180° about X
                [0.0, 0.0, 1.0, 0.0],  # 180° about Y
                [0.0, 0.0, 0.0, 1.0],  # 180° about Z
            ],
            dtype=float,
        )

    q_t = _normalize_quat(q_t)
    q_c = _normalize_quat(q_c)

    min_err = float("inf")
    for sym in symmetry_quats:
        q_equiv = _quat_multiply(q_t, sym)
        q_equiv = _normalize_quat(q_equiv)
        err = quat_angle_error(q_equiv, q_c)
        if err < min_err:
            min_err = err
    return min_err


def closest_symmetric_quat(
    q_target: Iterable[float],
    q_current: Iterable[float],
    symmetry_quats: Optional[np.ndarray] = None,
) -> np.ndarray:
    """
    Return the symmetric equivalent of q_target closest to q_current.
    """
    q_t = np.array(list(q_target), dtype=float)
    q_c = np.array(list(q_current), dtype=float)
    if q_t.shape != (4,) or q_c.shape != (4,):
        raise ValueError("Quaternion inputs must be shape (4,)")

    if symmetry_quats is None:
        symmetry_quats = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],  # identity
                [0.0, 1.0, 0.0, 0.0],  # 180 deg about X
                [0.0, 0.0, 1.0, 0.0],  # 180 deg about Y
                [0.0, 0.0, 0.0, 1.0],  # 180 deg about Z
            ],
            dtype=float,
        )

    q_t = _normalize_quat(q_t)
    q_c = _normalize_quat(q_c)

    best_q = q_t
    min_err = float("inf")
    for sym in symmetry_quats:
        q_equiv = _quat_multiply(q_t, sym)
        q_equiv = _normalize_quat(q_equiv)
        err = quat_angle_error(q_equiv, q_c)
        if err < min_err:
            min_err = err
            best_q = q_equiv

    return best_q


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Quaternion multiply (wxyz)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion (wxyz)."""
    norm = np.linalg.norm(q)
    if norm <= 1e-12:
        return q
    return q / norm

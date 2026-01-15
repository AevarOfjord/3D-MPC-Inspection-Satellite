"""
Trajectory Utilities

Shared helpers for building time-parameterized trajectories with
curvature-based speed profiles.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np


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


def apply_hold_segments(
    trajectory: np.ndarray,
    dt: float,
    hold_start: float,
    hold_end: float,
) -> Tuple[np.ndarray, float]:
    """Insert hold segments at start and end of a trajectory."""
    if trajectory.size == 0:
        return trajectory, 0.0

    dt = float(dt) if dt and dt > 0 else 0.05
    hold_start = float(max(hold_start, 0.0))
    hold_end = float(max(hold_end, 0.0))

    start_samples = int(round(hold_start / dt))
    end_samples = int(round(hold_end / dt))

    start_state = trajectory[0].copy()
    start_state[0] = 0.0
    start_state[4:7] = 0.0

    end_state = trajectory[-1].copy()
    end_state[4:7] = 0.0

    segments = []
    time_offset = 0.0

    if start_samples > 0:
        start_block = np.repeat(start_state[None, :], start_samples, axis=0)
        start_block[:, 0] = np.arange(start_samples) * dt
        segments.append(start_block)
        time_offset = hold_start

    traj_body = trajectory.copy()
    traj_body[:, 0] += time_offset
    segments.append(traj_body)
    time_offset = traj_body[-1, 0]

    if end_samples > 0:
        end_block = np.repeat(end_state[None, :], end_samples, axis=0)
        end_block[:, 0] = time_offset + np.arange(1, end_samples + 1) * dt
        segments.append(end_block)
        time_offset = end_block[-1, 0]

    full_traj = np.vstack(segments)
    return full_traj, float(time_offset)

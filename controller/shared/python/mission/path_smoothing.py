"""Path smoothing utilities for nonlinear, bump-free mission references."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np


def _as_points(path: Sequence[Sequence[float]]) -> np.ndarray:
    points = np.array(path, dtype=float)
    if points.ndim != 2 or points.shape[0] == 0:
        return np.zeros((0, 3), dtype=float)
    if points.shape[1] < 3:
        padded = np.zeros((points.shape[0], 3), dtype=float)
        padded[:, : points.shape[1]] = points
        return padded
    return points[:, :3]


def _remove_consecutive_duplicates(points: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    if points.shape[0] <= 1:
        return points
    kept = [points[0]]
    for i in range(1, points.shape[0]):
        if float(np.linalg.norm(points[i] - kept[-1])) > eps:
            kept.append(points[i])
    return np.array(kept, dtype=float)


def _resample_polyline(points: np.ndarray, ds_target_m: float) -> np.ndarray:
    if points.shape[0] <= 1:
        return points.copy()

    ds = max(1e-4, float(ds_target_m))
    seg = np.linalg.norm(points[1:] - points[:-1], axis=1)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    total = float(cum[-1])
    if total <= 1e-12:
        return np.vstack((points[0], points[-1]))

    n_samples = max(2, int(np.ceil(total / ds)) + 1)
    sample_s = np.linspace(0.0, total, num=n_samples)
    out = np.zeros((n_samples, 3), dtype=float)
    seg_idx = 0
    for i, s in enumerate(sample_s):
        while seg_idx + 1 < len(cum) and cum[seg_idx + 1] < s:
            seg_idx += 1
        if seg_idx >= points.shape[0] - 1:
            out[i] = points[-1]
            continue
        s0 = float(cum[seg_idx])
        s1 = float(cum[seg_idx + 1])
        span = max(1e-12, s1 - s0)
        t = (s - s0) / span
        out[i] = points[seg_idx] + t * (points[seg_idx + 1] - points[seg_idx])

    out[0] = points[0]
    out[-1] = points[-1]
    return out


def _catmull_rom_point(
    p0: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
    t0: float,
    t1: float,
    t2: float,
    t3: float,
    t: float,
) -> np.ndarray:
    def _interp(
        pa: np.ndarray, pb: np.ndarray, ta: float, tb: float, tq: float
    ) -> np.ndarray:
        if abs(tb - ta) <= 1e-12:
            return pa
        w0 = (tb - tq) / (tb - ta)
        w1 = (tq - ta) / (tb - ta)
        return w0 * pa + w1 * pb

    a1 = _interp(p0, p1, t0, t1, t)
    a2 = _interp(p1, p2, t1, t2, t)
    a3 = _interp(p2, p3, t2, t3, t)
    b1 = _interp(a1, a2, t0, t2, t)
    b2 = _interp(a2, a3, t1, t3, t)
    return _interp(b1, b2, t1, t2, t)


def sample_centripetal_catmull_rom(
    path: Sequence[Sequence[float]],
    *,
    alpha: float = 0.5,
    ds_nominal_m: float = 0.05,
    min_samples_per_segment: int = 4,
) -> np.ndarray:
    """Return a dense centripetal Catmull-Rom curve that passes through waypoints."""
    points = _remove_consecutive_duplicates(_as_points(path))
    n = points.shape[0]
    if n <= 2:
        return points.copy()

    out: list[np.ndarray] = [points[0]]
    ds_nominal = max(1e-4, float(ds_nominal_m))
    alpha_val = float(alpha)

    for i in range(n - 1):
        p1 = points[i]
        p2 = points[i + 1]
        p0 = points[i - 1] if i > 0 else (p1 + (p1 - p2))
        p3 = points[i + 2] if i + 2 < n else (p2 + (p2 - p1))

        def _next_t(prev_t: float, pa: np.ndarray, pb: np.ndarray) -> float:
            d = float(np.linalg.norm(pb - pa))
            return prev_t + max(d**alpha_val, 1e-6)

        t0 = 0.0
        t1 = _next_t(t0, p0, p1)
        t2 = _next_t(t1, p1, p2)
        t3 = _next_t(t2, p2, p3)

        seg_len = float(np.linalg.norm(p2 - p1))
        n_seg = max(min_samples_per_segment, int(np.ceil(seg_len / (0.5 * ds_nominal))))

        for j in range(1, n_seg + 1):
            # Include each segment endpoint; skip duplicate across segments.
            if i < n - 2 and j == n_seg:
                continue
            u = j / float(n_seg)
            tq = t1 + (t2 - t1) * u
            out.append(_catmull_rom_point(p0, p1, p2, p3, t0, t1, t2, t3, tq))

    out_arr = np.array(out, dtype=float)
    out_arr[0] = points[0]
    out_arr[-1] = points[-1]
    return out_arr


def _point_to_segment_distance(
    point: np.ndarray, a: np.ndarray, b: np.ndarray
) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - a))
    t = float(np.dot(point - a, ab) / denom)
    t = min(1.0, max(0.0, t))
    proj = a + t * ab
    return float(np.linalg.norm(point - proj))


def max_distance_to_polyline(points: np.ndarray, polyline: np.ndarray) -> float:
    if points.shape[0] == 0 or polyline.shape[0] == 0:
        return 0.0
    if polyline.shape[0] == 1:
        return float(np.max(np.linalg.norm(points - polyline[0], axis=1)))

    max_d = 0.0
    for p in points:
        best = float("inf")
        for i in range(polyline.shape[0] - 1):
            d = _point_to_segment_distance(p, polyline[i], polyline[i + 1])
            if d < best:
                best = d
        if best > max_d:
            max_d = best
    return float(max_d)


def smooth_polyline_centripetal(
    path: Sequence[Sequence[float]],
    *,
    ds_target_m: float = 0.05,
    max_deviation_m: float = 0.02,
) -> tuple[list[tuple[float, float, float]], dict[str, Any]]:
    """Smooth + resample a path with a deviation-safe fallback to linear resampling."""
    original = _remove_consecutive_duplicates(_as_points(path))
    metadata: dict[str, Any] = {
        "active": True,
        "fallback": False,
        "method": "centripetal_catmull_rom",
        "ds_target_m": float(max(1e-4, ds_target_m)),
        "max_deviation_m": float(max(0.0, max_deviation_m)),
        "measured_deviation_m": 0.0,
    }

    if original.shape[0] <= 1:
        return [tuple(map(float, p)) for p in original], metadata

    linear_resampled = _resample_polyline(original, metadata["ds_target_m"])
    if original.shape[0] <= 2:
        return [tuple(map(float, p)) for p in linear_resampled], metadata

    try:
        dense = sample_centripetal_catmull_rom(
            original,
            alpha=0.5,
            ds_nominal_m=metadata["ds_target_m"],
        )
        smooth_resampled = _resample_polyline(dense, metadata["ds_target_m"])
        deviation = max_distance_to_polyline(smooth_resampled, original)
        metadata["measured_deviation_m"] = float(deviation)
        if deviation > metadata["max_deviation_m"]:
            metadata["fallback"] = True
            final = linear_resampled
        else:
            final = smooth_resampled
    except Exception:
        metadata["fallback"] = True
        metadata["measured_deviation_m"] = float("inf")
        final = linear_resampled

    final[0] = original[0]
    final[-1] = original[-1]
    return [tuple(map(float, p)) for p in final], metadata

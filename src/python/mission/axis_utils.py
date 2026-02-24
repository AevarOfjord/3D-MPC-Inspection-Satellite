"""
Shared axis inference utilities for scan path geometry analysis.

Extracted here to avoid circular imports between unified_compiler and
runtime_loader (which import from each other via compile_unified_mission_path).
"""

from __future__ import annotations

import numpy as np


def infer_scan_axis_from_path(points: np.ndarray) -> np.ndarray | None:
    """Infer the scan axis from a point cloud using SVD.

    Handles two path shapes:

    **Case 1 — Linear path** (e.g. a straight approach along an OBJ axis):
        The dominant direction (``vt[0]``, max-variance) is the scan axis.
        Detected when ``σ₀ ≥ 10 × σ₁``.

    **Case 2 — Helical / circular path** (spiral scan around an object):
        The helix axis is the *minimum*-variance direction (``vt[-1]``).
        The two in-plane singular values (σ₀ ≈ σ₁) dominate the axial one
        (σ₂).  Detected when ``σ₀ ≥ 3 × σ₂``.

    Returns an unsigned unit vector along the inferred axis, or ``None`` if
    the path shape is ambiguous and callers should fall back to the declared
    JSON axis.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 5:
        return None
    centered = pts - np.mean(pts, axis=0)
    try:
        _, sv, vt = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    if sv.size < 3:
        return None

    s0, s1, s2 = float(sv[0]), float(sv[1]), float(sv[2])

    def _unit(v: np.ndarray) -> np.ndarray | None:
        n = float(np.linalg.norm(v))
        return np.array(v, dtype=float) / n if n > 1e-12 else None

    # Case 1: nearly linear path — dominant direction is the scan axis.
    if s0 > 1e-9 and (s1 <= 1e-9 or s0 >= 10.0 * s1):
        return _unit(vt[0])

    # Case 2: helical / circular path — helix axis is the minimum-variance direction.
    if s2 > 1e-9 and s0 >= 3.0 * s2:
        return _unit(vt[-1])

    return None


def snap_axis_if_near_cardinal(axis: np.ndarray) -> np.ndarray:
    """Snap to nearest ±X/Y/Z if the axis is already very close (|dot| ≥ 0.98).

    Keeps mission intent crisp when asset paths have small numerical tilt.
    Returns the original (unnormalised) vector if no snap is possible.
    """
    vec = np.array(axis, dtype=float).reshape(-1)
    if vec.size != 3:
        return axis
    mags = np.abs(vec)
    idx = int(np.argmax(mags))
    if mags[idx] < 0.98:
        return axis
    snapped = np.zeros(3, dtype=float)
    snapped[idx] = 1.0 if vec[idx] >= 0.0 else -1.0
    return snapped

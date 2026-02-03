"""
MPCC Test Path Generators

Complex, parameterized paths for MPCC validation and demos.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Tuple

import numpy as np

PathPoints = List[Tuple[float, float, float]]


def _compute_path_length(path: PathPoints) -> float:
    if len(path) < 2:
        return 0.0
    pts = np.array(path, dtype=float)
    return float(np.sum(np.linalg.norm(pts[1:] - pts[:-1], axis=1)))


def _densify_waypoints(waypoints: PathPoints, step_size: float = 0.1) -> PathPoints:
    if len(waypoints) < 2:
        return waypoints
    dense: PathPoints = [tuple(map(float, waypoints[0]))]
    for a, b in zip(waypoints, waypoints[1:]):
        p0 = np.array(a, dtype=float)
        p1 = np.array(b, dtype=float)
        seg = p1 - p0
        dist = float(np.linalg.norm(seg))
        if dist < 1e-9:
            continue
        steps = max(2, int(np.ceil(dist / max(step_size, 1e-3))))
        for i in range(1, steps + 1):
            p = p0 + seg * (i / steps)
            dense.append(tuple(float(x) for x in p))
    return dense


def build_helical_arc_path(
    radius: float = 2.0,
    z_end: float = 2.0,
    turns: float = 0.25,
    num_points: int = 240,
) -> PathPoints:
    t = np.linspace(0.0, 2.0 * np.pi * turns, num_points)
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    z = np.linspace(0.0, z_end, num_points)
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def build_figure_eight_path(
    scale: float = 2.0,
    z_amp: float = 0.4,
    num_points: int = 400,
) -> PathPoints:
    t = np.linspace(0.0, 2.0 * np.pi, num_points)
    x = scale * np.sin(t)
    y = scale * np.sin(t) * np.cos(t)
    z = z_amp * np.sin(2.0 * t)
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def build_s_curve_path(
    length: float = 6.0,
    y_amp: float = 1.0,
    z_amp: float = 0.6,
    num_points: int = 260,
) -> PathPoints:
    t = np.linspace(0.0, 1.0, num_points)
    x = -0.5 * length + length * t
    y = y_amp * np.sin(2.0 * np.pi * t)
    z = z_amp * np.sin(np.pi * t)
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def build_spiral_inward_path(
    r_start: float = 2.5,
    r_end: float = 0.4,
    z_end: float = 1.5,
    turns: float = 2.0,
    num_points: int = 360,
) -> PathPoints:
    t = np.linspace(0.0, 1.0, num_points)
    r = r_start + (r_end - r_start) * t
    theta = 2.0 * np.pi * turns * t
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = z_end * t
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def build_lissajous_path(
    x_amp: float = 2.2,
    y_amp: float = 1.6,
    z_amp: float = 0.8,
    num_points: int = 520,
) -> PathPoints:
    t = np.linspace(0.0, 2.0 * np.pi, num_points)
    x = x_amp * np.sin(3.0 * t)
    y = y_amp * np.sin(4.0 * t + 0.5 * np.pi)
    z = z_amp * np.sin(2.0 * t)
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def build_clover_path(
    scale: float = 2.0,
    z_amp: float = 0.5,
    num_points: int = 480,
) -> PathPoints:
    t = np.linspace(0.0, 2.0 * np.pi, num_points)
    r = scale * np.sin(2.0 * t)
    x = r * np.cos(t)
    y = r * np.sin(t)
    z = z_amp * np.cos(3.0 * t)
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def build_helix_wave_path(
    radius: float = 1.8,
    wave_amp: float = 0.5,
    z_end: float = 2.5,
    turns: float = 2.5,
    num_points: int = 420,
) -> PathPoints:
    t = np.linspace(0.0, 2.0 * np.pi * turns, num_points)
    r = radius + wave_amp * np.sin(3.0 * t)
    x = r * np.cos(t)
    y = r * np.sin(t)
    z = np.linspace(0.0, z_end, num_points)
    return list(zip(x.tolist(), y.tolist(), z.tolist()))


def build_box_spiral_path(
    half_extent: float = 2.5,
    shrink: float = 0.6,
    layers: int = 4,
    z_step: float = 0.3,
    step_size: float = 0.12,
) -> PathPoints:
    waypoints: PathPoints = []
    half = half_extent
    z = 0.0
    x = half
    y = half
    waypoints.append((x, y, z))
    for _ in range(layers):
        waypoints.append((-half, half, z))
        z += z_step
        waypoints.append((-half, -half, z))
        z += z_step
        waypoints.append((half, -half, z))
        z += z_step
        waypoints.append((half, half, z))
        half = max(half - shrink, 0.4)
    return _densify_waypoints(waypoints, step_size=step_size)


def build_waypoint_maze_path(step_size: float = 0.1) -> PathPoints:
    waypoints: PathPoints = [
        (0.0, 0.0, 0.0),
        (1.5, 0.0, 0.2),
        (1.5, 1.5, 0.4),
        (0.0, 1.5, 0.6),
        (-1.5, 1.0, 0.8),
        (-1.0, -1.0, 1.0),
        (0.5, -1.5, 0.8),
        (1.5, -0.5, 0.5),
    ]
    return _densify_waypoints(waypoints, step_size=step_size)


MPCC_TEST_CASES: Dict[str, Dict[str, object]] = {
    "helical_arc": {
        "title": "Helical Arc (3D)",
        "description": "Quarter-turn arc with vertical rise.",
        "builder": build_helical_arc_path,
        "path_speed": 0.2,
    },
    "figure_eight": {
        "title": "Figure-8 Lemniscate",
        "description": "Crossing loop with mild Z oscillation.",
        "builder": build_figure_eight_path,
        "path_speed": 0.15,
    },
    "s_curve": {
        "title": "S-Curve with Climb",
        "description": "Long S with vertical lift.",
        "builder": build_s_curve_path,
        "path_speed": 0.12,
    },
    "spiral_inward": {
        "title": "Inward Spiral",
        "description": "Multi-turn spiral closing to center.",
        "builder": build_spiral_inward_path,
        "path_speed": 0.18,
    },
    "lissajous": {
        "title": "Lissajous Sweep (3D)",
        "description": "Multi-frequency weave with 3D oscillation.",
        "builder": build_lissajous_path,
        "path_speed": 0.14,
    },
    "clover": {
        "title": "Clover Loop",
        "description": "Four-leaf clover with vertical ripple.",
        "builder": build_clover_path,
        "path_speed": 0.13,
    },
    "helix_wave": {
        "title": "Helix with Radius Wave",
        "description": "Helix with radial modulation and climb.",
        "builder": build_helix_wave_path,
        "path_speed": 0.16,
    },
    "box_spiral": {
        "title": "Box Spiral Ramp",
        "description": "Rectangular spiral with altitude steps.",
        "builder": build_box_spiral_path,
        "path_speed": 0.12,
    },
    "waypoint_maze": {
        "title": "Waypoint Maze (3D)",
        "description": "Multi-segment 3D path with corners.",
        "builder": build_waypoint_maze_path,
        "path_speed": 0.1,
    },
}


def build_test_path(case_key: str) -> Tuple[PathPoints, float, float]:
    case = MPCC_TEST_CASES[case_key]
    builder = case["builder"]
    assert isinstance(builder, Callable)
    path = builder()
    length = _compute_path_length(path)
    speed = float(case.get("path_speed", 0.1))
    return path, length, speed

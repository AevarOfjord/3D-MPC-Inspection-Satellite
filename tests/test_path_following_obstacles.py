import numpy as np
from satellite_control.config.constants import Constants
from satellite_control.mission.path_following import build_point_to_point_path


def _segment_distance(start: np.ndarray, end: np.ndarray, point: np.ndarray) -> float:
    seg = end - start
    seg_len_sq = float(np.dot(seg, seg))
    if seg_len_sq < 1e-12:
        return float(np.linalg.norm(point - start))
    t = float(np.dot(point - start, seg) / seg_len_sq)
    t = min(max(t, 0.0), 1.0)
    closest = start + t * seg
    return float(np.linalg.norm(point - closest))


def _min_path_distance(path, center):
    center = np.array(center, dtype=float)
    min_dist = float("inf")
    for i in range(len(path) - 1):
        p0 = np.array(path[i], dtype=float)
        p1 = np.array(path[i + 1], dtype=float)
        min_dist = min(min_dist, _segment_distance(p0, p1, center))
    return min_dist


def test_single_obstacle_arc_detour():
    start = (0.0, 0.0, 0.0)
    end = (4.0, 0.0, 0.0)
    obstacle = (2.0, 0.0, 0.0, 0.5)

    path = build_point_to_point_path(
        waypoints=[start, end],
        obstacles=[obstacle],
        step_size=0.1,
    )

    assert np.allclose(path[0], start)
    assert np.allclose(path[-1], end)

    direct_dist = float(np.linalg.norm(np.array(end) - np.array(start)))
    path_len = float(
        np.sum(
            np.linalg.norm(
                np.array(path[1:], dtype=float) - np.array(path[:-1], dtype=float),
                axis=1,
            )
        )
    )
    assert path_len > direct_dist

    r_eff = (
        obstacle[3]
        + Constants.OBSTACLE_SAFETY_MARGIN
        + Constants.OBSTACLE_TURNING_MARGIN
    )
    min_dist = _min_path_distance(path, obstacle[:3])
    assert min_dist >= r_eff - 1e-3


def test_multiple_obstacles_detour():
    start = (0.0, 0.0, 0.0)
    end = (6.0, 0.0, 0.0)
    obstacles = [
        (2.0, 0.0, 0.0, 0.4),
        (4.0, 0.3, 0.0, 0.4),
    ]

    path = build_point_to_point_path(
        waypoints=[start, end],
        obstacles=obstacles,
        step_size=0.1,
    )

    assert np.allclose(path[0], start)
    assert np.allclose(path[-1], end)

    for obs in obstacles:
        r_eff = (
            obs[3]
            + Constants.OBSTACLE_SAFETY_MARGIN
            + Constants.OBSTACLE_TURNING_MARGIN
        )
        min_dist = _min_path_distance(path, obs[:3])
        assert min_dist >= r_eff - 1e-3

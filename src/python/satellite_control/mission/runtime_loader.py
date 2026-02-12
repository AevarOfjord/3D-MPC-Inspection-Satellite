"""
Shared unified-mission loading pipeline for CLI and dashboard entry points.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.mission.mission_types import Obstacle
from satellite_control.mission.path_assets import load_path_asset
from satellite_control.mission.unified_compiler import compile_unified_mission_path
from satellite_control.mission.unified_mission import MissionDefinition, SegmentType


@dataclass
class UnifiedMissionRuntime:
    """Compiled mission artifacts required by runtime entry points."""

    simulation_config: SimulationConfig
    path: list[tuple[float, float, float]]
    path_length: float
    path_speed: float
    start_pos: tuple[float, float, float]
    end_pos: tuple[float, float, float]


def parse_unified_mission_payload(payload: Mapping[str, Any]) -> MissionDefinition:
    """
    Parse a unified mission payload.

    Raises:
        ValueError: if the payload does not match the unified mission v2 contract.
    """
    if not isinstance(payload, Mapping):
        raise ValueError("Mission payload must be an object.")
    if "segments" not in payload or "start_pose" not in payload:
        raise ValueError(
            "Unsupported legacy mission format. Expected unified mission v2."
        )
    try:
        return MissionDefinition.from_dict(dict(payload))
    except Exception as exc:
        raise ValueError(f"Invalid unified mission: {exc}") from exc


def compile_unified_mission_runtime(
    mission: MissionDefinition,
    *,
    simulation_config: SimulationConfig | None = None,
    output_frame: str | None = None,
) -> UnifiedMissionRuntime:
    """
    Compile a unified mission into a simulation-ready configuration.
    """
    base_config = simulation_config or SimulationConfig.create_default()

    # Create mutable working copy to avoid frozen dataclass violations
    sim_config = base_config.clone()

    path, path_length, path_speed, origin = compile_unified_mission_path(
        mission=mission,
        sim_config=sim_config,
        output_frame=output_frame,
    )

    # Disable Two-Body gravity (1/r^2) for runtime missions.
    # This allows simulation in relative frames (e.g., LVLH with coordinates ~10m)
    # without the physics engine interpreting them as being at the Earth's center (r=10m).
    # This ensures high-precision visualization (no jitter) while avoiding physics singularities.
    sim_config.app_config.physics.use_two_body_gravity = False

    sim_config.app_config.mpc.path_speed = float(path_speed)
    mission_state = sim_config.mission_state
    mission_state.obstacles = _to_runtime_obstacles(mission.obstacles)
    mission_state.obstacles_enabled = bool(mission_state.obstacles)
    mission_state.path_waypoints = path
    mission_state.path_length = float(path_length)
    mission_state.path_speed = float(path_speed)
    mission_state.frame_origin = origin
    (
        mission_state.scan_attitude_center,
        mission_state.scan_attitude_axis,
        mission_state.scan_attitude_direction,
    ) = _resolve_scan_attitude_context(
        mission=mission,
        origin=np.array(origin, dtype=float),
        output_frame=output_frame,
    )

    start_pos = tuple(path[0]) if path else tuple(mission.start_pose.position)
    end_pos = tuple(path[-1]) if path else start_pos

    return UnifiedMissionRuntime(
        simulation_config=sim_config,
        path=path,
        path_length=float(path_length),
        path_speed=float(path_speed),
        start_pos=start_pos,
        end_pos=end_pos,
    )


def _to_runtime_obstacles(
    obstacles: Sequence[Any],
) -> list[Obstacle]:
    runtime_obstacles: list[Obstacle] = []
    for obstacle in obstacles:
        runtime_obstacles.append(
            Obstacle(
                position=np.array(obstacle.position, dtype=float),
                radius=float(obstacle.radius),
            )
        )
    return runtime_obstacles


def _normalize_frame(frame: Any) -> str:
    """Normalize frame enum/string to uppercase token."""
    if hasattr(frame, "value"):
        return str(frame.value).upper()
    return str(frame).upper()


def _mission_uses_lvlh(mission: MissionDefinition) -> bool:
    """Check whether mission path compilation runs in LVLH frame."""
    if _normalize_frame(getattr(mission.start_pose, "frame", "ECI")) == "LVLH":
        return True
    for segment in mission.segments:
        if segment.type == SegmentType.TRANSFER:
            if _normalize_frame(getattr(segment.end_pose, "frame", "ECI")) == "LVLH":
                return True
        if segment.type == SegmentType.SCAN:
            if _normalize_frame(getattr(segment.scan, "frame", "ECI")) == "LVLH":
                return True
    return False


def _convert_position(
    position: Sequence[float],
    source_frame: str,
    target_frame: str,
    origin: np.ndarray,
) -> np.ndarray:
    """Convert position between ECI/LVLH using runtime origin translation."""
    src = _normalize_frame(source_frame)
    dst = _normalize_frame(target_frame)
    pos = np.array(position, dtype=float)
    if src == dst:
        return pos
    if src == "ECI" and dst == "LVLH":
        return pos - origin
    if src == "LVLH" and dst == "ECI":
        return pos + origin
    return pos


def _axis_to_vector(axis_token: Any) -> np.ndarray:
    """Parse '+X/-X/+Y/-Y/+Z/-Z' axis token into unit vector."""
    token = str(getattr(axis_token, "value", axis_token)).upper().strip()
    mapping = {
        "+X": np.array([1.0, 0.0, 0.0], dtype=float),
        "-X": np.array([-1.0, 0.0, 0.0], dtype=float),
        "+Y": np.array([0.0, 1.0, 0.0], dtype=float),
        "-Y": np.array([0.0, -1.0, 0.0], dtype=float),
        "+Z": np.array([0.0, 0.0, 1.0], dtype=float),
        "-Z": np.array([0.0, 0.0, -1.0], dtype=float),
    }
    return mapping.get(token, np.array([0.0, 0.0, 1.0], dtype=float))


def _quat_rotate_vector(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate vector by quaternion [w, x, y, z]."""
    q = np.array(quat_wxyz, dtype=float).reshape(-1)
    if q.size != 4:
        return vec
    q_norm = np.linalg.norm(q)
    if q_norm <= 1e-12:
        return vec
    q = q / q_norm
    w, x, y, z = q
    q_vec = np.array([x, y, z], dtype=float)
    uv = np.cross(q_vec, vec)
    uuv = np.cross(q_vec, uv)
    return vec + 2.0 * (w * uv + uuv)


def _infer_axis_from_points(points: np.ndarray) -> np.ndarray | None:
    """Infer dominant scan-axis direction from asset path samples."""
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 5:
        return None
    centered = pts - np.mean(pts, axis=0)
    try:
        _, singular_vals, vt = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    if singular_vals.size < 2:
        return None
    # Require a clear dominant direction; otherwise fall back to mission axis token.
    if singular_vals[0] <= 1e-9 or singular_vals[0] < 1.10 * singular_vals[1]:
        return None
    axis = np.array(vt[0], dtype=float)
    axis_norm = np.linalg.norm(axis)
    if axis_norm <= 1e-12:
        return None
    return axis / axis_norm


def _snap_axis_if_near_cardinal(axis: np.ndarray) -> np.ndarray:
    """
    Snap to nearest +/-X/Y/Z if already very close.

    Keeps mission intent crisp when asset paths have small numerical tilt.
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


def _resolve_scan_attitude_context(
    mission: MissionDefinition,
    origin: np.ndarray,
    output_frame: str | None,
) -> tuple[
    tuple[float, float, float] | None,
    tuple[float, float, float] | None,
    str,
]:
    """Extract scan-center and scan-axis context for MPC scan-attitude mode."""
    frame_mode = (
        output_frame.upper()
        if output_frame is not None
        else ("LVLH" if _mission_uses_lvlh(mission) else "ECI")
    )

    for segment in mission.segments:
        if segment.type != SegmentType.SCAN:
            continue

        scan_cfg = segment.scan
        scan_frame = _normalize_frame(getattr(scan_cfg, "frame", "ECI"))

        direction = getattr(scan_cfg, "direction", "CW")
        direction = str(getattr(direction, "value", direction))

        target_pose = getattr(segment, "target_pose", None)
        if target_pose is not None and getattr(target_pose, "position", None) is not None:
            center_raw = np.array(target_pose.position, dtype=float)
            center_frame = _normalize_frame(getattr(target_pose, "frame", scan_frame))
        else:
            center_raw = np.zeros(3, dtype=float)
            center_frame = scan_frame
        center = _convert_position(center_raw, center_frame, frame_mode, origin)

        axis_vec = _axis_to_vector(getattr(scan_cfg, "axis", "+Z"))

        asset_id = getattr(segment, "path_asset", None)
        if asset_id:
            try:
                asset = load_path_asset(str(asset_id))
                asset_path = np.array(asset.get("path") or [], dtype=float)
                inferred_axis = _infer_axis_from_points(asset_path)
                if inferred_axis is not None:
                    if asset_path.shape[0] >= 2:
                        displacement = asset_path[-1] - asset_path[0]
                        if np.dot(inferred_axis, displacement) < 0.0:
                            inferred_axis = -inferred_axis
                    if (
                        bool(asset.get("relative_to_obj", True))
                        and target_pose is not None
                        and getattr(target_pose, "orientation", None) is not None
                    ):
                        inferred_axis = _quat_rotate_vector(
                            np.array(target_pose.orientation, dtype=float),
                            inferred_axis,
                        )
                    axis_vec = _snap_axis_if_near_cardinal(inferred_axis)
            except Exception:
                # Fall back to mission-declared axis token.
                pass

        axis_norm = np.linalg.norm(axis_vec)
        if axis_norm <= 1e-12:
            axis_vec = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            axis_vec = axis_vec / axis_norm

        return (
            tuple(map(float, center)),
            tuple(map(float, axis_vec)),
            direction,
        )

    return None, None, "CW"

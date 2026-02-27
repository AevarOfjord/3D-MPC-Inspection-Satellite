"""
Shared unified-mission loading pipeline for CLI and dashboard entry points.
"""

from __future__ import annotations

import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from config.paths import normalize_repo_relative_str
from config.simulation_config import SimulationConfig
from mission.axis_utils import infer_scan_axis_from_path, snap_axis_if_near_cardinal
from mission.path_assets import load_path_asset
from mission.unified_compiler import compile_unified_mission_path
from mission.unified_mission import MissionDefinition, SegmentType
from runtime.policy import (
    MissionRuntimePlan,
    compile_mission_runtime_plan,
)
from utils.orientation_utils import quat_wxyz_to_euler_xyz


@dataclass
class UnifiedMissionRuntime:
    """Compiled mission artifacts required by runtime entry points."""

    simulation_config: SimulationConfig
    path: list[tuple[float, float, float]]
    path_length: float
    path_speed: float
    start_pos: tuple[float, float, float]
    end_pos: tuple[float, float, float]
    runtime_plan: MissionRuntimePlan | None = None


_STUDIO_OBJ_TARGET_PREFIX = "STUDIO_OBJ::"
_STUDIO_LOCAL_TARGET_ID = "STUDIO_LOCAL_ORIGIN"
_CANONICAL_ORBIT_TARGET_ORIGINS: dict[str, tuple[float, float, float]] = {
    "ISS": (6_799_250.0, 0.0, 0.0),
    "STARLINK-1008": (0.0, 6_854_420.0, 0.0),
}


def _axis_token_to_letter(axis_token: Any) -> str | None:
    token = str(getattr(axis_token, "value", axis_token)).strip().upper()
    if token.endswith("X"):
        return "X"
    if token.endswith("Y"):
        return "Y"
    if token.endswith("Z"):
        return "Z"
    return None


def infer_path_asset_dominant_axis_letter(asset_id: str) -> str | None:
    if not str(asset_id or "").strip():
        return None
    try:
        asset = load_path_asset(str(asset_id))
    except Exception:
        return None
    raw_path = asset.get("path") or []
    try:
        points = np.array(raw_path, dtype=float)
    except Exception:
        return None
    inferred_axis = _infer_axis_from_points(points)
    if inferred_axis is None:
        return None
    idx = int(np.argmax(np.abs(inferred_axis)))
    return ("X", "Y", "Z")[idx]


def collect_scan_axis_asset_mismatches(
    mission: MissionDefinition,
) -> list[dict[str, Any]]:
    mismatches: list[dict[str, Any]] = []
    for segment_index, segment in enumerate(mission.segments):
        if segment.type != SegmentType.SCAN:
            continue
        asset_id = str(getattr(segment, "path_asset", "") or "").strip()
        if not asset_id:
            continue
        inferred_axis = infer_path_asset_dominant_axis_letter(asset_id)
        if inferred_axis is None:
            continue
        scan_cfg = getattr(segment, "scan", None)
        declared_token = str(
            getattr(
                getattr(scan_cfg, "axis", "+Z"),
                "value",
                getattr(scan_cfg, "axis", "+Z"),
            )
        ).upper()
        declared_axis = _axis_token_to_letter(declared_token)
        if declared_axis == inferred_axis:
            continue
        mismatches.append(
            {
                "segment_index": int(segment_index),
                "path_asset": asset_id,
                "declared_axis": declared_token,
                "inferred_axis": inferred_axis,
            }
        )
    return mismatches


def apply_scan_axis_asset_migration(
    mission: MissionDefinition,
) -> list[str]:
    notices: list[str] = []
    for mismatch in collect_scan_axis_asset_mismatches(mission):
        seg_idx = int(mismatch["segment_index"])
        if seg_idx < 0 or seg_idx >= len(mission.segments):
            continue
        segment = mission.segments[seg_idx]
        if segment.type != SegmentType.SCAN:
            continue
        scan_cfg = getattr(segment, "scan", None)
        if scan_cfg is None:
            continue
        inferred_axis = str(mismatch["inferred_axis"]).upper()
        migrated_token = f"+{inferred_axis}"
        current_axis_value = getattr(scan_cfg, "axis", "+Z")
        axis_type = type(current_axis_value)
        try:
            if hasattr(axis_type, "__members__"):
                setattr(scan_cfg, "axis", axis_type(migrated_token))
            else:
                setattr(scan_cfg, "axis", migrated_token)
        except Exception:
            setattr(scan_cfg, "axis", migrated_token)
        notices.append(
            "segment[{idx}] scan.axis migrated {old_axis} -> {new_axis} using path_asset '{asset}'".format(
                idx=seg_idx,
                old_axis=str(mismatch["declared_axis"]),
                new_axis=migrated_token,
                asset=str(mismatch["path_asset"]),
            )
        )
    return notices


def parse_unified_mission_payload(payload: Mapping[str, Any]) -> MissionDefinition:
    """
    Parse a unified mission payload.

    Raises:
        ValueError: if the payload does not match the unified mission contract.
    """
    if not isinstance(payload, Mapping):
        raise ValueError("Mission payload must be an object.")
    if "segments" not in payload or "start_pose" not in payload:
        raise ValueError("Unsupported legacy mission format. Expected unified mission.")
    try:
        return MissionDefinition.from_dict(dict(payload))
    except Exception as exc:
        raise ValueError(f"Invalid unified mission: {exc}") from exc


def _resolve_reference_scan_segment(
    mission: MissionDefinition,
) -> tuple[Any | None, str]:
    target_id = str(getattr(mission, "start_target_id", "") or "").strip()
    if target_id:
        for segment in mission.segments:
            if segment.type == SegmentType.SCAN and str(segment.target_id).strip() == target_id:
                return segment, target_id
    for segment in mission.segments:
        if segment.type == SegmentType.SCAN:
            fallback_target = str(getattr(segment, "target_id", "") or "").strip()
            if fallback_target:
                return segment, fallback_target
    return None, target_id


def _canonical_target_from_studio_obj(target_raw: str) -> str | None:
    if not target_raw.startswith(_STUDIO_OBJ_TARGET_PREFIX):
        return None
    encoded_path = target_raw[len(_STUDIO_OBJ_TARGET_PREFIX) :].strip()
    if not encoded_path:
        return None
    normalized = encoded_path.replace("\\", "/").lower()
    if normalized.endswith("starlink.obj") or "/starlink/" in normalized:
        return "STARLINK-1008"
    if normalized.endswith("iss.obj") or "/iss/" in normalized:
        return "ISS"
    return None


def _canonical_orbit_target_id(target_raw: str) -> str | None:
    raw = str(target_raw or "").strip()
    if not raw:
        return None
    mapped_from_obj = _canonical_target_from_studio_obj(raw)
    if mapped_from_obj:
        return mapped_from_obj
    upper = raw.upper()
    if upper in _CANONICAL_ORBIT_TARGET_ORIGINS:
        return upper
    if "STARLINK" in upper:
        return "STARLINK-1008"
    if "ISS" in upper:
        return "ISS"
    return None


def _resolve_runtime_frame_origin(
    mission: MissionDefinition,
    *,
    compiled_origin: np.ndarray,
    output_frame: str,
) -> np.ndarray:
    # Studio built-in OBJ targets (ISS/Starlink) are authored in local LVLH but
    # should run around the existing orbiting Viewer targets.
    if _normalize_frame(output_frame) != "LVLH":
        return np.array(compiled_origin, dtype=float)
    _, target_id = _resolve_reference_scan_segment(mission)
    target_raw = str(target_id or "").strip()
    if not target_raw.startswith(_STUDIO_OBJ_TARGET_PREFIX):
        return np.array(compiled_origin, dtype=float)
    canonical = _canonical_orbit_target_id(target_raw)
    if not canonical:
        return np.array(compiled_origin, dtype=float)
    origin = _CANONICAL_ORBIT_TARGET_ORIGINS.get(canonical)
    if origin is None:
        return np.array(compiled_origin, dtype=float)
    return np.array(origin, dtype=float)


def _resolve_visualization_scan_object(
    mission: MissionDefinition,
    *,
    origin: np.ndarray,
    output_frame: str,
) -> dict[str, Any] | None:
    segment, target_id = _resolve_reference_scan_segment(mission)
    target_raw = str(target_id or "").strip()
    if not target_raw or target_raw.upper() == _STUDIO_LOCAL_TARGET_ID:
        return None
    canonical_target = _canonical_orbit_target_id(target_raw)
    if canonical_target in _CANONICAL_ORBIT_TARGET_ORIGINS:
        # Viewer already renders canonical orbit targets; do not add duplicates.
        return None

    position = np.zeros(3, dtype=float)
    orientation_euler = [0.0, 0.0, 0.0]
    if segment is not None and getattr(segment, "target_pose", None) is not None:
        target_pose = segment.target_pose
        position = _convert_position(
            position=getattr(target_pose, "position", (0.0, 0.0, 0.0)),
            source_frame=_normalize_frame(getattr(target_pose, "frame", output_frame)),
            target_frame=output_frame,
            origin=origin,
        )
        orientation = getattr(target_pose, "orientation", None)
        if orientation is not None:
            try:
                euler = quat_wxyz_to_euler_xyz(np.array(orientation, dtype=float))
                orientation_euler = [
                    float(euler[0]),
                    float(euler[1]),
                    float(euler[2]),
                ]
            except Exception:
                orientation_euler = [0.0, 0.0, 0.0]

    upper = target_raw.upper()
    if target_raw.startswith(_STUDIO_OBJ_TARGET_PREFIX):
        encoded_path = target_raw[len(_STUDIO_OBJ_TARGET_PREFIX) :].strip()
        if not encoded_path:
            return None
        return {
            "type": "mesh",
            "position": [float(position[0]), float(position[1]), float(position[2])],
            "orientation": orientation_euler,
            "radius": 1.0,
            "height": 1.0,
            "obj_path": normalize_repo_relative_str(encoded_path),
        }
    if "STARLINK" in upper or "ISS" in upper:
        return None
    return None


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
    scan_axis_migration_notices = apply_scan_axis_asset_migration(mission)

    resolved_output_frame = (
        output_frame.upper()
        if output_frame is not None
        else ("LVLH" if _mission_uses_lvlh(mission) else "ECI")
    )

    path, path_length, path_speed, origin, pointing_spans = (
        compile_unified_mission_path(
            mission=mission,
            sim_config=sim_config,
            output_frame=resolved_output_frame,
            include_pointing_spans=True,
        )
    )

    runtime_plan = compile_mission_runtime_plan(
        mission=mission,
        path_length_m=float(path_length),
        default_path_speed=float(sim_config.app_config.mpc.path_speed),
        path_speed_min=float(getattr(sim_config.app_config.mpc, "path_speed_min", 0.0)),
        path_speed_max=float(getattr(sim_config.app_config.mpc, "path_speed_max", 0.0)),
        hold_duration_s=float(getattr(sim_config.mission_state, "path_hold_end", 10.0)),
        margin_s=float(
            getattr(
                sim_config.app_config.reference_scheduler, "duration_margin_s", 30.0
            )
        ),
    )

    # Disable Two-Body gravity (1/r^2) for runtime missions.
    # This allows simulation in relative frames (e.g., LVLH with coordinates ~10m)
    # without the physics engine interpreting them as being at the Earth's center (r=10m).
    # This ensures high-precision visualization (no jitter) while avoiding physics singularities.
    sim_config.app_config.physics.use_two_body_gravity = False

    runtime_path_speed = float(runtime_plan.path_speed_mps)
    sim_config.app_config.mpc.path_speed = runtime_path_speed
    mission_state = sim_config.mission_state
    mission_state.path_waypoints = path
    mission_state.path_length = float(path_length)
    mission_state.path_speed = runtime_path_speed
    overrides = getattr(mission, "overrides", None)
    raw_hold_schedule = getattr(overrides, "hold_schedule", []) if overrides else []
    mission_state.path_hold_schedule = []
    for item in raw_hold_schedule or []:
        if not isinstance(item, dict):
            continue
        try:
            path_index = int(item.get("path_index", 0))
            duration_s = max(0.0, float(item.get("duration_s", 0.0)))
        except Exception:
            continue
        mission_state.path_hold_schedule.append(
            {"path_index": path_index, "duration_s": duration_s}
        )
    mission_state.path_hold_schedule.sort(
        key=lambda item: int(item.get("path_index", 0))
    )
    mission_state.path_hold_active_index = None
    mission_state.path_hold_started_at_s = None
    mission_state.path_hold_completed = set()
    mission_state.path_tracking_estimated_duration = float(
        runtime_plan.required_duration_s
    )
    runtime_origin = _resolve_runtime_frame_origin(
        mission,
        compiled_origin=np.array(origin, dtype=float),
        output_frame=resolved_output_frame,
    )
    mission_state.frame_origin = tuple(map(float, runtime_origin))
    mission_state.path_frame = resolved_output_frame
    mission_state.pointing_path_spans = list(pointing_spans or [])
    mission_state.scan_axis_migration_notices = list(scan_axis_migration_notices)
    scan_axis_source = str(
        getattr(
            getattr(sim_config.app_config, "controller_contracts", None),
            "scan_axis_source",
            "planner",
        )
    )
    (
        mission_state.scan_attitude_center,
        mission_state.scan_attitude_axis,
        mission_state.scan_attitude_direction,
    ) = _resolve_scan_attitude_context(
        mission=mission,
        origin=runtime_origin,
        output_frame=resolved_output_frame,
        scan_axis_source=scan_axis_source,
    )
    mission_state.visualization_scan_object = _resolve_visualization_scan_object(
        mission,
        origin=runtime_origin,
        output_frame=resolved_output_frame,
    )

    start_pos = tuple(path[0]) if path else tuple(mission.start_pose.position)
    end_pos = tuple(path[-1]) if path else start_pos

    sim_max_duration = float(sim_config.app_config.simulation.max_duration or 0.0)
    required_duration = float(runtime_plan.required_duration_s)
    scheduler_cfg = sim_config.app_config.reference_scheduler
    is_contract_run = os.environ.get("SATCTRL_CONTRACT_SCENARIO", "0").strip() in {
        "1",
        "true",
        "TRUE",
    }
    enforce_contract_min_duration = bool(
        getattr(scheduler_cfg, "enforce_contract_min_duration", True)
    )
    auto_extend_manual_duration = bool(
        getattr(scheduler_cfg, "auto_extend_manual_duration", True)
    )

    if (
        is_contract_run
        and enforce_contract_min_duration
        and (sim_max_duration <= 0.0 or sim_max_duration < required_duration)
    ):
        sim_config.app_config.simulation.max_duration = required_duration
    elif (
        auto_extend_manual_duration
        and sim_max_duration > 0.0
        and sim_max_duration < required_duration
    ):
        sim_config.app_config.simulation.max_duration = required_duration

    return UnifiedMissionRuntime(
        simulation_config=sim_config,
        path=path,
        path_length=float(path_length),
        path_speed=runtime_path_speed,
        start_pos=start_pos,
        end_pos=end_pos,
        runtime_plan=runtime_plan,
    )


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
    """Infer the helix/spiral axis from asset path samples.

    Delegates to axis_utils.infer_scan_axis_from_path which uses the
    minimum-variance SVD direction (vt[-1]) — the correct helix axis.
    """
    return infer_scan_axis_from_path(np.asarray(points, dtype=float))


def _snap_axis_if_near_cardinal(axis: np.ndarray) -> np.ndarray:
    """Snap to nearest ±X/Y/Z if already very close. Delegates to axis_utils."""
    return snap_axis_if_near_cardinal(axis)


def _resolve_scan_attitude_context(
    mission: MissionDefinition,
    origin: np.ndarray,
    output_frame: str | None,
    scan_axis_source: str = "planner",
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
        if (
            target_pose is not None
            and getattr(target_pose, "position", None) is not None
        ):
            center_raw = np.array(target_pose.position, dtype=float)
            center_frame = _normalize_frame(getattr(target_pose, "frame", scan_frame))
        else:
            center_raw = np.zeros(3, dtype=float)
            center_frame = scan_frame
        center = _convert_position(center_raw, center_frame, frame_mode, origin)

        axis_vec = _axis_to_vector(getattr(scan_cfg, "axis", "+Z"))

        axis_source = str(scan_axis_source).strip().lower()
        asset_id = getattr(segment, "path_asset", None)

        # Priority 1: explicit scan_axis stored in the path asset (set by
        # the user in the path-maker UI when the asset was generated).
        asset_scan_axis_applied = False
        if asset_id:
            try:
                asset = load_path_asset(str(asset_id))
                stored_axis = asset.get("scan_axis")
                if stored_axis is not None:
                    token = str(stored_axis).strip().upper()
                    if token and token[-1] in ("X", "Y", "Z"):
                        candidate = _axis_to_vector(f"+{token[-1]}")
                        # Rotate by target orientation when path is OBJ-relative.
                        if (
                            bool(asset.get("relative_to_obj", True))
                            and target_pose is not None
                            and getattr(target_pose, "orientation", None) is not None
                        ):
                            candidate = _quat_rotate_vector(
                                np.array(target_pose.orientation, dtype=float),
                                candidate,
                            )
                        candidate = _snap_axis_if_near_cardinal(candidate)
                        axis_vec = candidate
                        asset_scan_axis_applied = True
            except Exception:
                pass  # Fall through to other strategies.

        # Priority 2: SVD inference from the path geometry (when configured).
        if not asset_scan_axis_applied and axis_source == "asset_infer" and asset_id:
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

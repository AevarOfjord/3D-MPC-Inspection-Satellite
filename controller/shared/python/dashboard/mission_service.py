"""
Mission service helpers.

Provides:
- compatibility mission migration helpers
- validation/preview support
- mission/draft persistence utilities
- compatibility API deprecation header constants
"""

from __future__ import annotations

import hashlib
import json
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from fastapi import HTTPException

from controller.configs.paths import DASHBOARD_DATA_ROOT
from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.dashboard.models import (
    LegacyMissionMigrateRequestModel,
    MissionConstraintSummaryModel,
    MissionDraftResponseModel,
    MissionDraftSaveRequestModel,
    MissionSummaryModel,
    PreviewMissionResponseModel,
    SaveMissionResponseModel,
    UnifiedMissionModel,
    ValidationIssueModel,
    ValidationReportModel,
    ValidationSummaryModel,
)
from controller.shared.python.mission.repository import (
    MISSIONS_DIR,
    sanitize_mission_name,
    with_json_extension,
)
from controller.shared.python.mission.runtime_loader import (
    collect_scan_axis_asset_mismatches,
    compile_unified_mission_runtime,
    parse_unified_mission_payload,
)

DRAFTS_DIR = DASHBOARD_DATA_ROOT / "mission_drafts"
MIGRATION_DOC_LINK = "https://github.com/AevarOfjord/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel/blob/main/docs/MIGRATION.md"
LEGACY_SUNSET_HTTP = "Mon, 18 May 2026 00:00:00 GMT"
LEGACY_DEPRECATION_HEADERS: dict[str, str] = {
    "Deprecation": "true",
    "Sunset": LEGACY_SUNSET_HTTP,
    "Link": f'<{MIGRATION_DOC_LINK}>; rel="deprecation"',
}
SCAN_AXIS_ASSET_MISMATCH_CODE = "SCAN_AXIS_ASSET_MISMATCH"
SCAN_AXIS_MIGRATION_NOTICE_TAG = "migration:scan_axis_asset_mismatch"
STUDIO_LOCAL_TARGET_ID = "STUDIO_LOCAL_ORIGIN"
STUDIO_MISSION_TAG = "studio"


def _now_iso() -> str:
    return datetime.now(UTC).isoformat()


def _mission_files() -> list[Path]:
    if not MISSIONS_DIR.exists():
        return []
    return sorted(
        (path for path in MISSIONS_DIR.glob("*.json") if path.is_file()),
        key=lambda item: item.stat().st_mtime,
        reverse=True,
    )


def _default_segment_id(index: int, seg_type: str) -> str:
    return f"seg_{index + 1:03d}_{seg_type}"


def _default_mission_id(name: str, epoch: str) -> str:
    digest = hashlib.sha1(f"{name}|{epoch}".encode()).hexdigest()[:10]
    safe = sanitize_mission_name(name) or "mission"
    return f"mission_{safe}_{digest}"


def _strip_segment_fields(segment: dict[str, Any]) -> dict[str, Any]:
    payload = dict(segment)
    payload.pop("segment_id", None)
    payload.pop("title", None)
    payload.pop("notes", None)
    return payload


def to_legacy_payload(
    mission: UnifiedMissionModel | dict[str, Any],
) -> dict[str, Any]:
    """
    Convert a v2 mission payload to the compatibility unified mission contract.
    """
    mission_dict = (
        mission.model_dump(mode="json")
        if isinstance(mission, UnifiedMissionModel)
        else dict(mission)
    )
    return {
        "epoch": mission_dict["epoch"],
        "start_pose": mission_dict["start_pose"],
        "start_target_id": mission_dict.get("start_target_id"),
        "segments": [
            _strip_segment_fields(segment)
            for segment in mission_dict.get("segments", [])
        ],
        "overrides": mission_dict.get("overrides"),
    }


def migrate_legacy_payload(
    payload: dict[str, Any],
    *,
    name_hint: str | None = None,
) -> UnifiedMissionModel:
    """
    Migrate a compatibility unified mission payload to the v2 model.
    """
    mission_def = parse_unified_mission_payload(payload)
    legacy = mission_def.to_dict()
    mission_name = (name_hint or payload.get("name") or "Mission").strip() or "Mission"
    mission_id = str(payload.get("mission_id") or "").strip()
    if not mission_id:
        mission_id = _default_mission_id(mission_name, legacy.get("epoch", ""))

    now_iso = _now_iso()
    segment_payloads: list[dict[str, Any]] = []
    for idx, segment in enumerate(legacy.get("segments", [])):
        segment_payload = {
            "segment_id": str(
                segment.get("segment_id")
                or _default_segment_id(idx, segment.get("type", "segment"))
            ),
            "title": segment.get("title"),
            "notes": segment.get("notes"),
            **segment,
        }
        segment_payloads.append(segment_payload)

    metadata = (
        payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
    )
    migrated = {
        "schema_version": 2,
        "mission_id": mission_id,
        "name": mission_name,
        "epoch": legacy["epoch"],
        "start_pose": legacy["start_pose"],
        "start_target_id": payload.get("start_target_id"),
        "segments": segment_payloads,
        "overrides": legacy.get("overrides"),
        "metadata": {
            "version": int(metadata.get("version") or 1),
            "created_at": metadata.get("created_at") or now_iso,
            "updated_at": metadata.get("updated_at") or now_iso,
            "tags": metadata.get("tags") or [],
        },
    }
    return UnifiedMissionModel.model_validate(migrated)


def ensure_payload(
    payload: dict[str, Any],
    *,
    name_hint: str | None = None,
) -> UnifiedMissionModel:
    """
    Ensure arbitrary mission payload is represented as UnifiedMissionModel.
    """
    if (
        int(payload.get("schema_version") or 0) == 2
        and isinstance(payload.get("mission_id"), str)
        and isinstance(payload.get("metadata"), dict)
    ):
        return UnifiedMissionModel.model_validate(payload)
    return migrate_legacy_payload(payload, name_hint=name_hint)


def migrate_legacy_request(
    request: LegacyMissionMigrateRequestModel,
) -> UnifiedMissionModel:
    return ensure_payload(request.payload, name_hint=request.name_hint)


def summarize_constraints(
    mission: UnifiedMissionModel,
) -> MissionConstraintSummaryModel:
    speed_values: list[float] = []
    accel_values: list[float] = []
    angular_values: list[float] = []
    for segment in mission.segments:
        constraints = segment.constraints
        if constraints is None:
            continue
        if constraints.speed_max is not None:
            speed_values.append(float(constraints.speed_max))
        if constraints.accel_max is not None:
            accel_values.append(float(constraints.accel_max))
        if constraints.angular_rate_max is not None:
            angular_values.append(float(constraints.angular_rate_max))
    return MissionConstraintSummaryModel(
        speed_max=max(speed_values) if speed_values else None,
        accel_max=max(accel_values) if accel_values else None,
        angular_rate_max=max(angular_values) if angular_values else None,
    )


def _build_scan_axis_mismatch_map(
    mission: UnifiedMissionModel,
) -> dict[int, dict[str, Any]]:
    try:
        mission_def = parse_unified_mission_payload(to_legacy_payload(mission))
    except Exception:
        return {}
    mismatches = collect_scan_axis_asset_mismatches(mission_def)
    return {int(item["segment_index"]): item for item in mismatches}


def _with_scan_axis_migration_notice(
    mission: UnifiedMissionModel,
) -> tuple[UnifiedMissionModel, list[str]]:
    mismatch_map = _build_scan_axis_mismatch_map(mission)
    existing_tags = list(mission.metadata.tags or [])
    existing_tags_set = set(existing_tags)
    notices: list[str] = []

    if not mismatch_map:
        if SCAN_AXIS_MIGRATION_NOTICE_TAG in existing_tags_set:
            tags = [
                tag for tag in existing_tags if tag != SCAN_AXIS_MIGRATION_NOTICE_TAG
            ]
            return (
                mission.model_copy(
                    update={
                        "metadata": mission.metadata.model_copy(update={"tags": tags})
                    }
                ),
                notices,
            )
        return mission, notices

    updated_segments = []
    for index, segment in enumerate(mission.segments):
        mismatch = mismatch_map.get(index)
        if segment.type != "scan" or mismatch is None:
            updated_segments.append(segment)
            continue
        inferred_axis = str(mismatch.get("inferred_axis", "Z")).upper()
        migrated_axis = f"+{inferred_axis}"
        updated_segments.append(
            segment.model_copy(
                update={
                    "scan": segment.scan.model_copy(update={"axis": migrated_axis}),
                }
            )
        )
        notices.append(
            f"segments[{index}] scan.axis migrated "
            f"{mismatch.get('declared_axis')} -> {migrated_axis} "
            f"using path_asset '{mismatch.get('path_asset')}'"
        )

    tags = list(existing_tags)
    if SCAN_AXIS_MIGRATION_NOTICE_TAG not in existing_tags_set:
        tags.append(SCAN_AXIS_MIGRATION_NOTICE_TAG)

    migrated = mission.model_copy(
        update={
            "segments": updated_segments,
            "metadata": mission.metadata.model_copy(update={"tags": tags}),
        }
    )
    return migrated, notices


def build_validation_report(mission: UnifiedMissionModel) -> ValidationReportModel:
    issues: list[ValidationIssueModel] = []
    axis_mismatch_map = _build_scan_axis_mismatch_map(mission)
    mission_tags = {
        str(tag).strip().lower()
        for tag in (mission.metadata.tags or [])
        if str(tag).strip()
    }
    is_studio_mission = STUDIO_MISSION_TAG in mission_tags

    if not mission.name.strip():
        issues.append(
            ValidationIssueModel(
                code="MISSION_NAME_REQUIRED",
                severity="error",
                path="name",
                message="Mission name is required.",
                suggestion="Set a non-empty mission name.",
            )
        )

    if (
        mission.start_pose.frame == "LVLH"
        and not (mission.start_target_id or "").strip()
    ):
        issues.append(
            ValidationIssueModel(
                code="START_TARGET_REQUIRED",
                severity="error",
                path="start_target_id",
                message="LVLH start pose requires start_target_id.",
                suggestion="Choose the reference object for LVLH mission coordinates.",
            )
        )

    if not mission.segments:
        issues.append(
            ValidationIssueModel(
                code="MISSION_SEGMENTS_REQUIRED",
                severity="error",
                path="segments",
                message="Mission must contain at least one segment.",
                suggestion="Add at least one transfer, scan, or hold segment.",
            )
        )

    segment_ids: set[str] = set()
    for index, segment in enumerate(mission.segments):
        seg_path = f"segments[{index}]"
        if segment.segment_id in segment_ids:
            issues.append(
                ValidationIssueModel(
                    code="SEGMENT_ID_DUPLICATE",
                    severity="error",
                    path=f"{seg_path}.segment_id",
                    message=f"Duplicate segment_id: {segment.segment_id}",
                    suggestion="Use unique segment identifiers.",
                )
            )
        segment_ids.add(segment.segment_id)

        if segment.type == "scan":
            if not segment.target_id.strip():
                issues.append(
                    ValidationIssueModel(
                        code="SCAN_TARGET_REQUIRED",
                        severity="error",
                        path=f"{seg_path}.target_id",
                        message="Scan segment requires target_id.",
                        suggestion="Select a target object for scan segments.",
                    )
                )
            if not segment.path_asset and not is_studio_mission:
                issues.append(
                    ValidationIssueModel(
                        code="SCAN_PATH_ASSET_RECOMMENDED",
                        severity="warning",
                        path=f"{seg_path}.path_asset",
                        message="Scan segment has no path asset.",
                        suggestion="Attach a path asset generated in Scan Planner.",
                    )
                )
            mismatch = axis_mismatch_map.get(index)
            if mismatch is not None:
                inferred_axis = str(mismatch.get("inferred_axis", "Z")).upper()
                issues.append(
                    ValidationIssueModel(
                        code=SCAN_AXIS_ASSET_MISMATCH_CODE,
                        severity="warning",
                        path=f"{seg_path}.scan.axis",
                        message=(
                            "scan.axis does not match dominant path_asset axis "
                            f"({mismatch.get('declared_axis')} vs +{inferred_axis})."
                        ),
                        suggestion=(
                            "Set pair axis in Planner Step 1 and save mission to sync "
                            "scan.axis metadata."
                        ),
                    )
                )

        if (
            segment.type == "transfer"
            and segment.end_pose.frame == "LVLH"
            and not (segment.target_id or "").strip()
            and (mission.start_target_id or "").strip().upper()
            != STUDIO_LOCAL_TARGET_ID
        ):
            issues.append(
                ValidationIssueModel(
                    code="TRANSFER_TARGET_REQUIRED",
                    severity="error",
                    path=f"{seg_path}.target_id",
                    message="LVLH transfer segment requires target_id.",
                    suggestion="Select a transfer reference object.",
                )
            )

        if segment.type == "hold" and segment.duration < 0:
            issues.append(
                ValidationIssueModel(
                    code="HOLD_DURATION_INVALID",
                    severity="error",
                    path=f"{seg_path}.duration",
                    message="Hold duration must be non-negative.",
                    suggestion="Use duration >= 0.",
                )
            )

        constraints = segment.constraints
        if constraints is not None:
            for key in ("speed_max", "accel_max", "angular_rate_max"):
                value = getattr(constraints, key)
                if value is None:
                    continue
                if value <= 0:
                    issues.append(
                        ValidationIssueModel(
                            code="CONSTRAINT_NON_POSITIVE",
                            severity="error",
                            path=f"{seg_path}.constraints.{key}",
                            message=f"{key} must be > 0.",
                            suggestion="Use positive constraint values.",
                        )
                    )
                if value > 500:
                    issues.append(
                        ValidationIssueModel(
                            code="CONSTRAINT_OUTLIER",
                            severity="warning",
                            path=f"{seg_path}.constraints.{key}",
                            message=f"{key} appears unusually high ({value}).",
                            suggestion="Confirm units and intended magnitude.",
                        )
                    )

    manual_path = mission.overrides.manual_path if mission.overrides is not None else []
    if len(manual_path) > 3000:
        issues.append(
            ValidationIssueModel(
                code="MANUAL_PATH_DENSE",
                severity="warning",
                path="overrides.manual_path",
                message=f"Manual path contains {len(manual_path)} points.",
                suggestion="Downsample manual path for faster preview and run-start time.",
            )
        )

    if len(mission.segments) > 30:
        issues.append(
            ValidationIssueModel(
                code="SEGMENT_COUNT_HIGH",
                severity="warning",
                path="segments",
                message=f"Mission contains {len(mission.segments)} segments.",
                suggestion="Consider splitting very large missions for easier debugging.",
            )
        )

    summary = ValidationSummaryModel(
        errors=sum(1 for issue in issues if issue.severity == "error"),
        warnings=sum(1 for issue in issues if issue.severity == "warning"),
        info=sum(1 for issue in issues if issue.severity == "info"),
    )
    return ValidationReportModel(
        valid=summary.errors == 0,
        issues=issues,
        summary=summary,
    )


def preview_mission(mission: UnifiedMissionModel) -> PreviewMissionResponseModel:
    report = build_validation_report(mission)
    if not report.valid:
        raise HTTPException(
            status_code=422,
            detail={
                "message": "Mission failed validation.",
                "report": report.model_dump(mode="json"),
            },
        )

    mission_def = parse_unified_mission_payload(to_legacy_payload(mission))
    runtime = compile_unified_mission_runtime(
        mission_def,
        simulation_config=SimulationConfig.create_default(),
        output_frame="LVLH",
    )
    path_speed = float(runtime.path_speed)
    path_length = float(runtime.path_length)
    eta_s = float(path_length / path_speed) if path_speed > 1e-9 else 0.0

    risk_flags: list[str] = []
    if report.summary.warnings > 0:
        risk_flags.append("validation_warnings")
    if path_length > 5000:
        risk_flags.append("long_path")
    if path_speed > 3.0:
        risk_flags.append("high_speed")

    return PreviewMissionResponseModel(
        path=[list(point) for point in runtime.path],
        path_length=path_length,
        path_speed=path_speed,
        eta_s=eta_s,
        risk_flags=risk_flags,
        constraint_summary=summarize_constraints(mission),
    )


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def list_missions() -> list[MissionSummaryModel]:
    summaries: list[MissionSummaryModel] = []
    for mission_file in _mission_files():
        try:
            payload = _read_json(mission_file)
            mission = ensure_payload(payload, name_hint=mission_file.stem)
        except Exception:
            continue
        summaries.append(
            MissionSummaryModel(
                name=mission.name,
                mission_id=mission.mission_id,
                updated_at=mission.metadata.updated_at,
                segments_count=len(mission.segments),
                filename=mission_file.name,
                schema_version=2,
            )
        )
    return summaries


def load_mission(mission_id_or_name: str) -> UnifiedMissionModel:
    key = mission_id_or_name.strip()
    if not key:
        raise HTTPException(status_code=400, detail="Mission identifier is required.")

    for mission_file in _mission_files():
        try:
            payload = _read_json(mission_file)
            mission = ensure_payload(payload, name_hint=mission_file.stem)
        except Exception:
            continue
        if key in {
            mission.mission_id,
            mission.name,
            mission_file.stem,
            mission_file.name,
        }:
            migrated, _ = _with_scan_axis_migration_notice(mission)
            return migrated
    raise HTTPException(
        status_code=404, detail=f"Mission not found: {mission_id_or_name}"
    )


def save_mission(name: str, mission: UnifiedMissionModel) -> SaveMissionResponseModel:
    safe_name = sanitize_mission_name(name) or sanitize_mission_name(mission.name)
    if not safe_name:
        raise HTTPException(status_code=400, detail="Mission name is required.")
    filename = with_json_extension(safe_name)
    file_path = MISSIONS_DIR / filename

    now_iso = _now_iso()
    existing: UnifiedMissionModel | None = None
    if file_path.exists():
        try:
            existing = ensure_payload(_read_json(file_path), name_hint=file_path.stem)
        except Exception:
            existing = None

    created_at = (
        existing.metadata.created_at
        if existing and existing.metadata.created_at
        else mission.metadata.created_at or now_iso
    )
    version = (
        existing.metadata.version + 1
        if existing is not None
        else max(int(mission.metadata.version), 1)
    )
    mission_id = existing.mission_id if existing is not None else mission.mission_id

    stored = mission.model_copy(
        update={
            "name": name.strip() or mission.name,
            "mission_id": mission_id,
            "metadata": mission.metadata.model_copy(
                update={
                    "created_at": created_at,
                    "updated_at": now_iso,
                    "version": version,
                }
            ),
        }
    )
    _write_json(file_path, stored.model_dump(mode="json"))
    return SaveMissionResponseModel(
        mission_id=stored.mission_id,
        version=int(stored.metadata.version),
        saved_at=now_iso,
        filename=file_path.name,
    )


def _draft_path(draft_id: str) -> Path:
    safe = sanitize_mission_name(draft_id)
    if not safe:
        raise HTTPException(status_code=400, detail="Invalid draft id.")
    return DRAFTS_DIR / f"{safe}.json"


def save_draft(
    request: MissionDraftSaveRequestModel,
) -> MissionDraftResponseModel:
    draft_id = (
        request.draft_id.strip()
        if request.draft_id
        else f"draft_{uuid.uuid4().hex[:12]}"
    )
    path = _draft_path(draft_id)

    revision = 1
    if path.exists():
        existing = _read_json(path)
        existing_revision = int(existing.get("revision") or 0)
        if (
            request.base_revision is not None
            and request.base_revision != existing_revision
        ):
            raise HTTPException(
                status_code=409,
                detail={
                    "message": "Draft revision conflict.",
                    "expected_revision": existing_revision,
                },
            )
        revision = existing_revision + 1

    now_iso = _now_iso()
    payload = {
        "schema_version": "mission_draft",
        "draft_id": draft_id,
        "revision": revision,
        "saved_at": now_iso,
        "mission": request.mission.model_dump(mode="json"),
    }
    _write_json(path, payload)
    return MissionDraftResponseModel(
        draft_id=draft_id,
        revision=revision,
        saved_at=now_iso,
        mission=request.mission,
    )


def list_draft_ids(limit: int = 200) -> list[str]:
    drafts: list[tuple[float, str]] = []
    if not DRAFTS_DIR.exists():
        return []
    for path in DRAFTS_DIR.glob("*.json"):
        if not path.is_file():
            continue
        try:
            drafts.append((path.stat().st_mtime, path.stem))
        except OSError:
            continue
    drafts.sort(key=lambda item: item[0], reverse=True)
    return [draft_id for _, draft_id in drafts[:limit]]


def load_draft(draft_id: str) -> MissionDraftResponseModel:
    path = _draft_path(draft_id)
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"Draft not found: {draft_id}")
    payload = _read_json(path)
    return MissionDraftResponseModel(
        draft_id=str(payload.get("draft_id") or draft_id),
        revision=int(payload.get("revision") or 1),
        saved_at=str(payload.get("saved_at") or _now_iso()),
        mission=UnifiedMissionModel.model_validate(payload.get("mission") or {}),
    )

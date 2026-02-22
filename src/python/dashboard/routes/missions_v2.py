"""V2 mission authoring API routes."""

from __future__ import annotations

from dashboard.mission_v2_service import (
    build_validation_report,
    list_draft_ids_v2,
    list_missions_v2,
    load_draft_v2,
    load_mission_v2,
    migrate_legacy_request_to_v2,
    preview_v2_mission,
    save_draft_v2,
    save_mission_v2,
)
from dashboard.models import (
    LegacyMissionMigrateRequestV2Model,
    MissionDraftResponseV2Model,
    MissionDraftSaveRequestV2Model,
    MissionSummaryV2Model,
    PreviewMissionV2ResponseModel,
    SaveMissionV2Request,
    SaveMissionV2ResponseModel,
    UnifiedMissionV2Model,
    ValidationReportV2Model,
)
from fastapi import APIRouter

router = APIRouter(prefix="/api/v2", tags=["missions-v2"])


@router.post("/missions/validate", response_model=ValidationReportV2Model)
async def validate_mission_v2(mission: UnifiedMissionV2Model):
    return build_validation_report(mission)


@router.post("/missions/preview", response_model=PreviewMissionV2ResponseModel)
async def preview_mission_v2(mission: UnifiedMissionV2Model):
    return preview_v2_mission(mission)


@router.post("/missions", response_model=SaveMissionV2ResponseModel)
async def create_or_update_mission_v2(request: SaveMissionV2Request):
    return save_mission_v2(request.name, request.mission)


@router.get("/missions", response_model=list[MissionSummaryV2Model])
async def list_missions_endpoint_v2():
    return list_missions_v2()


@router.get("/missions/{mission_id}", response_model=UnifiedMissionV2Model)
async def get_mission_v2(mission_id: str):
    return load_mission_v2(mission_id)


@router.post("/missions/drafts", response_model=MissionDraftResponseV2Model)
async def save_mission_draft_v2(request: MissionDraftSaveRequestV2Model):
    return save_draft_v2(request)


@router.get("/missions/drafts/list")
async def list_mission_drafts_v2():
    return {"draft_ids": list_draft_ids_v2()}


@router.get("/missions/drafts/{draft_id}", response_model=MissionDraftResponseV2Model)
async def get_mission_draft_v2(draft_id: str):
    return load_draft_v2(draft_id)


@router.post("/missions/migrate_legacy", response_model=UnifiedMissionV2Model)
async def migrate_legacy_mission_payload_v2(
    request: LegacyMissionMigrateRequestV2Model,
):
    return migrate_legacy_request_to_v2(request)

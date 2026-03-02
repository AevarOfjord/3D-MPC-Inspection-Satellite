"""Mission authoring API routes."""

from __future__ import annotations

from fastapi import APIRouter

from controller.shared.python.dashboard.mission_service import (
    build_validation_report,
    list_draft_ids,
    list_missions,
    load_draft,
    load_mission,
    migrate_legacy_request,
    preview_mission,
    save_draft,
    save_mission,
)
from controller.shared.python.dashboard.models import (
    LegacyMissionMigrateRequestModel,
    MissionDraftResponseModel,
    MissionDraftSaveRequestModel,
    MissionSummaryModel,
    PreviewMissionResponseModel,
    SaveMissionRequest,
    SaveMissionResponseModel,
    UnifiedMissionModel,
    ValidationReportModel,
)

router = APIRouter(prefix="/api/v2", tags=["missions"])


@router.post("/missions/validate", response_model=ValidationReportModel)
async def validate_mission(mission: UnifiedMissionModel):
    return build_validation_report(mission)


@router.post("/missions/preview", response_model=PreviewMissionResponseModel)
async def preview_mission_endpoint(mission: UnifiedMissionModel):
    return preview_mission(mission)


@router.post("/missions", response_model=SaveMissionResponseModel)
async def create_or_update_mission(request: SaveMissionRequest):
    return save_mission(request.name, request.mission)


@router.get("/missions", response_model=list[MissionSummaryModel])
async def list_missions_endpoint():
    return list_missions()


@router.get("/missions/{mission_id}", response_model=UnifiedMissionModel)
async def get_mission(mission_id: str):
    return load_mission(mission_id)


@router.post("/missions/drafts", response_model=MissionDraftResponseModel)
async def save_mission_draft(request: MissionDraftSaveRequestModel):
    return save_draft(request)


@router.get("/missions/drafts/list")
async def list_mission_drafts():
    return {"draft_ids": list_draft_ids()}


@router.get("/missions/drafts/{draft_id}", response_model=MissionDraftResponseModel)
async def get_mission_draft(draft_id: str):
    return load_draft(draft_id)


@router.post("/missions/migrate_legacy", response_model=UnifiedMissionModel)
async def migrate_legacy_mission_payload(
    request: LegacyMissionMigrateRequestModel,
):
    return migrate_legacy_request(request)

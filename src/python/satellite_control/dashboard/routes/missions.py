"""Mission CRUD and execution routes."""

import logging
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException

from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.dashboard.models import (
    PreviewUnifiedMissionResponse,
    RunMissionRequest,
    SaveUnifiedMissionRequest,
    UnifiedMissionModel,
)
from satellite_control.mission.repository import (
    list_mission_names,
    load_mission_json,
    save_mission_json,
)
from satellite_control.mission.repository import (
    resolve_mission_file as resolve_mission_file_repo,
)
from satellite_control.mission.runtime_loader import (
    compile_unified_mission_runtime,
    parse_unified_mission_payload,
)
from satellite_control.mission.unified_mission import MissionDefinition

logger = logging.getLogger("dashboard")

router = APIRouter()

# Injected at startup
_sim_manager = None


def set_dependencies(sim_manager) -> None:
    global _sim_manager
    _sim_manager = sim_manager


def _get_sim_manager():
    assert _sim_manager is not None, "set_dependencies() not called"
    return _sim_manager


def _resolve_mission_file(mission_name: str) -> Path:
    try:
        return resolve_mission_file_repo(mission_name, source_priority=("local",))
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


def _parse_unified_mission(data: dict[str, Any]) -> MissionDefinition:
    try:
        return parse_unified_mission_payload(data)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))


@router.post("/mission_v2")
async def update_mission_v2(config: UnifiedMissionModel):
    mission_def = _parse_unified_mission(config.model_dump())
    await _get_sim_manager().update_unified_mission(mission_def)
    return {
        "status": "success",
        "message": "Unified mission applied.",
    }


@router.post("/mission_v2/preview", response_model=PreviewUnifiedMissionResponse)
async def preview_mission_v2(config: UnifiedMissionModel):
    mission_def = _parse_unified_mission(config.model_dump())
    mission_runtime = compile_unified_mission_runtime(
        mission_def,
        simulation_config=SimulationConfig.create_default(),
        output_frame="ECI",
    )
    return {
        "path": [list(p) for p in mission_runtime.path],
        "path_length": float(mission_runtime.path_length),
        "path_speed": float(mission_runtime.path_speed),
    }


@router.get("/mission_v2")
async def get_current_mission_v2():
    mgr = _get_sim_manager()
    if not mgr.current_unified_mission:
        raise HTTPException(status_code=404, detail="No unified mission set")
    return mgr.current_unified_mission.to_dict()


@router.post("/save_mission_v2")
async def save_mission_v2(request: SaveUnifiedMissionRequest):
    """Save a unified mission configuration to JSON."""
    mission_def = _parse_unified_mission(request.config.model_dump())

    try:
        mission_file = save_mission_json(
            name=request.name,
            payload=mission_def.to_dict(),
            source="local",
        )
        return {"status": "success", "filename": mission_file.name}
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    except Exception as exc:
        logger.error(f"Failed to save unified mission: {exc}")
        raise HTTPException(status_code=500, detail=str(exc))


@router.get("/saved_missions_v2")
async def list_saved_missions_v2():
    """List all saved unified mission JSON files."""
    mission_names = list_mission_names(source_priority=("local",))
    return {"missions": mission_names}


@router.get("/mission_v2/{mission_name}")
async def load_mission_v2(mission_name: str):
    try:
        data = load_mission_json(mission_name, source_priority=("local",))
    except FileNotFoundError:
        raise HTTPException(
            status_code=404, detail=f"Unified mission not found: {mission_name}"
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Failed to read mission: {exc}")

    mission_def = _parse_unified_mission(data)
    return mission_def.to_dict()


@router.post("/run_mission")
async def run_mission(request: RunMissionRequest):
    """Spawn a subprocess to run a saved mission via CLI."""
    import subprocess
    import sys

    mission_file = _resolve_mission_file(request.mission_name)

    logger.info(f"[RUN_MISSION] Starting mission: {mission_file}")

    try:
        process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "satellite_control.cli",
                "--mission",
                str(mission_file.absolute()),
                "--no-anim",
            ],
            cwd=Path.cwd(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        logger.info(f"[RUN_MISSION] Started process PID: {process.pid}")

        return {
            "status": "started",
            "pid": process.pid,
            "mission_file": str(mission_file),
        }
    except Exception as e:
        logger.error(f"[RUN_MISSION] Failed to start: {e}")
        raise HTTPException(status_code=500, detail=str(e))

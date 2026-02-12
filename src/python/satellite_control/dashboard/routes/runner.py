"""
Routes for controlling the simulation runner.
"""
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from satellite_control.dashboard.runner_manager import RunnerManager

router = APIRouter(prefix="/runner", tags=["runner"])

# Singleton instance will be injected or accessed
_runner_manager: RunnerManager | None = None

def set_runner_manager(manager: RunnerManager):
    global _runner_manager
    _runner_manager = manager

def get_runner_manager() -> RunnerManager:
    if _runner_manager is None:
        raise RuntimeError("RunnerManager not initialized")
    return _runner_manager

from pydantic import BaseModel
from fastapi import HTTPException

class StartSimulationRequest(BaseModel):
    mission_name: str | None = None


class PresetSaveRequest(BaseModel):
    name: str
    config: dict[str, Any]


class PresetApplyRequest(BaseModel):
    name: str

@router.post("/start")
async def start_simulation(request: StartSimulationRequest | None = None):
    """Start the simulation process."""
    manager = get_runner_manager()
    mission_name = request.mission_name if request else None
    await manager.start_simulation(mission_name)
    return {"status": "started", "mission": mission_name}

@router.post("/stop")
async def stop_simulation():
    """Stop the simulation process."""
    manager = get_runner_manager()
    await manager.stop_simulation()
    return {"status": "stopped"}

@router.get("/config")
def get_config():
    """Get the current simulation configuration (defaults + overrides)."""
    manager = get_runner_manager()
    return manager.get_config()

@router.post("/config")
def update_config(overrides: dict):
    """Update the simulation configuration overrides."""
    manager = get_runner_manager()
    manager.update_config(overrides)
    return {"status": "updated", "config": manager.get_config()}

@router.post("/config/reset")
def reset_config():
    """Reset simulation configuration overrides back to defaults."""
    manager = get_runner_manager()
    manager.reset_config()
    return {"status": "reset", "config": manager.get_config()}


@router.get("/presets")
def list_presets():
    """List all saved presets."""
    manager = get_runner_manager()
    return {"presets": manager.list_presets()}


@router.post("/presets")
def save_preset(payload: PresetSaveRequest):
    """Create or update a preset."""
    manager = get_runner_manager()
    try:
        saved = manager.save_preset(payload.name, payload.config)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"status": "saved", "name": payload.name.strip(), "preset": saved}


@router.delete("/presets/{preset_name}")
def delete_preset(preset_name: str):
    """Delete a preset by name."""
    manager = get_runner_manager()
    deleted = manager.delete_preset(preset_name)
    if not deleted:
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' not found")
    return {"status": "deleted", "name": preset_name}


@router.post("/presets/apply")
def apply_preset(payload: PresetApplyRequest):
    """Apply a saved preset to active runner config overrides."""
    manager = get_runner_manager()
    try:
        config = manager.apply_preset(payload.name)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=f"Preset '{payload.name}' not found") from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"status": "applied", "name": payload.name, "config": config}


@router.post("/presets/reset")
def reset_presets():
    """Clear all saved presets."""
    manager = get_runner_manager()
    manager.clear_presets()
    return {"status": "reset"}

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for log streaming."""
    print("DEBUG: WebSocket connection attempt at /runner/ws")
    manager = get_runner_manager()
    await manager.connect(websocket)
    try:
        while True:
            # We just need to keep the connection open and listen for client disconnects
            # The server pushes data, client doesn't send much (maybe commands?)
            # For now, just wait for message and ignore or handle commands
            data = await websocket.receive_text()
            # Optional: handle commands from WS too
            if data == "start":
                await manager.start_simulation()
            elif data == "stop":
                await manager.stop_simulation()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        # manager.disconnect(websocket) # usually handled in disconnect or finally
        pass
    finally:
        manager.disconnect(websocket)

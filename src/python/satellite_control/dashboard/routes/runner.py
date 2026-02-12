"""
Routes for controlling the simulation runner.
"""
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

class StartSimulationRequest(BaseModel):
    mission_name: str | None = None

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

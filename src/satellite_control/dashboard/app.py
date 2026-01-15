import asyncio
import json
import logging
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Optional, Literal

from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation
from src.satellite_control.mission.mission_types import Mission
from src.satellite_control.mission.mission_state_manager import MissionStateManager
from src.satellite_control.config.timing import SIMULATION_DT, CONTROL_DT

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dashboard")

app = FastAPI()

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all for dev
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        # Convert numpy types to native types for JSON serialization
        json_str = json.dumps(
            message,
            default=lambda x: x.tolist() if isinstance(x, np.ndarray) else str(x),
        )
        for connection in self.active_connections:
            try:
                await connection.send_text(json_str)
            except Exception as e:
                logger.error(f"Error sending message: {e}")


manager = ConnectionManager()
from pydantic import BaseModel


# --- Data Models ---
class ObstacleModel(BaseModel):
    position: List[float]
    radius: float


class MissionConfigModel(BaseModel):
    start_position: List[float]
    target_position: List[float]
    target_orientation: List[float]
    obstacles: List[ObstacleModel]


# --- Global State ---
sim_instance = None
simulation_task = None
current_mission_config: Optional[MissionConfigModel] = (
    None  # To persist config across restarts
)
simulation_paused = True  # Start paused until explicit Run command
simulation_speed = 1.0  # 1.0x realtime
pending_steps = 0  # Queue of single-step requests while paused


async def run_simulation_loop():
    """
    Background task to run the simulation loop and broadcast state.
    """
    global sim_instance, current_mission_config, simulation_paused, simulation_speed, pending_steps
    logger.info("Starting simulation loop...")

    # Initialize simulation if not exists
    if sim_instance is None:
        from src.satellite_control.mission import create_flyby_mission
        from src.satellite_control.config.simulation_config import SimulationConfig
        from src.satellite_control.mission.mission_types import Obstacle
    from src.satellite_control.config.simulation_config import SimulationConfig
    from src.satellite_control.mission.mission_types import Obstacle

    start_pos = (
        tuple(current_mission_config.start_position)
        if current_mission_config
        else (10.0, 0.0, 0.0)
    )
    target_pos = (
        tuple(current_mission_config.target_position)
        if current_mission_config
        and getattr(current_mission_config, "target_position", None)
        else (0.0, 0.0, 0.0)
    )
    target_angle = (
        tuple(current_mission_config.target_orientation)
        if current_mission_config
        and getattr(current_mission_config, "target_orientation", None)
        else (0.0, 0.0, 0.0)
    )

    obstacles = (
        [
            (o.position[0], o.position[1], o.position[2], o.radius)
            for o in current_mission_config.obstacles
        ]
        if current_mission_config
        else []
    )

    if sim_instance is None or current_mission_config is not None:
        # Create configuration
        sim_config = SimulationConfig.create_default()
        if obstacles:
            # Convert obstacle tuples to Obstacle objects for sim_config
            sim_config.mission_state.obstacles = [
                Obstacle(position=np.array(o[:3]), radius=o[3]) for o in obstacles
            ]
            sim_config.mission_state.obstacles_enabled = len(obstacles) > 0

        # Initialize simulation
        sim_instance = SatelliteMPCLinearizedSimulation(
            simulation_config=sim_config,
            start_pos=start_pos,
            target_pos=target_pos,
            start_angle=(0.0, 0.0, 0.0),
            target_angle=target_angle,
        )

    frame_dt = 0.016

    try:
        while True:
            # Check Pause State
            if simulation_paused and pending_steps <= 0:
                await asyncio.sleep(0.1)
                continue

            # Sync Target with Config (Enable Dynamic Updates)
            tgt_pos = target_pos  # Default from init
            tgt_ori = target_angle  # Default from init

            if sim_instance and current_mission_config:
                tgt_pos = (
                    tuple(current_mission_config.target_position)
                    if hasattr(current_mission_config, "target_position")
                    else (0.0, 0.0, 0.0)
                )
                tgt_ori = (
                    tuple(current_mission_config.target_orientation)
                    if hasattr(current_mission_config, "target_orientation")
                    else (0.0, 0.0, 0.0)
                )
                sim_instance.set_target(tgt_pos, tgt_ori)
                # Ensure we stay in continuous mode
                sim_instance.set_continuous(True)
                # Trigger RRT* only when obstacles exist
                if current_mission_config.obstacles:
                    sim_instance.replan_path()

            # Step simulation (~60Hz) with speed multiplier
            if simulation_speed >= 1.0:
                steps_per_frame = max(1, int((frame_dt * simulation_speed) / SIMULATION_DT))
                sleep_time = frame_dt
            else:
                steps_per_frame = 1
                sleep_time = frame_dt / max(simulation_speed, 0.1)

            if simulation_paused and pending_steps > 0:
                step_count = max(1, pending_steps)
                pending_steps = 0
                for _ in range(step_count):
                    sim_instance.step()
            else:
                for _ in range(steps_per_frame):
                    sim_instance.step()
                # Continuous mode enabled: no auto-reset needed
                # if sim_instance.is_complete():
                #    ... reset ...

            # Broadcast state
            state = sim_instance.get_current_state()

            telemetry = {
                "time": sim_instance.simulation_time,
                "position": state[0:3],
                "quaternion": state[3:7],
                "velocity": state[7:10],
                "angular_velocity": state[10:13],
                "target_position": sim_instance.target_state[0:3]
                if hasattr(sim_instance, "target_state")
                else tgt_pos,
                "target_orientation": tgt_ori,  # Pass the commanded orientation
                "thrusters": (
                    sim_instance.last_control_output[
                        : getattr(sim_instance.mpc_controller, "num_thrusters", 12)
                    ]
                    if hasattr(sim_instance, "last_control_output")
                    else []
                ),
                "rw_torque": (
                    sim_instance.last_control_output[
                        getattr(sim_instance.mpc_controller, "num_thrusters", 12) :
                    ]
                    if hasattr(sim_instance, "last_control_output")
                    else []
                ),
                "obstacles": [
                    {"position": list(o.position), "radius": o.radius}
                    for o in (
                        sim_instance.simulation_config.mission_state.obstacles or []
                    )
                ]
                if hasattr(sim_instance, "simulation_config")
                and hasattr(sim_instance.simulation_config, "mission_state")
                else [],
                "planned_path": getattr(sim_instance, "planned_path", []),
                "paused": simulation_paused,
                "sim_speed": simulation_speed,
                "solve_time": getattr(sim_instance, "last_solve_time", 0.0),
                "pos_error": getattr(sim_instance, "last_pos_error", 0.0),
                "ang_error": getattr(sim_instance, "last_ang_error", 0.0),
            }

            await manager.broadcast(telemetry)
            await asyncio.sleep(sleep_time)

    except asyncio.CancelledError:
        logger.info("Simulation loop cancelled.")


@app.post("/mission")
async def update_mission(config: MissionConfigModel):
    """
    Update mission parameters and restart simulation.
    """
    global sim_instance, simulation_task, current_mission_config, simulation_paused

    logger.info(f"Received new mission config: {config}")

    # 1. Stop current simulation
    if simulation_task:
        simulation_task.cancel()
        try:
            await simulation_task
        except asyncio.CancelledError:
            pass

    # 2. Update config and reset instance
    current_mission_config = config
    sim_instance = None  # Force re-init in loop

    # 3. Restart loop
    simulation_task = asyncio.create_task(run_simulation_loop())

    # 4. Unpause
    simulation_paused = False

    return {"status": "success", "message": "Simulation restarted with new config"}


class ControlCommand(BaseModel):
    action: Literal["pause", "resume", "step"]
    steps: int = 1


class SpeedCommand(BaseModel):
    speed: float


@app.post("/control")
async def control_simulation(cmd: ControlCommand):
    """
    Pause, resume, or single-step the simulation.
    """
    global simulation_paused, pending_steps

    if cmd.action == "pause":
        simulation_paused = True
    elif cmd.action == "resume":
        simulation_paused = False
    elif cmd.action == "step":
        simulation_paused = True
        pending_steps += max(1, cmd.steps)

    return {
        "status": "success",
        "paused": simulation_paused,
        "pending_steps": pending_steps,
    }


@app.post("/speed")
async def update_speed(cmd: SpeedCommand):
    """
    Update simulation speed multiplier.
    """
    global simulation_speed
    simulation_speed = max(0.1, min(cmd.speed, 10.0))
    return {"status": "success", "sim_speed": simulation_speed}


@app.post("/replan")
async def replan_path():
    """
    Trigger an explicit RRT* replanning step.
    """
    if not sim_instance:
        return {"status": "error", "message": "Simulation not initialized"}

    sim_instance.replan_path()
    return {
        "status": "success",
        "waypoints": len(getattr(sim_instance, "planned_path", [])),
    }


@app.post("/reset")
async def reset_simulation():
    """
    Reset the simulation loop as if the server was restarted.
    """
    global sim_instance, simulation_task, current_mission_config
    global simulation_paused, simulation_speed, pending_steps

    if simulation_task:
        simulation_task.cancel()
        try:
            await simulation_task
        except asyncio.CancelledError:
            pass

    sim_instance = None
    current_mission_config = None
    simulation_paused = True
    simulation_speed = 1.0
    pending_steps = 0

    simulation_task = None

    return {"status": "success", "message": "Simulation reset (paused)"}


@app.on_event("startup")
async def startup_event():
    global simulation_task
    simulation_task = asyncio.create_task(run_simulation_loop())


@app.on_event("shutdown")
async def shutdown_event():
    if simulation_task:
        simulation_task.cancel()
        try:
            await simulation_task
        except asyncio.CancelledError:
            pass


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

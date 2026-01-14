import asyncio
import json
import logging
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from typing import List

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
    obstacles: List[ObstacleModel]


# --- Global State ---
sim_instance = None
simulation_task = None
current_mission_config = None  # To persist config across restarts


async def run_simulation_loop():
    """
    Background task to run the simulation loop and broadcast state.
    """
    global sim_instance, current_mission_config
    logger.info("Starting simulation loop...")

    # Initialize simulation if not exists
    if sim_instance is None:
        from src.satellite_control.mission import create_flyby_mission
        from src.satellite_control.config.simulation_config import SimulationConfig
        from src.satellite_control.mission.mission_types import Obstacle

        # Use provided config or default
        if current_mission_config:
            start_pos = tuple(current_mission_config.start_position)
            obstacles = [
                Obstacle(position=np.array(o.position), radius=o.radius)
                for o in current_mission_config.obstacles
            ]
        else:
            # Default Flyby
            default_mission = create_flyby_mission()
            start_pos = tuple(default_mission.start_position)
            obstacles = default_mission.obstacles

        # Create configuration
        sim_config = SimulationConfig.create_default()
        if obstacles:
            sim_config.mission_state.obstacles = obstacles
            sim_config.mission_state.obstacles_enabled = len(obstacles) > 0

        # Initialize simulation
        sim_instance = SatelliteMPCLinearizedSimulation(
            simulation_config=sim_config,
            start_pos=start_pos,
            target_pos=(0.0, 0.0, 0.0),
            start_angle=(0.0, 0.0, 0.0),
            target_angle=(0.0, 0.0, 0.0),
        )

    try:
        while True:
            # Step simulation (~60Hz)
            steps_per_frame = int(0.016 / SIMULATION_DT)
            for _ in range(steps_per_frame):
                sim_instance.step()
                if sim_instance.is_complete():
                    logger.info("Mission Complete. Resetting...")
                    sim_instance.reset()
                    break

            # Broadcast state
            state = sim_instance.get_current_state()  # FIX: get_current_state()

            telemetry = {
                "time": sim_instance.sim_time,
                "position": state[0:3],
                "quaternion": state[3:7],
                "velocity": state[7:10],
                "angular_velocity": state[10:13],
                "thrusters": sim_instance.last_control_output[:12]
                if hasattr(sim_instance, "last_control_output")
                else [],
                "rw_torque": sim_instance.last_control_output[12:]
                if hasattr(sim_instance, "last_control_output")
                else [],
                "obstacles": [
                    {"position": obs.position.tolist(), "radius": obs.radius}
                    for obs in sim_instance.simulation_config.mission_state.obstacles
                ]
                if sim_instance.simulation_config
                and sim_instance.simulation_config.mission_state.obstacles
                else [],
            }

            await manager.broadcast(telemetry)
            await asyncio.sleep(0.016)

    except asyncio.CancelledError:
        logger.info("Simulation loop cancelled.")


@app.post("/mission")
async def update_mission(config: MissionConfigModel):
    """
    Update mission parameters and restart simulation.
    """
    global sim_instance, simulation_task, current_mission_config

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

    return {"status": "success", "message": "Simulation restarted with new config"}


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

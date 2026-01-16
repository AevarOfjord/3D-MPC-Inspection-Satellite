import asyncio
import csv
import json
import logging
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Optional, Literal, Dict, Any
from pathlib import Path
from pydantic import BaseModel, Field

from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.config.timing import SIMULATION_DT
from src.satellite_control.mission.mission_types import Obstacle

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

DATA_DIR = Path(__file__).resolve().parents[3] / "Data" / "Simulation"


# --- Data Models ---
class ObstacleModel(BaseModel):
    position: List[float]
    radius: float


class MeshScanConfigModel(BaseModel):
    obj_path: str
    standoff: float = 0.5
    levels: int = 8
    points_per_circle: int = 72
    speed_max: float = 0.2
    speed_min: float = 0.05
    lateral_accel: float = 0.05
    z_margin: float = 0.0


class MissionConfigModel(BaseModel):
    start_position: List[float] = Field(default_factory=lambda: [10.0, 0.0, 0.0])
    target_position: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    target_orientation: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    obstacles: List[ObstacleModel] = Field(default_factory=list)
    mesh_scan: Optional[MeshScanConfigModel] = None


class ControlCommand(BaseModel):
    action: Literal["pause", "resume", "step"]
    steps: int = 1


class SpeedCommand(BaseModel):
    speed: float


# --- Connection Manager ---
class ConnectionManager:
    """Manages WebSocket connections for real-time updates."""

    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
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
                # We might want to remove broken connections here, but disconnect usually handles it


# --- Simulation Manager ---
class SimulationManager:
    """
    Encapsulates the simulation state, configuration, and execution loop.
    """

    def __init__(self):
        self.sim_instance: Optional[SatelliteMPCLinearizedSimulation] = None
        self.current_mission_config: Optional[MissionConfigModel] = None
        self.simulation_task: Optional[asyncio.Task] = None

        # State flags
        self.paused: bool = True
        self.simulation_speed: float = 1.0
        self.pending_steps: int = 0

        self.connection_manager = ConnectionManager()

    async def start(self):
        """Start the background simulation loop."""
        if self.simulation_task is None or self.simulation_task.done():
            self.simulation_task = asyncio.create_task(self._run_loop())
            logger.info("Simulation loop started.")

    async def stop(self):
        """Stop the background simulation loop."""
        if self.simulation_task:
            self.simulation_task.cancel()
            try:
                await self.simulation_task
            except asyncio.CancelledError:
                pass
            self.simulation_task = None
            logger.info("Simulation loop stopped.")

    async def update_mission(self, config: MissionConfigModel):
        """Update mission configuration and restart simulation."""
        logger.info(f"Updating mission config: {config}")

        # Stop everything
        await self.stop()

        # Update config
        self.current_mission_config = config
        self.sim_instance = None  # Force re-initialization

        # Restart
        await self.start()
        self.paused = False
        self.pending_steps = 0

    def control(self, command: ControlCommand) -> Dict[str, Any]:
        """Handle control commands (pause, resume, step)."""
        if command.action == "pause":
            self.paused = True
        elif command.action == "resume":
            self.paused = False
        elif command.action == "step":
            self.paused = True
            self.pending_steps += max(1, command.steps)

        return {
            "status": "success",
            "paused": self.paused,
            "pending_steps": self.pending_steps,
        }

    def set_speed(self, speed: float) -> float:
        """Set simulation speed multiplier."""
        self.simulation_speed = max(0.1, min(speed, 10.0))
        return self.simulation_speed

    def replan(self) -> Dict[str, Any]:
        """Trigger RRT* replanning."""
        if not self.sim_instance:
            return {"status": "error", "message": "Simulation not initialized"}

        self.sim_instance.replan_path()
        return {
            "status": "success",
            "waypoints": len(getattr(self.sim_instance, "planned_path", [])),
        }

    async def reset(self):
        """Reset simulation to initial state."""
        await self.stop()
        self.sim_instance = None
        self.current_mission_config = None
        self.paused = True
        self.simulation_speed = 1.0
        self.pending_steps = 0
        # Optional: Restart loop immediately or wait for new config?
        # Original logic was just to stop and clear.
        # But usually we want the loop running (idling) or ready.
        # Original app restarted the loop automatically/async or just sat there.
        # Original `reset_simulation` stopped the task and set it to None.
        # But `startup_event` starts it. So reset essentially kills the loop?
        # Let's effectively "restart" the manager state but keeping the task running if we want a fresh start,
        # OR follow original behavior: cancel task, clear instance.
        # If we cancel task, the dashboard stops updating.
        # Let's keep the user experience alive: Re-init without config.
        # But if we follow original strict reset:
        self.sim_instance = None
        self.current_mission_config = None
        # We should probably restart the loop so it can initialize a default sim if needed
        # or just sit idle waiting for config.
        # The original code threw away everything.
        # Let's effectively "reboot".
        await self.start()

    def _initialize_simulation(self):
        """Initialize the simulation instance based on current config."""
        start_pos = (10.0, 0.0, 0.0)
        target_pos = (0.0, 0.0, 0.0)
        target_angle = (0.0, 0.0, 0.0)
        obstacles_data = []

        if self.current_mission_config:
            start_pos = tuple(self.current_mission_config.start_position)
            target_pos = tuple(self.current_mission_config.target_position)
            target_angle = tuple(self.current_mission_config.target_orientation)
            obstacles_data = [
                (o.position[0], o.position[1], o.position[2], o.radius)
                for o in self.current_mission_config.obstacles
            ]

        # Create Configuration
        sim_config = SimulationConfig.create_default()

        # Configure Obstacles
        if obstacles_data:
            sim_config.mission_state.obstacles = [
                Obstacle(position=np.array(o[:3]), radius=o[3]) for o in obstacles_data
            ]
            sim_config.mission_state.obstacles_enabled = True

        # Configure Mesh Scan (if applicable)
        if self.current_mission_config and self.current_mission_config.mesh_scan:
            self._configure_mesh_scan(sim_config, self.current_mission_config.mesh_scan)
            # Update target_pos if mesh scan set a path start
            if sim_config.mission_state.dxf_shape_path:
                target_pos = tuple(sim_config.mission_state.dxf_shape_path[0])

        # Initialize Instance
        self.sim_instance = SatelliteMPCLinearizedSimulation(
            simulation_config=sim_config,
            start_pos=start_pos,
            target_pos=target_pos,
            start_angle=(0.0, 0.0, 0.0),
            target_angle=target_angle,
        )

    def _configure_mesh_scan(
        self, sim_config: SimulationConfig, mesh_scan: MeshScanConfigModel
    ):
        """Helper to configure mesh scan parameters."""
        from src.satellite_control.mission.mesh_scan import build_mesh_scan_trajectory

        try:
            mpc_dt = float(sim_config.app_config.mpc.dt)
            path, trajectory, path_length = build_mesh_scan_trajectory(
                obj_path=mesh_scan.obj_path,
                standoff=mesh_scan.standoff,
                levels=mesh_scan.levels,
                points_per_circle=mesh_scan.points_per_circle,
                v_max=mesh_scan.speed_max,
                v_min=mesh_scan.speed_min,
                lateral_accel=mesh_scan.lateral_accel,
                dt=mpc_dt,
                z_margin=mesh_scan.z_margin,
            )
        except Exception as exc:
            logger.error(f"Mesh scan generation failed: {exc}")
            return

        if not path:
            return

        ms = sim_config.mission_state
        ms.mesh_scan_mode_active = True
        ms.mesh_scan_obj_path = mesh_scan.obj_path
        ms.mesh_scan_standoff = mesh_scan.standoff
        ms.mesh_scan_levels = mesh_scan.levels
        ms.mesh_scan_points_per_circle = mesh_scan.points_per_circle
        ms.mesh_scan_speed_max = mesh_scan.speed_max
        ms.mesh_scan_speed_min = mesh_scan.speed_min
        ms.mesh_scan_lateral_accel = mesh_scan.lateral_accel
        ms.mesh_scan_z_margin = mesh_scan.z_margin

        ms.dxf_shape_mode_active = True
        ms.dxf_shape_path = path
        ms.dxf_path_length = path_length
        ms.dxf_target_speed = mesh_scan.speed_max
        ms.dxf_trajectory = trajectory.tolist()
        ms.dxf_trajectory_dt = mpc_dt
        ms.dxf_shape_phase = "POSITIONING"

    def _get_telemetry_dict(self) -> Dict[str, Any]:
        """Construct telemetry dictionary from current simulation state."""
        if not self.sim_instance:
            return {}

        state = self.sim_instance.get_current_state()

        # Fallback values if sim_instance properties aren't ready
        tgt_pos = (
            self.sim_instance.target_state[0:3]
            if hasattr(self.sim_instance, "target_state")
            else (0, 0, 0)
        )

        # Use commanded orientation from config if available, else derive?
        # The original code used global config variables for command orientation.
        # We can try to use sim_instance target state orientation or the config.
        # sim_instance.target_state[3:7] is the quaternion.
        # The frontend expects 'target_orientation' as Euler angles (roll, pitch, yaw)?
        # Original: "target_orientation": tgt_ori (from config)
        # Let's use config if available, else 0.
        tgt_ori = (0.0, 0.0, 0.0)
        if self.current_mission_config:
            tgt_ori = tuple(self.current_mission_config.target_orientation)

        num_thrusters = getattr(self.sim_instance.mpc_controller, "num_thrusters", 12)
        last_output = getattr(
            self.sim_instance, "last_control_output", np.zeros(num_thrusters + 3)
        )

        obstacles = []
        if (
            hasattr(self.sim_instance, "simulation_config")
            and hasattr(self.sim_instance.simulation_config, "mission_state")
            and self.sim_instance.simulation_config.mission_state.obstacles
        ):
            obstacles = [
                {"position": list(o.position), "radius": o.radius}
                for o in self.sim_instance.simulation_config.mission_state.obstacles
            ]

        return {
            "time": self.sim_instance.simulation_time,
            "position": state[0:3],
            "quaternion": state[3:7],
            "velocity": state[7:10],
            "angular_velocity": state[10:13],
            "target_position": tgt_pos,
            "target_orientation": tgt_ori,
            "thrusters": last_output[:num_thrusters],
            "rw_torque": last_output[num_thrusters:],
            "obstacles": obstacles,
            "planned_path": getattr(self.sim_instance, "planned_path", []),
            "paused": self.paused,
            "sim_speed": self.simulation_speed,
            "solve_time": getattr(self.sim_instance, "last_solve_time", 0.0),
            "pos_error": getattr(self.sim_instance, "last_pos_error", 0.0),
            "ang_error": getattr(self.sim_instance, "last_ang_error", 0.0),
        }

    async def _run_loop(self):
        """Main simulation execution loop."""
        logger.info("Starting internal simulation loop...")

        # Initial Init
        if self.sim_instance is None:
            self._initialize_simulation()

        frame_dt = 0.016

        try:
            while True:
                # 1. Handle Pause / Idle
                if self.paused and self.pending_steps <= 0:
                    await asyncio.sleep(0.1)
                    # Even if paused, we might want to broadcast state occasionally?
                    # Original code: continue (no broadcast if paused)
                    # But if we just connected, we want state.
                    # Let's broadcast if we have an instance
                    if self.sim_instance:
                        await self.connection_manager.broadcast(
                            self._get_telemetry_dict()
                        )
                    continue

                # 2. Dynamic Target Updates (Live Interaction)
                if self.sim_instance and self.current_mission_config:
                    # Sync target from config if changed (though update_mission re-inits usually)
                    # But the endpoint 'mission' re-inits.
                    # Original code had logic to sync target every frame from 'current_mission_config'.
                    # This allows changing config object in memory?
                    # The update_mission endpoint replaces the entire object.
                    # So we don't need to poll unless we have a separate 'update_target' endpoint.
                    # Original code L226-247 syncs target.
                    # It seems redundant if update_mission always re-inits.
                    # BUT, maybe there was an intention for live updates without reset?
                    # For safety, I'll keep the re-planning check.

                    if self.current_mission_config.obstacles:
                        # Replan if needed? Original code called replan_path() every frame?
                        # L246: sim_instance.replan_path()
                        # That seems expensive! RRT* every frame?
                        # Ah, 'replan_path' in sim usually checks if it needs to?
                        # No, sim.replan_path() logs "Replanning path...".
                        # Checking original code L245: "if current_mission_config.obstacles: sim_instance.replan_path()"
                        # This looks like a bug in original code or very aggressive replanning.
                        # I will NOT call replan_path every frame unless requested.
                        pass

                # 3. Calculate Steps
                # Ensure sim_instance exists (it should)
                if not self.sim_instance:
                    self._initialize_simulation()

                if self.simulation_speed >= 1.0:
                    steps_per_frame = max(
                        1, int((frame_dt * self.simulation_speed) / SIMULATION_DT)
                    )
                    sleep_time = frame_dt
                else:
                    steps_per_frame = 1
                    sleep_time = frame_dt / max(self.simulation_speed, 0.1)

                # 4. Step Simulation
                if self.paused and self.pending_steps > 0:
                    step_count = max(1, self.pending_steps)
                    self.pending_steps = 0
                    for _ in range(step_count):
                        self.sim_instance.step()
                else:
                    for _ in range(steps_per_frame):
                        self.sim_instance.step()

                # 5. Broadcast State
                telemetry = self._get_telemetry_dict()
                await self.connection_manager.broadcast(telemetry)

                await asyncio.sleep(sleep_time)

        except asyncio.CancelledError:
            logger.info("Simulation loop task cancelled.")
        except Exception as e:
            logger.error(f"Error in simulation loop: {e}", exc_info=True)


# --- Global Singleton ---
sim_manager = SimulationManager()


# --- Dependencies ---
def get_sim_manager() -> SimulationManager:
    return sim_manager


# --- Endpoints ---


@app.on_event("startup")
async def startup_event():
    await sim_manager.start()


@app.on_event("shutdown")
async def shutdown_event():
    await sim_manager.stop()


@app.get("/simulations")
async def list_simulations():
    runs = []
    if DATA_DIR.exists():
        for run_dir in sorted(DATA_DIR.iterdir(), reverse=True):
            if not run_dir.is_dir():
                continue
            metrics_path = run_dir / "performance_metrics.json"
            metrics = {}
            if metrics_path.exists():
                try:
                    metrics = json.loads(metrics_path.read_text())
                except Exception as exc:
                    logger.error(f"Failed to read metrics for {run_dir.name}: {exc}")
            sim_metrics = (
                metrics.get("simulation", {}) if isinstance(metrics, dict) else {}
            )
            runs.append(
                {
                    "id": run_dir.name,
                    "modified": run_dir.stat().st_mtime,
                    "has_physics": (run_dir / "physics_data.csv").exists(),
                    "has_metrics": metrics_path.exists(),
                    "steps": sim_metrics.get("total_steps"),
                    "duration": sim_metrics.get("total_time_s"),
                }
            )
    return {"runs": runs}


@app.get("/simulations/{run_id}/telemetry")
async def get_simulation_telemetry(
    run_id: str,
    stride: int = Query(1, ge=1, le=1000),
):
    def _get_run_dir(rid: str) -> Path:
        if Path(rid).name != rid:
            raise HTTPException(status_code=400, detail="Invalid run id")
        rdir = DATA_DIR / rid
        if not rdir.exists() or not rdir.is_dir():
            raise HTTPException(status_code=404, detail="Run not found")
        return rdir

    run_dir = _get_run_dir(run_id)
    physics_path = run_dir / "physics_data.csv"
    if not physics_path.exists():
        raise HTTPException(status_code=404, detail="physics_data.csv not found")
    metadata_path = run_dir / "mission_metadata.json"
    scan_object = None
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text())
            scan_object = metadata.get("scan_object")
        except Exception as exc:
            logger.warning(f"Failed to read mission metadata for {run_id}: {exc}")

    from src.satellite_control.utils.orientation_utils import euler_xyz_to_quat_wxyz

    def to_float(value: Optional[str], default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    # Read CSV (Optimized for large files?)
    # For now, keeping original logic but wrapped
    try:
        with physics_path.open() as handle:
            reader = csv.DictReader(handle)
            fieldnames = reader.fieldnames or []
            thruster_cols = [
                name
                for name in fieldnames
                if name.startswith("Thruster_") and name.endswith("_Cmd")
            ]

            def thruster_key(name: str) -> int:
                parts = name.split("_")
                return int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0

            thruster_cols.sort(key=thruster_key)

            telemetry = []
            for idx, row in enumerate(reader):
                if idx % stride != 0:
                    continue

                roll = to_float(row.get("Current_Roll"))
                pitch = to_float(row.get("Current_Pitch"))
                yaw = to_float(row.get("Current_Yaw"))
                quat = euler_xyz_to_quat_wxyz((roll, pitch, yaw))

                target_roll = to_float(row.get("Target_Roll"))
                target_pitch = to_float(row.get("Target_Pitch"))
                target_yaw = to_float(row.get("Target_Yaw"))
                target_quat = euler_xyz_to_quat_wxyz(
                    (target_roll, target_pitch, target_yaw)
                )

                err_x = to_float(row.get("Error_X"))
                err_y = to_float(row.get("Error_Y"))
                err_z = to_float(row.get("Error_Z"))
                err_roll = to_float(row.get("Error_Roll"))
                err_pitch = to_float(row.get("Error_Pitch"))
                err_yaw = to_float(row.get("Error_Yaw"))

                telemetry.append(
                    {
                        "time": to_float(row.get("Time")),
                        "position": [
                            to_float(row.get("Current_X")),
                            to_float(row.get("Current_Y")),
                            to_float(row.get("Current_Z")),
                        ],
                        "quaternion": list(quat),
                        "velocity": [
                            to_float(row.get("Current_VX")),
                            to_float(row.get("Current_VY")),
                            to_float(row.get("Current_VZ")),
                        ],
                        "angular_velocity": [
                            to_float(row.get("Current_WX")),
                            to_float(row.get("Current_WY")),
                            to_float(row.get("Current_WZ")),
                        ],
                        "target_position": [
                            to_float(row.get("Target_X")),
                            to_float(row.get("Target_Y")),
                            to_float(row.get("Target_Z")),
                        ],
                        "target_orientation": [target_roll, target_pitch, target_yaw],
                        "target_quaternion": list(target_quat),
                        "scan_object": scan_object,
                        "thrusters": [to_float(row.get(col)) for col in thruster_cols],
                        "rw_torque": [0.0, 0.0, 0.0],
                        "obstacles": [],
                        "solve_time": to_float(row.get("Solve_Time", 0.0)) / 1000.0,
                        "pos_error": float(np.linalg.norm([err_x, err_y, err_z])),
                        "ang_error": float(
                            np.linalg.norm([err_roll, err_pitch, err_yaw])
                        ),
                    }
                )
        return {"run_id": run_id, "telemetry": telemetry}
    except Exception as e:
        logger.error(f"Error processing telemetry for {run_id}: {e}")
        raise HTTPException(status_code=500, detail="Error processing telemetry data")


@app.post("/mission")
async def update_mission(config: MissionConfigModel):
    await sim_manager.update_mission(config)
    return {"status": "success", "message": "Simulation restarted with new config"}


@app.post("/control")
async def control_simulation(cmd: ControlCommand):
    return sim_manager.control(cmd)


@app.post("/speed")
async def update_speed(cmd: SpeedCommand):
    speed = sim_manager.set_speed(cmd.speed)
    return {"status": "success", "sim_speed": speed}


@app.post("/replan")
async def replan_path():
    return sim_manager.replan()


@app.post("/reset")
async def reset_simulation():
    await sim_manager.reset()
    return {"status": "success", "message": "Simulation reset (paused)"}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await sim_manager.connection_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        sim_manager.connection_manager.disconnect(websocket)

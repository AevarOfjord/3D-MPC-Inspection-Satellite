import asyncio
import csv
import json
import logging
import numpy as np
from fastapi import (
    FastAPI,
    WebSocket,
    WebSocketDisconnect,
    HTTPException,
    Query,
    File,
    Form,
)
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from typing import Annotated, List, Optional, Literal, Dict, Any, Union
from pathlib import Path
from pydantic import BaseModel, Field, field_validator

from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.config.timing import SIMULATION_DT
from src.satellite_control.mission.mission_types import Obstacle
from src.satellite_control.mission.unified_mission import MissionDefinition
from src.satellite_control.mission.unified_compiler import compile_unified_mission_path
from src.satellite_control.utils.orientation_utils import quat_wxyz_to_euler_xyz

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
    level_spacing: Optional[float] = (
        None  # Alternative: distance between levels in meters
    )
    points_per_circle: int = 72
    speed_max: float = 0.2
    speed_min: float = 0.05
    lateral_accel: float = 0.05
    z_margin: float = 0.0
    scan_axis: str = "Z"  # "X", "Y", or "Z"


class MissionConfigModel(BaseModel):
    start_position: List[float] = Field(default_factory=lambda: [10.0, 0.0, 0.0])
    end_position: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    end_orientation: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    obstacles: List[ObstacleModel] = Field(default_factory=list)
    mesh_scan: Optional[MeshScanConfigModel] = None


class PoseModel(BaseModel):
    frame: Literal["ECI", "LVLH"]
    position: List[float]
    orientation: Optional[List[float]] = None  # quaternion [w, x, y, z]

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: List[float]) -> List[float]:
        if len(value) != 3:
            raise ValueError("position must have length 3")
        return value

    @field_validator("orientation")
    @classmethod
    def validate_orientation(cls, value: Optional[List[float]]) -> Optional[List[float]]:
        if value is None:
            return value
        if len(value) != 4:
            raise ValueError("orientation must have length 4")
        return value


class ConstraintsModel(BaseModel):
    speed_max: Optional[float] = None
    accel_max: Optional[float] = None
    angular_rate_max: Optional[float] = None


class SplineControlModel(BaseModel):
    position: List[float]
    weight: float = 1.0

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: List[float]) -> List[float]:
        if len(value) != 3:
            raise ValueError("spline control position must have length 3")
        return value


class TransferSegmentModel(BaseModel):
    type: Literal["transfer"]
    end_pose: PoseModel
    constraints: Optional[ConstraintsModel] = None


class ScanConfigModel(BaseModel):
    frame: Literal["ECI", "LVLH"] = "LVLH"
    axis: Literal["+X", "-X", "+Y", "-Y", "+Z", "-Z", "custom"] = "+Z"
    standoff: float = 10.0
    overlap: float = 0.25
    fov_deg: float = 60.0
    pitch: Optional[float] = None
    revolutions: int = 4
    direction: Literal["CW", "CCW"] = "CW"
    sensor_axis: Literal["+Y", "-Y"] = "+Y"


class ScanSegmentModel(BaseModel):
    type: Literal["scan"]
    target_id: str
    target_pose: Optional[PoseModel] = None
    scan: ScanConfigModel
    constraints: Optional[ConstraintsModel] = None


class HoldSegmentModel(BaseModel):
    type: Literal["hold"]
    duration: float = 0.0
    constraints: Optional[ConstraintsModel] = None


MissionSegmentModel = Annotated[
    Union[TransferSegmentModel, ScanSegmentModel, HoldSegmentModel],
    Field(discriminator="type"),
]


class MissionOverridesModel(BaseModel):
    spline_controls: List[SplineControlModel] = Field(default_factory=list)


class UnifiedMissionModel(BaseModel):
    epoch: str
    start_pose: PoseModel
    segments: List[MissionSegmentModel]
    obstacles: List[ObstacleModel] = Field(default_factory=list)
    overrides: Optional[MissionOverridesModel] = None


class PreviewUnifiedMissionResponse(BaseModel):
    path: List[List[float]]
    path_length: float
    path_speed: float


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
        self.current_unified_mission: Optional[MissionDefinition] = None
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
        self.current_unified_mission = None
        self.sim_instance = None  # Force re-initialization

        # Restart
        await self.start()
        self.paused = False
        self.pending_steps = 0

    async def update_unified_mission(self, mission: MissionDefinition):
        """Update unified mission config and restart simulation."""
        logger.info("Updating unified mission config (v2).")

        await self.stop()

        self.current_unified_mission = mission
        self.current_mission_config = None
        self.sim_instance = None

        sim_config = SimulationConfig.create_default()

        path, path_length, path_speed = compile_unified_mission_path(
            mission=mission,
            sim_config=sim_config,
        )

        sim_config.app_config.mpc.path_speed = float(path_speed)
        if mission.obstacles:
            sim_config.mission_state.obstacles = [
                Obstacle(position=np.array(o.position), radius=o.radius)
                for o in mission.obstacles
            ]
            sim_config.mission_state.obstacles_enabled = True
        sim_config.mission_state.mpcc_path_waypoints = path
        sim_config.mission_state.dxf_shape_path = path
        sim_config.mission_state.dxf_path_length = float(path_length)
        sim_config.mission_state.dxf_path_speed = float(path_speed)
        sim_config.mission_state.trajectory_mode_active = False
        sim_config.mission_state.dxf_shape_mode_active = False

        start_pos = tuple(mission.start_pose.position)
        end_pos = tuple(path[-1]) if path else start_pos

        self.sim_instance = SatelliteMPCLinearizedSimulation(
            simulation_config=sim_config,
            start_pos=start_pos,
            end_pos=end_pos,
            start_angle=(0.0, 0.0, 0.0),
            end_angle=(0.0, 0.0, 0.0),
        )

        await self.start()
        self.paused = False
        self.pending_steps = 0

    def set_unified_mission(self, mission: MissionDefinition) -> None:
        """Store unified mission definition (v2) without altering simulation yet."""
        self.current_unified_mission = mission

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
        self.current_unified_mission = None
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
        end_pos = (0.0, 0.0, 0.0)
        end_angle = (0.0, 0.0, 0.0)
        obstacles_data = []

        if self.current_mission_config:
            start_pos = tuple(self.current_mission_config.start_position)
            end_pos = tuple(self.current_mission_config.end_position)
            end_angle = tuple(self.current_mission_config.end_orientation)
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
            # Update end_pos if mesh scan set a path start
            if sim_config.mission_state.dxf_shape_path:
                end_pos = tuple(sim_config.mission_state.dxf_shape_path[0])

        # Initialize Instance
        self.sim_instance = SatelliteMPCLinearizedSimulation(
            simulation_config=sim_config,
            start_pos=start_pos,
            end_pos=end_pos,
            start_angle=(0.0, 0.0, 0.0),
            end_angle=end_angle,
        )

    def _configure_mesh_scan(
        self, sim_config: SimulationConfig, mesh_scan: MeshScanConfigModel
    ):
        """Helper to configure mesh scan parameters."""
        from src.satellite_control.mission.mesh_scan import build_mesh_scan_trajectory

        try:
            mpc_dt = float(sim_config.app_config.mpc.dt)
            path, _, path_length = build_mesh_scan_trajectory(
                obj_path=mesh_scan.obj_path,
                standoff=mesh_scan.standoff,
                levels=mesh_scan.levels,
                points_per_circle=mesh_scan.points_per_circle,
                v_max=mesh_scan.speed_max,
                v_min=mesh_scan.speed_min,
                lateral_accel=mesh_scan.lateral_accel,
                dt=mpc_dt,
                z_margin=mesh_scan.z_margin,
                build_trajectory=False,
            )
        except Exception as exc:
            logger.error(f"Mesh scan generation failed: {exc}")
            return

        if not path:
            return

        # MPCC is now the only mode
        sim_config.app_config.mpc.path_speed = mesh_scan.speed_max
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

        ms.dxf_shape_mode_active = False
        ms.dxf_shape_path = path
        ms.dxf_path_length = path_length
        ms.dxf_path_speed = mesh_scan.speed_max
        ms.mpcc_path_waypoints = path
        ms.trajectory_mode_active = False

    def _get_telemetry_dict(self) -> Dict[str, Any]:
        """Construct telemetry dictionary from current simulation state."""
        if not self.sim_instance:
            return {}

        state = self.sim_instance.get_current_state()

        # Fallback values if sim_instance properties aren't ready
        ref_state = getattr(self.sim_instance, "reference_state", None)
        ref_pos = (0.0, 0.0, 0.0)
        ref_quat = None
        ref_ori = (0.0, 0.0, 0.0)

        if ref_state is not None and len(ref_state) >= 7:
            ref_pos = tuple(float(v) for v in ref_state[0:3])
            ref_quat = tuple(float(v) for v in ref_state[3:7])
            try:
                ref_ori = tuple(quat_wxyz_to_euler_xyz(ref_quat).tolist())
            except Exception:
                ref_ori = (0.0, 0.0, 0.0)
        elif self.current_mission_config:
            ref_pos = tuple(self.current_mission_config.end_position)
            ref_ori = tuple(self.current_mission_config.end_orientation)

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
            "reference_position": ref_pos,
            "reference_orientation": ref_ori,
            "reference_quaternion": ref_quat,
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
        if self.sim_instance is None and self.current_mission_config is not None:
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

                # 2. Dynamic Reference Updates (Live Interaction)
                if self.sim_instance and self.current_mission_config:
                    # Sync reference from config if changed (though update_mission re-inits usually)
                    # But the endpoint 'mission' re-inits.
                    # Original code had logic to sync reference every frame from 'current_mission_config'.
                    # This allows changing config object in memory?
                    # The update_mission endpoint replaces the entire object.
                    # So we don't need to poll unless we have a separate 'update_reference' endpoint.
                    # Original code L226-247 syncs reference.
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
        # Only scan the 50 most recent directories to avoid slowdown
        count = 0
        for run_dir in sorted(DATA_DIR.iterdir(), reverse=True):
            if not run_dir.is_dir():
                continue
            # Skip runs without physics data (speedup)
            physics_path = run_dir / "physics_data.csv"
            if not physics_path.exists():
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
                    "has_physics": True,
                    "has_metrics": metrics_path.exists(),
                    "steps": sim_metrics.get("total_steps"),
                    "duration": sim_metrics.get("total_time_s"),
                }
            )
            count += 1
            if count >= 50:
                break
    return {"runs": runs}


def _get_run_dir(rid: str) -> Path:
    """Helper to get and validate a simulation run directory."""
    if Path(rid).name != rid:
        raise HTTPException(status_code=400, detail="Invalid run id")
    rdir = DATA_DIR / rid
    if not rdir.exists() or not rdir.is_dir():
        raise HTTPException(status_code=404, detail="Run not found")
    return rdir


@app.get("/simulations/{run_id}/telemetry")
async def get_simulation_telemetry(
    run_id: str,
    stride: int = Query(1, ge=1, le=1000),
):
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

    # Read CSV
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

                reference_roll = to_float(row.get("Reference_Roll"))
                reference_pitch = to_float(row.get("Reference_Pitch"))
                reference_yaw = to_float(row.get("Reference_Yaw"))
                reference_quat = euler_xyz_to_quat_wxyz(
                    (reference_roll, reference_pitch, reference_yaw)
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
                        "reference_position": [
                            to_float(row.get("Reference_X")),
                            to_float(row.get("Reference_Y")),
                            to_float(row.get("Reference_Z")),
                        ],
                        "reference_orientation": [
                            reference_roll,
                            reference_pitch,
                            reference_yaw,
                        ],
                        "reference_quaternion": list(reference_quat),
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


@app.get("/simulations/{run_id}/video")
async def get_simulation_video(run_id: str):
    run_dir = _get_run_dir(run_id)
    video_path = run_dir / "Simulation_3D_Render.mp4"
    if not video_path.exists():
        raise HTTPException(status_code=404, detail="Video animation not found")

    return FileResponse(
        path=video_path, filename=f"simulation_{run_id}.mp4", media_type="video/mp4"
    )


@app.get("/api/models/serve")
async def serve_model_file(path: str):
    """Serve a model file from the filesystem."""
    file_path = Path(path)

    # If relative path, resolve from project root (where server runs)
    if not file_path.is_absolute():
        file_path = Path.cwd() / file_path

    logger.info(f"[MODEL SERVE] Requested: {path}, Resolved to: {file_path}")

    if not file_path.exists() or not file_path.is_file():
        logger.warning(f"[MODEL SERVE] File not found: {file_path}")
        raise HTTPException(
            status_code=404, detail=f"Model file not found: {file_path}"
        )

    return FileResponse(path=file_path, filename=file_path.name)


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


# --- Trajectory Builder Endpoints ---


@app.post("/upload_object")
async def upload_object(file: bytes = File(...), filename: str = Form(...)):
    """Upload an OBJ file to the server."""
    try:
        import aiofiles
    except Exception:
        aiofiles = None

    upload_dir = Path("OBJ_files/uploads")
    upload_dir.mkdir(parents=True, exist_ok=True)

    # Sanitize filename
    safe_name = Path(filename).name
    file_path = upload_dir / safe_name

    if aiofiles is None:
        with open(file_path, "wb") as out_file:
            out_file.write(file)
    else:
        async with aiofiles.open(file_path, "wb") as out_file:
            await out_file.write(file)

    return {"status": "success", "path": str(file_path), "filename": safe_name}


@app.post("/preview_trajectory")
async def preview_trajectory(config: MeshScanConfigModel):
    """Generate a preview of the mesh scan trajectory without running simulation."""
    from src.satellite_control.mission.mesh_scan import (
        build_mesh_scan_trajectory,
        load_obj_vertices,
        compute_mesh_bounds,
    )

    try:
        # Determine levels: use level_spacing if provided, otherwise use levels count
        levels = config.levels
        if config.level_spacing and config.level_spacing > 0:
            # Compute levels from object height and spacing
            try:
                vertices = load_obj_vertices(config.obj_path)
                _, max_bounds, _, _ = compute_mesh_bounds(vertices)
                min_bounds = vertices.min(axis=0)
                # Select axis dimension based on scan_axis
                axis_idx = 2
                if config.scan_axis == "X":
                    axis_idx = 0
                elif config.scan_axis == "Y":
                    axis_idx = 1

                object_height = max_bounds[axis_idx] - min_bounds[axis_idx]
                levels = max(1, int(object_height / config.level_spacing))
            except Exception:
                # Fallback to default if can't read object
                levels = 8

        # Use simple default dt for preview
        dt = 0.1
        path, _, path_length = build_mesh_scan_trajectory(
            obj_path=config.obj_path,
            standoff=config.standoff,
            levels=levels,
            points_per_circle=config.points_per_circle,
            v_max=config.speed_max,
            v_min=config.speed_min,
            lateral_accel=config.lateral_accel,
            dt=dt,
            z_margin=config.z_margin,
            scan_axis=config.scan_axis,
            build_trajectory=False,
        )

        # Calculate approximate duration
        speed = max(float(config.speed_max), 1e-3)
        duration = float(path_length) / speed

        return {
            "status": "success",
            "path": path,  # List of (x,y,z) tuples
            "points": len(path),
            "estimated_duration": duration,
            "path_length": path_length,
            "computed_levels": levels,
        }
    except Exception as e:
        logger.error(f"Preview generation failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


class SaveMissionRequest(BaseModel):
    name: str
    config: MissionConfigModel


@app.post("/save_mission")
async def save_mission(request: SaveMissionRequest):
    """Save the current mission configuration to a JSON file."""
    missions_dir = Path("missions")
    missions_dir.mkdir(exist_ok=True)

    # Sanitize name
    safe_name = "".join(
        c for c in request.name if c.isalnum() or c in ("-", "_")
    ).strip()
    if not safe_name:
        raise HTTPException(status_code=400, detail="Invalid mission name")

    file_path = missions_dir / f"{safe_name}.json"

    try:
        with open(file_path, "w") as f:
            f.write(request.config.model_dump_json(indent=2))

        return {"status": "success", "filename": f"{safe_name}.json"}
    except Exception as e:
        logger.error(f"Failed to save mission: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/saved_missions")
async def list_saved_missions():
    """List all saved mission JSON files."""
    missions_dir = Path("missions")
    if not missions_dir.exists():
        return {"missions": []}

    files = sorted(missions_dir.glob("*.json"))
    return {"missions": [f.name for f in files]}


class SaveUnifiedMissionRequest(BaseModel):
    name: str
    config: UnifiedMissionModel


def _sanitize_mission_name(name: str) -> str:
    safe_name = "".join(c for c in name if c.isalnum() or c in ("-", "_")).strip()
    if not safe_name:
        raise HTTPException(status_code=400, detail="Invalid mission name")
    return safe_name


def _parse_unified_mission(data: Dict[str, Any]) -> MissionDefinition:
    try:
        return MissionDefinition.from_dict(data)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"Invalid unified mission: {exc}")


@app.post("/mission_v2")
async def update_mission_v2(config: UnifiedMissionModel):
    mission_def = _parse_unified_mission(config.model_dump())
    await sim_manager.update_unified_mission(mission_def)
    return {
        "status": "success",
        "message": "Unified mission applied.",
    }


@app.post("/mission_v2/preview", response_model=PreviewUnifiedMissionResponse)
async def preview_mission_v2(config: UnifiedMissionModel):
    mission_def = _parse_unified_mission(config.model_dump())
    sim_config = SimulationConfig.create_default()
    path, path_length, path_speed = compile_unified_mission_path(
        mission=mission_def,
        sim_config=sim_config,
    )
    return {
        "path": [list(p) for p in path],
        "path_length": float(path_length),
        "path_speed": float(path_speed),
    }


@app.get("/mission_v2")
async def get_current_mission_v2():
    if not sim_manager.current_unified_mission:
        raise HTTPException(status_code=404, detail="No unified mission set")
    return sim_manager.current_unified_mission.to_dict()


@app.post("/save_mission_v2")
async def save_mission_v2(request: SaveUnifiedMissionRequest):
    """Save a unified mission configuration to JSON."""
    missions_dir = Path("missions_unified")
    missions_dir.mkdir(exist_ok=True)

    safe_name = _sanitize_mission_name(request.name)
    file_path = missions_dir / f"{safe_name}.json"

    # Validate and normalize before saving
    mission_def = _parse_unified_mission(request.config.model_dump())

    try:
        with open(file_path, "w") as f:
            json.dump(mission_def.to_dict(), f, indent=2)
        return {"status": "success", "filename": f"{safe_name}.json"}
    except Exception as exc:
        logger.error(f"Failed to save unified mission: {exc}")
        raise HTTPException(status_code=500, detail=str(exc))


@app.get("/saved_missions_v2")
async def list_saved_missions_v2():
    """List all saved unified mission JSON files."""
    missions_dir = Path("missions_unified")
    if not missions_dir.exists():
        return {"missions": []}
    files = sorted(missions_dir.glob("*.json"))
    return {"missions": [f.name for f in files]}


@app.get("/mission_v2/{mission_name}")
async def load_mission_v2(mission_name: str):
    missions_dir = Path("missions_unified")
    mission_file = missions_dir / f"{mission_name}.json"

    if not mission_file.exists():
        if not mission_name.endswith(".json"):
            mission_file = missions_dir / f"{mission_name}.json"
        if not mission_file.exists():
            raise HTTPException(
                status_code=404, detail=f"Unified mission not found: {mission_name}"
            )

    try:
        data = json.loads(mission_file.read_text())
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Failed to read mission: {exc}")

    mission_def = _parse_unified_mission(data)
    return mission_def.to_dict()


class RunMissionRequest(BaseModel):
    mission_name: str


@app.post("/run_mission")
async def run_mission(request: RunMissionRequest):
    """Spawn a subprocess to run a saved mission via CLI."""
    import subprocess
    import sys

    # Find mission file
    missions_dir = Path("missions")
    mission_file = missions_dir / f"{request.mission_name}.json"

    if not mission_file.exists():
        # Try with .json extension if not included
        if not request.mission_name.endswith(".json"):
            mission_file = missions_dir / f"{request.mission_name}.json"
        if not mission_file.exists():
            raise HTTPException(
                status_code=404, detail=f"Mission not found: {request.mission_name}"
            )

    logger.info(f"[RUN_MISSION] Starting mission: {mission_file}")

    try:
        # Spawn the simulation as a subprocess using the CLI
        # The CLI will load the mission JSON and run the simulation
        # Use DEVNULL to prevent pipe buffer from filling and blocking
        process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "src.satellite_control.cli",
                "--mission",
                str(mission_file.absolute()),
                "--headless",
            ],
            cwd=Path.cwd(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,  # Detach from parent process
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

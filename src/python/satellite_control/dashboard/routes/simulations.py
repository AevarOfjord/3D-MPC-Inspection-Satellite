"""Simulation browser & live-control routes."""

import csv
import json
import logging
from pathlib import Path

import numpy as np
from fastapi import APIRouter, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

from satellite_control.dashboard.models import ControlCommand, SpeedCommand

logger = logging.getLogger("dashboard")

router = APIRouter()

# Resolved lazily on first use via set_dependencies()
_sim_manager = None
_data_dir: Path | None = None


def set_dependencies(sim_manager, data_dir: Path) -> None:
    """Inject shared singletons after app creation."""
    global _sim_manager, _data_dir
    _sim_manager = sim_manager
    _data_dir = data_dir


def _get_sim_manager():
    assert _sim_manager is not None, "set_dependencies() not called"
    return _sim_manager


def _get_run_dir(rid: str) -> Path:
    """Helper to get and validate a simulation run directory."""
    assert _data_dir is not None
    if Path(rid).name != rid:
        raise HTTPException(status_code=400, detail="Invalid run id")
    rdir = _data_dir / rid
    if not rdir.exists() or not rdir.is_dir():
        raise HTTPException(status_code=404, detail="Run not found")
    return rdir


@router.get("/simulations")
async def list_simulations():
    assert _data_dir is not None
    runs: list[dict] = []
    if _data_dir.exists():
        count = 0
        for run_dir in sorted(_data_dir.iterdir(), reverse=True):
            if not run_dir.is_dir():
                continue
            physics_path = run_dir / "physics_data.csv"
            if not physics_path.exists():
                continue
            metrics_path = run_dir / "performance_metrics.json"
            metrics: dict = {}
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


@router.get("/simulations/{run_id}/telemetry")
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
    planned_path = None
    frame = "ECI"
    frame_origin = None
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text())
            scan_object = metadata.get("scan_object")
            planned_path = metadata.get("planned_path")
            if scan_object and scan_object.get("position") is not None:
                frame = "LVLH"
                frame_origin = scan_object.get("position")
        except Exception as exc:
            logger.warning(f"Failed to read mission metadata for {run_id}: {exc}")

    if planned_path:
        logger.info(
            f"[TELEMETRY] Loaded {len(planned_path)} points from metadata for {run_id}"
        )
    else:
        logger.info(
            f"[TELEMETRY] No planned_path in metadata for {run_id}, will compute from CSV"
        )

    from satellite_control.utils.orientation_utils import euler_xyz_to_quat_wxyz

    def to_float(value: str | None, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

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
            computed_path: list[list[float]] = []
            last_ref = None
            min_ref_step = 0.02

            for idx, row in enumerate(reader):
                ref_x = to_float(row.get("Reference_X"))
                ref_y = to_float(row.get("Reference_Y"))
                ref_z = to_float(row.get("Reference_Z"))

                if planned_path is None:
                    if last_ref is None:
                        computed_path.append([ref_x, ref_y, ref_z])
                        last_ref = (ref_x, ref_y, ref_z)
                    else:
                        dx = ref_x - last_ref[0]
                        dy = ref_y - last_ref[1]
                        dz = ref_z - last_ref[2]
                        if (dx * dx + dy * dy + dz * dz) ** 0.5 >= min_ref_step:
                            computed_path.append([ref_x, ref_y, ref_z])
                            last_ref = (ref_x, ref_y, ref_z)

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

                # Determine Frame Origin from CSV (preferred) or Metadata
                current_origin = frame_origin
                current_frame = frame
                if "Frame_Origin_X" in row:
                    ox = to_float(row.get("Frame_Origin_X"))
                    oy = to_float(row.get("Frame_Origin_Y"))
                    oz = to_float(row.get("Frame_Origin_Z"))
                    # If we have an origin, we treat this as a relative frame (LVLH-like)
                    # even if the origin is [0,0,0] (which would just be ECI centered)
                    current_origin = [ox, oy, oz]
                    current_frame = "LVLH"

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
                        "reference_position": [ref_x, ref_y, ref_z],
                        "reference_orientation": [
                            reference_roll,
                            reference_pitch,
                            reference_yaw,
                        ],
                        "reference_quaternion": list(reference_quat),
                        "scan_object": scan_object,
                        "thrusters": [to_float(row.get(col)) for col in thruster_cols],
                        "rw_torque": [
                            to_float(row.get("RW_Torque_X")),
                            to_float(row.get("RW_Torque_Y")),
                            to_float(row.get("RW_Torque_Z")),
                        ],
                        "obstacles": [],
                        "solve_time": to_float(row.get("Solve_Time", 0.0)) / 1000.0,
                        "pos_error": float(np.linalg.norm([err_x, err_y, err_z])),
                        "ang_error": float(
                            np.linalg.norm([err_roll, err_pitch, err_yaw])
                        ),
                        "frame": current_frame,
                        "frame_origin": current_origin,
                    }
                )
        if planned_path is None:
            planned_path = computed_path
        return {
            "run_id": run_id,
            "telemetry": telemetry,
            "planned_path": planned_path or [],
        }
    except Exception as e:
        logger.error(f"Error processing telemetry for {run_id}: {e}")
        raise HTTPException(status_code=500, detail="Error processing telemetry data")


@router.get("/simulations/{run_id}/video")
async def get_simulation_video(run_id: str):
    run_dir = _get_run_dir(run_id)
    video_path = run_dir / "Simulation_3D_Render.mp4"
    if not video_path.exists():
        raise HTTPException(status_code=404, detail="Video animation not found")

    return FileResponse(
        path=video_path, filename=f"simulation_{run_id}.mp4", media_type="video/mp4"
    )


@router.post("/control")
async def control_simulation(cmd: ControlCommand):
    return _get_sim_manager().control(cmd)


@router.post("/speed")
async def update_speed(cmd: SpeedCommand):
    speed = _get_sim_manager().set_speed(cmd.speed)
    return {"status": "success", "sim_speed": speed}


@router.post("/reset")
async def reset_simulation():
    await _get_sim_manager().reset()
    return {"status": "success", "message": "Simulation reset (paused)"}


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    mgr = _get_sim_manager()
    await mgr.connection_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        mgr.connection_manager.disconnect(websocket)

"""Simulation browser & live-control routes."""

import asyncio
import csv
import json
import logging
import time
from pathlib import Path

import numpy as np
from dashboard.models import ControlCommand, SpeedCommand
from fastapi import APIRouter, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

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


def _collect_runs(limit: int = 50) -> list[dict]:
    """Collect simulation run metadata from disk."""
    assert _data_dir is not None
    runs: list[dict] = []
    if not _data_dir.exists():
        return runs

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
        sim_metrics = metrics.get("simulation", {}) if isinstance(metrics, dict) else {}
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
        if count >= limit:
            break
    return runs


def _runs_signature(runs: list[dict]) -> str:
    """Stable signature used to detect run-list changes."""
    compact = [
        (
            run.get("id"),
            run.get("modified"),
            run.get("steps"),
            run.get("duration"),
            run.get("has_metrics"),
        )
        for run in runs
    ]
    return json.dumps(compact, separators=(",", ":"))


@router.get("/simulations")
async def list_simulations():
    runs = _collect_runs()
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
    obstacles: list[dict[str, object]] = []
    # Playback should be interpreted as LVLH by default.
    frame = "LVLH"
    frame_origin = [0.0, 0.0, 0.0]
    planned_path_frame = "LVLH"
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text())
            scan_object = metadata.get("scan_object")
            planned_path = metadata.get("planned_path")
            metadata_obstacles = metadata.get("obstacles")
            if isinstance(metadata_obstacles, list):
                parsed_obstacles: list[dict[str, object]] = []
                for obstacle in metadata_obstacles:
                    if not isinstance(obstacle, dict):
                        continue
                    position = obstacle.get("position")
                    radius = obstacle.get("radius")
                    if not isinstance(position, list) or len(position) < 3:
                        continue
                    try:
                        parsed_obstacles.append(
                            {
                                "position": [
                                    float(position[0]),
                                    float(position[1]),
                                    float(position[2]),
                                ],
                                "radius": float(radius),
                            }
                        )
                    except (TypeError, ValueError):
                        continue
                obstacles = parsed_obstacles
            planned_path_frame_raw = str(
                metadata.get("planned_path_frame", "LVLH")
            ).upper()
            planned_path_frame = (
                planned_path_frame_raw
                if planned_path_frame_raw in {"LVLH", "ECI"}
                else "LVLH"
            )
            frame = planned_path_frame
            meta_origin = metadata.get("frame_origin")
            if isinstance(meta_origin, list) and len(meta_origin) >= 3:
                frame_origin = [
                    float(meta_origin[0]),
                    float(meta_origin[1]),
                    float(meta_origin[2]),
                ]
            elif scan_object and scan_object.get("position") is not None:
                pos = scan_object.get("position")
                if isinstance(pos, list) and len(pos) >= 3:
                    frame_origin = [float(pos[0]), float(pos[1]), float(pos[2])]
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

    from utils.orientation_utils import (
        quat_angle_error,
        quat_wxyz_to_euler_xyz,
    )

    def to_float(value: str | None, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def parse_quat_wxyz(row: dict[str, str | None], prefix: str) -> np.ndarray:
        """Parse quaternion columns in wxyz order."""
        qw = row.get(f"{prefix}_QW")
        qx = row.get(f"{prefix}_QX")
        qy = row.get(f"{prefix}_QY")
        qz = row.get(f"{prefix}_QZ")
        quat = np.array(
            [
                to_float(qw, 1.0),
                to_float(qx, 0.0),
                to_float(qy, 0.0),
                to_float(qz, 0.0),
            ],
            dtype=float,
        )

        norm = float(np.linalg.norm(quat))
        if norm <= 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return quat / norm

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
            last_curr_yaw_deg: float | None = None
            curr_yaw_unwrapped_deg: float | None = None
            last_curr_euler_deg: np.ndarray | None = None
            curr_euler_unwrapped_deg: np.ndarray | None = None

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
                    # Keep yaw unwrapping continuous even when downsampling output rows.
                    quat_skip = parse_quat_wxyz(row, "Current")
                    curr_euler_skip = quat_wxyz_to_euler_xyz(quat_skip)
                    curr_euler_deg_skip = np.degrees(
                        np.array(curr_euler_skip, dtype=float)
                    )
                    if last_curr_euler_deg is None:
                        curr_euler_unwrapped_deg = curr_euler_deg_skip.copy()
                    else:
                        delta_euler = curr_euler_deg_skip - last_curr_euler_deg
                        delta_euler = ((delta_euler + 180.0) % 360.0) - 180.0
                        curr_euler_unwrapped_deg = (
                            curr_euler_unwrapped_deg + delta_euler
                            if curr_euler_unwrapped_deg is not None
                            else curr_euler_deg_skip.copy()
                        )
                    last_curr_euler_deg = curr_euler_deg_skip.copy()
                    curr_yaw_deg_skip = float(np.degrees(curr_euler_skip[2]))
                    if last_curr_yaw_deg is None:
                        curr_yaw_unwrapped_deg = curr_yaw_deg_skip
                    else:
                        delta = curr_yaw_deg_skip - last_curr_yaw_deg
                        delta = ((delta + 180.0) % 360.0) - 180.0
                        curr_yaw_unwrapped_deg = (curr_yaw_unwrapped_deg or 0.0) + delta
                    last_curr_yaw_deg = curr_yaw_deg_skip
                    continue

                quat = parse_quat_wxyz(row, "Current")
                curr_euler = quat_wxyz_to_euler_xyz(quat)
                curr_euler_deg = np.degrees(np.array(curr_euler, dtype=float))
                if last_curr_euler_deg is None:
                    curr_euler_unwrapped_deg = curr_euler_deg.copy()
                else:
                    delta_euler = curr_euler_deg - last_curr_euler_deg
                    delta_euler = ((delta_euler + 180.0) % 360.0) - 180.0
                    curr_euler_unwrapped_deg = (
                        curr_euler_unwrapped_deg + delta_euler
                        if curr_euler_unwrapped_deg is not None
                        else curr_euler_deg.copy()
                    )
                last_curr_euler_deg = curr_euler_deg.copy()
                curr_pitch_deg = float(np.degrees(curr_euler[1]))
                curr_yaw_deg = float(np.degrees(curr_euler[2]))
                if last_curr_yaw_deg is None:
                    curr_yaw_unwrapped_deg = curr_yaw_deg
                else:
                    delta = curr_yaw_deg - last_curr_yaw_deg
                    delta = ((delta + 180.0) % 360.0) - 180.0
                    curr_yaw_unwrapped_deg = (curr_yaw_unwrapped_deg or 0.0) + delta
                last_curr_yaw_deg = curr_yaw_deg

                reference_quat = parse_quat_wxyz(row, "Reference")
                reference_euler = quat_wxyz_to_euler_xyz(reference_quat)

                err_x = to_float(row.get("Error_X"))
                err_y = to_float(row.get("Error_Y"))
                err_z = to_float(row.get("Error_Z"))

                # Determine Frame Origin from CSV (preferred) or Metadata
                current_origin = frame_origin if frame == "LVLH" else None
                current_frame = frame
                if frame == "LVLH" and "Frame_Origin_X" in row:
                    ox = to_float(row.get("Frame_Origin_X"))
                    oy = to_float(row.get("Frame_Origin_Y"))
                    oz = to_float(row.get("Frame_Origin_Z"))
                    # If we have an origin, we treat this as a relative frame (LVLH-like)
                    # even if the origin is [0,0,0] (which would just be ECI centered)
                    current_origin = [ox, oy, oz]
                    current_frame = "LVLH"
                elif current_frame == "LVLH" and current_origin is None:
                    current_origin = [0.0, 0.0, 0.0]

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
                            float(reference_euler[0]),
                            float(reference_euler[1]),
                            float(reference_euler[2]),
                        ],
                        "reference_quaternion": list(reference_quat),
                        "scan_object": scan_object,
                        "thrusters": [to_float(row.get(col)) for col in thruster_cols],
                        "rw_torque": [
                            to_float(row.get("RW_Torque_X")),
                            to_float(row.get("RW_Torque_Y")),
                            to_float(row.get("RW_Torque_Z")),
                        ],
                        "obstacles": obstacles,
                        "solve_time": to_float(row.get("Solve_Time", 0.0)) / 1000.0,
                        "pos_error": float(np.linalg.norm([err_x, err_y, err_z])),
                        "ang_error": float(quat_angle_error(reference_quat, quat)),
                        "yaw_unwrapped_deg": float(curr_yaw_unwrapped_deg or 0.0),
                        "orientation_unwrapped_deg": [
                            float(curr_euler_unwrapped_deg[0])
                            if curr_euler_unwrapped_deg is not None
                            else float(curr_euler_deg[0]),
                            float(curr_euler_unwrapped_deg[1])
                            if curr_euler_unwrapped_deg is not None
                            else float(curr_euler_deg[1]),
                            float(curr_euler_unwrapped_deg[2])
                            if curr_euler_unwrapped_deg is not None
                            else float(curr_euler_deg[2]),
                        ],
                        "euler_unreliable": bool(abs(curr_pitch_deg) > 85.0),
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
            "planned_path_frame": planned_path_frame,
            "frame_origin": frame_origin,
        }
    except Exception as e:
        logger.error(f"Error processing telemetry for {run_id}: {e}")
        raise HTTPException(status_code=500, detail="Error processing telemetry data")


@router.get("/simulations/{run_id}/files")
async def list_simulation_files(run_id: str):
    """List all files in the simulation directory."""
    run_dir = _get_run_dir(run_id)
    files = []

    # We only want to list the files in the run directory and its immediate subdirectories (like Plots)
    # For now, let's keep it simple: generic recursive list or just top level + specific known folders?
    # Let's do a simple recursive walk for the response.

    for path in run_dir.rglob("*"):
        if path.is_file():
            rel_path = path.relative_to(run_dir)
            files.append(
                {
                    "path": str(rel_path),
                    "name": path.name,
                    "size": path.stat().st_size,
                    "type": "file",
                }
            )
        elif path.is_dir():
            rel_path = path.relative_to(run_dir)
            files.append(
                {"path": str(rel_path), "name": path.name, "type": "directory"}
            )

    # Sort by type (dir first) then name
    files.sort(key=lambda x: (x["type"] == "file", x["path"]))
    return {"files": files}


@router.get("/simulations/{run_id}/files/{file_path:path}")
async def get_simulation_file(run_id: str, file_path: str):
    """Serve a specific file from the simulation directory."""
    run_dir = _get_run_dir(run_id)
    target_path = (run_dir / file_path).resolve()

    # Security check: Ensure we haven't escaped the run dir
    if not str(target_path).startswith(str(run_dir.resolve())):
        raise HTTPException(status_code=403, detail="Access denied")

    if not target_path.exists() or not target_path.is_file():
        raise HTTPException(status_code=404, detail="File not found")

    media_type = None
    if target_path.suffix.lower() == ".json":
        media_type = "application/json"
    elif target_path.suffix.lower() == ".csv":
        media_type = "text/csv"
    elif target_path.suffix.lower() in [".txt", ".log"]:
        media_type = "text/plain"
    elif target_path.suffix.lower() == ".mp4":
        media_type = "video/mp4"
    elif target_path.suffix.lower() in [".png", ".jpg", ".jpeg", ".gif"]:
        media_type = "image/jpeg"  # or determine dynamically

    return FileResponse(
        path=target_path, filename=target_path.name, media_type=media_type
    )


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


@router.websocket("/simulations/runs/ws")
async def runs_updates_websocket(websocket: WebSocket):
    """Push simulation run list updates to clients in near-real-time."""
    await websocket.accept()
    last_signature: str | None = None

    try:
        while True:
            runs = _collect_runs()
            signature = _runs_signature(runs)
            if signature != last_signature:
                await websocket.send_json(
                    {
                        "type": "runs_snapshot"
                        if last_signature is None
                        else "runs_updated",
                        "runs": runs,
                        "latest_run_id": runs[0]["id"] if runs else None,
                        "updated_at": time.time(),
                    }
                )
                last_signature = signature
            await asyncio.sleep(1.5)
    except WebSocketDisconnect:
        return
    except Exception as exc:
        logger.debug("Runs updates websocket closed: %s", exc)

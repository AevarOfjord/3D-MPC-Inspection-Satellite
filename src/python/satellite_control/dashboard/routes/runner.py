"""
Routes for controlling the simulation runner.
"""
import asyncio
import importlib
import io
import json
import os
import re
import shutil
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from fastapi import (
    APIRouter,
    File,
    Form,
    HTTPException,
    Query,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import FileResponse, StreamingResponse

from satellite_control.dashboard.runner_manager import (
    PRESETS_FILE,
    PROJECT_ROOT,
    SIMULATION_SCRIPT,
    RunnerManager,
)
from satellite_control.mission.repository import (
    MISSIONS_DIR,
    save_mission_json,
)

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

_package_task: asyncio.Task | None = None
_package_state: dict[str, Any] = {
    "status": "idle",
    "started_at": None,
    "finished_at": None,
    "return_code": None,
    "archive_path": None,
    "log_lines": [],
    "error": None,
}
_MAX_PACKAGE_LOG_LINES = 300
_RELEASE_DIR = (PROJECT_ROOT / "release").resolve()
_SIMULATION_DIR = (PROJECT_ROOT / "Data" / "Simulation").resolve()

class StartSimulationRequest(BaseModel):
    mission_name: str | None = None


class PresetSaveRequest(BaseModel):
    name: str
    config: dict[str, Any]


class PresetApplyRequest(BaseModel):
    name: str


def _now_iso() -> str:
    return datetime.now(UTC).isoformat()


def _append_package_log(line: str) -> None:
    logs = _package_state.setdefault("log_lines", [])
    logs.append(line.rstrip("\n"))
    if len(logs) > _MAX_PACKAGE_LOG_LINES:
        del logs[: len(logs) - _MAX_PACKAGE_LOG_LINES]


async def _consume_package_stream(stream: asyncio.StreamReader) -> None:
    while True:
        chunk = await stream.readline()
        if not chunk:
            break
        text = chunk.decode("utf-8", errors="replace")
        _append_package_log(text)
        match = re.search(r"Created app archive:\s*(.+?)\s*$", text.strip())
        if match:
            _package_state["archive_path"] = match.group(1).strip()


async def _run_package_job() -> None:
    global _package_task
    _package_state["status"] = "running"
    _package_state["started_at"] = _now_iso()
    _package_state["finished_at"] = None
    _package_state["return_code"] = None
    _package_state["archive_path"] = None
    _package_state["error"] = None
    _package_state["log_lines"] = []
    _append_package_log(">>> Starting: make package-app")

    try:
        process = await asyncio.create_subprocess_exec(
            "make",
            "package-app",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            cwd=str(PROJECT_ROOT),
            env=os.environ.copy(),
        )

        assert process.stdout is not None
        assert process.stderr is not None
        await asyncio.gather(
            _consume_package_stream(process.stdout),
            _consume_package_stream(process.stderr),
        )
        return_code = await process.wait()
        _package_state["return_code"] = int(return_code)
        _package_state["status"] = "completed" if return_code == 0 else "failed"
        if return_code != 0 and not _package_state.get("error"):
            _package_state["error"] = f"make package-app exited with code {return_code}"
    except Exception as exc:
        _package_state["status"] = "failed"
        _package_state["error"] = str(exc)
        _append_package_log(f">>> Packaging failed: {exc}")
    finally:
        _package_state["finished_at"] = _now_iso()
        _package_task = None


def _resolve_latest_archive() -> Path | None:
    candidates = sorted(
        _RELEASE_DIR.glob("*.tar.gz"),
        key=lambda p: p.stat().st_mtime if p.exists() else 0.0,
        reverse=True,
    )
    for path in candidates:
        if path.is_file():
            return path.resolve()
    return None


def _iter_mission_files() -> list[Path]:
    if not MISSIONS_DIR.exists():
        return []
    return sorted(path for path in MISSIONS_DIR.glob("*.json") if path.is_file())


def _iter_simulation_run_dirs() -> list[Path]:
    if not _SIMULATION_DIR.exists():
        return []
    return sorted(path for path in _SIMULATION_DIR.iterdir() if path.is_dir())


def _parse_json_list(value: str | None) -> set[str]:
    if not value:
        return set()
    try:
        parsed = json.loads(value)
    except Exception:
        return set()
    if not isinstance(parsed, list):
        return set()
    out: set[str] = set()
    for item in parsed:
        if isinstance(item, str):
            trimmed = item.strip()
            if trimmed:
                out.add(trimmed)
    return out


def _inspect_workspace_zip(raw_bytes: bytes, manager: RunnerManager) -> dict[str, Any]:
    import zipfile

    try:
        zf = zipfile.ZipFile(io.BytesIO(raw_bytes))
    except zipfile.BadZipFile as exc:
        raise HTTPException(status_code=400, detail="Invalid zip file") from exc

    bundle_mission_names: list[str] = []
    bundle_preset_names: list[str] = []
    bundle_simulation_runs: list[str] = []
    has_runner_overrides = False

    with zf:
        names = zf.namelist()
        has_runner_overrides = "runner_overrides.json" in names

        for name in names:
            normalized = name.strip().replace("\\", "/")
            if normalized.startswith("missions/") and normalized.endswith(".json"):
                mission_name = Path(normalized).name
                if mission_name:
                    bundle_mission_names.append(mission_name)
            if normalized.startswith("simulation_runs/"):
                parts = Path(normalized).parts
                if len(parts) >= 2 and parts[1]:
                    bundle_simulation_runs.append(parts[1])

        if "runner_presets.json" in names:
            try:
                presets_payload = json.loads(
                    zf.read("runner_presets.json").decode("utf-8")
                )
                presets = presets_payload.get("presets", {})
                if isinstance(presets, dict):
                    bundle_preset_names = sorted(
                        [name for name, value in presets.items() if isinstance(name, str) and isinstance(value, dict)]
                    )
            except Exception:
                bundle_preset_names = []

    bundle_mission_names = sorted(set(bundle_mission_names))
    bundle_simulation_runs = sorted(set(bundle_simulation_runs))
    existing_mission_names = {path.name for path in _iter_mission_files()}
    existing_preset_names = set(manager.list_presets().keys())
    existing_sim_runs = {path.name for path in _iter_simulation_run_dirs()}

    mission_conflicts = sorted(
        [name for name in bundle_mission_names if name in existing_mission_names]
    )
    preset_conflicts = sorted(
        [name for name in bundle_preset_names if name in existing_preset_names]
    )
    simulation_conflicts = sorted(
        [name for name in bundle_simulation_runs if name in existing_sim_runs]
    )

    return {
        "schema_version": "workspace_inspect_v1",
        "bundle": {
            "missions": bundle_mission_names,
            "presets": bundle_preset_names,
            "simulation_runs": bundle_simulation_runs,
            "has_runner_overrides": has_runner_overrides,
        },
        "conflicts": {
            "missions": mission_conflicts,
            "presets": preset_conflicts,
            "simulation_runs": simulation_conflicts,
        },
        "counts": {
            "missions_total": len(bundle_mission_names),
            "presets_total": len(bundle_preset_names),
            "simulation_runs_total": len(bundle_simulation_runs),
            "mission_conflicts": len(mission_conflicts),
            "preset_conflicts": len(preset_conflicts),
            "simulation_run_conflicts": len(simulation_conflicts),
        },
    }


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


@router.post("/package_app/start")
async def start_package_app():
    """Start asynchronous make package-app job."""
    global _package_task
    if _package_task is not None and not _package_task.done():
        raise HTTPException(status_code=409, detail="Packaging job already running")
    _package_task = asyncio.create_task(_run_package_job())
    return {"status": "started"}


@router.get("/package_app/status")
def get_package_app_status():
    """Get package job status and recent logs."""
    running = bool(_package_task is not None and not _package_task.done())
    payload = dict(_package_state)
    payload["running"] = running
    return payload


@router.get("/package_app/download_latest")
def download_latest_package_app():
    """Download latest generated package archive."""
    archive_path_value = _package_state.get("archive_path")
    archive_path: Path | None = None

    if isinstance(archive_path_value, str) and archive_path_value.strip():
        candidate = Path(archive_path_value)
        if not candidate.is_absolute():
            candidate = (PROJECT_ROOT / candidate).resolve()
        else:
            candidate = candidate.resolve()
        if candidate.exists() and candidate.is_file():
            archive_path = candidate

    if archive_path is None:
        archive_path = _resolve_latest_archive()

    if archive_path is None:
        raise HTTPException(
            status_code=404,
            detail="No package archive found. Run package-app first.",
        )

    return FileResponse(
        path=archive_path,
        filename=archive_path.name,
        media_type="application/gzip",
    )


@router.get("/workspace/export")
def export_workspace_bundle(
    include_simulation_data: bool = Query(False),
):
    """Export workspace as zip bundle."""
    manager = get_runner_manager()
    buffer = io.BytesIO()

    config_payload = manager.get_config()
    config_sections: dict[str, Any] = {}
    for section in ("physics", "mpc", "simulation", "input_file_path"):
        value = config_payload.get(section)
        if value is not None:
            config_sections[section] = value

    mission_files = _iter_mission_files()
    sim_runs = _iter_simulation_run_dirs() if include_simulation_data else []
    manifest = {
        "schema_version": "workspace_bundle_v1",
        "generated_at": _now_iso(),
        "counts": {
            "missions": len(mission_files),
            "presets": len(manager.list_presets()),
            "simulation_runs": len(sim_runs),
        },
    }

    import zipfile

    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("workspace_manifest.json", json.dumps(manifest, indent=2))
        zf.writestr(
            "runner_overrides.json",
            json.dumps(config_sections, indent=2),
        )
        zf.writestr(
            "runner_presets.json",
            json.dumps({"presets": manager.list_presets()}, indent=2),
        )
        for mission_file in mission_files:
            zf.writestr(
                f"missions/{mission_file.name}",
                mission_file.read_text(encoding="utf-8"),
            )
        if include_simulation_data:
            for run_dir in sim_runs:
                for artifact in run_dir.rglob("*"):
                    if not artifact.is_file():
                        continue
                    rel = artifact.relative_to(run_dir)
                    zf.writestr(
                        f"simulation_runs/{run_dir.name}/{rel.as_posix()}",
                        artifact.read_bytes(),
                    )

    buffer.seek(0)
    filename = f"satellite-workspace-{datetime.now().strftime('%Y%m%d_%H%M%S')}.zip"
    headers = {
        "Content-Disposition": f'attachment; filename="{filename}"',
    }
    return StreamingResponse(
        buffer,
        media_type="application/zip",
        headers=headers,
    )


@router.post("/workspace/inspect")
async def inspect_workspace_bundle(file: UploadFile = File(...)):
    """Inspect workspace bundle and report conflicts without importing."""
    manager = get_runner_manager()
    filename = file.filename or ""
    if not filename.lower().endswith(".zip"):
        raise HTTPException(status_code=400, detail="Expected a .zip workspace bundle")

    raw_bytes = await file.read()
    if not raw_bytes:
        raise HTTPException(status_code=400, detail="Uploaded file is empty")

    return _inspect_workspace_zip(raw_bytes, manager)


@router.post("/workspace/import")
async def import_workspace_bundle(
    file: UploadFile = File(...),
    replace_existing_missions: bool = Form(True),
    replace_existing_presets: bool = Form(False),
    replace_existing_simulation_runs: bool = Form(False),
    apply_runner_config: bool = Form(True),
    overwrite_missions_json: str | None = Form(None),
    overwrite_presets_json: str | None = Form(None),
    overwrite_simulation_runs_json: str | None = Form(None),
):
    """Import missions + presets + active config from workspace bundle zip."""
    manager = get_runner_manager()
    filename = file.filename or ""
    if not filename.lower().endswith(".zip"):
        raise HTTPException(status_code=400, detail="Expected a .zip workspace bundle")

    raw_bytes = await file.read()
    if not raw_bytes:
        raise HTTPException(status_code=400, detail="Uploaded file is empty")

    import zipfile

    try:
        zf = zipfile.ZipFile(io.BytesIO(raw_bytes))
    except zipfile.BadZipFile as exc:
        raise HTTPException(status_code=400, detail="Invalid zip file") from exc

    inspection = _inspect_workspace_zip(raw_bytes, manager)
    existing_mission_conflicts = set(inspection["conflicts"]["missions"])
    existing_preset_conflicts = set(inspection["conflicts"]["presets"])
    existing_sim_run_conflicts = set(inspection["conflicts"]["simulation_runs"])

    explicit_mission_overwrites = _parse_json_list(overwrite_missions_json)
    explicit_preset_overwrites = _parse_json_list(overwrite_presets_json)
    explicit_sim_overwrites = _parse_json_list(overwrite_simulation_runs_json)

    imported_missions = 0
    skipped_missions = 0
    imported_presets = 0
    skipped_presets = 0
    imported_simulation_runs = 0
    skipped_simulation_runs = 0
    imported_config = False

    with zf:
        for name in zf.namelist():
            normalized = name.strip().replace("\\", "/")
            if not normalized.startswith("missions/") or not normalized.endswith(".json"):
                continue
            mission_name = Path(normalized).name
            if not mission_name:
                continue
            allow_overwrite = replace_existing_missions or (
                mission_name in explicit_mission_overwrites
            )
            if (mission_name in existing_mission_conflicts) and (not allow_overwrite):
                skipped_missions += 1
                continue
            payload = json.loads(zf.read(name).decode("utf-8"))
            save_mission_json(name=Path(mission_name).stem, payload=payload, source="local")
            imported_missions += 1

        if "runner_presets.json" in zf.namelist():
            presets_payload = json.loads(zf.read("runner_presets.json").decode("utf-8"))
            presets = presets_payload.get("presets", {})
            if isinstance(presets, dict):
                for preset_name, preset_value in presets.items():
                    if not isinstance(preset_name, str) or not isinstance(preset_value, dict):
                        continue
                    preset_config = preset_value.get("config")
                    if not isinstance(preset_config, dict):
                        continue
                    allow_overwrite = replace_existing_presets or (
                        preset_name in explicit_preset_overwrites
                    )
                    if (preset_name in existing_preset_conflicts) and (not allow_overwrite):
                        skipped_presets += 1
                        continue
                    manager.save_preset(preset_name, preset_config)
                    imported_presets += 1

        sim_run_members: dict[str, list[str]] = {}
        for name in zf.namelist():
            normalized = name.strip().replace("\\", "/")
            if not normalized.startswith("simulation_runs/"):
                continue
            parts = Path(normalized).parts
            if len(parts) < 3:
                continue
            run_name = parts[1]
            if not run_name:
                continue
            sim_run_members.setdefault(run_name, []).append(normalized)

        _SIMULATION_DIR.mkdir(parents=True, exist_ok=True)
        for run_name, member_paths in sim_run_members.items():
            allow_overwrite = replace_existing_simulation_runs or (
                run_name in explicit_sim_overwrites
            )
            run_dir = (_SIMULATION_DIR / run_name).resolve()
            if run_name in existing_sim_run_conflicts and not allow_overwrite:
                skipped_simulation_runs += 1
                continue
            if run_dir.exists() and run_name in existing_sim_run_conflicts:
                shutil.rmtree(run_dir)
            run_dir.mkdir(parents=True, exist_ok=True)

            for member in member_paths:
                parts = Path(member).parts
                rel_parts = parts[2:]
                if not rel_parts:
                    continue
                dest = (run_dir / Path(*rel_parts)).resolve()
                try:
                    dest.relative_to(run_dir)
                except ValueError:
                    continue
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(zf.read(member))

            imported_simulation_runs += 1

        if apply_runner_config and "runner_overrides.json" in zf.namelist():
            config_payload = json.loads(zf.read("runner_overrides.json").decode("utf-8"))
            if isinstance(config_payload, dict):
                manager.update_config(config_payload)
                imported_config = True

    return {
        "status": "imported",
        "options": {
            "replace_existing_missions": replace_existing_missions,
            "replace_existing_presets": replace_existing_presets,
            "replace_existing_simulation_runs": replace_existing_simulation_runs,
            "apply_runner_config": apply_runner_config,
        },
        "explicit_overwrites": {
            "missions": sorted(explicit_mission_overwrites),
            "presets": sorted(explicit_preset_overwrites),
            "simulation_runs": sorted(explicit_sim_overwrites),
        },
        "missions_imported": imported_missions,
        "missions_skipped": skipped_missions,
        "presets_imported": imported_presets,
        "presets_skipped": skipped_presets,
        "simulation_runs_imported": imported_simulation_runs,
        "simulation_runs_skipped": skipped_simulation_runs,
        "config_imported": imported_config,
    }


@router.get("/system_status")
def get_system_status():
    """Return runtime readiness status for web-only operation."""
    manager = get_runner_manager()
    ui_dist_index = (PROJECT_ROOT / "ui" / "dist" / "index.html").resolve()
    data_sim_dir = (PROJECT_ROOT / "Data" / "Simulation").resolve()
    data_dashboard_dir = (PROJECT_ROOT / "Data" / "Dashboard").resolve()
    src_python_dir = (PROJECT_ROOT / "src" / "python").resolve()

    checks = {
        "ui_dist_ready": ui_dist_index.exists() and ui_dist_index.is_file(),
        "simulation_script_ready": SIMULATION_SCRIPT.exists() and SIMULATION_SCRIPT.is_file(),
        "src_python_ready": src_python_dir.exists() and src_python_dir.is_dir(),
        "data_sim_ready": data_sim_dir.exists() and data_sim_dir.is_dir(),
        "data_dashboard_ready": data_dashboard_dir.exists() and data_dashboard_dir.is_dir(),
        "python_executable_ready": bool(sys.executable),
    }

    dependency_names = ("fastapi", "uvicorn", "numpy", "pydantic")
    dependency_status: dict[str, bool] = {}
    for name in dependency_names:
        try:
            importlib.import_module(name)
            dependency_status[name] = True
        except Exception:
            dependency_status[name] = False

    missing_checks = [name for name, ok in checks.items() if not ok]
    missing_deps = [name for name, ok in dependency_status.items() if not ok]
    ready_for_runner = not missing_checks and not missing_deps
    runner_active = bool(manager.process and manager.process.returncode is None)

    return {
        "ready_for_runner": ready_for_runner,
        "runner_active": runner_active,
        "checks": checks,
        "dependencies": dependency_status,
        "missing_checks": missing_checks,
        "missing_dependencies": missing_deps,
        "paths": {
            "project_root": str(PROJECT_ROOT),
            "ui_dist_index": str(ui_dist_index),
            "simulation_script": str(SIMULATION_SCRIPT),
            "presets_file": str(PRESETS_FILE),
            "data_sim_dir": str(data_sim_dir),
            "data_dashboard_dir": str(data_dashboard_dir),
            "src_python_dir": str(src_python_dir),
        },
        "python": {
            "executable": sys.executable,
            "version": sys.version.split()[0],
            "pid": os.getpid(),
        },
    }


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
    except Exception:
        # manager.disconnect(websocket) # usually handled in disconnect or finally
        pass
    finally:
        manager.disconnect(websocket)

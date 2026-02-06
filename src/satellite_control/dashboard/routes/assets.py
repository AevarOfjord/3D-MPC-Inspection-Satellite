"""Model-file serving, OBJ upload, mesh-scan preview, and path-asset routes."""

import logging
import math
from pathlib import Path
from typing import Dict, List

from fastapi import APIRouter, File, Form, HTTPException
from fastapi.responses import FileResponse

from src.satellite_control.dashboard.models import MeshScanConfigModel, PathAssetSaveRequest
from src.satellite_control.mission.path_assets import (
    list_path_assets,
    load_path_asset,
    save_path_asset,
)

logger = logging.getLogger("dashboard")

router = APIRouter()

# Injected at startup
_project_root: Path = Path(".")
_model_allowed_roots: tuple = ()


def set_dependencies(project_root: Path, model_allowed_roots: tuple) -> None:
    global _project_root, _model_allowed_roots
    _project_root = project_root
    _model_allowed_roots = model_allowed_roots


def _resolve_allowed_model_path(path_value: str) -> Path:
    """Resolve a model path while restricting access to known model roots."""
    candidate = Path(path_value)
    if not candidate.is_absolute():
        candidate = _project_root / candidate

    resolved = candidate.resolve()
    for allowed_root in _model_allowed_roots:
        if resolved == allowed_root or allowed_root in resolved.parents:
            return resolved

    raise HTTPException(
        status_code=400,
        detail="Model path must be inside OBJ_files or ui/public/OBJ_files",
    )


# --- Model file routes ---


@router.get("/api/models/serve")
async def serve_model_file(path: str):
    """Serve a model file from the filesystem."""
    file_path = _resolve_allowed_model_path(path)

    logger.info(f"[MODEL SERVE] Requested: {path}, Resolved to: {file_path}")

    if not file_path.exists() or not file_path.is_file():
        logger.warning(f"[MODEL SERVE] File not found: {file_path}")
        raise HTTPException(
            status_code=404, detail=f"Model file not found: {file_path}"
        )

    return FileResponse(path=file_path, filename=file_path.name)


@router.get("/api/models/list")
async def list_model_files():
    """List available OBJ models in the repository."""
    search_dirs = [
        _project_root / "OBJ_files",
        _project_root / "OBJ_files" / "uploads",
    ]
    models: List[Dict[str, str]] = []
    for base in search_dirs:
        if not base.exists():
            continue
        for obj_file in sorted(base.rglob("*.obj")):
            try:
                rel_path = obj_file.relative_to(_project_root)
            except ValueError:
                rel_path = obj_file
            models.append(
                {
                    "name": obj_file.stem,
                    "filename": obj_file.name,
                    "path": str(rel_path),
                }
            )
    return {"models": models}


@router.get("/api/models/bounds")
async def get_model_bounds(path: str):
    """Compute basic bounds for an OBJ model."""
    from src.satellite_control.mission.mesh_scan import (
        load_obj_vertices,
        compute_mesh_bounds,
    )

    file_path = _resolve_allowed_model_path(path)

    if not file_path.exists() or not file_path.is_file():
        raise HTTPException(status_code=404, detail=f"Model file not found: {path}")

    vertices = load_obj_vertices(str(file_path))
    min_bounds, max_bounds, center, _ = compute_mesh_bounds(vertices)
    extents = (max_bounds - min_bounds).tolist()

    return {
        "center": center.tolist(),
        "min_bounds": min_bounds.tolist(),
        "max_bounds": max_bounds.tolist(),
        "extents": extents,
    }


# --- Upload ---


@router.post("/upload_object")
async def upload_object(file: bytes = File(...), filename: str = Form(...)):
    """Upload an OBJ file to the server."""
    try:
        import aiofiles
    except Exception:
        aiofiles = None

    upload_dir = Path("OBJ_files/uploads")
    upload_dir.mkdir(parents=True, exist_ok=True)

    safe_name = Path(filename).name
    file_path = upload_dir / safe_name

    if aiofiles is None:
        with open(file_path, "wb") as out_file:
            out_file.write(file)
    else:
        async with aiofiles.open(file_path, "wb") as out_file:
            await out_file.write(file)

    return {"status": "success", "path": str(file_path), "filename": safe_name}


# --- Trajectory preview ---


@router.post("/preview_trajectory")
async def preview_trajectory(config: MeshScanConfigModel):
    """Generate a preview of the mesh scan trajectory without running simulation."""
    from src.satellite_control.mission.mesh_scan import (
        build_mesh_scan_trajectory,
        build_mesh_spiral_trajectory,
        load_obj_vertices,
        compute_mesh_bounds,
    )

    try:
        levels = config.levels
        if config.level_spacing and config.level_spacing > 0:
            try:
                vertices = load_obj_vertices(config.obj_path)
                _, max_bounds, _, _ = compute_mesh_bounds(vertices)
                min_bounds = vertices.min(axis=0)
                axis_idx = 2
                if config.scan_axis == "X":
                    axis_idx = 0
                elif config.scan_axis == "Y":
                    axis_idx = 1

                object_height = max_bounds[axis_idx] - min_bounds[axis_idx]
                levels = max(
                    1,
                    int(math.ceil(object_height / max(config.level_spacing, 1e-6))),
                )
            except Exception:
                levels = 8

        dt = 0.1
        axis = str(config.scan_axis).upper().strip()
        if str(config.pattern).lower() == "spiral":
            path, _, path_length = build_mesh_spiral_trajectory(
                obj_path=config.obj_path,
                standoff=config.standoff,
                levels=levels,
                points_per_circle=config.points_per_circle,
                v_max=config.speed_max,
                v_min=config.speed_min,
                lateral_accel=config.lateral_accel,
                dt=dt,
                z_margin=config.z_margin,
                scan_axis=axis,
                build_trajectory=False,
            )
        else:
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
                scan_axis=axis,
                build_trajectory=False,
            )

        speed = max(float(config.speed_max), 1e-3)
        duration = float(path_length) / speed

        return {
            "status": "success",
            "path": path,
            "points": len(path),
            "estimated_duration": duration,
            "path_length": path_length,
            "computed_levels": levels,
        }
    except Exception as e:
        logger.error(f"Preview generation failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# --- Path assets ---


@router.get("/path_assets")
async def get_path_assets():
    """List saved path assets."""
    try:
        assets = list_path_assets()
        return {"assets": assets}
    except Exception as e:
        logger.error(f"Failed to list path assets: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/path_assets/{asset_id}")
async def get_path_asset(asset_id: str):
    """Load a specific path asset by id."""
    try:
        asset = load_path_asset(asset_id)
        return asset
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail=f"Path asset not found: {asset_id}")
    except Exception as e:
        logger.error(f"Failed to load path asset {asset_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/path_assets")
async def create_path_asset(request: PathAssetSaveRequest):
    """Save a path asset for reuse in missions."""
    try:
        payload = request.model_dump()
        asset = save_path_asset(payload)
        return asset
    except Exception as e:
        logger.error(f"Failed to save path asset: {e}")
        raise HTTPException(status_code=500, detail=str(e))

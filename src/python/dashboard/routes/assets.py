"""Model-file serving, OBJ upload, mesh-scan preview, and path-asset routes."""

import logging
import math
from pathlib import Path

import numpy as np
from config.paths import (
    ASSET_MODEL_FILES_ROOT,
    LEGACY_ASSET_MODEL_FILES_ROOT,
    normalize_repo_relative_str,
    resolve_repo_path,
)
from dashboard.models import (
    CompileScanProjectRequestModel,
    MeshScanConfigModel,
    PathAssetSaveRequest,
    ScanProjectModel,
)
from fastapi import APIRouter, File, Form, HTTPException
from fastapi.responses import FileResponse
from mission.path_assets import (
    list_path_assets,
    load_path_asset,
    save_path_asset,
)
from mission.scan_projects import (
    compile_scan_project,
    list_scan_projects,
    load_scan_project,
    save_scan_project,
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
    resolved = resolve_repo_path(path_value).resolve()
    for allowed_root in _model_allowed_roots:
        if resolved == allowed_root or allowed_root in resolved.parents:
            return resolved

    raise HTTPException(
        status_code=400,
        detail="Model path must be inside model_files or ui/public/model_files",
    )


def _normalize_model_path_for_storage(path_value: Path) -> str:
    return normalize_repo_relative_str(path_value)


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
        ASSET_MODEL_FILES_ROOT,
        LEGACY_ASSET_MODEL_FILES_ROOT,
        ASSET_MODEL_FILES_ROOT / "uploads",
        LEGACY_ASSET_MODEL_FILES_ROOT / "uploads",
    ]
    models: list[dict[str, str]] = []
    seen_paths: set[Path] = set()
    for base in search_dirs:
        if not base.exists():
            continue
        for obj_file in sorted(base.rglob("*.obj")):
            resolved_obj = obj_file.resolve()
            if resolved_obj in seen_paths:
                continue
            seen_paths.add(resolved_obj)
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
    from mission.mesh_scan import (
        compute_mesh_bounds,
        load_obj_vertices,
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

    upload_dir = ASSET_MODEL_FILES_ROOT / "uploads"
    upload_dir.mkdir(parents=True, exist_ok=True)

    safe_name = Path(filename).name
    file_path = upload_dir / safe_name

    if aiofiles is None:
        with open(file_path, "wb") as out_file:
            out_file.write(file)
    else:
        async with aiofiles.open(file_path, "wb") as out_file:
            await out_file.write(file)

    return {
        "status": "success",
        "path": normalize_repo_relative_str(file_path),
        "filename": safe_name,
    }


# --- Trajectory preview ---


@router.post("/preview_trajectory")
async def preview_trajectory(config: MeshScanConfigModel):
    """Generate a preview of the mesh scan trajectory without running simulation."""
    from mission.mesh_scan import (
        apply_mesh_section_filter,
        build_mesh_scan_trajectory,
        build_mesh_spiral_trajectory,
        compute_mesh_bounds,
        load_obj_vertices,
    )

    def resolve_section_filter(
        pass_cfg: dict,
    ) -> tuple[
        list[float] | None,
        list[float] | None,
        list[float] | None,
        float | None,
        float | None,
    ]:
        section_mode = str(pass_cfg.get("section_mode", "none")).lower()
        region_center = None
        region_size = None
        plane_normal = None
        plane_offset_min = None
        plane_offset_max = None

        # Backward-compatible AABB support (legacy region flags)
        use_region = bool(pass_cfg.get("region_enabled")) or section_mode == "aabb"
        if use_region:
            center = pass_cfg.get("region_center")
            size = pass_cfg.get("region_size")
            if (
                isinstance(center, list)
                and isinstance(size, list)
                and len(center) == 3
                and len(size) == 3
            ):
                region_center = center
                region_size = size

        if section_mode == "plane_slab":
            pn = pass_cfg.get("plane_normal")
            d0 = pass_cfg.get("plane_offset_min")
            d1 = pass_cfg.get("plane_offset_max")
            if (
                isinstance(pn, list)
                and len(pn) == 3
                and d0 is not None
                and d1 is not None
            ):
                plane_normal = pn
                plane_offset_min = float(d0)
                plane_offset_max = float(d1)

        return (
            region_center,
            region_size,
            plane_normal,
            plane_offset_min,
            plane_offset_max,
        )

    def compute_levels_for(
        axis_token: str,
        level_spacing: float | None,
        fallback: int,
        pass_cfg: dict,
        obj_path: str,
    ) -> int:
        levels = int(max(1, fallback))
        if not level_spacing or level_spacing <= 0:
            return levels
        try:
            vertices = load_obj_vertices(obj_path)
            (
                region_center,
                region_size,
                plane_normal,
                plane_offset_min,
                plane_offset_max,
            ) = resolve_section_filter(pass_cfg)
            vertices, _ = apply_mesh_section_filter(
                vertices,
                np.empty((0, 3), dtype=int),
                region_center=region_center,
                region_size=region_size,
                plane_normal=plane_normal,
                plane_offset_min=plane_offset_min,
                plane_offset_max=plane_offset_max,
            )
            _, max_bounds, _, _ = compute_mesh_bounds(vertices)
            min_bounds = vertices.min(axis=0)
            axis_idx = 2
            axis_u = str(axis_token).upper().strip()
            if axis_u == "X":
                axis_idx = 0
            elif axis_u == "Y":
                axis_idx = 1
            object_height = max_bounds[axis_idx] - min_bounds[axis_idx]
            return max(1, int(math.ceil(object_height / max(level_spacing, 1e-6))))
        except Exception:
            return levels

    def build_one_pass(
        pass_cfg: dict,
        obj_path: str,
    ) -> tuple[list[tuple[float, float, float]], float, int]:
        levels = compute_levels_for(
            pass_cfg["scan_axis"],
            pass_cfg.get("level_spacing"),
            int(pass_cfg["levels"]),
            pass_cfg,
            obj_path,
        )
        axis = str(pass_cfg["scan_axis"]).upper().strip()
        (
            region_center,
            region_size,
            plane_normal,
            plane_offset_min,
            plane_offset_max,
        ) = resolve_section_filter(pass_cfg)
        dt = 0.1
        if str(pass_cfg["pattern"]).lower() == "spiral":
            path, _, path_length = build_mesh_spiral_trajectory(
                obj_path=obj_path,
                standoff=float(pass_cfg["standoff"]),
                levels=levels,
                points_per_circle=int(pass_cfg["points_per_circle"]),
                v_max=float(pass_cfg["speed_max"]),
                v_min=float(pass_cfg["speed_min"]),
                lateral_accel=float(pass_cfg["lateral_accel"]),
                dt=dt,
                z_margin=float(pass_cfg["z_margin"]),
                scan_axis=axis,
                region_center=region_center,
                region_size=region_size,
                plane_normal=plane_normal,
                plane_offset_min=plane_offset_min,
                plane_offset_max=plane_offset_max,
                build_trajectory=False,
            )
        else:
            path, _, path_length = build_mesh_scan_trajectory(
                obj_path=obj_path,
                standoff=float(pass_cfg["standoff"]),
                levels=levels,
                points_per_circle=int(pass_cfg["points_per_circle"]),
                v_max=float(pass_cfg["speed_max"]),
                v_min=float(pass_cfg["speed_min"]),
                lateral_accel=float(pass_cfg["lateral_accel"]),
                dt=dt,
                z_margin=float(pass_cfg["z_margin"]),
                scan_axis=axis,
                region_center=region_center,
                region_size=region_size,
                plane_normal=plane_normal,
                plane_offset_min=plane_offset_min,
                plane_offset_max=plane_offset_max,
                build_trajectory=False,
            )
        return path, float(path_length), levels

    def connect_transition(
        start_pt: tuple[float, float, float],
        end_pt: tuple[float, float, float],
        max_step: float = 1.0,
    ) -> list[tuple[float, float, float]]:
        a = np.array(start_pt, dtype=float)
        b = np.array(end_pt, dtype=float)
        dist = float(np.linalg.norm(b - a))
        if dist <= 1e-6:
            return []
        steps = max(2, int(math.ceil(dist / max(max_step, 1e-3))))
        out: list[tuple[float, float, float]] = []
        for i in range(1, steps):
            t = i / float(steps)
            p = a + t * (b - a)
            out.append((float(p[0]), float(p[1]), float(p[2])))
        return out

    try:
        resolved_obj_path = _resolve_allowed_model_path(config.obj_path)
        resolved_obj_path_str = str(resolved_obj_path)
        pass_summaries: list[dict] = []
        path: list[tuple[float, float, float]] = []
        total_length = 0.0
        total_duration = 0.0

        passes = []
        if config.passes:
            for p in config.passes:
                payload = p.model_dump()
                if payload.get("enabled", True):
                    passes.append(payload)

        if not passes:
            passes = [
                {
                    "label": "Pass 1",
                    "standoff": config.standoff,
                    "levels": config.levels,
                    "level_spacing": config.level_spacing,
                    "points_per_circle": config.points_per_circle,
                    "speed_max": config.speed_max,
                    "speed_min": config.speed_min,
                    "lateral_accel": config.lateral_accel,
                    "z_margin": config.z_margin,
                    "scan_axis": config.scan_axis,
                    "pattern": config.pattern,
                    "region_enabled": False,
                    "region_center": None,
                    "region_size": None,
                    "section_mode": "none",
                    "plane_normal": None,
                    "plane_offset_min": None,
                    "plane_offset_max": None,
                }
            ]

        for idx, pass_cfg in enumerate(passes):
            pass_path, pass_len, pass_levels = build_one_pass(
                pass_cfg, resolved_obj_path_str
            )
            if not pass_path:
                continue
            if path:
                transition = connect_transition(path[-1], pass_path[0], max_step=1.0)
                if transition:
                    trans_len = float(
                        np.sum(
                            np.linalg.norm(
                                np.diff(
                                    np.array(
                                        [path[-1], *transition, pass_path[0]],
                                        dtype=float,
                                    ),
                                    axis=0,
                                ),
                                axis=1,
                            )
                        )
                    )
                    total_length += trans_len
                    transition_speed = max(
                        float(pass_cfg.get("speed_max", config.speed_max)),
                        1e-3,
                    )
                    total_duration += trans_len / transition_speed
                    path.extend(transition)
            path.extend(pass_path)
            total_length += pass_len
            speed = max(float(pass_cfg.get("speed_max", config.speed_max)), 1e-3)
            total_duration += pass_len / speed
            pass_summaries.append(
                {
                    "index": idx,
                    "label": pass_cfg.get("label") or f"Pass {idx + 1}",
                    "scan_axis": pass_cfg.get("scan_axis"),
                    "pattern": pass_cfg.get("pattern"),
                    "computed_levels": pass_levels,
                    "points": len(pass_path),
                    "path_length": pass_len,
                }
            )

        if not path:
            raise ValueError("No path generated from provided pass configuration.")

        computed_levels = (
            pass_summaries[0]["computed_levels"] if pass_summaries else config.levels
        )
        return {
            "status": "success",
            "path": path,
            "points": len(path),
            "estimated_duration": total_duration,
            "path_length": total_length,
            "computed_levels": computed_levels,
            "pass_summaries": pass_summaries,
        }
    except Exception as e:
        logger.error(f"Preview generation failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# --- Path assets ---


@router.get("/scan_projects")
async def get_scan_projects():
    """List saved scan projects."""
    try:
        projects = list_scan_projects()
        return {"projects": projects}
    except Exception as e:
        logger.error(f"Failed to list scan projects: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/scan_projects/{project_id}")
async def get_scan_project(project_id: str):
    """Load a specific scan project by id."""
    try:
        return load_scan_project(project_id)
    except FileNotFoundError:
        raise HTTPException(
            status_code=404, detail=f"Scan project not found: {project_id}"
        )
    except Exception as e:
        logger.error(f"Failed to load scan project {project_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/scan_projects")
async def create_scan_project(request: ScanProjectModel):
    """Save an editable scan project."""
    try:
        payload = request.model_dump()
        resolved_obj = _resolve_allowed_model_path(str(payload.get("obj_path") or ""))
        payload["obj_path"] = _normalize_model_path_for_storage(resolved_obj)
        return save_scan_project(payload)
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to save scan project: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/scan_projects/compile")
async def compile_scan_project_preview(request: CompileScanProjectRequestModel):
    """Compile a scan project into final scan/connector path output."""
    try:
        payload = request.project.model_dump()
        resolved_obj = _resolve_allowed_model_path(payload["obj_path"])
        payload["obj_path"] = str(resolved_obj)
        return compile_scan_project(
            payload,
            quality=request.quality,
            include_collision=request.include_collision,
            collision_threshold_m=request.collision_threshold_m,
        )
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Failed to compile scan project: {e}")
        raise HTTPException(status_code=500, detail=str(e))


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
        obj_path = str(payload.get("obj_path") or "").strip()
        if obj_path:
            resolved_obj = _resolve_allowed_model_path(obj_path)
            payload["obj_path"] = _normalize_model_path_for_storage(resolved_obj)
        asset = save_path_asset(payload)
        return asset
    except Exception as e:
        logger.error(f"Failed to save path asset: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# --- Studio spiral path generation ---


from pydantic import BaseModel  # noqa: E402


class _KeyLevelIn(BaseModel):
    id: str = ""
    t: float = 0.5
    radius_x: float = 5.0
    radius_y: float = 5.0
    rotation_deg: float = 0.0
    offset_x: float = 0.0
    offset_y: float = 0.0


class _GenerateScanPathRequest(BaseModel):
    axis_seed: str = "Z"
    plane_a: dict = {"position": [0.0, 0.0, -5.0], "orientation": [1.0, 0.0, 0.0, 0.0]}
    plane_b: dict = {"position": [0.0, 0.0, 5.0], "orientation": [1.0, 0.0, 0.0, 0.0]}
    ellipse: dict = {"radius_x": 5.0, "radius_y": 5.0}
    level_spacing_m: float = 0.5
    point_density_scale: float = 1.0
    key_levels: list[_KeyLevelIn] = []


def _normalize3(value: list[float] | tuple[float, ...], fallback: tuple[float, float, float]) -> np.ndarray:
    try:
        arr = np.array(value, dtype=float).reshape(-1)
        if arr.size >= 3 and np.all(np.isfinite(arr[:3])):
            return arr[:3]
    except Exception:
        pass
    return np.array(fallback, dtype=float)


def _normalize_quat_wxyz(value: list[float] | tuple[float, ...]) -> np.ndarray:
    try:
        arr = np.array(value, dtype=float).reshape(-1)
        if arr.size >= 4 and np.all(np.isfinite(arr[:4])):
            q = arr[:4]
            n = float(np.linalg.norm(q))
            if n > 1e-9:
                return q / n
    except Exception:
        pass
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def _quat_to_matrix_wxyz(q: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q[:4]]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _quat_slerp_wxyz(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    qa = np.array(q0, dtype=float)
    qb = np.array(q1, dtype=float)
    dot = float(np.dot(qa, qb))
    if dot < 0.0:
        qb = -qb
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        q = qa + t * (qb - qa)
        n = float(np.linalg.norm(q))
        return q / n if n > 1e-9 else qa
    theta_0 = float(math.acos(dot))
    sin_theta_0 = float(math.sin(theta_0))
    theta = theta_0 * float(t)
    sin_theta = float(math.sin(theta))
    s0 = float(math.sin(theta_0 - theta) / max(sin_theta_0, 1e-9))
    s1 = float(sin_theta / max(sin_theta_0, 1e-9))
    q = s0 * qa + s1 * qb
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-9 else qa


def _axis_frame(axis_seed: str) -> tuple[np.ndarray, np.ndarray]:
    _ = str(axis_seed).upper().strip().lstrip("+")
    # Local in-plane basis for a plane mesh whose local +Z is the normal.
    # Axis seed controls initial plane placement only, not ellipse frame math.
    return np.array([1.0, 0.0, 0.0], dtype=float), np.array([0.0, 1.0, 0.0], dtype=float)


@router.post("/api/models/generate_scan_path")
async def generate_scan_path(request: _GenerateScanPathRequest):
    """Generate a spiral scan path from two 6-DOF planes and uniform ellipse radii."""
    plane_a = dict(request.plane_a or {})
    plane_b = dict(request.plane_b or {})
    ellipse = dict(request.ellipse or {})

    pos_a = _normalize3(plane_a.get("position") or [0.0, 0.0, -5.0], (0.0, 0.0, -5.0))
    pos_b = _normalize3(plane_b.get("position") or [0.0, 0.0, 5.0], (0.0, 0.0, 5.0))
    quat_a = _normalize_quat_wxyz(plane_a.get("orientation") or [1.0, 0.0, 0.0, 0.0])
    radius_x = max(0.1, float(ellipse.get("radius_x", 5.0) or 5.0))
    radius_y = max(0.1, float(ellipse.get("radius_y", 5.0) or 5.0))
    level_spacing = max(0.05, float(request.level_spacing_m or 0.5))
    density_scale = float(np.clip(float(request.point_density_scale or 1.0), 0.25, 25.0))

    axis_vec = pos_b - pos_a
    span = float(np.linalg.norm(axis_vec))
    if span < 1e-6:
        span = 10.0
        axis_vec = np.array([0.0, 0.0, 1.0], dtype=float)
    n_axis = axis_vec / max(float(np.linalg.norm(axis_vec)), 1e-9)
    turns = max(1.0, span / level_spacing)
    points_per_turn = max(4, int(round(32 * density_scale)))
    total_points = max(8, int(math.ceil(turns * points_per_turn)))
    base_u, _base_v = _axis_frame(request.axis_seed)
    rot_a = _quat_to_matrix_wxyz(quat_a)
    u_seed = rot_a @ base_u
    # Spiral must remain centered about A->B line. Build a stable frame (u,v)
    # perpendicular to this centerline.
    u_proj = u_seed - float(np.dot(u_seed, n_axis)) * n_axis
    u_norm = float(np.linalg.norm(u_proj))
    if u_norm <= 1e-9:
        fallback = (
            np.array([0.0, 0.0, 1.0], dtype=float)
            if abs(float(np.dot(n_axis, np.array([0.0, 0.0, 1.0], dtype=float)))) < 0.9
            else np.array([1.0, 0.0, 0.0], dtype=float)
        )
        u_proj = np.cross(fallback, n_axis)
        u_norm = float(np.linalg.norm(u_proj))
    u_axis = u_proj / max(u_norm, 1e-9)
    v_axis = np.cross(n_axis, u_axis)
    v_norm = float(np.linalg.norm(v_axis))
    if v_norm <= 1e-9:
        v_axis = np.array([0.0, 1.0, 0.0], dtype=float)
    else:
        v_axis = v_axis / v_norm

    waypoints: list[list[float]] = []
    for i in range(total_points + 1):
        t = i / float(max(1, total_points))
        center = pos_a + (pos_b - pos_a) * t
        ang = 2.0 * math.pi * turns * t
        local_u = radius_x * math.cos(ang)
        local_v = radius_y * math.sin(ang)
        world = center + (u_axis * local_u) + (v_axis * local_v)
        waypoints.append([float(world[0]), float(world[1]), float(world[2])])

    return {"waypoints": waypoints}

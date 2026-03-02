"""
Dashboard FastAPI application.

Thin bootstrapping: creates the FastAPI app, applies middleware,
registers route modules, and wires up lifecycle hooks.
"""

import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

from controller.configs.paths import (
    ASSET_MODEL_FILES_ROOT,
    LEGACY_ASSET_MODEL_FILES_ROOT,
    PROJECT_ROOT,
    SIMULATION_DATA_ROOT,
    UI_PUBLIC_MODEL_FILES_ROOT,
    ensure_runtime_data_dirs,
)
from controller.configs.paths import (
    UI_DIST_DIR as UI_DIST_ROOT,
)
from controller.shared.python.dashboard.routes import assets as asset_routes
from controller.shared.python.dashboard.routes import missions as mission_routes
from controller.shared.python.dashboard.routes import missions_api as mission_api_routes
from controller.shared.python.dashboard.routes import runner as runner_routes
from controller.shared.python.dashboard.routes import simulations as sim_routes
from controller.shared.python.dashboard.runner_manager import RunnerManager
from controller.shared.python.dashboard.simulation_manager import SimulationManager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dashboard")


# --- Shared constants ---
DATA_DIR = SIMULATION_DATA_ROOT


def _dedupe_paths(*paths: Path) -> tuple[Path, ...]:
    unique: list[Path] = []
    for path in paths:
        resolved = path.resolve()
        if resolved not in unique:
            unique.append(resolved)
    return tuple(unique)


MODEL_ALLOWED_ROOTS = _dedupe_paths(
    ASSET_MODEL_FILES_ROOT,
    LEGACY_ASSET_MODEL_FILES_ROOT,
    UI_PUBLIC_MODEL_FILES_ROOT,
)
UI_DIST_DIR = UI_DIST_ROOT.resolve()
SPA_BLOCKED_PREFIXES = (
    "api",
    "runner",
    "simulations",
    "mission",
    "save_mission",
    "saved_missions",
    "path_assets",
    "scan_projects",
    "upload_object",
    "preview_trajectory",
    "control",
    "speed",
    "reset",
    "docs",
    "redoc",
    "openapi.json",
)

# --- Global Singleton ---
ensure_runtime_data_dirs()
sim_manager = SimulationManager()
runner_manager = RunnerManager()


def get_sim_manager() -> SimulationManager:
    return sim_manager


# --- Lifespan (replaces deprecated on_event) ---
@asynccontextmanager
async def lifespan(_: FastAPI):
    logger.info("Dashboard startup: Initializing services...")
    await sim_manager.start()
    yield
    await sim_manager.stop()
    await runner_manager.stop_simulation()


# --- App creation ---
app = FastAPI(lifespan=lifespan)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "http://127.0.0.1:5173",
        "http://localhost:8000",
        "http://127.0.0.1:8000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Wire dependencies and register routers ---
sim_routes.set_dependencies(sim_manager, DATA_DIR)
mission_routes.set_dependencies(sim_manager)
asset_routes.set_dependencies(PROJECT_ROOT, MODEL_ALLOWED_ROOTS)
runner_routes.set_runner_manager(runner_manager)

app.include_router(sim_routes.router)
app.include_router(mission_routes.router)
app.include_router(mission_api_routes.router)
app.include_router(asset_routes.router)
app.include_router(runner_routes.router)


def _is_path_within_ui_dist(path: Path) -> bool:
    try:
        path.relative_to(UI_DIST_DIR)
        return True
    except ValueError:
        return False


def _is_spa_path_blocked(path: str) -> bool:
    if not path:
        return False
    first = path.split("/", 1)[0]
    return first in SPA_BLOCKED_PREFIXES


if UI_DIST_DIR.exists():
    logger.info("Serving prebuilt UI from %s", UI_DIST_DIR)

    @app.get("/", include_in_schema=False)
    async def serve_ui_index():
        return FileResponse(UI_DIST_DIR / "index.html")

    @app.get("/{ui_path:path}", include_in_schema=False)
    async def serve_ui_or_asset(ui_path: str):
        if _is_spa_path_blocked(ui_path):
            raise HTTPException(status_code=404, detail="Not found")

        requested = (UI_DIST_DIR / ui_path).resolve()
        if _is_path_within_ui_dist(requested) and requested.is_file():
            return FileResponse(requested)

        return FileResponse(UI_DIST_DIR / "index.html")
else:
    logger.info(
        "No prebuilt UI found at %s; use Vite dev server on :5173.", UI_DIST_DIR
    )

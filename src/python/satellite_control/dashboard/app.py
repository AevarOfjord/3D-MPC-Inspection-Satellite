"""
Dashboard FastAPI application.

Thin bootstrapping: creates the FastAPI app, applies middleware,
registers route modules, and wires up lifecycle hooks.
"""

import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from satellite_control.dashboard.routes import assets as asset_routes
from satellite_control.dashboard.routes import missions as mission_routes
from satellite_control.dashboard.routes import runner as runner_routes
from satellite_control.dashboard.routes import simulations as sim_routes
from satellite_control.dashboard.runner_manager import RunnerManager
from satellite_control.dashboard.simulation_manager import SimulationManager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dashboard")

# --- Shared constants ---
PROJECT_ROOT = Path(__file__).resolve().parents[4]
DATA_DIR = PROJECT_ROOT / "Data" / "Simulation"
MODEL_ALLOWED_ROOTS = (
    (PROJECT_ROOT / "assets" / "model_files").resolve(),
    (PROJECT_ROOT / "ui" / "public" / "model_files").resolve(),
)

# --- Global Singleton ---
sim_manager = SimulationManager()
runner_manager = RunnerManager()


def get_sim_manager() -> SimulationManager:
    return sim_manager


# --- Lifespan (replaces deprecated on_event) ---
@asynccontextmanager
async def lifespan(application: FastAPI):
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
# --- Wire dependencies and register routers ---
sim_routes.set_dependencies(sim_manager, DATA_DIR)
mission_routes.set_dependencies(sim_manager)
asset_routes.set_dependencies(PROJECT_ROOT, MODEL_ALLOWED_ROOTS)
runner_routes.set_runner_manager(runner_manager)

app.include_router(sim_routes.router)
app.include_router(mission_routes.router)
app.include_router(asset_routes.router)
app.include_router(runner_routes.router)

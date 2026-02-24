# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

All primary commands are via `make` from the repo root:

```bash
make install       # Create .venv311, build C++ extensions, install Python deps (editable)
make rebuild       # Clean + full rebuild from scratch
make run           # Start backend (FastAPI) + frontend (Vite) dev servers
make run-app       # Backend serving prebuilt ui/dist (production mode)
make test          # Run pytest suite (from repo root)
make test-cov      # Run pytest with 30% coverage gate
make lint          # Run ruff (backend) + eslint (frontend)
make lint-backend  # ruff check only
make sim           # Run CLI simulation interactively
make ui-build      # Build production React bundle
make clean         # Remove .venv311, build artifacts, caches
make sync-ui-model-assets  # Mirror data/assets/model_files/ → ui/dist/model_files/
make package-app   # Create distributable .tar.gz bundle
```

**Running a single test** — Python working directory must be `src/python`:
```bash
cd src/python
python -m pytest ../../tests/test_mpc.py::TestMPCController::test_basic_solve -q --tb=short

# Skip slow/e2e tests:
python -m pytest ../../tests/ -m "not slow and not e2e" -q
```

**Frontend tests:**
```bash
cd ui
npm run test          # Vitest unit tests (once)
npm run test:watch    # Vitest watch mode
npm run test:e2e      # Playwright e2e (requires running server on :5173 or :8000)
```

**Verifying C++ build:**
```bash
cd src/python && python -c "from cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('OK')"
```

**Codegen** (CasADi → C Jacobians, auto-cached by source hash):
```bash
cd src/python && python -m control.codegen.generate         # regenerate if stale
cd src/python && python -m control.codegen.generate --force # force full rebuild
```
Codegen must be rerun if `control/codegen/satellite_dynamics.py` or `cost_functions.py` change. Output goes to `codegen_cache/` (not committed).

## Architecture Overview

This is a **hybrid Python + C++ + React** satellite simulation and control system.

### C++ Extension Modules (`src/cpp/`)

Three pybind11 modules compiled via CMake + scikit-build-core:

| Module | Source Dir | Purpose |
|--------|-----------|---------|
| `_cpp_mpc` | `src/cpp/mpc/` | RTI-SQP MPC solver (OSQP-backed) |
| `_cpp_sim` | `src/cpp/sim/` | RK4 physics integrator (16-state plant) |
| `_cpp_physics` | `src/cpp/sim/` | Orbital mechanics utilities |

C++ dependencies (fetched by CMake): Eigen3 3.4.0, OSQP 0.6.3. Python bindings via pybind11.

Clangd LSP will show `'Eigen/Dense' file not found` — this is a false positive because clangd lacks the CMake FetchContent build environment. Ignore these; the actual build works fine.

### Plant State (16-dimensional)

```
x = [r(3), q(4), v(3), w(3), wr(3)]
  r   = relative position (m)
  q   = quaternion (scalar-first / wxyz)
  v   = relative velocity (m/s)
  w   = body angular velocity (rad/s)
  wr  = reaction-wheel speeds (rad/s, 3 wheels)
```

The MPC augments this to 17D by adding `s` (path progress parameter).

### Python Package Layout (`src/python/`)

- **`simulation/`** — `engine.py` (main sim class), `loop.py`, `context.py`, `initialization.py`, `cpp_backend.py`, telemetry utilities
- **`runtime/`** — `control_loop.py`, `mpc_runner.py`, `thruster_manager.py`, `path_completion.py`, `v6_policy.py`, `performance_monitor.py`
- **`control/`** — `mpc_controller.py` (Python wrapper around `_cpp_mpc`), `codegen/` (CasADi symbolic dynamics & Jacobian generation)
- **`mission/`** — `runtime_loader.py` (parse/compile missions), `state.py` (MissionState), `unified_compiler.py`, `unified_mission.py` (Pydantic schema)
- **`config/`** — Pydantic models (`models.py`), constants, physics/timing parameters
- **`physics/`** — `orbital_config.py` (OrbitalConfig), `orbital_dynamics.py`
- **`dashboard/`** — FastAPI app (`app.py`), routes (`simulations.py`, `runner.py`, `missions.py`, `assets.py`)
- **`exceptions.py`** — Top-level `SimulationError` and siblings (not in a subpackage)

Key import paths:
```python
from simulation.engine import SatelliteMPCLinearizedSimulation
from runtime.mpc_runner import MPCRunner
from runtime.control_loop import ControlLoop
from exceptions import SimulationError
from mission.state import MissionState
from physics.orbital_config import OrbitalConfig
from config.simulation_config import SimulationConfig
```

**`src/python` must be the working directory** (or on `sys.path`) — there is no `pyproject.toml` entry adding it automatically. `make test` and `make run` handle this; manual invocations need `cd src/python` first.

### Control Loop Data Flow

```
Mission JSON
  → RuntimeLoader.parse_unified_mission_payload()
  → UnifiedCompiler.compile_unified_mission_runtime()
  → SatelliteMPCLinearizedSimulation (engine.py)
      ├─ MPCController  →  C++ SQPController (RTI-SQP + OSQP)
      ├─ SimulationEngine  →  C++ RK4 integrator
      ├─ ThrusterManager  (PWM / continuous actuation)
      └─ PerformanceMonitor
  → sim.run_simulation()  (loop.py)
      For each step:
        1. Build reference slice from path progress
        2. MPC solve (C++ with CasADi-generated Jacobians)
        3. ThrusterManager maps force/torque → actuator commands
        4. SimulationEngine.step() propagates state (RK4)
        5. Log telemetry; check termination
  → Output: telemetry JSON, matplotlib/plotly plots, optional video
```

### MPC Controller Modes

Modes: `TRACK → RECOVER → SETTLE → HOLD → COMPLETE` — managed by `v6_policy.py` and `control_loop.py`.

| Transition | Trigger |
|-----------|---------|
| TRACK → RECOVER | Solver degraded or tracking error exceeds tolerance |
| RECOVER → TRACK | Errors subside (hysteretic — different entry/exit thresholds) |
| TRACK/RECOVER → SETTLE | Path arc-length exhausted (all waypoints traversed) |
| SETTLE → HOLD | Position/angle/velocity within completion gates for `hold_required_s` |
| SETTLE/HOLD → COMPLETE | All contract gates satisfied for `hold_duration_s` |

Solver health escalation: `ok → degraded → hard_limit_breach` (latched, prevents flapping). Mode is logged to telemetry as `mode_state.current_mode`.

### Mission JSON Schema

Two versions coexist; compiler auto-migrates v1 → v2. Current schema (`schema_version: 2`):

```json
{
  "schema_version": 2,
  "mission_id": "...",
  "epoch": "ISO 8601",
  "start_pose": { "frame": "ECI|LVLH", "position": [x,y,z], "orientation": [w,x,y,z] },
  "segments": [
    { "type": "transfer", "end_pose": {...}, "constraints": {...} },
    { "type": "scan", "scan": { "standoff": 10.0, "fov_deg": 60.0, "revolutions": 4, ... } },
    { "type": "hold", "duration": 10.0 }
  ]
}
```

### Backend API

FastAPI served on `:8000`. Key route groups:

- `GET /simulations` — list runs; `GET /simulations/{id}/telemetry` — CSV physics data with continuous Euler unwrapping
- `GET/POST /runner/config` — read/update MPC params, physics, actuator policy at runtime
- `GET/POST/DELETE /runner/presets` — save/load/apply named config presets
- `GET /runner/workspace/export` / `POST /runner/workspace/import` — ZIP bundle missions + presets + sim data
- `WS /ws` — live telemetry stream; `WS /runner/ws` — simulation log stream; `WS /simulations/runs/ws` — run list push updates

SPA fallback: requests not matching known API prefixes → `index.html`.

### Frontend (`ui/`)

React 19 + Three.js (`@react-three/fiber`) for 3D visualization. Zustand for state, Vite for bundling, Vitest + Playwright for tests. Dev server on `:5173`.

**Three Zustand stores** (`ui/src/store/`):
- **`telemetryStore`** — `latest` frame, 1200-point rolling `history`, full `playbackData[]`, `playbackIndex`, `events[]` (100-event log)
- **`cameraStore`** — `focusTarget`, `focusDistance`, `viewPreset`, `controls` (OrbitControls ref)
- **`viewportStore`** — `canvas` ref, `recording` flag

**Inside a `<Canvas>`** (React Three Fiber), only 3D elements are valid — wrap fallbacks in `<ErrorBoundary>` + `<Suspense>` for components that use `useLoader`. Lazy-loaded models (OBJ) must have both.

### Tests

Pytest markers defined in `tests/conftest.py`:
- `slow`, `e2e` — manual
- `unit`, `integration`, `hardware` — auto-applied by file name pattern

Fixtures auto-applied to every test: `fresh_config` (default `SimulationConfig`), `cleanup_matplotlib` (closes pyplot figures).

**Expected baseline:** 163 passed, 1 skipped, 2 failed.

## Pre-existing Test Failures

Two failures exist on all branches (not regressions):

- `tests/test_mission_workflow.py::TestMissionState::test_mission_state_roundtrip` — `MissionState` missing `get_current_mission_type()`
- `tests/test_mission_workflow.py::TestMissionState::test_mission_reset` — `MissionState` missing `reset()`

## Environment Variables

- `SATELLITE_HEADLESS=0` — Enable GUI windows (default: headless)
- `PYTHONPATH` — Set automatically by `make` targets; manual runs need `cd src/python`
- `VITE_API_BASE` / `VITE_WS_BASE` — Override API/WebSocket base URL in frontend (default: auto-detect from `window.location`)

## Documentation

- `ARCHITECTURE.md` — system design and key interfaces
- `MATHEMATICS.md` — MPC formulation, cost function, DARE terminal costs
- `PHYSICS-ENGINE.md` — RK4 integrator, gravity models, thruster/wheel dynamics

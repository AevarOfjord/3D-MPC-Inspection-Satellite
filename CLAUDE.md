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
make sync-ui-model-assets  # Mirror canonical model assets into the built UI bundle
make package-app   # Create distributable .tar.gz bundle
```

**Running a single test** — run from the repo root (conftest.py adds it to sys.path):
```bash
python -m pytest tests/test_mpc.py::TestMPCController::test_basic_solve -q --tb=short

# Skip slow/e2e tests:
python -m pytest tests/ -m "not slow and not e2e" -q
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
python -c "from controller.shared.python.cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('OK')"
```

**Codegen** (CasADi → C Jacobians, auto-cached by source hash):
```bash
python -m controller.shared.python.control_common.codegen.generate         # regenerate if stale
python -m controller.shared.python.control_common.codegen.generate --force # force full rebuild
```
Codegen must be rerun if `controller/shared/python/control_common/codegen/satellite_dynamics.py` or `cost_functions.py` change. Output goes to `codegen_cache/` (not committed).

## Architecture Overview

This is a **hybrid Python + C++ + React** satellite simulation and control system.

### C++ Extension Modules

Five pybind11 modules compiled via CMake + scikit-build-core, loaded through `controller/shared/python/cpp/__init__.py`:

| Module | Purpose |
| ------ | ------- |
| `_cpp_mpc` | RTI-SQP MPC solver (hybrid/default) |
| `_cpp_mpc_nonlinear` | SQP solver for exact nonlinear linearization |
| `_cpp_mpc_linear` | SQP solver for linear profile |
| `_cpp_sim` | RK4 physics integrator (16-state plant) |
| `_cpp_physics` | Orbital mechanics utilities |

C++ dependencies (fetched by CMake): Eigen3 3.4.0, OSQP 0.6.3. Python bindings via pybind11.

Clangd LSP will show `'Eigen/Dense' file not found` — false positive, ignore. The actual build works fine.

### Plant State (16-dimensional)

```
x = [r(3), q(4), v(3), w(3), wr(3)]
  r   = relative position (m)
  q   = quaternion (scalar-first / wxyz)
  v   = relative velocity (m/s)
  w   = body angular velocity (rad/s)
  wr  = reaction-wheel speeds (rad/s, 3 wheels)
```

The MPC augments this to 17D by adding `s` (path progress parameter). Control vector is 10D: `[τ_rw(3), u_thr(6), v_s(1)]`.

### Controller Profiles

Six canonical profiles are selectable via `AppConfig.mpc_core.controller_profile`:

| Profile ID | Class | Main backend | Linearization / solve style |
| ---------- | ----- | ------------ | ---------------------------- |
| `cpp_hybrid_rti_osqp` (default) | `HybridMPCController` | `_cpp_mpc` | stage-wise RTI linearization with tolerant stale reuse |
| `cpp_nonlinear_rti_osqp` | `NonlinearMPCController` | `_cpp_mpc_nonlinear` | exact stage-wise CasADi linearization, optional outer SQP loop |
| `cpp_linearized_rti_osqp` | `LinearMPCController` | `_cpp_mpc_linear` | frozen affine model reused across horizon |
| `cpp_nonlinear_fullnlp_ipopt` | `NmpcController` | CasADi Opti + IPOPT | full nonlinear NLP |
| `cpp_nonlinear_rti_hpipm` | `AcadosRtiController` | acados + HPIPM | exact nonlinear OCP with `SQP_RTI` |
| `cpp_nonlinear_sqp_hpipm` | `AcadosSqpController` | acados + HPIPM | exact nonlinear OCP with full `SQP` |

Legacy names such as `hybrid`, `nonlinear`, `linear`, `nmpc`, `acados_rti`, and `acados_sqp` are normalized in `controller/registry.py`. Entry point: `controller.factory.create_controller(cfg)`.

### Package Layout

```
controller/
  factory.py              # create_controller() dispatcher
  registry.py             # profile constants (HYBRID_PROFILE, etc.)
  exceptions.py           # SimulationError and siblings
  cli.py                  # CLI entry point
  configs/                # Pydantic config models, constants, physics/timing params
    models.py             # AppConfig (top-level config with mpc_core field)
    simulation_config.py  # SimulationConfig.create_default()
  linear/python/controller.py
  nonlinear/python/controller.py
  hybrid/python/controller.py
  nmpc/python/controller.py
  acados_rti/python/controller.py
  acados_sqp/python/controller.py
  acados_shared/python/base.py
  shared/python/
    control_common/
      base.py             # Controller ABC
      mpc_controller.py   # Shared MPCController base class
      codegen/            # CasADi symbolic dynamics & Jacobian generation
        satellite_dynamics.py
        cost_functions.py
        generate.py
    simulation/
      engine.py           # SatelliteMPCLinearizedSimulation (main sim class)
      loop.py             # run_simulation() step loop
      context.py, initialization.py, cpp_backend.py
      logger.py, step_logging.py, data_logger.py
    runtime/
      mpc_runner.py, thruster_manager.py, path_completion.py
      policy.py           # Mode state machine
      performance_monitor.py
    mission/
      runtime_loader.py   # parse_unified_mission_payload()
      unified_compiler.py # compile_unified_mission_runtime()
      unified_mission.py  # Pydantic schema (v2)
      state.py            # MissionState
    physics/
      orbital_config.py   # OrbitalConfig
      orbital_dynamics.py
    dashboard/
      app.py              # FastAPI app
      routes/             # simulations.py, runner.py, missions.py, missions_api.py, assets.py
    cpp/
      __init__.py         # Loads all _cpp_* extensions; adds build/ to sys.path
    utils/, visualization/, benchmarks/
```

Key import paths (import from repo root):

```python
from controller.factory import create_controller
from controller.shared.python.simulation.engine import SatelliteMPCLinearizedSimulation
from controller.shared.python.runtime.mpc_runner import MPCRunner
from controller.shared.python.mission.state import MissionState
from controller.shared.python.physics.orbital_config import OrbitalConfig
from controller.configs.simulation_config import SimulationConfig
from controller.exceptions import SimulationError
```

### Control Loop Data Flow

```
Mission JSON
  → RuntimeLoader.parse_unified_mission_payload()
  → UnifiedCompiler.compile_unified_mission_runtime()
  → SatelliteMPCLinearizedSimulation (engine.py)
      ├─ create_controller(cfg)  →  selected profile controller
      │                              ├─ OSQP-family C++ SQP runtime, or
      │                              ├─ CasADi Opti + IPOPT, or
      │                              └─ acados nonlinear OCP runtime
      ├─ SimulationEngine  →  C++ RK4 integrator (_cpp_sim)
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

Modes: `TRACK → RECOVER → SETTLE → HOLD → COMPLETE` — managed by `policy.py` and `control_loop.py`.

| Transition | Trigger |
| ---------- | ------- |
| TRACK → RECOVER | Solver degraded or tracking error exceeds tolerance |
| RECOVER → TRACK | Errors subside (hysteretic — different entry/exit thresholds) |
| TRACK/RECOVER → SETTLE | Path arc-length exhausted (all waypoints traversed) |
| SETTLE → HOLD | Position/angle/velocity within completion gates for `hold_required_s` |
| SETTLE/HOLD → COMPLETE | All contract gates satisfied for `hold_duration_s` |

Solver health escalation: `ok → degraded → hard_limit_breach` (latched). Mode logged to telemetry as `mode_state.current_mode`.

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

- `GET /simulations` — list runs; `GET /simulations/{id}/telemetry` — CSV physics data
- `GET/POST /runner/config` — read/update MPC params, physics, actuator policy at runtime
- `GET/POST/DELETE /runner/presets` — save/load/apply named config presets
- `GET /runner/workspace/export` / `POST /runner/workspace/import` — ZIP bundle
- `WS /ws` — live telemetry; `WS /runner/ws` — simulation log; `WS /simulations/runs/ws` — run list updates

SPA fallback: requests not matching API prefixes → `index.html`.

### Frontend (`ui/`)

React 19 + Three.js (`@react-three/fiber`) for 3D visualization. Zustand for state, Vite for bundling, Vitest + Playwright for tests. Dev server on `:5173`.

**Three Zustand stores** (`ui/src/store/`):

- **`telemetryStore`** — `latest` frame, 1200-point rolling `history`, full `playbackData[]`, `playbackIndex`, `events[]` (100-event log)
- **`cameraStore`** — `focusTarget`, `focusDistance`, `viewPreset`, `controls` (OrbitControls ref)
- **`viewportStore`** — `canvas` ref, `recording` flag

**Inside a `<Canvas>`** (React Three Fiber), only 3D elements are valid — wrap fallbacks in `<ErrorBoundary>` + `<Suspense>`. Lazy-loaded OBJ models must have both.

### Tests

Pytest markers defined in `tests/conftest.py`:

- `slow`, `e2e` — manual
- `unit`, `integration`, `hardware` — auto-applied by file name pattern

Fixtures auto-applied to every test: `fresh_config` (yields `SimulationConfig.create_default()`), `cleanup_matplotlib` (closes pyplot figures).

Do not rely on fixed test-count baselines; use current gate outputs (`make test`, `npm --prefix ui run test`) as the source of truth.

## Environment Variables

- `SATELLITE_HEADLESS=0` — Enable GUI windows (default: headless)
- `VITE_API_BASE` / `VITE_WS_BASE` — Override API/WebSocket base URL in frontend (default: auto-detect from `window.location`)

## Documentation

- `ARCHITECTURE.md` — system design and key interfaces
- `MATH/README.md` — shared controller formulation and comparison framing
- `MATH/*.md` — profile-specific controller mathematics
- `PHYSICS-ENGINE.md` — RK4 integrator, gravity models, thruster/wheel dynamics

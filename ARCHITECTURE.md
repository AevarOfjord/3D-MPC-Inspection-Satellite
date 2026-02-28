# Architecture

This document explains how the project is structured, how data/control flow through the system, and how major modules connect.

## 1. System Overview

The repository is a hybrid Python + C++ + React system:

- Python orchestrates mission loading, runtime control flow, simulation management, API serving, and plotting.
- C++ implements the high-performance MPC core, dynamics linearization, and simulation engine (bound into Python via pybind11).
- UI (`ui/`) provides mission authoring, run control, and telemetry visualization through REST + WebSocket APIs.

Primary runtime loop:

1. Load/compile mission and config.
2. Initialize simulation + MPC controller.
3. Run control loop:
   - Build references and runtime mode state.
   - Solve MPC in C++.
   - Apply actuator commands.
   - Advance physics simulation.
   - Log telemetry/metrics.
4. Persist outputs and visualize in dashboard/plots.

## 2. End-to-End Execution Flow

## 2.1 Config and Mission Intake

- Defaults and schema:
  - `src/python/config/defaults.py`
  - `src/python/config/models.py`
  - `src/python/config/validator.py`
- Mission payload parsing and runtime compilation:
  - `src/python/mission/runtime_loader.py`
  - `src/python/mission/unified_compiler.py`
  - `src/python/mission/unified_mission.py`

Result: a validated runtime mission representation with path/reference context and controller settings.

## 2.2 Controller and Simulation Initialization

- High-level simulation object:
  - `src/python/simulation/engine.py`
- Init wiring:
  - `src/python/simulation/initialization.py`
  - `src/python/simulation/cpp_backend.py`
  - `src/python/control/mpc_controller.py`
  - `src/python/runtime/policy.py`

Python wrapper `MPCController` loads C++ extension `cpp._cpp_mpc` (RTI-SQP backend) and passes:

- physical parameters (mass, inertia, thruster geometry, RW config),
- MPC parameters (weights, horizons, bounds, policies),
- path samples for MPCC.

At each solve step, CasADi-generated exact Jacobians are computed in Python and
injected into the C++ SQP controller, which builds and solves the QP via OSQP.

## 2.3 Runtime Control Loop

Main loop modules:

- `src/python/simulation/loop.py`
- `src/python/runtime/control_loop.py`
- `src/python/runtime/mpc_runner.py`
- `src/python/runtime/thruster_manager.py`

Per control step:

1. Compute current mode/state contracts (TRACK/RECOVER/SETTLE/HOLD/COMPLETE).
2. Build reference slice/path context.
3. Call C++ MPC solve.
4. Process actuator policy and safety/fallback behavior.
5. Apply controls to simulation engine.
6. Log state/control/solver timing and completion metrics.

## 2.4 C++ Core Responsibilities

- MPC RTI-SQP solver (CasADi Jacobians + OSQP QP):
  - `src/cpp/mpc/sqp_controller.cpp`
  - `src/cpp/mpc/sqp_controller.hpp`
  - `src/cpp/mpc/sqp_types.cpp`
  - `src/cpp/mpc/sqp_types.hpp`
- CasADi symbolic dynamics and cost codegen (Python-side):
  - `src/python/control/codegen/satellite_dynamics.py`
  - `src/python/control/codegen/cost_functions.py`
  - `src/python/control/codegen/generate.py`
- Orbital dynamics:
  - `src/cpp/sim/orbital_dynamics.cpp`
  - `src/cpp/sim/orbital_dynamics.hpp`
- Simulation engine:
  - `src/cpp/sim/simulation_engine.cpp`
  - `src/cpp/sim/simulation_engine.hpp`
- Python bindings:
  - `src/cpp/mpc/bindings.cpp` (MPC module `_cpp_mpc`)
  - `src/cpp/sim/bindings_sim.cpp`
  - `src/cpp/sim/bindings_physics.cpp`

## 2.5 Dashboard and UI Flow

Backend API:

- FastAPI app: `src/python/dashboard/app.py`
- Routes:
  - `src/python/dashboard/routes/runner.py`
  - `src/python/dashboard/routes/simulations.py`
  - `src/python/dashboard/routes/missions.py`
  - `src/python/dashboard/routes/missions_api.py`
  - `src/python/dashboard/routes/assets.py`

Frontend:

- React entry: `ui/src/main.tsx`, `ui/src/App.tsx`
- API clients: `ui/src/api/*.ts`
- Mission/planner state hooks: `ui/src/hooks/*`
- 3D/telemetry UI: `ui/src/components/*`, `ui/src/store/*`

Run output and artifacts are written under `data/simulation_data/<run_id>/` with:

- `Plots/` for generated plots and plot manifests.
- `Data/01_timeseries/` for control/physics CSV streams and derived step stats.
- `Data/02_metadata/` for run metadata/config/status/performance payloads.
- `Data/03_diagnostics/` for KPI, constraints, controller-health and timelines.
- `Data/04_manifests/` for checksums and artifact indexes.
- `Data/05_notes/` for human-readable summaries/notes.
- `Data/06_media/` for rendered videos/images.

## 3. Key Interfaces and Contracts

## 3.1 Python <-> C++

- Extension loader:
  - `src/python/cpp/__init__.py`
- MPC wrapper:
  - `src/python/control/mpc_controller.py`
- C++ API contract includes:
  - state/control vectors,
  - path data (`s,x,y,z`),
  - solver status/timing fields,
  - projected progress/error metrics.

## 3.2 Backend <-> UI

- REST for mission compile/save/load, run control, and telemetry retrieval.
- WebSocket for run-time streaming/log/status updates.
- JSON payloads for mission definitions, run presets, telemetry series, and artifacts metadata.

## 4. Entry Points

- CLI:
  - `src/python/cli.py`
- Dashboard server:
  - `src/python/dashboard/app.py`
- Script launchers:
  - `scripts/start_app.py`
  - `scripts/run_simulation.py`
  - `scripts/run_mpc_quality_suite.py`
- Python tests:
  - `tests/`
- UI tests:
  - `ui/tests/` and `ui/src/utils/*test*`

## 5. Full Code Tree (Source-Centric)

The tree below focuses on maintained source and runtime assets (excluding generated/cache folders like `.git`, `build`, `.venv`, `.pytest_cache`, `.ruff_cache`).

```text
Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel/
├── src/
│   ├── python/
│   │   ├── cli.py
│   │   ├── exceptions.py
│   │   ├── config/
│   │   │   ├── constants.py
│   │   │   ├── defaults.py
│   │   │   ├── models.py
│   │   │   ├── paths.py
│   │   │   ├── physics.py
│   │   │   ├── reaction_wheel_config.py
│   │   │   ├── simulation_config.py
│   │   │   ├── timing.py
│   │   │   └── validator.py
│   │   ├── control/
│   │   │   ├── base.py
│   │   │   ├── mpc_controller.py
│   │   │   └── codegen/
│   │   │       ├── satellite_dynamics.py
│   │   │       ├── cost_functions.py
│   │   │       └── generate.py
│   │   ├── core/           (compatibility namespace; no active modules)
│   │   ├── cpp/
│   │   │   └── __init__.py
│   │   ├── dashboard/
│   │   │   ├── app.py
│   │   │   ├── mission_service.py
│   │   │   ├── models.py
│   │   │   ├── runner_manager.py
│   │   │   ├── simulation_manager.py
│   │   │   └── routes/
│   │   │       ├── assets.py
│   │   │       ├── missions.py
│   │   │       ├── missions_api.py
│   │   │       ├── runner.py
│   │   │       └── simulations.py
│   │   ├── mission/
│   │   │   ├── mesh_scan.py
│   │   │   ├── mission_report_generator.py
│   │   │   ├── mission_types.py
│   │   │   ├── path_assets.py
│   │   │   ├── path_following.py
│   │   │   ├── repository.py
│   │   │   ├── runtime_loader.py
│   │   │   ├── scan_projects.py
│   │   │   ├── state.py
│   │   │   ├── trajectory_utils.py
│   │   │   ├── unified_compiler.py
│   │   │   └── unified_mission.py
│   │   ├── physics/
│   │   │   ├── orbital_config.py
│   │   │   └── orbital_dynamics.py
│   │   ├── runtime/
│   │   │   ├── control_loop.py
│   │   │   ├── mpc_runner.py
│   │   │   ├── path_completion.py
│   │   │   ├── performance_monitor.py
│   │   │   ├── policy.py
│   │   │   ├── thruster_manager.py
│   │   │   └── __init__.py
│   │   ├── simulation/
│   │   │   ├── context.py
│   │   │   ├── cpp_backend.py
│   │   │   ├── data_logger.py
│   │   │   ├── engine.py
│   │   │   ├── initialization.py
│   │   │   ├── io.py
│   │   │   ├── logger.py
│   │   │   ├── loop.py
│   │   │   ├── reference.py
│   │   │   ├── state_validator.py
│   │   │   └── step_logging.py
│   │   ├── utils/
│   │   │   ├── logging_config.py
│   │   │   ├── navigation_utils.py
│   │   │   └── orientation_utils.py
│   │   └── visualization/
│   │       ├── actuator_plots.py
│   │       ├── command_utils.py
│   │       ├── diagnostics_plots.py
│   │       ├── plot_data_utils.py
│   │       ├── plot_generator.py
│   │       ├── plot_style.py
│   │       ├── simulation_visualization.py
│   │       ├── state_plots.py
│   │       ├── trajectory_plots.py
│   │       ├── unified_visualizer.py
│   │       └── video_renderer.py
│   └── cpp/
│       ├── satellite_params.hpp
│       ├── mpc/
│       │   ├── bindings.cpp
│       │   ├── sqp_controller.cpp
│       │   ├── sqp_controller.hpp
│       │   ├── sqp_types.cpp
│       │   └── sqp_types.hpp
│       └── sim/
│           ├── bindings_physics.cpp
│           ├── bindings_sim.cpp
│           ├── orbital_dynamics.cpp
│           ├── orbital_dynamics.hpp
│           ├── simulation_engine.cpp
│           └── simulation_engine.hpp
├── ui/
│   ├── src/
│   │   ├── main.tsx
│   │   ├── App.tsx
│   │   ├── api/
│   │   ├── components/
│   │   ├── config/
│   │   ├── data/
│   │   ├── feedback/
│   │   ├── hooks/
│   │   ├── services/
│   │   ├── store/
│   │   ├── types/
│   │   └── utils/
│   ├── tests/
│   │   ├── e2e/
│   │   └── unit/
│   ├── package.json
│   ├── vite.config.ts
│   ├── tsconfig.json
│   └── playwright.config.ts
├── tests/
│   ├── conftest.py
│   ├── test_benchmark.py
│   ├── test_config.py
│   ├── test_cpp_integration.py
│   ├── test_dashboard_api.py
│   ├── test_e2e_simulation.py
│   ├── test_math.py
│   ├── test_mission_workflow.py
│   ├── test_mpc.py
│   ├── test_mpc_monte_carlo.py
│   ├── test_missions_api.py
│   ├── test_path_planning.py
│   ├── test_performance_monitor.py
│   ├── test_property_based.py
│   ├── test_runner_workspace_routes.py
│   ├── test_runtime_policy.py
│   ├── test_scan_project_pipeline.py
│   ├── test_state_validation.py
│   ├── test_termination_contract.py
│   ├── test_thruster_logic.py
│   ├── test_unified_compiler.py
│   ├── verify_runner_manager.py
│   └── verify_runner_mission.py
├── scripts/
│   ├── packaged_entrypoint.py
│   ├── run_mpc_quality_suite.py
│   ├── run_simulation.py
│   ├── smoke_test_packaged_app.py
│   ├── start_app.py
│   └── tuning/
├── missions/
├── data/
│   ├── assets/
│   ├── dashboard/
│   └── simulation_data/
├── ARCHITECTURE.md
├── CLAUDE.md
├── MATHEMATICS.md
├── PHYSICS-ENGINE.md
├── README.md
├── pyproject.toml
├── MANIFEST.in
└── Makefile
```

## 6. Notes

- The canonical source of MPC math and objective details is `MATHEMATICS.md`.
- Architecture and behavior should be read with current code defaults from `config/defaults.py` and `config/models.py`.

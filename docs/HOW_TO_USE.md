# How To Use Mission Control

This guide is for running and operating the app day-to-day from the web interface.

## 1) Start The App

### Option A: Standard local run

```bash
make run-app
```

Then open:

- `http://localhost:8000`

### Option B: macOS zero-terminal launch

- Double-click `Start_Mission_Control.command`

### Option C: packaged handoff build

If you received a packaged archive:

- run `./RUN_APP.command` (macOS) or `./RUN_APP.sh`
- open `http://localhost:8000`

## 2) Main Web Workflow

1. `Planner`: use the guided 5-step rail (`Path Maker -> Transfer -> Obstacles -> Path Edit -> Mission Saver`).
2. `Planner` defaults to `Guided` mode; switch to `Advanced` for free step navigation.
3. `Path Maker` step: define plane pairs, shape spirals with handles, and connect endpoints.
4. `Transfer` step: set start pose, select a spiral endpoint, and generate transfer.
5. `Obstacles` step: place spherical obstacles (`position + radius` only, visual diagnostics only).
6. `Path Edit` step: manually drag/add/delete spline points and resolve warnings.
7. `Mission Saver` step: validation + naming helper + save mission.
8. Launch runs from `Runner` mode (Planner save step is save-only in V4.2).
9. `Settings`: tune MPC/simulation settings and click `Save Changes`.
10. `Runner`: start/stop simulation and watch logs.
11. `Viewer`: inspect motion/attitude behavior.
12. `Data`: browse generated files and download artifacts.

### Mission Draft Recovery

- Draft autosave runs every ~5 seconds in Planner.
- On reload, Planner shows a one-shot restore card with `Restore` and `Discard`.

### Planner Onboarding (V4.0)

- First planner open shows a non-blocking `Take 60s Tour` banner.
- Tour coachmarks are dismissable and optional.
- `Never show again` persists per browser profile.

### Mission Templates

- `Quick Inspect`
- `Single Target Spiral`
- `Transfer + Scan`

## 3) Build & Package From UI

Go to `Settings` -> `Build & Package`.

- `Start Packaging`: runs backend packaging job.
- `Download Latest Archive`: downloads newest built app package.
- `Refresh`: updates package status/logs.

## 3.1) Keyboard Workflow (V3.1)

- Open command palette: `Ctrl/Cmd + K`
- Open shortcut help: `?`
- Switch modes: `Ctrl/Cmd + 1..5`
  - `1 Viewer`, `2 Planner`, `3 Runner`, `4 Data`, `5 Settings`
- Planner step jump: `Alt + 1..5`
  - `1 Path Maker`, `2 Transfer`, `3 Obstacles`, `4 Path Edit`, `5 Mission Saver`
- Planner actions:
  - `Ctrl/Cmd + Shift + V`: validate
  - `Ctrl/Cmd + S`: save mission

## 4) Workspace Backup/Restore

Go to `Settings` -> `Build & Package`.

### Export Workspace

- Click `Export Workspace`.
- Optional: enable `Include simulation run data in export` first.
  - Use this only when needed; export can become large.

### Import Workspace

1. Select `.zip`.
2. Click `Inspect Workspace`.
3. Review conflicts:
   - missions
   - presets
   - simulation runs
4. Choose overwrite behavior:
   - global toggles (`Replace existing ...`)
   - per-item checkboxes in conflict lists
5. Click `Import Workspace`.

## 5) System Readiness Check

In `Settings` -> `System Readiness`, verify:

- UI dist present
- runner script paths valid
- required dependencies available

If something is missing, fix that first before running/packaging.

## 6) Useful Commands

```bash
make install        # install deps + build C++ bindings
make ui-build       # build production frontend bundle
make run            # backend + Vite dev mode
make run-app        # backend serves prebuilt UI on :8000
make package-app    # create distributable bundle in ./release
make package-pyinstaller  # build native PyInstaller artifact for current OS
make stop           # stop app processes on known ports
.venv311/bin/python scripts/migrate_missions_v1_to_v2.py missions_unified --recursive --output-dir missions_v2_migrated
.venv311/bin/python scripts/run_mpc_quality_suite.py --fail-on-breach
.venv311/bin/python scripts/run_mpc_quality_suite.py --full --fail-on-breach
.venv311/bin/python scripts/check_v6_cutover_readiness.py --suite-summary Data/Simulation/quality_fast.json --suite-summary Data/Simulation/quality_full.json --schema-migration-ok --fail-on-not-ready
```

## 6.1) Runner Config Schema (V6)

- Canonical runner payload is now:
  - `schema_version: "app_config_v3"`
  - `app_config: { physics, reference_scheduler, mpc_core, actuator_policy, controller_contracts, simulation, input_file_path }`
- `Runner` endpoints still dual-read legacy payloads:
  - legacy `{ control: { mpc: ... }, sim: ... }`
  - v1 flat `{ physics, mpc, simulation, input_file_path }`
  - v2 envelope `{ schema_version: "app_config_v2", app_config: { ... } }`
- New writes from UI/API persist v3 envelope while responses still mirror `physics/mpc/simulation` top-level fields for transition compatibility.
- `config_meta.deprecations` in `/runner/config` indicates the active one-release compatibility sunset window.

## 6.2) Path Termination Contract (V6)

- Path-following completion is strict at the final waypoint and must satisfy all terminal thresholds:
  - position error `<= 0.10 m`
  - angle error `<= 2 deg`
  - linear velocity error `<= 0.05 m/s`
  - angular velocity error `<= 2 deg/s`
- These thresholds must remain satisfied continuously for `10 s` before simulation auto-terminates by default.
- Hold duration is overridable per mission/preset via `mission_state.path_hold_end`.

## 6.3) V6 Diagnostics Artifacts

Each run now includes additional controller diagnostics artifacts:

- `mode_timeline.csv`
- `completion_gate_trace.csv`
- `controller_health.json`
- `contract_report_v6.json` (from quality-suite scenarios)

## 6.4) Solver Fallback Policy (V6)

- On solver non-success, fallback is bounded and time-decayed (not indefinite):
  - hold full last-feasible command for `0.30s`
  - linear decay for `0.70s`
  - force zero at/after `1.00s`
- Runtime telemetry exposes:
  - `fallback_active`
  - `fallback_age_s`
  - `fallback_scale`

## 7) Troubleshooting

- Missing `ui/dist/index.html`:
  - run `make ui-build`
- Packaging fails:
  - open `Settings` -> `Build & Package` and read job logs
- Import didn’t overwrite expected items:
  - re-run `Inspect Workspace`
  - verify global replace toggles and per-item selections
- Mission save blocked by validation:
  - open the `Mission Saver` step and click issue rows to jump to affected fields

## 8) Release Ops

- V4 release and tagging checklist: `docs/RELEASE_V4.md`
- V6 default-cutover readiness gate:
  - Collect quality-suite summaries with `scripts/run_mpc_quality_suite.py --output ...`.
  - Run `scripts/check_v6_cutover_readiness.py` with those summaries and `--schema-migration-ok`.

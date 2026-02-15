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

1. `Mission Planner` or `Scan Planner`: create/edit mission.
2. `Settings`: tune MPC/simulation settings and click `Save Changes`.
3. `Runner`: start/stop simulation and watch logs.
4. `Viewer`: inspect motion/attitude behavior.
5. `Data`: browse generated files and download artifacts.

## 3) Build & Package From UI

Go to `Settings` -> `Build & Package`.

- `Start Packaging`: runs backend packaging job.
- `Download Latest Archive`: downloads newest built app package.
- `Refresh`: updates package status/logs.

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
make stop           # stop app processes on known ports
```

## 7) Troubleshooting

- Missing `ui/dist/index.html`:
  - run `make ui-build`
- Packaging fails:
  - open `Settings` -> `Build & Package` and read job logs
- Import didn’t overwrite expected items:
  - re-run `Inspect Workspace`
  - verify global replace toggles and per-item selections

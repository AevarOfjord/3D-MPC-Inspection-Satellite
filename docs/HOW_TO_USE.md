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

1. `Planner`: use the unified guided rail (`Target -> Segments -> Scan Definition -> Constraints -> Validate -> Save/Launch`).
2. `Planner` defaults to `Guided` mode; switch to `Advanced` for free step navigation.
3. `Scan Definition` step: use basics first, then optional advanced geometry/connectors.
4. `Validate` step: issues are grouped by severity; click an issue row to jump to its field context.
5. `Save/Launch` step: use preflight checklist + naming helper before launch.
6. `Settings`: tune MPC/simulation settings and click `Save Changes`.
7. `Runner`: start/stop simulation and watch logs.
8. `Viewer`: inspect motion/attitude behavior.
9. `Data`: browse generated files and download artifacts.

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
- Planner step jump: `Alt + 1..6`
  - `1 Target`, `2 Segments`, `3 Scan Definition`, `4 Constraints`, `5 Validate`, `6 Save/Launch`
- Planner actions:
  - `Ctrl/Cmd + Shift + V`: validate
  - `Ctrl/Cmd + S`: save mission
  - `Ctrl/Cmd + Enter`: launch mission

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
```

## 7) Troubleshooting

- Missing `ui/dist/index.html`:
  - run `make ui-build`
- Packaging fails:
  - open `Settings` -> `Build & Package` and read job logs
- Import didn’t overwrite expected items:
  - re-run `Inspect Workspace`
  - verify global replace toggles and per-item selections
- Mission save blocked by validation:
  - open the `Validate` step and click issue rows to jump to affected fields

## 8) Release Ops

- V4 release and tagging checklist: `docs/RELEASE_V4.md`

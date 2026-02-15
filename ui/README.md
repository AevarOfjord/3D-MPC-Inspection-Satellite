# Mission Control UI

React + TypeScript + Vite frontend for the Mission Control interface.

## Prerequisites

- Node.js 18+
- Backend running on `http://localhost:8000` (see `run_dashboard.py`)

## Configure API Endpoints (optional)

The UI defaults to `http://localhost:8000` and `ws://localhost:8000/ws`.
Override with:

```
VITE_API_BASE=http://localhost:8000
VITE_WS_BASE=ws://localhost:8000
```

Backward-compatible vars are still accepted:

```
VITE_API_URL=http://localhost:8000
VITE_WS_URL=ws://localhost:8000
```

## Settings Lifecycle

- **Save Changes** stores MPC/simulation overrides in backend memory.
- Starting a run uses those saved overrides.
- Overrides persist across multiple runs until cleared.
- **Reset** in Settings now calls backend reset and restores defaults.
- Restarting backend also clears overrides.
- Presets are stored by backend in `Data/Dashboard/runner_presets.json` and remain available after restart.
- Presets are shared across browsers/users pointing at the same backend.

The Runner panel displays:

- `Config Hash`: short hash of active AppConfig for reproducibility.
- `Overrides Active` vs `Defaults Active`: current config mode.

## Development

```
npm install
npm run dev
```

Startup behavior:
- The UI remembers the last selected top-level tab.
- First launch defaults to `Runner` to avoid loading the 3D stack until needed.
- Hovering/focusing `Viewer`, `Mission Planner`, or `Scan Planner` prefetches 3D modules for faster open.
- `Viewer`, `Mission Planner`, and `Scan Planner` are loaded as separate lazy mode chunks.
- Playback/Data run lists auto-refresh every 5 seconds and when the window regains focus.
- Playback/Data also receive backend push updates over `/simulations/runs/ws` for near-instant new-run discovery.

## Production build

```
npm run build
npm run preview
```

You can also serve `ui/dist` directly from the FastAPI backend:

```
make ui-build
make run-app
# then open http://localhost:8000
```

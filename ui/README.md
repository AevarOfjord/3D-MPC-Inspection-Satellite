# Mission Control UI

React + TypeScript + Vite frontend for the Mission Control interface.

## Prerequisites

- Node.js 18+
- Backend running on `http://localhost:8000` (see `run_dashboard.py`)

## Configure API Endpoints (optional)

The UI defaults to API base `http://localhost:8000` and WS base `ws://localhost:8000`.
Runner logs stream over `ws://localhost:8000/runner/ws` (derived from `VITE_WS_BASE`).
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
- Hovering/focusing `Viewer` or `Planner` prefetches 3D modules for faster open.
- `Viewer` and unified `Planner` are loaded as separate lazy mode chunks.
- V4.0 planner uses a guided UX by default with a 3-column layout and optional coachmarks.
- Planner UX mode persistence key: `mission_control_planner_ux_mode_v1` (`guided` or `advanced`).
- Planner onboarding persistence key: `mission_control_coachmarks_v1`.
- V3.1 adds command palette (`Ctrl/Cmd+K`) and shortcut help (`?`) for quick navigation and planner actions.
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

## V4 Readiness Checks

```
npx playwright test tests/e2e/v4_readiness.spec.ts --config playwright.config.ts
```

Covers V4 KPI flow, desktop layout stability (1280/1440/1920), and keyboard focus/navigation checks.

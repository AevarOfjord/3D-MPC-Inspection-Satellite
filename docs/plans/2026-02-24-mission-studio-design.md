# Mission Studio Design

**Date:** 2026-02-24
**Replaces:** Current 5-step planner (`PlannerModeViewV4`, `FlowStepCardsV42`, `PlannerStepRailV4`)

## Problem

The current planner is a form-wizard: 5 sequential steps, sidebar-heavy layout, and low-level controls ("plane_a", "plane_b", "axis=X"). Users have to understand internal data structures to build a mission. Simple missions are over-engineered; complex multi-pass missions are barely possible.

## Vision

A **3D-first mission editor**. The viewport is the primary workspace. Users load an OBJ model, draw scan paths around it visually, connect segments by dragging wires between glowing endpoint nodes, and attach holds by clicking waypoints. A right-side assembly panel shows the growing mission segment list. No wizard steps. Build in any order.

---

## Layout

```
┌─────────────────────────────────────────────────────────────────────┐
│  [header nav: VIEWER | MISSION STUDIO | RUNNER | DATA | SETTINGS]   │
├──────────────┬──────────────────────────────────────┬───────────────┤
│  LEFT PANEL  │                                      │  RIGHT PANEL  │
│  (280px)     │         3D CANVAS                    │  (260px)      │
│              │    (full-bleed, deep-blue bg,         │               │
│  + Start Pos │     loaded OBJ at origin,             │  MISSION      │
│  + Scan Path │     paths, wires, nodes)              │  ASSEMBLY     │
│  + Transfer  │                                      │               │
│  + Hold      │                                      │  segment list │
│              │                                      │  reorderable  │
│  ──────────  │                                      │               │
│  OBJ Loader  │                                      │  [Validate]   │
│  Model: ...  │                                      │  [Save]       │
│              │                                      │               │
│  ──────────  │                                      │               │
│  Cross-sec.  │                                      │               │
│  editor      │                                      │               │
└──────────────┴──────────────────────────────────────┴───────────────┘
```

**Left panel:** Three collapsible sections:
1. **Add Segment** — action buttons: Start Position, Scan Path, Transfer, Hold
2. **Model** — OBJ file loader + axis selector (X/Y/Z toggle) + plane gap controls
3. **Shape Editor** — 2D cross-section mini-canvas (visible only when a scan pass is selected)

**Right panel:** Mission Assembly list + Validate + Save buttons.

**3D canvas:** Full-bleed between panels. Background color `#070b14` (existing `--v4-bg`). No Earth, no orbit targets, no other models — just the user's OBJ and the mission geometry.

---

## Scene Objects

| Object | Visual | Interaction |
|---|---|---|
| OBJ model | Rendered normally, slightly dimmed (opacity 0.85) | Click to select (reference only) |
| Scan path | Colored polyline — each pass a distinct hue (cyan, violet, amber, ...) | Click to select pass |
| Cross-section ring | Cyan wireframe ring at mid-height of selected pass | Drag 8 control points to reshape |
| Path waypoints | Small white dots along spiral | Click+drag = spline nudge (C1, Gaussian falloff σ=3 points) |
| Endpoint nodes | Large glowing spheres at path start/end and satellite start | Drag wire from node to node to create transfer |
| Transfer wire | Dashed arc between two endpoint nodes | Click to select; shown in assembly list |
| Hold marker | Orange diamond on a waypoint | Click to edit hold duration inline |
| Obstacles | Translucent red spheres | Drag to move; surface handle to resize radius |
| Satellite start | Glowing white crosshair + +X axis arrow | Drag to reposition; arrow auto-points to next waypoint |

---

## Interaction Flows

### Creating a scan pass
1. Press **+ Scan Path** → axis selector (X/Y/Z) appears, two plane handles on model
2. Drag plane handles to position and set gap
3. Cross-section ring appears at mid-height — drag 8 control points to shape
4. Press **Generate** → spiral appears; level height slider in left panel controls turn density
5. Path is now a node with two glowing endpoint spheres (start + end)

### Connecting segments (Transfer)
1. Press **+ Transfer** → all unconnected endpoint nodes pulse
2. Hover an endpoint node → brightens, cursor becomes crosshair
3. Drag from node → dashed wire follows cursor
4. Drop onto another endpoint node → transfer arc snaps in, segment added to assembly list
5. Escape cancels

### Adding a hold
1. Press **+ Hold** → all path waypoints become clickable targets
2. Click any waypoint → orange diamond appears + floating inline duration input
3. Confirm → hold segment inserted into assembly list at that position

### Setting satellite start
1. Press **+ Start Position** → crosshair appears at `[0,0,0]`
2. Drag to reposition (XZ plane by default; hold Shift for Y axis)
3. +X arrow auto-points toward next connected waypoint

### Nudging a waypoint (spline deform)
1. Click any path point → highlights; surrounding ±5 points dim
2. Drag in 3D → path deforms with Gaussian falloff (σ = 3 point spacing), C1 continuous
3. Release → path locks in

### Adding obstacles
1. Press **+ Obstacle** (in Add Segment section) → sphere appears at `[0,0,0]`
2. Drag to position; drag surface handle to set radius
3. Obstacle is a visual reference for routing — also exported to mission JSON

---

## Cross-Section Editor (left panel)

A 200×200px 2D mini-canvas. Shows the spiral cross-section as a closed polygon.

- **8 draggable control points** — drag to reshape
- **Presets:** Circle (8pts), Rectangle (4pts), Hexagon (6pts)
- **Add/remove points:** `[+]` inserts at midpoint of longest edge; `[-]` removes selected point
- **Live 3D preview** — spiral updates in real time (debounced 100ms)
- **Level height slider** — 0.01–2.0m; turn count = plane gap ÷ level height (displayed read-only)
- Shape is uniform across all levels (the "consistent throughout" guarantee)

---

## Mission Assembly Panel (right panel)

```
MISSION ASSEMBLY
────────────────
⠿ ① 🛰  Start Position
⠿ ② ↗  Transfer
⠿ ③ 🔄 Scan A  [X]  ✕
⠿ ④ ↗  Transfer     ✕
⠿ ⑤ 🔄 Scan B  [Z]  ✕
⠿ ⑥ ⏸  Hold 5.0s   ✕
────────────────
6 segments · 847 pts

[  Validate  ]
[    Save    ]
```

- **Drag handle** `⠿` — reorder segments. Breaking a connection shows inline warning: "⚠ Transfer connection broken"
- **Click row** — highlights object in viewport, camera focuses smoothly
- **Axis badge** `[X]` `[Y]` `[Z]` on scan rows — click to change axis inline
- **Delete** `✕` — removes segment and connected transfers (confirmation if breaking connections)
- **Path stats** — live total waypoints + segment count
- **Validate** — runs backend validation; issues appear inline under affected row
- **Save** — enabled when: start set + ≥1 scan + all scans reachable from start + validation passed

---

## Backend Integration

Mission Studio produces the same `UnifiedMission` JSON (schema v2) the backend already consumes: `transfer`, `scan`, `hold` segments in order. No backend changes required. `UnifiedCompiler` already handles this. The new studio is a **frontend-only** replacement for the current planner.

The existing `useMissionBuilder` hook will be partially reused for backend API calls (save, validate). The 3D scene management (nodes, wires, cross-section, spline nudge) will be new React Three Fiber components.

---

## What Does NOT Change

- Backend API, UnifiedMission schema, UnifiedCompiler — untouched
- Viewer mode, Runner mode, Data view, Settings — untouched
- `HudComponents`, `TelemetryPanel`, `ControllerActuatorPanel` — untouched
- The nav tab label changes from "PLANNER" to "MISSION STUDIO"

---

## New Files (approximate)

```
ui/src/components/MissionStudio/
  MissionStudioLayout.tsx       — top-level layout (left panel, canvas, right panel)
  MissionStudioCanvas.tsx       — R3F scene (model, paths, nodes, wires)
  MissionStudioLeftPanel.tsx    — Add Segment buttons + OBJ loader + shape editor
  MissionStudioRightPanel.tsx   — Mission Assembly list
  CrossSectionEditor.tsx        — 2D mini-canvas control point editor
  ScanPathObject.tsx            — R3F: renders one spiral pass + endpoint nodes
  TransferWireObject.tsx        — R3F: renders dashed transfer arc
  HoldMarkerObject.tsx          — R3F: renders orange hold diamond
  SatelliteStartObject.tsx      — R3F: renders crosshair + axis arrow
  ObstacleObject.tsx            — R3F: renders draggable red sphere
  useStudioState.ts             — Zustand store for all studio state
  useSpiralGenerator.ts         — generates spiral waypoints from planes + cross-section
  useSplineNudge.ts             — spline deform with Gaussian falloff
  useWireDrag.ts                — drag-wire interaction state machine
```

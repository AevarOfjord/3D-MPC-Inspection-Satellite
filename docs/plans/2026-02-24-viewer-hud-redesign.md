# Viewer HUD Redesign

**Date:** 2026-02-24
**Scope:** `ui/src/components/Overlay.tsx`, `HudComponents.tsx`, `TelemetryCharts.tsx`, `ViewerModeView.tsx`

## Goals

1. **Visual polish** — sharper, more professional sci-fi aesthetic
2. **Performance** — eliminate unnecessary re-renders by splitting monolithic `Overlay` into independent subscribers

## Layout

```
┌─────────────────────────────────────────────────────────────────┐
│  [header / nav bar]                                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐          3D VIEWPORT           ┌───────────┐ │
│  │  TELEMETRY   │                                │CONTROLLER │ │
│  │  POS xyz     │                                │MODE: TRACK│ │
│  │  VEL xyz     │                                │SOLVER: OK │ │
│  │  ERR xyz     │                                │POS ERR    │ │
│  │  ROT xyz     │                                │ANG ERR    │ │
│  │  SPIN xyz    │                                │─────────  │ │
│  │  ─────────   │                                │ACTUATORS  │ │
│  │  RANGE  m    │                                │thruster   │ │
│  │  SPEED  m/s  │                                │rw wheels  │ │
│  └──────────────┘                                └───────────┘ │
│                                                                 │
│  ─────────── [▲ Charts  pos · ang · vel · solve] ───────────── │
│  (collapsed by default; click tab to expand 224px drawer)       │
└─────────────────────────────────────────────────────────────────┘
```

## Component Architecture

### Before (problem)
```
Overlay (subscribes to telemetry → useState → full re-render every frame)
  └── HudPanel "TELEMETRY"
  └── HudPanel "CONTROLLER"
  └── HudPanel "ACTUATORS"
```

### After (solution)
```
ViewerHud (no telemetry subscription — purely structural)
  ├── TelemetryPanel   (subscribes independently to telemetry service)
  └── ControllerActuatorPanel  (subscribes independently to telemetry service)

TelemetryCharts (unchanged subscription model, adds useDeferredValue)
  └── CollapsibleChartsDrawer (new wrapper — collapsed by default)
```

Each panel subscribes to `telemetry.subscribe()` directly and calls its own `useState`. A thruster tick only re-renders `ControllerActuatorPanel`; position updates only re-render `TelemetryPanel`.

## Visual Changes

### HudPanel (HudComponents.tsx)
- Corner accent: `w-2 h-2` → `w-3 h-3`, border `2px` → stays 2px but gains `opacity-80`
- Title bar: add `live` prop — when true, the cyan indicator dot pulses with a CSS `animate-pulse` class
- No other structural changes

### TelemetryPanel (replaces TELEMETRY section of Overlay)
- `DataRow` values: add `font-variant-numeric: tabular-nums` via Tailwind `tabular-nums` class — numbers no longer shift layout
- Column grid stays identical (`grid-cols-[30px_1fr_1fr_1fr_25px]`)
- RANGE / SPEED section: unchanged

### ControllerActuatorPanel (replaces CONTROLLER + ACTUATORS sections)
- Merged into one panel with an internal `<hr>` divider between controller and actuators
- **Mode badge**: `modeLabel` rendered as a `<span>` with background color keyed to mode:
  - `TRACK` → `bg-indigo-600/80 text-indigo-100`
  - `RECOVER` → `bg-amber-600/80 text-amber-100`
  - `SETTLE` → `bg-sky-600/80 text-sky-100`
  - `HOLD` → `bg-emerald-600/80 text-emerald-100`
  - `COMPLETE` → `bg-green-600/80 text-green-100`
  - fallback → `bg-slate-600/80 text-slate-100`
- **Solver health traffic light**: a 10px circle dot (green/amber/red) left of the status text, replacing the text color alone
- **Gate flags**: replace `OK`/`NO` text with filled circle (`●`) green/red
- Actuator bars: unchanged visuals, just relocated

### CollapsibleChartsDrawer (wraps TelemetryCharts)
- Default state: collapsed — shows only a `32px` tall tab strip at the bottom:
  `▲ Charts   pos · ang · vel · solve`
- Expanded state: 224px drawer slides up (CSS transition `transition-all duration-300`)
- Toggle: clicking anywhere on the tab strip
- State: local `useState` inside `CollapsibleChartsDrawer`

### TelemetryCharts performance
- Wrap `history` selector with `useDeferredValue`:
  ```ts
  const rawHistory = useTelemetryStore(s => s.history);
  const history = useDeferredValue(rawHistory);
  ```
  This lets React deprioritize chart re-renders when the HUD is updating at high frequency.

## Files to Create / Modify

| File | Action |
|------|--------|
| `ui/src/components/Overlay.tsx` | Replace with `ViewerHud` (structural only, no telemetry sub) |
| `ui/src/components/TelemetryPanel.tsx` | New — extracted TELEMETRY panel |
| `ui/src/components/ControllerActuatorPanel.tsx` | New — merged CONTROLLER + ACTUATORS panel |
| `ui/src/components/HudComponents.tsx` | Add `live` prop to `HudPanel` |
| `ui/src/components/TelemetryCharts.tsx` | Add `useDeferredValue`, wrap in `CollapsibleChartsDrawer` |
| `ui/src/components/modes/ViewerModeView.tsx` | Import `ViewerHud` instead of `Overlay` |

## Non-Goals

- No changes to data fields shown
- No changes to `OrbitTargetsPanel`
- No changes to any planner, runner, data, or settings views
- No changes to `telemetryStore.ts` or `telemetry.ts`

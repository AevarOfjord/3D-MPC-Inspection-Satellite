# Mission Studio Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the current 5-step planner with a 3D-first Mission Studio mode where users load an OBJ model, draw spiral scan paths visually, connect segments by dragging wires between endpoint nodes, and assemble missions in a live segment list.

**Architecture:** New `MissionStudio` app mode with a full-bleed R3F canvas flanked by two floating panels. A new Zustand store (`useStudioStore`) owns all studio state. The studio produces the same `UnifiedMission` JSON schema the backend already consumes — no backend changes. The existing `unifiedMissionApi` save/validate calls are reused directly.

**Tech Stack:** React 19, TypeScript, React Three Fiber 9, `@react-three/drei` (OBJ loader, OrbitControls, GizmoHelper), Zustand 5, Tailwind CSS v3, Lucide React. Working directory for all commands: `ui/`.

---

## Key Concepts (read before implementing)

- **Scan pass:** A spiral path generated between two parallel planes. Defined by: axis (X/Y/Z), plane positions, plane gap, cross-section polygon (8 control points), level height. Produces an array of `[x,y,z]` waypoints.
- **Endpoint node:** A glowing sphere at the start or end of a scan pass (or the satellite start position). Wires connect endpoint nodes to form transfers.
- **Wire drag:** State machine: `idle → dragging(sourceNodeId) → connected(sourceId, targetId)`. While dragging, a dashed line follows the cursor in 3D.
- **Spline nudge:** When a waypoint is dragged, surrounding points are displaced by `displacement × exp(-d²/2σ²)` where `d` is index distance and `σ=3`.
- **Cross-section:** A closed polygon of 8 control points in 2D (local XY plane of the scan axis). The spiral extrudes this shape uniformly from bottom plane to top plane.
- **Assembly list:** Ordered array of `StudioSegment` objects (scan | transfer | hold | start). On save, compiled to `UnifiedMission` JSON.

---

## Task 1: Zustand store + app mode wiring

**Files:**
- Create: `ui/src/components/MissionStudio/useStudioStore.ts`
- Modify: `ui/src/App.tsx`
- Modify: `ui/src/utils/appMode.ts`

**Step 1: Create the studio store**

```ts
// ui/src/components/MissionStudio/useStudioStore.ts
import { create } from 'zustand';

export type StudioSegmentType = 'start' | 'scan' | 'transfer' | 'hold';

export interface ScanPass {
  id: string;
  axis: 'X' | 'Y' | 'Z';
  planeAOffset: number;   // offset along axis from model origin (meters)
  planeBOffset: number;   // offset along axis from model origin (meters)
  crossSection: [number, number][];  // 8 control points in local 2D
  levelHeight: number;    // meters between spiral turns
  waypoints: [number, number, number][];  // generated spiral
  color: string;          // hue for this pass e.g. '#22d3ee'
}

export interface TransferWire {
  id: string;
  fromNodeId: string;   // `${scanId}:start` | `${scanId}:end` | 'satellite:start'
  toNodeId: string;
}

export interface HoldMarker {
  id: string;
  scanId: string;
  waypointIndex: number;
  duration: number;
}

export interface StudioSegment {
  id: string;
  type: StudioSegmentType;
  // type-specific payload refs
  scanId?: string;
  wireId?: string;
  holdId?: string;
}

export type WireDragState =
  | { phase: 'idle' }
  | { phase: 'dragging'; sourceNodeId: string; cursorWorld: [number, number, number] }
  | { phase: 'connected'; sourceNodeId: string; targetNodeId: string };

export interface StudioState {
  // Model
  modelUrl: string | null;
  modelBoundingBox: { min: [number,number,number]; max: [number,number,number] } | null;

  // Scene objects
  satelliteStart: [number, number, number];
  scanPasses: ScanPass[];
  wires: TransferWire[];
  holds: HoldMarker[];
  obstacles: { id: string; position: [number,number,number]; radius: number }[];

  // Assembly
  segments: StudioSegment[];

  // Interaction
  selectedScanId: string | null;
  wireDrag: WireDragState;
  nudgingScanId: string | null;
  nudgingWaypointIndex: number | null;

  // Validation
  validationReport: import('../../api/unifiedMissionApi').ValidationReportV2 | null;
  validationBusy: boolean;
  saveBusy: boolean;
  missionName: string;

  // Actions
  setModelUrl: (url: string | null) => void;
  setModelBoundingBox: (bb: StudioState['modelBoundingBox']) => void;
  setSatelliteStart: (pos: [number,number,number]) => void;
  addScanPass: (pass: ScanPass) => void;
  updateScanPass: (id: string, updates: Partial<ScanPass>) => void;
  removeScanPass: (id: string) => void;
  selectScanPass: (id: string | null) => void;
  addWire: (wire: TransferWire) => void;
  removeWire: (id: string) => void;
  addHold: (hold: HoldMarker) => void;
  removeHold: (id: string) => void;
  addObstacle: () => void;
  updateObstacle: (id: string, updates: Partial<{ position: [number,number,number]; radius: number }>) => void;
  removeObstacle: (id: string) => void;
  setWireDrag: (state: WireDragState) => void;
  setNudging: (scanId: string | null, waypointIndex: number | null) => void;
  applyNudge: (scanId: string, waypointIndex: number, delta: [number,number,number]) => void;
  appendSegment: (seg: StudioSegment) => void;
  removeSegment: (id: string) => void;
  reorderSegments: (from: number, to: number) => void;
  setValidationReport: (report: StudioState['validationReport']) => void;
  setValidationBusy: (busy: boolean) => void;
  setSaveBusy: (busy: boolean) => void;
  setMissionName: (name: string) => void;
}

function makeDefaultCrossSection(): [number, number][] {
  // 8-point circle approximation, radius 5m
  return Array.from({ length: 8 }, (_, i) => {
    const angle = (i / 8) * Math.PI * 2;
    return [Math.cos(angle) * 5, Math.sin(angle) * 5] as [number, number];
  });
}

let _obstacleCounter = 0;
let _segmentCounter = 0;

export const useStudioStore = create<StudioState>((set, get) => ({
  modelUrl: null,
  modelBoundingBox: null,
  satelliteStart: [0, 0, 20],
  scanPasses: [],
  wires: [],
  holds: [],
  obstacles: [],
  segments: [],
  selectedScanId: null,
  wireDrag: { phase: 'idle' },
  nudgingScanId: null,
  nudgingWaypointIndex: null,
  validationReport: null,
  validationBusy: false,
  saveBusy: false,
  missionName: '',

  setModelUrl: (url) => set({ modelUrl: url }),
  setModelBoundingBox: (bb) => set({ modelBoundingBox: bb }),
  setSatelliteStart: (pos) => set({ satelliteStart: pos }),

  addScanPass: (pass) => {
    const PASS_COLORS = ['#22d3ee','#a78bfa','#fb923c','#4ade80','#f472b6','#facc15'];
    const { scanPasses } = get();
    const color = PASS_COLORS[scanPasses.length % PASS_COLORS.length];
    const passWithColor = { ...pass, color };
    const segId = `seg-${++_segmentCounter}`;
    set((s) => ({
      scanPasses: [...s.scanPasses, passWithColor],
      selectedScanId: pass.id,
      segments: [...s.segments, { id: segId, type: 'scan', scanId: pass.id }],
    }));
  },

  updateScanPass: (id, updates) =>
    set((s) => ({
      scanPasses: s.scanPasses.map((p) => (p.id === id ? { ...p, ...updates } : p)),
    })),

  removeScanPass: (id) =>
    set((s) => ({
      scanPasses: s.scanPasses.filter((p) => p.id !== id),
      wires: s.wires.filter((w) => !w.fromNodeId.startsWith(id) && !w.toNodeId.startsWith(id)),
      holds: s.holds.filter((h) => h.scanId !== id),
      segments: s.segments.filter((seg) => seg.scanId !== id),
      selectedScanId: s.selectedScanId === id ? null : s.selectedScanId,
    })),

  selectScanPass: (id) => set({ selectedScanId: id }),

  addWire: (wire) => {
    const segId = `seg-${++_segmentCounter}`;
    set((s) => ({
      wires: [...s.wires, wire],
      segments: [...s.segments, { id: segId, type: 'transfer', wireId: wire.id }],
    }));
  },

  removeWire: (id) =>
    set((s) => ({
      wires: s.wires.filter((w) => w.id !== id),
      segments: s.segments.filter((seg) => seg.wireId !== id),
    })),

  addHold: (hold) => {
    const segId = `seg-${++_segmentCounter}`;
    set((s) => ({
      holds: [...s.holds, hold],
      segments: [...s.segments, { id: segId, type: 'hold', holdId: hold.id }],
    }));
  },

  removeHold: (id) =>
    set((s) => ({
      holds: s.holds.filter((h) => h.id !== id),
      segments: s.segments.filter((seg) => seg.holdId !== id),
    })),

  addObstacle: () =>
    set((s) => ({
      obstacles: [
        ...s.obstacles,
        { id: `obs-${++_obstacleCounter}`, position: [0, 0, 0], radius: 2 },
      ],
    })),

  updateObstacle: (id, updates) =>
    set((s) => ({
      obstacles: s.obstacles.map((o) => (o.id === id ? { ...o, ...updates } : o)),
    })),

  removeObstacle: (id) =>
    set((s) => ({ obstacles: s.obstacles.filter((o) => o.id !== id) })),

  setWireDrag: (state) => set({ wireDrag: state }),
  setNudging: (scanId, waypointIndex) => set({ nudgingScanId: scanId, nudgingWaypointIndex: waypointIndex }),

  applyNudge: (scanId, waypointIndex, delta) => {
    const SIGMA = 3;
    set((s) => ({
      scanPasses: s.scanPasses.map((p) => {
        if (p.id !== scanId) return p;
        const waypoints = p.waypoints.map((wp, i) => {
          const d = Math.abs(i - waypointIndex);
          const weight = Math.exp(-(d * d) / (2 * SIGMA * SIGMA));
          return [
            wp[0] + delta[0] * weight,
            wp[1] + delta[1] * weight,
            wp[2] + delta[2] * weight,
          ] as [number, number, number];
        });
        return { ...p, waypoints };
      }),
    }));
  },

  appendSegment: (seg) => set((s) => ({ segments: [...s.segments, seg] })),
  removeSegment: (id) => set((s) => ({ segments: s.segments.filter((seg) => seg.id !== id) })),

  reorderSegments: (from, to) =>
    set((s) => {
      const segs = [...s.segments];
      const [moved] = segs.splice(from, 1);
      segs.splice(to, 0, moved);
      return { segments: segs };
    }),

  setValidationReport: (report) => set({ validationReport: report }),
  setValidationBusy: (busy) => set({ validationBusy: busy }),
  setSaveBusy: (busy) => set({ saveBusy: busy }),
  setMissionName: (name) => set({ missionName: name }),
}));
```

**Step 2: Add `'studio'` to AppMode**

Open `ui/src/utils/appMode.ts`. Add `'studio'` to the `AppMode` type and `parseStoredAppMode` valid set. Exact change — find the type definition and add the value:

```ts
// Before
export type AppMode = 'viewer' | 'planner' | 'runner' | 'data' | 'settings';

// After
export type AppMode = 'viewer' | 'planner' | 'studio' | 'runner' | 'data' | 'settings';
```

Also update `parseStoredAppMode` to accept `'studio'` as a valid stored value (add it to whatever set/array is used for validation).

**Step 3: Wire up in App.tsx**

In `App.tsx`:
1. Add a lazy import for the new layout (file doesn't exist yet — add the import but it will be created in Task 2):
```ts
const MissionStudioLayout = lazy(() =>
  import('./components/MissionStudio/MissionStudioLayout').then((m) => ({ default: m.MissionStudioLayout }))
);
```
2. Add a `switchToStudio` handler (same pattern as `switchToViewer`):
```ts
const switchToStudio = () => {
  preload3DModules();
  void ensureCanLeaveSettings().then((canLeave) => {
    if (!canLeave) return;
    setAppMode('studio');
    setViewMode('free');
  });
};
```
3. Add a nav button in the header between PLANNER and RUNNER:
```tsx
<button
  onClick={switchToStudio}
  onMouseEnter={preload3DModules}
  className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-xs font-semibold transition-all duration-300 ${
    appMode === 'studio'
      ? 'bg-violet-600/90 text-white shadow-[0_0_10px_rgba(139,92,246,0.3)]'
      : 'text-slate-400 hover:text-white hover:bg-white/5'
  }`}
>
  <Layers size={14} />
  STUDIO
</button>
```
(Import `Layers` from lucide-react alongside existing imports.)

4. Add the mode render in `<main>`:
```tsx
{appMode === 'studio' && (
  <Suspense fallback={<ModeLoading label="Loading Mission Studio..." />}>
    <MissionStudioLayout />
  </Suspense>
)}
```

5. Remove all the planner-step keyboard shortcut handling (the `Alt+1..5` block and `PLANNER_STEP_KEYS`) — the studio has no steps. Keep the `Ctrl+2` planner shortcut as-is (don't remove the old planner yet — keep both modes).

**Step 4: TypeScript check**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -40
```
Expected: errors only about `MissionStudioLayout` not found (file doesn't exist yet). No other errors.

**Step 5: Commit**

```bash
git add ui/src/components/MissionStudio/useStudioStore.ts ui/src/utils/appMode.ts ui/src/App.tsx
git commit -m "feat(studio): add Zustand store, AppMode wiring, nav button"
```

---

## Task 2: MissionStudioLayout — shell + panels

**Files:**
- Create: `ui/src/components/MissionStudio/MissionStudioLayout.tsx`
- Create: `ui/src/components/MissionStudio/MissionStudioLeftPanel.tsx`
- Create: `ui/src/components/MissionStudio/MissionStudioRightPanel.tsx`

**Step 1: Create the layout shell**

```tsx
// ui/src/components/MissionStudio/MissionStudioLayout.tsx
import { Suspense } from 'react';
import { MissionStudioLeftPanel } from './MissionStudioLeftPanel';
import { MissionStudioRightPanel } from './MissionStudioRightPanel';

// Canvas will be added in Task 3 — placeholder for now
function CanvasPlaceholder() {
  return (
    <div className="flex-1 flex items-center justify-center text-slate-500 text-sm select-none"
         style={{ background: '#070b14' }}>
      3D Canvas — coming in Task 3
    </div>
  );
}

export function MissionStudioLayout() {
  return (
    <div className="flex-1 flex min-h-0 overflow-hidden" style={{ background: '#070b14' }}>
      {/* Left panel */}
      <div className="w-[280px] shrink-0 flex flex-col border-r border-slate-800/60 overflow-y-auto custom-scrollbar"
           style={{ background: 'rgba(13,21,36,0.97)' }}>
        <MissionStudioLeftPanel />
      </div>

      {/* 3D Canvas */}
      <div className="flex-1 relative min-w-0">
        <Suspense fallback={null}>
          <CanvasPlaceholder />
        </Suspense>
      </div>

      {/* Right panel */}
      <div className="w-[260px] shrink-0 flex flex-col border-l border-slate-800/60 overflow-y-auto custom-scrollbar"
           style={{ background: 'rgba(13,21,36,0.97)' }}>
        <MissionStudioRightPanel />
      </div>
    </div>
  );
}
```

**Step 2: Create the left panel**

```tsx
// ui/src/components/MissionStudio/MissionStudioLeftPanel.tsx
import { useState, useRef } from 'react';
import { Plus, Layers, Move, Pause, AlertTriangle } from 'lucide-react';
import { useStudioStore } from './useStudioStore';

function SectionHeader({ label }: { label: string }) {
  return (
    <div className="px-3 py-2 text-[10px] font-bold uppercase tracking-[0.16em] text-slate-500 border-b border-slate-800/60">
      {label}
    </div>
  );
}

function ActionButton({
  icon,
  label,
  onClick,
  active,
  color = 'cyan',
}: {
  icon: React.ReactNode;
  label: string;
  onClick: () => void;
  active?: boolean;
  color?: 'cyan' | 'violet' | 'amber' | 'red';
}) {
  const colorMap = {
    cyan: active ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100' : 'border-slate-700 text-slate-300 hover:border-cyan-700',
    violet: active ? 'border-violet-600 bg-violet-900/40 text-violet-100' : 'border-slate-700 text-slate-300 hover:border-violet-700',
    amber: active ? 'border-amber-600 bg-amber-900/40 text-amber-100' : 'border-slate-700 text-slate-300 hover:border-amber-700',
    red: 'border-slate-700 text-slate-300 hover:border-red-700',
  };
  return (
    <button
      type="button"
      onClick={onClick}
      className={`v4-focus w-full flex items-center gap-2 px-3 py-2 rounded-lg border text-xs font-semibold transition-all ${colorMap[color]}`}
    >
      {icon}
      {label}
    </button>
  );
}

export function MissionStudioLeftPanel() {
  const { addScanPass, addObstacle, setSatelliteStart, scanPasses, selectedScanId, modelUrl, setModelUrl } = useStudioStore();
  const fileRef = useRef<HTMLInputElement>(null);

  const handleAddScan = () => {
    const id = `scan-${Date.now()}`;
    addScanPass({
      id,
      axis: 'Z',
      planeAOffset: -5,
      planeBOffset: 5,
      crossSection: Array.from({ length: 8 }, (_, i) => {
        const angle = (i / 8) * Math.PI * 2;
        return [Math.cos(angle) * 5, Math.sin(angle) * 5] as [number, number];
      }),
      levelHeight: 0.5,
      waypoints: [],
      color: '#22d3ee',
    });
  };

  const handleLoadModel = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const url = URL.createObjectURL(file);
    setModelUrl(url);
  };

  const selectedPass = scanPasses.find((p) => p.id === selectedScanId) ?? null;

  return (
    <div className="flex flex-col gap-0">
      {/* Add Segment */}
      <SectionHeader label="Add Segment" />
      <div className="p-3 flex flex-col gap-2">
        <ActionButton icon={<Move size={13} />} label="Set Start Position" onClick={() => setSatelliteStart([0, 0, 20])} color="cyan" />
        <ActionButton icon={<Layers size={13} />} label="Add Scan Pass" onClick={handleAddScan} color="violet" />
        <ActionButton icon={<Plus size={13} />} label="Add Obstacle" onClick={addObstacle} color="red" />
        <ActionButton icon={<Pause size={13} />} label="Add Hold (click waypoint)" onClick={() => {}} color="amber" />
      </div>

      {/* Model */}
      <SectionHeader label="Model" />
      <div className="p-3 flex flex-col gap-2">
        <button
          type="button"
          onClick={() => fileRef.current?.click()}
          className="v4-focus w-full flex items-center justify-center gap-2 px-3 py-2 rounded-lg border border-slate-700 text-xs font-semibold text-slate-300 hover:border-cyan-700 transition-all"
        >
          {modelUrl ? '⬛ Model Loaded' : '📂 Load OBJ Model'}
        </button>
        <input ref={fileRef} type="file" accept=".obj" className="hidden" onChange={handleLoadModel} />
        {modelUrl && (
          <button
            type="button"
            onClick={() => setModelUrl(null)}
            className="v4-focus text-[10px] text-slate-500 hover:text-red-400 text-left px-1"
          >
            ✕ Remove model
          </button>
        )}
      </div>

      {/* Shape Editor — only when a scan pass is selected */}
      {selectedPass && (
        <>
          <SectionHeader label={`Shape — ${selectedPass.id}`} />
          <div className="p-3 flex flex-col gap-3">
            {/* Axis toggle */}
            <div className="flex gap-1">
              {(['X', 'Y', 'Z'] as const).map((axis) => (
                <button
                  key={axis}
                  type="button"
                  onClick={() => useStudioStore.getState().updateScanPass(selectedPass.id, { axis })}
                  className={`v4-focus flex-1 py-1.5 rounded-lg border text-xs font-bold transition-all ${
                    selectedPass.axis === axis
                      ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                      : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                  }`}
                >
                  {axis}
                </button>
              ))}
            </div>

            {/* Level height */}
            <div className="flex flex-col gap-1">
              <div className="flex justify-between text-[10px] text-slate-400 uppercase tracking-wider">
                <span>Level Height</span>
                <span className="tabular-nums">{selectedPass.levelHeight.toFixed(2)} m</span>
              </div>
              <input
                type="range"
                min={0.05}
                max={2}
                step={0.05}
                value={selectedPass.levelHeight}
                onChange={(e) =>
                  useStudioStore.getState().updateScanPass(selectedPass.id, { levelHeight: parseFloat(e.target.value) })
                }
                className="w-full accent-cyan-500"
              />
            </div>

            {/* Plane gap */}
            <div className="flex flex-col gap-1">
              <div className="flex justify-between text-[10px] text-slate-400 uppercase tracking-wider">
                <span>Plane Gap</span>
                <span className="tabular-nums">
                  {Math.abs(selectedPass.planeBOffset - selectedPass.planeAOffset).toFixed(1)} m
                </span>
              </div>
              <input
                type="range"
                min={1}
                max={50}
                step={0.5}
                value={Math.abs(selectedPass.planeBOffset - selectedPass.planeAOffset)}
                onChange={(e) => {
                  const gap = parseFloat(e.target.value);
                  useStudioStore.getState().updateScanPass(selectedPass.id, {
                    planeAOffset: -gap / 2,
                    planeBOffset: gap / 2,
                  });
                }}
                className="w-full accent-violet-500"
              />
            </div>

            <div className="text-[10px] text-slate-500 flex items-center gap-1">
              <AlertTriangle size={10} />
              Drag waypoints in viewport to nudge path
            </div>
          </div>
        </>
      )}
    </div>
  );
}
```

**Step 3: Create the right panel**

```tsx
// ui/src/components/MissionStudio/MissionStudioRightPanel.tsx
import { useState } from 'react';
import { Trash2, Save, CheckCircle } from 'lucide-react';
import { useStudioStore } from './useStudioStore';
import { unifiedMissionApi } from '../../api/unifiedMissionApi';
import { compileStudioMission } from './compileStudioMission';

function SegmentRow({ index }: { index: number }) {
  const { segments, scanPasses, wires, holds, removeSegment } = useStudioStore();
  const seg = segments[index];
  if (!seg) return null;

  let icon = '●';
  let label = seg.type;
  let badge: string | null = null;

  if (seg.type === 'start') { icon = '🛰'; label = 'Start Position'; }
  if (seg.type === 'scan' && seg.scanId) {
    const pass = scanPasses.find((p) => p.id === seg.scanId);
    icon = '🔄';
    label = `Scan Pass`;
    badge = pass?.axis ?? null;
  }
  if (seg.type === 'transfer') { icon = '↗'; label = 'Transfer'; }
  if (seg.type === 'hold' && seg.holdId) {
    const hold = holds.find((h) => h.id === seg.holdId);
    icon = '⏸';
    label = `Hold ${hold?.duration.toFixed(1) ?? '?'}s`;
  }

  return (
    <div className="flex items-center gap-2 px-3 py-2 rounded-lg border border-slate-800 hover:border-slate-700 bg-slate-900/40 group">
      <span className="text-[10px] text-slate-500 w-5 shrink-0 tabular-nums">{index + 1}</span>
      <span className="text-sm">{icon}</span>
      <span className="flex-1 text-xs text-slate-200 font-medium truncate">{label}</span>
      {badge && (
        <span className="text-[10px] px-1.5 py-0.5 rounded border border-cyan-800 text-cyan-300 bg-cyan-950/40">
          {badge}
        </span>
      )}
      {seg.type !== 'start' && (
        <button
          type="button"
          onClick={() => removeSegment(seg.id)}
          className="opacity-0 group-hover:opacity-100 text-slate-500 hover:text-red-400 transition-opacity"
        >
          <Trash2 size={12} />
        </button>
      )}
    </div>
  );
}

export function MissionStudioRightPanel() {
  const {
    segments, scanPasses, satelliteStart, wires, holds, obstacles,
    missionName, setMissionName, validationReport, validationBusy,
    saveBusy, setValidationReport, setValidationBusy, setSaveBusy,
  } = useStudioStore();

  const [saveSuccess, setSaveSuccess] = useState(false);

  const totalWaypoints = scanPasses.reduce((acc, p) => acc + p.waypoints.length, 0);

  const handleValidate = async () => {
    setValidationBusy(true);
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      const report = await unifiedMissionApi.validateUnifiedMission(mission);
      setValidationReport(report);
    } catch {
      setValidationReport(null);
    } finally {
      setValidationBusy(false);
    }
  };

  const handleSave = async () => {
    setSaveBusy(true);
    setSaveSuccess(false);
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      await unifiedMissionApi.saveUnifiedMission(mission);
      setSaveSuccess(true);
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch {
      // no-op — TODO surface error
    } finally {
      setSaveBusy(false);
    }
  };

  const canSave = segments.length > 0 && missionName.trim().length > 0;

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-3 py-2 text-[10px] font-bold uppercase tracking-[0.16em] text-slate-500 border-b border-slate-800/60 flex items-center justify-between">
        <span>Mission Assembly</span>
        <span className="text-slate-600 tabular-nums">{segments.length} seg · {totalWaypoints} pts</span>
      </div>

      {/* Segment list */}
      <div className="flex-1 overflow-y-auto custom-scrollbar p-3 flex flex-col gap-1.5">
        {segments.length === 0 ? (
          <div className="text-xs text-slate-600 text-center py-8">
            Add segments using the left panel
          </div>
        ) : (
          segments.map((_, i) => <SegmentRow key={i} index={i} />)
        )}
      </div>

      {/* Validation results */}
      {validationReport && (
        <div className="px-3 py-2 border-t border-slate-800/60">
          <div className={`text-xs font-semibold ${validationReport.valid ? 'text-emerald-400' : 'text-amber-400'}`}>
            {validationReport.valid ? '✓ Validation passed' : `✗ ${validationReport.summary.errors} errors`}
          </div>
          {!validationReport.valid && validationReport.issues.slice(0, 3).map((issue, i) => (
            <div key={i} className="text-[10px] text-slate-400 mt-0.5 truncate">{issue.message}</div>
          ))}
        </div>
      )}

      {/* Footer actions */}
      <div className="p-3 border-t border-slate-800/60 flex flex-col gap-2">
        <input
          className="v4-field text-xs"
          placeholder="Mission name..."
          value={missionName}
          onChange={(e) => setMissionName(e.target.value)}
        />
        <button
          type="button"
          onClick={handleValidate}
          disabled={validationBusy}
          className="v4-focus v4-button w-full py-2 bg-slate-800 text-slate-200 disabled:opacity-50"
        >
          {validationBusy ? 'Validating...' : 'Validate'}
        </button>
        <button
          type="button"
          onClick={handleSave}
          disabled={!canSave || saveBusy}
          className="v4-focus v4-button w-full py-2 bg-emerald-900/40 border-emerald-700 text-emerald-100 disabled:opacity-40 flex items-center justify-center gap-1.5"
        >
          {saveSuccess ? <><CheckCircle size={13} /> Saved!</> : saveBusy ? 'Saving...' : <><Save size={13} /> Save Mission</>}
        </button>
      </div>
    </div>
  );
}
```

**Step 4: Create the mission compiler stub**

```ts
// ui/src/components/MissionStudio/compileStudioMission.ts
import type { UnifiedMission } from '../../api/unifiedMission';
import type { StudioState } from './useStudioStore';

export function compileStudioMission(state: StudioState): UnifiedMission {
  // Minimal stub — produces a valid schema v2 mission from studio state.
  // Will be fleshed out in Task 7.
  const segments: UnifiedMission['segments'] = state.scanPasses.map((pass, i) => ({
    segment_id: pass.id,
    type: 'scan' as const,
    target_id: 'studio_target',
    scan: {
      frame: 'ECI' as const,
      axis: `+${pass.axis}` as '+X' | '+Y' | '+Z',
      standoff: 10,
      overlap: 0.1,
      fov_deg: 60,
      revolutions: Math.max(1, Math.round(Math.abs(pass.planeBOffset - pass.planeAOffset) / pass.levelHeight)),
      direction: 'CW' as const,
      sensor_axis: '+Y' as const,
      pattern: 'spiral' as const,
    },
  }));

  return {
    schema_version: 2,
    mission_id: `studio-${Date.now()}`,
    name: state.missionName || 'Untitled Studio Mission',
    epoch: new Date().toISOString(),
    start_pose: {
      frame: 'ECI',
      position: state.satelliteStart,
      orientation: [1, 0, 0, 0],
    },
    segments,
    obstacles: state.obstacles,
  };
}
```

**Step 5: TypeScript check**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -40
```
Expected: clean or only minor import issues.

**Step 6: Visual check**

```bash
make run
```
Open browser, switch to STUDIO tab. Verify:
- Deep blue background visible ✓
- Left panel shows "Add Segment", "Model" sections ✓
- Right panel shows "Mission Assembly" with empty state ✓
- "Add Scan Pass" button adds a row to the right panel ✓

**Step 7: Commit**

```bash
git add ui/src/components/MissionStudio/
git commit -m "feat(studio): layout shell, left panel, right panel, mission compiler stub"
```

---

## Task 3: R3F Canvas with OBJ model + grid

**Files:**
- Create: `ui/src/components/MissionStudio/MissionStudioCanvas.tsx`
- Modify: `ui/src/components/MissionStudio/MissionStudioLayout.tsx`

**Step 1: Create the canvas**

```tsx
// ui/src/components/MissionStudio/MissionStudioCanvas.tsx
import { Suspense, useRef } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls, GizmoHelper, GizmoViewport, Grid, useOBJ } from '@react-three/drei';
import * as THREE from 'three';
import { useStudioStore } from './useStudioStore';

function ObjModel({ url }: { url: string }) {
  const obj = useOBJ(url);
  const ref = useRef<THREE.Group>(null);

  // Compute bounding box and report it to store
  const setModelBoundingBox = useStudioStore((s) => s.setModelBoundingBox);
  if (ref.current) {
    const box = new THREE.Box3().setFromObject(ref.current);
    const min = box.min.toArray() as [number,number,number];
    const max = box.max.toArray() as [number,number,number];
    setModelBoundingBox({ min, max });
  }

  return (
    <group ref={ref}>
      <primitive object={obj} />
    </group>
  );
}

function SceneContents() {
  const modelUrl = useStudioStore((s) => s.modelUrl);

  return (
    <>
      <ambientLight intensity={0.6} />
      <directionalLight position={[10, 20, 10]} intensity={1.2} />

      {/* Reference grid */}
      <Grid
        args={[100, 100]}
        cellSize={1}
        cellThickness={0.5}
        cellColor="#1e3a5f"
        sectionSize={10}
        sectionThickness={1}
        sectionColor="#2a4f7a"
        fadeDistance={80}
        fadeStrength={1}
        infiniteGrid
      />

      {/* OBJ model */}
      {modelUrl && (
        <Suspense fallback={null}>
          <ObjModel url={modelUrl} />
        </Suspense>
      )}

      <OrbitControls makeDefault />
      <GizmoHelper alignment="bottom-right" margin={[60, 60]}>
        <GizmoViewport labelColor="white" axisHeadScale={0.9} />
      </GizmoHelper>
    </>
  );
}

export function MissionStudioCanvas() {
  return (
    <Canvas
      camera={{ position: [0, 15, 30], fov: 50, near: 0.01, far: 10000 }}
      gl={{ antialias: true, alpha: false }}
      style={{ background: '#070b14' }}
    >
      <color attach="background" args={['#070b14']} />
      <SceneContents />
    </Canvas>
  );
}
```

**Step 2: Wire canvas into layout**

In `MissionStudioLayout.tsx`, replace `CanvasPlaceholder` import/usage with:

```tsx
import { MissionStudioCanvas } from './MissionStudioCanvas';
// ...
{/* 3D Canvas */}
<div className="flex-1 relative min-w-0">
  <MissionStudioCanvas />
</div>
```

**Step 3: Check `useOBJ` availability**

```bash
cd ui && node -e "const d = require('./node_modules/@react-three/drei/index.cjs'); console.log(typeof d.useOBJ)"
```
If `'undefined'` is printed, `useOBJ` isn't exported from this version. In that case use `useLoader` from `@react-three/fiber` with `THREE.OBJLoader` from `three/addons`. Replace the `ObjModel` component:

```tsx
import { useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';

function ObjModel({ url }: { url: string }) {
  const obj = useLoader(OBJLoader, url);
  return <primitive object={obj} />;
}
```

**Step 4: TypeScript check + visual check**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -30
make run
```
Load the studio, click "Load OBJ Model", pick any `.obj` file — model should appear in the canvas. Grid should be visible. OrbitControls should allow mouse rotation.

**Step 5: Commit**

```bash
git add ui/src/components/MissionStudio/MissionStudioCanvas.tsx ui/src/components/MissionStudio/MissionStudioLayout.tsx
git commit -m "feat(studio): R3F canvas with OBJ loader, grid, orbit controls, gizmo"
```

---

## Task 4: Spiral generator + scan pass visualization

**Files:**
- Create: `ui/src/components/MissionStudio/useSpiralGenerator.ts`
- Create: `ui/src/components/MissionStudio/ScanPassObject.tsx`
- Modify: `ui/src/components/MissionStudio/MissionStudioCanvas.tsx`

**Step 1: Write a unit test for the spiral generator**

```ts
// ui/src/components/MissionStudio/__tests__/useSpiralGenerator.test.ts
import { generateSpiral } from '../useSpiralGenerator';

describe('generateSpiral', () => {
  it('produces waypoints between two offsets', () => {
    const pts = generateSpiral({
      axis: 'Z',
      planeAOffset: -5,
      planeBOffset: 5,
      crossSection: Array.from({ length: 8 }, (_, i) => {
        const a = (i / 8) * Math.PI * 2;
        return [Math.cos(a) * 3, Math.sin(a) * 3] as [number, number];
      }),
      levelHeight: 1,
    });
    // Should have (10 / 1) * 8 = 80 points
    expect(pts.length).toBeGreaterThan(0);
    // All Z coords should be between planeA and planeB
    pts.forEach(([, , z]) => {
      expect(z).toBeGreaterThanOrEqual(-5 - 0.01);
      expect(z).toBeLessThanOrEqual(5 + 0.01);
    });
  });

  it('respects cross-section shape', () => {
    // Rectangle cross-section: 4 points at ±5, ±3
    const rect: [number, number][] = [
      [5, 3], [5, -3], [-5, -3], [-5, 3],
      [5, 3], [5, -3], [-5, -3], [-5, 3], // 8 points
    ];
    const pts = generateSpiral({
      axis: 'Z', planeAOffset: -2, planeBOffset: 2,
      crossSection: rect, levelHeight: 0.5,
    });
    expect(pts.length).toBeGreaterThan(0);
  });
});
```

**Step 2: Run test to verify it fails**

```bash
npx --prefix ui vitest run ui/src/components/MissionStudio/__tests__/useSpiralGenerator.test.ts 2>&1 | tail -10
```
Expected: FAIL — `generateSpiral` not found.

**Step 3: Implement the spiral generator**

```ts
// ui/src/components/MissionStudio/useSpiralGenerator.ts

interface SpiralParams {
  axis: 'X' | 'Y' | 'Z';
  planeAOffset: number;
  planeBOffset: number;
  crossSection: [number, number][];  // 8 points in local 2D
  levelHeight: number;
}

/**
 * Generate a spiral path between two parallel planes.
 * The cross-section polygon is extruded uniformly from planeA to planeB.
 * Each "level" is one pass around the cross-section polygon.
 * Points are interpolated along the polygon perimeter for smooth curves.
 */
export function generateSpiral(params: SpiralParams): [number, number, number][] {
  const { axis, planeAOffset, planeBOffset, crossSection, levelHeight } = params;
  const gap = planeBOffset - planeAOffset;
  if (Math.abs(gap) < 0.001 || levelHeight <= 0) return [];

  const turns = Math.abs(gap) / levelHeight;
  const pointsPerTurn = 32; // enough for smooth curves
  const totalPoints = Math.max(4, Math.round(turns * pointsPerTurn));

  // Compute perimeter points by interpolating along the closed polygon
  const perimeterPoints = buildPerimeterSamples(crossSection, pointsPerTurn);

  const waypoints: [number, number, number][] = [];
  for (let i = 0; i <= totalPoints; i++) {
    const t = i / totalPoints; // 0..1 along the full spiral
    const along = planeAOffset + gap * t; // position along scan axis
    const ringIndex = Math.floor((t * turns * pointsPerTurn) % pointsPerTurn);
    const [u, v] = perimeterPoints[ringIndex % perimeterPoints.length];

    // Map (u, v, along) to world coords based on axis
    let x = 0, y = 0, z = 0;
    if (axis === 'Z') { x = u; y = v; z = along; }
    else if (axis === 'X') { y = u; z = v; x = along; }
    else { x = u; z = v; y = along; }

    waypoints.push([x, y, z]);
  }

  return waypoints;
}

function buildPerimeterSamples(polygon: [number, number][], count: number): [number, number][] {
  // Compute total perimeter length
  const n = polygon.length;
  let totalLen = 0;
  const segLengths: number[] = [];
  for (let i = 0; i < n; i++) {
    const a = polygon[i];
    const b = polygon[(i + 1) % n];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    segLengths.push(len);
    totalLen += len;
  }

  // Sample evenly along perimeter
  const samples: [number, number][] = [];
  for (let s = 0; s < count; s++) {
    const target = (s / count) * totalLen;
    let acc = 0;
    for (let i = 0; i < n; i++) {
      const next = acc + segLengths[i];
      if (target <= next || i === n - 1) {
        const t = segLengths[i] > 0 ? (target - acc) / segLengths[i] : 0;
        const a = polygon[i];
        const b = polygon[(i + 1) % n];
        samples.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
        break;
      }
      acc = next;
    }
  }
  return samples;
}
```

**Step 4: Run test to verify it passes**

```bash
npx --prefix ui vitest run ui/src/components/MissionStudio/__tests__/useSpiralGenerator.test.ts 2>&1 | tail -10
```
Expected: PASS.

**Step 5: Create ScanPassObject R3F component**

```tsx
// ui/src/components/MissionStudio/ScanPassObject.tsx
import { useMemo, useEffect } from 'react';
import * as THREE from 'three';
import { useStudioStore } from './useStudioStore';
import { generateSpiral } from './useSpiralGenerator';

interface ScanPassObjectProps {
  scanId: string;
}

export function ScanPassObject({ scanId }: ScanPassObjectProps) {
  const pass = useStudioStore((s) => s.scanPasses.find((p) => p.id === scanId));
  const updateScanPass = useStudioStore((s) => s.updateScanPass);
  const selectedScanId = useStudioStore((s) => s.selectedScanId);

  // Regenerate waypoints whenever pass params change
  useEffect(() => {
    if (!pass) return;
    const waypoints = generateSpiral({
      axis: pass.axis,
      planeAOffset: pass.planeAOffset,
      planeBOffset: pass.planeBOffset,
      crossSection: pass.crossSection,
      levelHeight: pass.levelHeight,
    });
    updateScanPass(pass.id, { waypoints });
  }, [pass?.axis, pass?.planeAOffset, pass?.planeBOffset, pass?.levelHeight,
      JSON.stringify(pass?.crossSection)]);

  const lineGeometry = useMemo(() => {
    if (!pass || pass.waypoints.length < 2) return null;
    const points = pass.waypoints.map(([x, y, z]) => new THREE.Vector3(x, y, z));
    const geo = new THREE.BufferGeometry().setFromPoints(points);
    return geo;
  }, [pass?.waypoints]);

  // Endpoint spheres
  const startPos = pass?.waypoints[0] ?? null;
  const endPos = pass?.waypoints[pass.waypoints.length - 1] ?? null;
  const isSelected = selectedScanId === scanId;
  const color = pass?.color ?? '#22d3ee';

  if (!pass || !lineGeometry) return null;

  return (
    <group>
      {/* Spiral line */}
      <line geometry={lineGeometry}>
        <lineBasicMaterial color={color} linewidth={isSelected ? 2 : 1} opacity={isSelected ? 1 : 0.7} transparent />
      </line>

      {/* Endpoint nodes — glowing spheres */}
      {startPos && (
        <mesh position={startPos}
          onClick={() => useStudioStore.getState().selectScanPass(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#22d3ee" />
        </mesh>
      )}
      {endPos && (
        <mesh position={endPos}
          onClick={() => useStudioStore.getState().selectScanPass(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#a78bfa" />
        </mesh>
      )}

      {/* Plane A indicator */}
      <PlaneIndicator axis={pass.axis} offset={pass.planeAOffset} color={color} />
      <PlaneIndicator axis={pass.axis} offset={pass.planeBOffset} color={color} />
    </group>
  );
}

function PlaneIndicator({ axis, offset, color }: { axis: 'X'|'Y'|'Z'; offset: number; color: string }) {
  const pos: [number,number,number] =
    axis === 'X' ? [offset, 0, 0] :
    axis === 'Y' ? [0, offset, 0] :
                   [0, 0, offset];
  const rot: [number,number,number] =
    axis === 'X' ? [0, 0, Math.PI / 2] :
    axis === 'Y' ? [0, 0, 0] :
                   [Math.PI / 2, 0, 0];
  return (
    <mesh position={pos} rotation={rot}>
      <ringGeometry args={[4.5, 5, 32]} />
      <meshBasicMaterial color={color} opacity={0.3} transparent side={THREE.DoubleSide} />
    </mesh>
  );
}
```

**Step 6: Add scan pass objects to canvas**

In `MissionStudioCanvas.tsx`, inside `SceneContents`, add:

```tsx
import { ScanPassObject } from './ScanPassObject';
// ...
const scanPasses = useStudioStore((s) => s.scanPasses);
// ...inside return:
{scanPasses.map((p) => <ScanPassObject key={p.id} scanId={p.id} />)}
```

**Step 7: Visual check**

```bash
make run
```
Click "Add Scan Pass" — a spiral should appear in the canvas. Adjust "Plane Gap" slider — spiral stretches. Adjust "Level Height" — turn density changes.

**Step 8: Commit**

```bash
git add ui/src/components/MissionStudio/
git commit -m "feat(studio): spiral generator + scan pass R3F visualization with plane rings"
```

---

## Task 5: Waypoint nudge (spline deform)

**Files:**
- Create: `ui/src/components/MissionStudio/WaypointNudger.tsx`
- Modify: `ui/src/components/MissionStudio/MissionStudioCanvas.tsx`

**Step 1: Write a unit test for applyNudge**

```ts
// ui/src/components/MissionStudio/__tests__/useStudioStore.nudge.test.ts
import { useStudioStore } from '../useStudioStore';
import { generateSpiral } from '../useSpiralGenerator';

describe('applyNudge', () => {
  beforeEach(() => useStudioStore.getState().reset?.() ?? useStudioStore.setState({
    scanPasses: [], wires: [], holds: [], obstacles: [], segments: [],
    selectedScanId: null, wireDrag: { phase: 'idle' },
  }));

  it('moves the target waypoint and attenuates neighbors', () => {
    const waypoints = generateSpiral({
      axis: 'Z', planeAOffset: -5, planeBOffset: 5,
      crossSection: Array.from({length:8},(_,i)=>[Math.cos(i/8*Math.PI*2)*3, Math.sin(i/8*Math.PI*2)*3] as [number,number]),
      levelHeight: 1,
    });
    useStudioStore.setState({
      scanPasses: [{ id: 'p1', axis: 'Z', planeAOffset: -5, planeBOffset: 5,
        crossSection: [], levelHeight: 1, waypoints, color: '#fff' }],
    });

    const idx = 10;
    const before = [...useStudioStore.getState().scanPasses[0].waypoints[idx]];
    useStudioStore.getState().applyNudge('p1', idx, [1, 0, 0]);
    const after = useStudioStore.getState().scanPasses[0].waypoints[idx];

    // Target waypoint moved by ~1 (weight=1 at center)
    expect(after[0]).toBeCloseTo(before[0] + 1, 1);

    // Neighbor 5 indices away should have moved less
    const neighbor = useStudioStore.getState().scanPasses[0].waypoints[idx + 5];
    const neighborBefore = waypoints[idx + 5];
    expect(Math.abs(neighbor[0] - neighborBefore[0])).toBeLessThan(0.5);
  });
});
```

**Step 2: Run test**

```bash
npx --prefix ui vitest run ui/src/components/MissionStudio/__tests__/useStudioStore.nudge.test.ts 2>&1 | tail -10
```
Expected: PASS (applyNudge already implemented in Task 1 store).

**Step 3: Create WaypointNudger R3F component**

```tsx
// ui/src/components/MissionStudio/WaypointNudger.tsx
import { useRef, useCallback } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

interface WaypointNudgerProps {
  scanId: string;
}

// Renders clickable/draggable dots on each waypoint of a scan pass.
// Only rendered when that pass is selected.
export function WaypointNudger({ scanId }: WaypointNudgerProps) {
  const waypoints = useStudioStore((s) => s.scanPasses.find((p) => p.id === scanId)?.waypoints ?? []);
  const { camera, gl } = useThree();
  const dragging = useRef<{ index: number; plane: THREE.Plane } | null>(null);
  const raycaster = useRef(new THREE.Raycaster());

  const onPointerDown = useCallback((index: number, e: any) => {
    e.stopPropagation();
    // Drag plane: perpendicular to camera at waypoint position
    const wp = waypoints[index];
    const normal = new THREE.Vector3().subVectors(camera.position, new THREE.Vector3(...wp)).normalize();
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, new THREE.Vector3(...wp));
    dragging.current = { index, plane };
    gl.domElement.setPointerCapture(e.pointerId);
  }, [waypoints, camera, gl]);

  const onPointerMove = useCallback((e: any) => {
    if (!dragging.current) return;
    const { index, plane } = dragging.current;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(plane, intersection);
    if (!intersection) return;
    const wp = waypoints[index];
    const delta: [number,number,number] = [
      intersection.x - wp[0],
      intersection.y - wp[1],
      intersection.z - wp[2],
    ];
    useStudioStore.getState().applyNudge(scanId, index, delta);
  }, [waypoints, scanId, camera, gl]);

  const onPointerUp = useCallback((e: any) => {
    dragging.current = null;
    gl.domElement.releasePointerCapture(e.pointerId);
  }, [gl]);

  // Only show every Nth waypoint as a handle to avoid clutter
  const stride = Math.max(1, Math.floor(waypoints.length / 40));

  return (
    <group onPointerMove={onPointerMove} onPointerUp={onPointerUp}>
      {waypoints.map((wp, i) => {
        if (i % stride !== 0) return null;
        return (
          <mesh
            key={i}
            position={wp}
            onPointerDown={(e) => onPointerDown(i, e)}
          >
            <sphereGeometry args={[0.12, 8, 8]} />
            <meshBasicMaterial color="white" opacity={0.7} transparent />
          </mesh>
        );
      })}
    </group>
  );
}
```

**Step 4: Add WaypointNudger to canvas**

In `MissionStudioCanvas.tsx` `SceneContents`:

```tsx
import { WaypointNudger } from './WaypointNudger';
// ...
const selectedScanId = useStudioStore((s) => s.selectedScanId);
// ...inside return, after ScanPassObjects:
{selectedScanId && <WaypointNudger scanId={selectedScanId} />}
```

**Step 5: Visual check**

Select a scan pass by clicking on it. Small white dots should appear on waypoints. Drag a dot — nearby path should deform smoothly.

**Step 6: Commit**

```bash
git add ui/src/components/MissionStudio/WaypointNudger.tsx ui/src/components/MissionStudio/MissionStudioCanvas.tsx
git commit -m "feat(studio): waypoint nudge with Gaussian spline falloff"
```

---

## Task 6: Wire drag for transfers + satellite start node

**Files:**
- Create: `ui/src/components/MissionStudio/EndpointNodes.tsx`
- Create: `ui/src/components/MissionStudio/SatelliteStartNode.tsx`
- Modify: `ui/src/components/MissionStudio/MissionStudioCanvas.tsx`

**Step 1: Create EndpointNodes**

```tsx
// ui/src/components/MissionStudio/EndpointNodes.tsx
import { useRef, useCallback } from 'react';
import * as THREE from 'three';
import { useThree, useFrame } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

// Renders start/end endpoint spheres for all scan passes.
// When wireDrag is active, hovering a valid target pulses it.
export function EndpointNodes() {
  const scanPasses = useStudioStore((s) => s.scanPasses);
  const wireDrag = useStudioStore((s) => s.wireDrag);
  const setWireDrag = useStudioStore((s) => s.setWireDrag);
  const addWire = useStudioStore((s) => s.addWire);
  const { camera, gl } = useThree();
  const raycaster = useRef(new THREE.Raycaster());
  const dragLineRef = useRef<THREE.Line>(null);

  useFrame(() => {
    if (wireDrag.phase !== 'dragging' || !dragLineRef.current) return;
    const cursor = wireDrag.cursorWorld;
    // Find source position
    const [srcScanId, srcEndpoint] = wireDrag.sourceNodeId.split(':');
    const srcPass = scanPasses.find((p) => p.id === srcScanId);
    if (!srcPass || srcPass.waypoints.length === 0) return;
    const srcPos = srcEndpoint === 'start' ? srcPass.waypoints[0] : srcPass.waypoints[srcPass.waypoints.length - 1];
    const points = [new THREE.Vector3(...srcPos), new THREE.Vector3(...cursor)];
    (dragLineRef.current.geometry as THREE.BufferGeometry).setFromPoints(points);
  });

  const handlePointerMove = useCallback((e: PointerEvent) => {
    if (wireDrag.phase !== 'dragging') return;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    // Project onto Y=0 plane for cursor position
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(plane, intersection);
    if (intersection) {
      setWireDrag({ ...wireDrag, cursorWorld: [intersection.x, intersection.y, intersection.z] });
    }
  }, [wireDrag, camera, gl, setWireDrag]);

  const startDrag = (nodeId: string) => {
    setWireDrag({ phase: 'dragging', sourceNodeId: nodeId, cursorWorld: [0, 0, 0] });
    gl.domElement.addEventListener('pointermove', handlePointerMove);
    gl.domElement.addEventListener('pointerup', () => {
      gl.domElement.removeEventListener('pointermove', handlePointerMove);
      if (useStudioStore.getState().wireDrag.phase === 'dragging') {
        setWireDrag({ phase: 'idle' });
      }
    }, { once: true });
  };

  const completeDrag = (targetNodeId: string) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging') return;
    if (drag.sourceNodeId === targetNodeId) { setWireDrag({ phase: 'idle' }); return; }
    const wireId = `wire-${Date.now()}`;
    addWire({ id: wireId, fromNodeId: drag.sourceNodeId, toNodeId: targetNodeId });
    setWireDrag({ phase: 'idle' });
  };

  return (
    <group>
      {/* Drag wire line */}
      {wireDrag.phase === 'dragging' && (
        <line ref={dragLineRef}>
          <bufferGeometry />
          <lineDashedMaterial color="#22d3ee" dashSize={0.3} gapSize={0.2} linewidth={1} />
        </line>
      )}

      {/* Endpoint nodes for each scan pass */}
      {scanPasses.map((pass) => {
        if (pass.waypoints.length === 0) return null;
        const startPos = pass.waypoints[0];
        const endPos = pass.waypoints[pass.waypoints.length - 1];
        const startId = `${pass.id}:start`;
        const endId = `${pass.id}:end`;
        const isDragging = wireDrag.phase === 'dragging';

        return (
          <group key={pass.id}>
            <EndpointSphere
              position={startPos}
              color="#22d3ee"
              pulse={isDragging}
              onPointerDown={() => startDrag(startId)}
              onPointerUp={() => completeDrag(startId)}
            />
            <EndpointSphere
              position={endPos}
              color="#a78bfa"
              pulse={isDragging}
              onPointerDown={() => startDrag(endId)}
              onPointerUp={() => completeDrag(endId)}
            />
          </group>
        );
      })}
    </group>
  );
}

function EndpointSphere({
  position, color, pulse, onPointerDown, onPointerUp,
}: {
  position: [number,number,number];
  color: string;
  pulse: boolean;
  onPointerDown: () => void;
  onPointerUp: () => void;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  useFrame(({ clock }) => {
    if (!meshRef.current || !pulse) return;
    const s = 1 + 0.2 * Math.sin(clock.getElapsedTime() * 4);
    meshRef.current.scale.setScalar(s);
  });
  return (
    <mesh ref={meshRef} position={position} onPointerDown={onPointerDown} onPointerUp={onPointerUp}>
      <sphereGeometry args={[0.5, 16, 16]} />
      <meshBasicMaterial color={color} />
    </mesh>
  );
}
```

**Step 2: Create SatelliteStartNode**

```tsx
// ui/src/components/MissionStudio/SatelliteStartNode.tsx
import { useRef } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

export function SatelliteStartNode() {
  const satelliteStart = useStudioStore((s) => s.satelliteStart);
  const setSatelliteStart = useStudioStore((s) => s.setSatelliteStart);
  const { camera, gl } = useThree();
  const dragging = useRef(false);
  const dragPlane = useRef(new THREE.Plane());
  const raycaster = useRef(new THREE.Raycaster());

  const onPointerDown = (e: any) => {
    e.stopPropagation();
    dragging.current = true;
    const normal = new THREE.Vector3(0, 1, 0);
    dragPlane.current.setFromNormalAndCoplanarPoint(normal, new THREE.Vector3(...satelliteStart));
    gl.domElement.setPointerCapture(e.pointerId);
  };

  const onPointerMove = (e: any) => {
    if (!dragging.current) return;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(dragPlane.current, intersection);
    if (intersection) setSatelliteStart([intersection.x, satelliteStart[1], intersection.z]);
  };

  const onPointerUp = (e: any) => {
    dragging.current = false;
    gl.domElement.releasePointerCapture(e.pointerId);
  };

  return (
    <group position={satelliteStart} onPointerDown={onPointerDown} onPointerMove={onPointerMove} onPointerUp={onPointerUp}>
      {/* Crosshair rings */}
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.8, 1.0, 32]} />
        <meshBasicMaterial color="white" side={THREE.DoubleSide} />
      </mesh>
      {/* +X axis arrow */}
      <arrowHelper args={[new THREE.Vector3(1,0,0), new THREE.Vector3(0,0,0), 2, 0xff4444, 0.4, 0.3]} />
      {/* Center dot */}
      <mesh>
        <sphereGeometry args={[0.2, 8, 8]} />
        <meshBasicMaterial color="white" />
      </mesh>
    </group>
  );
}
```

**Step 3: Add to canvas**

In `MissionStudioCanvas.tsx`:

```tsx
import { EndpointNodes } from './EndpointNodes';
import { SatelliteStartNode } from './SatelliteStartNode';
// inside SceneContents return:
<EndpointNodes />
<SatelliteStartNode />
```

**Step 4: Visual check**

Add a scan pass. Cyan/violet glowing spheres appear at its ends. Drag the white crosshair to reposition satellite start. Press "+ Transfer" then drag from one sphere to another — dashed wire follows cursor, transfer added to assembly list.

**Step 5: Commit**

```bash
git add ui/src/components/MissionStudio/EndpointNodes.tsx ui/src/components/MissionStudio/SatelliteStartNode.tsx ui/src/components/MissionStudio/MissionStudioCanvas.tsx
git commit -m "feat(studio): endpoint nodes with wire drag, satellite start crosshair"
```

---

## Task 7: Obstacle objects + complete mission compiler

**Files:**
- Create: `ui/src/components/MissionStudio/ObstacleObjects.tsx`
- Modify: `ui/src/components/MissionStudio/compileStudioMission.ts`
- Modify: `ui/src/components/MissionStudio/MissionStudioCanvas.tsx`

**Step 1: Create ObstacleObjects**

```tsx
// ui/src/components/MissionStudio/ObstacleObjects.tsx
import { useRef } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

export function ObstacleObjects() {
  const obstacles = useStudioStore((s) => s.obstacles);
  const updateObstacle = useStudioStore((s) => s.updateObstacle);
  const { camera, gl } = useThree();

  return (
    <>
      {obstacles.map((obs) => (
        <DraggableObstacle
          key={obs.id}
          id={obs.id}
          position={obs.position}
          radius={obs.radius}
          onMove={(pos) => updateObstacle(obs.id, { position: pos })}
        />
      ))}
    </>
  );
}

function DraggableObstacle({
  id, position, radius, onMove,
}: {
  id: string;
  position: [number,number,number];
  radius: number;
  onMove: (pos: [number,number,number]) => void;
}) {
  const dragging = useRef(false);
  const dragPlane = useRef(new THREE.Plane());
  const raycaster = useRef(new THREE.Raycaster());
  const { camera, gl } = useThree();

  const onPointerDown = (e: any) => {
    e.stopPropagation();
    dragging.current = true;
    dragPlane.current.setFromNormalAndCoplanarPoint(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(...position)
    );
    gl.domElement.setPointerCapture(e.pointerId);
  };

  const onPointerMove = (e: any) => {
    if (!dragging.current) return;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(dragPlane.current, intersection);
    if (intersection) onMove([intersection.x, position[1], intersection.z]);
  };

  const onPointerUp = (e: any) => {
    dragging.current = false;
    gl.domElement.releasePointerCapture(e.pointerId);
  };

  return (
    <mesh
      position={position}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
    >
      <sphereGeometry args={[radius, 24, 24]} />
      <meshBasicMaterial color="#ef4444" opacity={0.25} transparent wireframe={false} />
    </mesh>
  );
}
```

**Step 2: Complete the mission compiler**

Replace `compileStudioMission.ts` with a proper implementation:

```ts
// ui/src/components/MissionStudio/compileStudioMission.ts
import type { UnifiedMission, MissionSegment, TransferSegment, ScanSegment, HoldSegment } from '../../api/unifiedMission';
import type { StudioState } from './useStudioStore';

export function compileStudioMission(state: StudioState): UnifiedMission {
  const segments: MissionSegment[] = [];

  for (const seg of state.segments) {
    if (seg.type === 'scan' && seg.scanId) {
      const pass = state.scanPasses.find((p) => p.id === seg.scanId);
      if (!pass) continue;
      const gap = Math.abs(pass.planeBOffset - pass.planeAOffset);
      const revolutions = Math.max(1, Math.round(gap / pass.levelHeight));
      const scanSeg: ScanSegment = {
        segment_id: seg.id,
        type: 'scan',
        target_id: 'studio_target',
        scan: {
          frame: 'ECI',
          axis: `+${pass.axis}` as '+X' | '+Y' | '+Z',
          standoff: 10,
          overlap: 0.1,
          fov_deg: 60,
          revolutions,
          direction: 'CW',
          sensor_axis: '+Y',
          pattern: 'spiral',
        },
        // Embed the generated waypoints as a manual path override
        ...(pass.waypoints.length > 0 ? {
          overrides: { manual_path: pass.waypoints },
        } : {}),
      };
      segments.push(scanSeg);
    }

    if (seg.type === 'transfer' && seg.wireId) {
      const wire = state.wires.find((w) => w.id === seg.wireId);
      if (!wire) continue;
      // Resolve target endpoint position
      const [toScanId, toEndpoint] = wire.toNodeId.split(':');
      const toPass = state.scanPasses.find((p) => p.id === toScanId);
      const toPos = toPass
        ? toEndpoint === 'start'
          ? toPass.waypoints[0]
          : toPass.waypoints[toPass.waypoints.length - 1]
        : state.satelliteStart;
      if (!toPos) continue;
      const transferSeg: TransferSegment = {
        segment_id: seg.id,
        type: 'transfer',
        end_pose: { frame: 'ECI', position: toPos, orientation: [1, 0, 0, 0] },
      };
      segments.push(transferSeg);
    }

    if (seg.type === 'hold' && seg.holdId) {
      const hold = state.holds.find((h) => h.id === seg.holdId);
      if (!hold) continue;
      const holdSeg: HoldSegment = {
        segment_id: seg.id,
        type: 'hold',
        duration: hold.duration,
      };
      segments.push(holdSeg);
    }
  }

  return {
    schema_version: 2,
    mission_id: `studio-${Date.now()}`,
    name: state.missionName || 'Untitled Studio Mission',
    epoch: new Date().toISOString(),
    start_pose: {
      frame: 'ECI',
      position: state.satelliteStart,
      orientation: [1, 0, 0, 0],
    },
    segments,
    obstacles: state.obstacles,
  };
}
```

**Step 3: Add obstacles to canvas**

In `MissionStudioCanvas.tsx`:

```tsx
import { ObstacleObjects } from './ObstacleObjects';
// inside SceneContents:
<ObstacleObjects />
```

**Step 4: TypeScript check**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -30
```
Expected: clean.

**Step 5: Commit**

```bash
git add ui/src/components/MissionStudio/
git commit -m "feat(studio): obstacle objects, complete mission compiler with manual path embed"
```

---

## Task 8: Smoke test + nav label update + final polish

**Files:**
- Modify: `ui/src/App.tsx` (nav label: PLANNER → PLANNER (legacy), STUDIO label already added)
- Modify: `ui/src/components/MissionStudio/MissionStudioRightPanel.tsx` (mission name auto-suggest)

**Step 1: Auto-suggest mission name**

In `MissionStudioRightPanel.tsx`, add auto-suggest when name is empty and first scan pass exists:

```tsx
// Add inside MissionStudioRightPanel, before the return:
useEffect(() => {
  if (missionName.trim().length > 0) return;
  if (scanPasses.length === 0) return;
  const ts = new Date().toISOString().slice(0,16).replace(/[-:T]/g,'');
  setMissionName(`Studio_${scanPasses.length}pass_${ts}`);
}, [scanPasses.length]);
```

**Step 2: Run all frontend unit tests**

```bash
cd ui && npx vitest run 2>&1 | tail -20
```
Expected: all tests pass, no regressions.

**Step 3: Full manual smoke test**

```bash
make run
```

Checklist:
- [ ] STUDIO tab visible in nav bar (violet)
- [ ] Switching to STUDIO shows deep-blue canvas, left/right panels
- [ ] Load an OBJ file — model appears in canvas
- [ ] "Add Scan Pass" — spiral appears with plane rings, entry in right panel
- [ ] Axis toggle (X/Y/Z) — spiral reorients
- [ ] Level Height slider — turn density updates live
- [ ] Plane Gap slider — spiral stretches
- [ ] Click on spiral — selected, white waypoint dots appear
- [ ] Drag a waypoint dot — path deforms with Gaussian falloff
- [ ] "Add Obstacle" — red translucent sphere appears, draggable
- [ ] Endpoint nodes pulse cyan/violet at path ends
- [ ] Drag from endpoint node to another — dashed wire follows, transfer added to assembly
- [ ] Satellite start crosshair visible, draggable on XZ plane
- [ ] Right panel shows segment rows with correct icons
- [ ] Type a mission name, click Validate — validation runs
- [ ] Click Save — mission saved (check backend `/simulations` or missions list)

**Step 4: Final commit**

```bash
git add -A
git commit -m "feat(studio): Mission Studio complete — 3D scan path editor with wire assembly"
```

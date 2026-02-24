# Viewer HUD Redesign Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Redesign the Viewer HUD into a split-panel layout (TELEMETRY top-left, CONTROLLER+ACTUATORS top-right, collapsible charts drawer at bottom) while fixing per-frame re-render performance by giving each panel its own telemetry subscription.

**Architecture:** Split the monolithic `Overlay` component into `TelemetryPanel` and `ControllerActuatorPanel`, each subscribing independently to the `telemetry` service so only the changed panel re-renders. Wrap `TelemetryCharts` in a `CollapsibleChartsDrawer` defaulting to collapsed. Polish `HudPanel` with a `live` pulsing indicator and add mode badges + solver traffic-light to the controller panel.

**Tech Stack:** React 19, TypeScript, Tailwind CSS v3, Zustand, Lucide React icons, the existing `telemetry` service singleton (`ui/src/services/telemetry.ts`).

---

### Task 1: Add `live` prop to `HudPanel` and polish corner accents

**Files:**
- Modify: `ui/src/components/HudComponents.tsx`

**Step 1: Read the current file**

Open `ui/src/components/HudComponents.tsx` and confirm it exports `HudPanel` with props `children`, `className`, `title`.

**Step 2: Implement the changes**

Replace the entire file content with:

```tsx
import React from 'react';

/**
 * A glassmorphism panel container with a sci-fi border.
 * Pass `live={true}` to pulse the title indicator dot while data is streaming.
 */
export function HudPanel({
  children,
  className = '',
  title,
  live = false,
}: {
  children: React.ReactNode;
  className?: string;
  title?: React.ReactNode;
  live?: boolean;
}) {
  return (
    <div className={`
      relative overflow-hidden
      bg-slate-950/80 backdrop-blur-md
      border border-slate-700/50 rounded-lg
      shadow-xl
      ${className}
    `}>
      {/* Sci-Fi Decorative Corners */}
      <div className="absolute top-0 left-0 w-3 h-3 border-t-2 border-l-2 border-cyan-500/70 rounded-tl-sm pointer-events-none" />
      <div className="absolute top-0 right-0 w-3 h-3 border-t-2 border-r-2 border-cyan-500/70 rounded-tr-sm pointer-events-none" />
      <div className="absolute bottom-0 left-0 w-3 h-3 border-b-2 border-l-2 border-cyan-500/70 rounded-bl-sm pointer-events-none" />
      <div className="absolute bottom-0 right-0 w-3 h-3 border-b-2 border-r-2 border-cyan-500/70 rounded-br-sm pointer-events-none" />

      {title && (
        <div className="px-3 py-2 border-b border-slate-800/50 bg-slate-900/50 flex items-center gap-2">
          <div className={`w-1.5 h-3 bg-cyan-500 rounded-full shadow-[0_0_5px_rgba(6,182,212,0.8)] ${live ? 'animate-pulse' : ''}`} />
          <span className="text-xs font-bold uppercase tracking-wider text-cyan-400">
            {title}
          </span>
        </div>
      )}
      <div className="p-3">
        {children}
      </div>
    </div>
  );
}
```

**Step 3: Verify no TypeScript errors**

Run from repo root:
```bash
cd ui && npx tsc --noEmit 2>&1 | head -30
```
Expected: no errors mentioning `HudComponents`.

**Step 4: Commit**

```bash
git add ui/src/components/HudComponents.tsx
git commit -m "feat(hud): add live pulse prop to HudPanel, enlarge corner accents"
```

---

### Task 2: Create `TelemetryPanel` component

**Files:**
- Create: `ui/src/components/TelemetryPanel.tsx`

**Step 1: Create the file**

```tsx
import { useEffect, useState } from 'react';
import { Quaternion, Euler } from 'three';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { HudPanel } from './HudComponents';

function DataRow({
  label,
  values,
  unit,
  colorClass = 'text-slate-200',
}: {
  label: string;
  values: number[] | Float32Array;
  unit: string;
  colorClass?: string;
}) {
  const fmt = (n: number) => (n >= 0 ? '+' : '') + n.toFixed(2);
  return (
    <div className="grid grid-cols-[30px_1fr_1fr_1fr_25px] gap-1 items-center hover:bg-white/5 rounded transition-colors">
      <span className="text-slate-500 font-bold text-[10px]">{label}</span>
      <span className={`text-right tabular-nums bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[0])}</span>
      <span className={`text-right tabular-nums bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[1])}</span>
      <span className={`text-right tabular-nums bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[2])}</span>
      <span className="text-slate-600 pl-1 text-[9px]">{unit}</span>
    </div>
  );
}

export function TelemetryPanel() {
  const [data, setData] = useState<TelemetryData | null>(null);
  const [attitude, setAttitude] = useState<[number, number, number]>([0, 0, 0]);

  useEffect(() => {
    return telemetry.subscribe((d) => {
      setData(d);
      if (d.orientation_unwrapped_deg?.length === 3) {
        setAttitude([
          d.orientation_unwrapped_deg[0],
          d.orientation_unwrapped_deg[1],
          d.orientation_unwrapped_deg[2],
        ]);
        return;
      }
      const q = new Quaternion(d.quaternion[1], d.quaternion[2], d.quaternion[3], d.quaternion[0]);
      const e = new Euler().setFromQuaternion(q, 'ZYX');
      const yawDeg = Number.isFinite(d.yaw_unwrapped_deg ?? NaN)
        ? (d.yaw_unwrapped_deg as number)
        : e.z * (180 / Math.PI);
      setAttitude([e.x * (180 / Math.PI), e.y * (180 / Math.PI), yawDeg]);
    });
  }, []);

  if (!data) return null;

  const { position, velocity, angular_velocity = [0, 0, 0], reference_position = [0, 0, 0] } = data;
  const speed = Math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2);
  const distance = Math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2);
  const delta: [number, number, number] = [
    reference_position[0] - position[0],
    reference_position[1] - position[1],
    reference_position[2] - position[2],
  ];
  const spinDeg: [number, number, number] = [
    angular_velocity[0] * 180 / Math.PI,
    angular_velocity[1] * 180 / Math.PI,
    angular_velocity[2] * 180 / Math.PI,
  ];

  return (
    <HudPanel title="TELEMETRY" live className="min-w-[240px]">
      <div className="flex flex-col gap-1 font-mono text-xs">
        <div className="grid grid-cols-[30px_1fr_1fr_1fr_25px] gap-1 text-center text-slate-500 text-[10px] mb-1">
          <div />
          <div>X</div>
          <div>Y</div>
          <div>Z</div>
          <div />
        </div>
        <DataRow label="POS" values={position} unit="m" />
        <DataRow label="VEL" values={velocity} unit="m/s" />
        <DataRow label="ERR" values={delta} unit="m" colorClass="text-orange-200" />
        <DataRow label="ROT" values={attitude} unit="deg" colorClass="text-yellow-200" />
        {data.euler_unreliable && (
          <div className="text-[10px] text-amber-300 px-1">
            Euler roll/yaw near singularity (|pitch| &gt; 85°); yaw shown unwrapped.
          </div>
        )}
        <DataRow label="SPIN" values={spinDeg} unit="°/s" colorClass="text-cyan-200" />

        <div className="mt-3 pt-2 border-t border-slate-700/50 flex justify-between px-1">
          <span className="text-slate-400 font-bold text-[10px]">RANGE</span>
          <span className="text-cyan-300 font-bold tabular-nums">{distance.toFixed(2)} m</span>
        </div>
        <div className="flex justify-between px-1">
          <span className="text-slate-400 font-bold text-[10px]">SPEED</span>
          <span className="text-cyan-300 font-bold tabular-nums">{speed.toFixed(3)} m/s</span>
        </div>
      </div>
    </HudPanel>
  );
}
```

**Step 2: Verify TypeScript**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -30
```
Expected: no errors in `TelemetryPanel.tsx`.

**Step 3: Commit**

```bash
git add ui/src/components/TelemetryPanel.tsx
git commit -m "feat(hud): add TelemetryPanel with independent telemetry subscription"
```

---

### Task 3: Create `ControllerActuatorPanel` component

**Files:**
- Create: `ui/src/components/ControllerActuatorPanel.tsx`

**Step 1: Create the file**

```tsx
import { useEffect, useState } from 'react';
import { Activity } from 'lucide-react';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { HudPanel } from './HudComponents';

// ── helpers ──────────────────────────────────────────────────────────────────

const MODE_STYLES: Record<string, string> = {
  TRACK:    'bg-indigo-600/80 text-indigo-100',
  RECOVER:  'bg-amber-600/80 text-amber-100',
  SETTLE:   'bg-sky-600/80 text-sky-100',
  HOLD:     'bg-emerald-600/80 text-emerald-100',
  COMPLETE: 'bg-green-600/80 text-green-100',
};

function ModeBadge({ label }: { label: string }) {
  const style = MODE_STYLES[label.toUpperCase()] ?? 'bg-slate-600/80 text-slate-100';
  return (
    <span className={`px-1.5 py-0.5 rounded text-[10px] font-bold uppercase tracking-wider ${style}`}>
      {label}
    </span>
  );
}

function SolverDot({ status }: { status: string }) {
  const color =
    status === 'ok' ? 'bg-green-400 shadow-[0_0_6px_rgba(74,222,128,0.8)]' :
    status === 'degraded' ? 'bg-yellow-400 shadow-[0_0_6px_rgba(250,204,21,0.8)]' :
    'bg-red-400 shadow-[0_0_6px_rgba(248,113,113,0.8)]';
  return <span className={`inline-block w-2.5 h-2.5 rounded-full ${color} flex-shrink-0`} />;
}

function GateFlag({ label, ok }: { label: string; ok: boolean }) {
  return (
    <div className="flex items-center justify-between bg-slate-900/40 rounded px-1 py-0.5">
      <span className="text-slate-400 font-bold text-[10px]">{label}</span>
      <span className={`text-sm leading-none ${ok ? 'text-green-400' : 'text-red-400'}`}>●</span>
    </div>
  );
}

function ThrusterBar({ value, label }: { value: number; label: string }) {
  const active = value > 0.01;
  return (
    <div className="flex flex-col items-center gap-1">
      <div className="relative w-2 h-8 bg-slate-800 rounded-sm overflow-hidden">
        <div
          className={`absolute bottom-0 left-0 right-0 transition-all duration-100 ${
            active ? 'bg-orange-500 shadow-[0_0_8px_rgba(249,115,22,0.8)]' : 'bg-slate-700'
          }`}
          style={{ height: `${Math.min(value * 100, 100)}%` }}
        />
      </div>
      <span className={`text-[8px] font-mono ${active ? 'text-orange-400 font-bold' : 'text-slate-600'}`}>
        {label}
      </span>
    </div>
  );
}

function ReactionWheelBar({ value, label }: { value: number; label: string }) {
  const active = Math.abs(value) > 0.00001;
  const heightPct = active
    ? Math.min(Math.max((Math.log10(Math.abs(value) * 1000 + 1) / 3) * 100, 15), 100)
    : 0;
  return (
    <div className="flex flex-col items-center gap-1">
      <div className="relative w-2 h-8 bg-slate-800 rounded-sm overflow-hidden">
        <div
          className={`absolute bottom-0 left-0 right-0 transition-all duration-100 ${
            active ? 'bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.8)]' : 'bg-slate-700'
          }`}
          style={{ height: `${heightPct}%` }}
        />
      </div>
      <span className={`text-[8px] font-mono ${active ? 'text-green-400 font-bold' : 'text-slate-600'}`}>
        {label}
      </span>
    </div>
  );
}

// ── main component ────────────────────────────────────────────────────────────

export function ControllerActuatorPanel() {
  const [data, setData] = useState<TelemetryData | null>(null);

  useEffect(() => {
    return telemetry.subscribe(setData);
  }, []);

  if (!data) return null;

  const {
    thrusters,
    rw_torque,
    solve_time = 0,
    pos_error = 0,
    ang_error = 0,
    mode_state = null,
    completion_gate = null,
    solver_health = null,
    pointing_status = null,
    controller_core = 'v6',
  } = data;

  const angErrorDeg = ang_error * (180 / Math.PI);
  const modeLabel = mode_state?.current_mode ?? 'TRACK';
  const modeTime = mode_state?.time_in_mode_s ?? 0;
  const holdElapsed = completion_gate?.hold_elapsed_s ?? 0;
  const holdRequired = completion_gate?.hold_required_s ?? 0;
  const holdProgress = holdRequired > 0 ? Math.min(1, holdElapsed / holdRequired) : 0;

  return (
    <HudPanel title="CONTROLLER / ACTUATORS" live className="min-w-[240px]">
      <div className="space-y-2 font-mono text-xs">

        {/* Solve time */}
        <div className="flex justify-between items-center">
          <div className="flex items-center gap-2">
            <Activity size={14} className="text-yellow-500" />
            <span className="text-slate-400 font-bold text-[10px]">SOLVE TIME</span>
          </div>
          <span className={`tabular-nums ${solve_time < 20 ? 'text-green-400' : solve_time < 40 ? 'text-yellow-400' : 'text-red-400'}`}>
            {(solve_time * 1000).toFixed(1)} ms
          </span>
        </div>

        <div className="h-px bg-slate-700/50" />

        {/* Mode */}
        <div className="flex justify-between items-center">
          <span className="text-slate-400 font-bold text-[10px]">MODE</span>
          <ModeBadge label={modeLabel} />
        </div>
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">CORE</span>
          <span className="text-indigo-300">{String(controller_core).toUpperCase()}</span>
        </div>
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">MODE TIME</span>
          <span className="text-slate-200 tabular-nums">{modeTime.toFixed(1)} s</span>
        </div>

        {/* Errors */}
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">POS ERROR</span>
          <span className={`tabular-nums ${pos_error < 0.1 ? 'text-green-400' : 'text-slate-200'}`}>
            {pos_error.toFixed(3)} m
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-slate-400 font-bold text-[10px]">ANG ERROR</span>
          <span className={`tabular-nums ${angErrorDeg < 1.0 ? 'text-green-400' : 'text-slate-200'}`}>
            {angErrorDeg.toFixed(1)}°
          </span>
        </div>

        {/* Pointing */}
        {pointing_status && (
          <>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">X AXIS ERR</span>
              <span className={`tabular-nums ${(Number(pointing_status.x_axis_error_deg ?? 0) <= 6.0) ? 'text-green-400' : 'text-amber-300'}`}>
                {Number(pointing_status.x_axis_error_deg ?? 0).toFixed(2)}°
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">Z AXIS ERR</span>
              <span className={`tabular-nums ${(Number(pointing_status.z_axis_error_deg ?? 0) <= 4.0) ? 'text-green-400' : 'text-amber-300'}`}>
                {Number(pointing_status.z_axis_error_deg ?? 0).toFixed(2)}°
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">POINTING</span>
              <span className={pointing_status.pointing_guardrail_breached ? 'text-amber-300' : 'text-emerald-300'}>
                {pointing_status.pointing_guardrail_breached ? 'BREACH' : 'OK'}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">VISIBLE SIDE</span>
              <span className="text-slate-200">{pointing_status.object_visible_side ?? '--'}</span>
            </div>
          </>
        )}

        {/* Completion gate */}
        {completion_gate && (
          <>
            <div className="h-px bg-slate-700/50" />
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">HOLD</span>
              <span className={`tabular-nums ${holdProgress >= 1 ? 'text-green-400' : 'text-slate-200'}`}>
                {holdElapsed.toFixed(1)} / {holdRequired.toFixed(1)} s
              </span>
            </div>
            <div className="w-full h-1 rounded bg-slate-800 overflow-hidden">
              <div
                className="h-full bg-emerald-500 transition-all duration-100"
                style={{ width: `${(holdProgress * 100).toFixed(1)}%` }}
              />
            </div>
            <div className="grid grid-cols-2 gap-1 text-[10px]">
              <GateFlag label="POS" ok={completion_gate.position_ok} />
              <GateFlag label="ANG" ok={completion_gate.angle_ok} />
              <GateFlag label="VEL" ok={completion_gate.velocity_ok} />
              <GateFlag label="OMEGA" ok={completion_gate.angular_velocity_ok} />
            </div>
            {completion_gate.last_breach_reason && (
              <div className="text-[10px] text-amber-300">
                Last breach: {completion_gate.last_breach_reason}
              </div>
            )}
          </>
        )}

        {/* Solver health */}
        {solver_health && (
          <>
            <div className="h-px bg-slate-700/50" />
            <div className="flex justify-between items-center">
              <span className="text-slate-400 font-bold text-[10px]">SOLVER</span>
              <div className="flex items-center gap-1.5">
                <SolverDot status={solver_health.status} />
                <span className="text-slate-200 uppercase text-[10px]">{solver_health.status}</span>
              </div>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">FALLBACKS</span>
              <span className="text-slate-200 tabular-nums">{solver_health.fallback_count}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-slate-400 font-bold text-[10px]">HARD BREACHES</span>
              <span className="text-slate-200 tabular-nums">{solver_health.hard_limit_breaches}</span>
            </div>
            {solver_health.fallback_active && (
              <div className="text-[10px] text-amber-300">
                Active fallback: age {Number(solver_health.fallback_age_s ?? 0).toFixed(2)} s, scale{' '}
                {Number(solver_health.fallback_scale ?? 0).toFixed(2)}
              </div>
            )}
            {solver_health.last_fallback_reason && (
              <div className="text-[10px] text-amber-300">
                Last fallback: {solver_health.last_fallback_reason}
              </div>
            )}
          </>
        )}

        {/* Actuators divider */}
        <div className="h-px bg-slate-700/50 mt-1" />
        <div className="flex gap-4 justify-between pt-1">
          <div className="flex flex-col items-center gap-2">
            <span className="text-[9px] font-bold text-slate-500 tracking-wider">THRUSTERS</span>
            <div className="flex gap-1">
              {thrusters.slice(0, 6).map((val, i) => (
                <ThrusterBar key={i} value={val} label={['+X', '-X', '+Y', '-Y', '+Z', '-Z'][i]} />
              ))}
            </div>
          </div>
          <div className="flex flex-col items-center gap-2">
            <span className="text-[9px] font-bold text-slate-500 tracking-wider">REACTION WHEELS</span>
            <div className="flex gap-1">
              {(rw_torque.length > 0 ? rw_torque : [0, 0, 0]).map((val, i) => (
                <ReactionWheelBar key={i} value={val} label={['X', 'Y', 'Z'][i]} />
              ))}
            </div>
          </div>
        </div>
      </div>
    </HudPanel>
  );
}
```

**Step 2: Verify TypeScript**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -30
```
Expected: no errors in `ControllerActuatorPanel.tsx`.

**Step 3: Commit**

```bash
git add ui/src/components/ControllerActuatorPanel.tsx
git commit -m "feat(hud): add ControllerActuatorPanel with mode badges, solver traffic-light"
```

---

### Task 4: Replace `Overlay` with structural `ViewerHud`

**Files:**
- Modify: `ui/src/components/Overlay.tsx`

**Step 1: Replace the file content**

The new `Overlay` is a purely structural shell — no telemetry subscription, no state:

```tsx
import { TelemetryPanel } from './TelemetryPanel';
import { ControllerActuatorPanel } from './ControllerActuatorPanel';

export function Overlay() {
  return (
    <div className="absolute inset-0 pointer-events-none p-4 z-10">
      {/* Top-left: Telemetry */}
      <div className="absolute top-4 left-4 pointer-events-auto">
        <TelemetryPanel />
      </div>

      {/* Top-right: Controller + Actuators */}
      <div className="absolute top-4 right-4 pointer-events-auto">
        <ControllerActuatorPanel />
      </div>
    </div>
  );
}
```

**Step 2: Verify TypeScript**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -30
```
Expected: clean.

**Step 3: Commit**

```bash
git add ui/src/components/Overlay.tsx
git commit -m "refactor(hud): replace monolithic Overlay with split-panel ViewerHud shell"
```

---

### Task 5: Add collapsible charts drawer to `TelemetryCharts`

**Files:**
- Modify: `ui/src/components/TelemetryCharts.tsx`

**Step 1: Read the current file**

Confirm `TelemetryCharts` renders a fixed `h-56` bottom strip with `absolute bottom-0 left-0 right-0`.

**Step 2: Wrap with collapsible drawer**

Replace the entire file:

```tsx
import { useMemo, useState, useDeferredValue } from 'react';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, CartesianGrid, ReferenceLine } from 'recharts';
import { Activity, ChevronUp, ChevronDown } from 'lucide-react';
import { useTelemetryStore } from '../store/telemetryStore';

export function TelemetryCharts() {
  const rawHistory = useTelemetryStore(s => s.history);
  const history = useDeferredValue(rawHistory);
  const [timeWindow, setTimeWindow] = useState(30);
  const [expanded, setExpanded] = useState(false);
  const [visible, setVisible] = useState({
    pos: true,
    ang: true,
    vel: true,
    solve: true,
  });

  const chartData = useMemo(() => {
    if (history.length === 0) return [];
    if (timeWindow === 0) return history;
    const latestTime = history[history.length - 1].time;
    return history.filter((p) => p.time >= latestTime - timeWindow);
  }, [history, timeWindow]);

  const visibleKeys = (['pos', 'ang', 'vel', 'solve'] as const).filter((k) => visible[k]);

  return (
    <div
      className={`absolute bottom-0 left-0 right-0 z-20 transition-all duration-300 ${
        expanded ? 'h-56' : 'h-8'
      }`}
    >
      {/* Toggle tab — always visible */}
      <button
        onClick={() => setExpanded((v) => !v)}
        className="absolute top-0 left-0 right-0 h-8 bg-black/70 backdrop-blur-sm border-t border-white/10 flex items-center justify-between px-4 text-xs text-gray-400 hover:text-gray-200 transition-colors cursor-pointer select-none"
      >
        <div className="flex items-center gap-3">
          {expanded ? <ChevronDown size={12} /> : <ChevronUp size={12} />}
          <span className="uppercase tracking-wider font-semibold">Charts</span>
          <span className="text-gray-600">
            {visibleKeys.map((k) => k.toUpperCase()).join(' · ')}
          </span>
        </div>
        {expanded && (
          <div className="flex items-center gap-2" onClick={(e) => e.stopPropagation()}>
            <span className="uppercase tracking-wider text-[10px] mr-1">Window</span>
            {[10, 30, 120, 0].map((range) => (
              <button
                key={range}
                onClick={() => setTimeWindow(range)}
                className={`px-2 py-0.5 rounded border text-[10px] uppercase ${
                  timeWindow === range ? 'border-blue-500 text-blue-300' : 'border-gray-700 text-gray-500 hover:border-gray-500'
                }`}
              >
                {range === 0 ? 'all' : `${range}s`}
              </button>
            ))}
            <div className="w-px h-3 bg-gray-700 mx-1" />
            {(['pos', 'ang', 'vel', 'solve'] as const).map((key) => (
              <button
                key={key}
                onClick={() => setVisible((prev) => ({ ...prev, [key]: !prev[key] }))}
                className={`px-2 py-0.5 rounded border text-[10px] uppercase ${
                  visible[key] ? 'border-green-500 text-green-300' : 'border-gray-700 text-gray-500'
                }`}
              >
                {key}
              </button>
            ))}
          </div>
        )}
      </button>

      {/* Chart content — only rendered when expanded */}
      {expanded && (
        <div className="absolute top-8 left-0 right-0 bottom-0 bg-black/80 backdrop-blur-md border-t border-white/5 flex px-4 pb-3 pt-2 gap-4 min-h-0">

          {visible.pos && (
            <ChartPane title="POSITION ERROR (m)" color="#60a5fa" dataKey="posError" icon="text-blue-400"
              refLines={[{ y: 0.1, color: '#22c55e' }, { y: 0.5, color: '#ef4444' }]}
              data={chartData} />
          )}
          {visible.ang && (
            <ChartPane title="ANGLE ERROR (deg)" color="#c084fc" dataKey="angError" icon="text-purple-400"
              refLines={[{ y: 1, color: '#22c55e' }, { y: 5, color: '#ef4444' }]}
              data={chartData} />
          )}
          {visible.vel && (
            <ChartPane title="VELOCITY (m/s)" color="#4ade80" dataKey="velocity" icon="text-green-400"
              refLines={[]}
              data={chartData} />
          )}
          {visible.solve && (
            <ChartPane title="SOLVE TIME (ms)" color="#facc15" dataKey="solveTime" icon="text-yellow-400"
              refLines={[{ y: 20, color: '#22c55e' }, { y: 40, color: '#ef4444' }]}
              data={chartData} />
          )}
        </div>
      )}
    </div>
  );
}

function ChartPane({
  title,
  color,
  dataKey,
  icon,
  refLines,
  data,
}: {
  title: string;
  color: string;
  dataKey: string;
  icon: string;
  refLines: { y: number; color: string }[];
  data: { time: number; [key: string]: number }[];
}) {
  return (
    <div className="flex-1 min-w-[200px] min-h-0 flex flex-col">
      <div className={`text-[10px] font-bold text-gray-400 mb-1 flex items-center gap-1.5`}>
        <Activity size={11} className={icon} />
        {title}
      </div>
      <div className="flex-1 min-h-0">
        <ResponsiveContainer width="100%" height="100%" minWidth={0}>
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="3 3" stroke="#222" />
            <XAxis dataKey="time" stroke="#555" fontSize={9} tickFormatter={(v) => `${v}s`}
              interval="preserveStartEnd" minTickGap={30} />
            <YAxis domain={['auto', 'auto']} stroke="#555" fontSize={9} width={42} />
            <Tooltip contentStyle={{ backgroundColor: '#111', border: '1px solid #333', fontSize: 11 }}
              labelFormatter={(l) => `${l}s`} />
            {refLines.map((r) => (
              <ReferenceLine key={r.y} y={r.y} stroke={r.color} strokeDasharray="4 4" />
            ))}
            <Line type="monotone" dataKey={dataKey} stroke={color} strokeWidth={1.5}
              dot={false} isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
```

**Step 3: Verify TypeScript**

```bash
cd ui && npx tsc --noEmit 2>&1 | head -30
```
Expected: clean.

**Step 4: Commit**

```bash
git add ui/src/components/TelemetryCharts.tsx
git commit -m "feat(hud): collapsible charts drawer with useDeferredValue, extracted ChartPane"
```

---

### Task 6: Verify the full build and run lint

**Step 1: TypeScript check**

```bash
cd ui && npx tsc --noEmit
```
Expected: 0 errors.

**Step 2: Lint**

```bash
cd ui && npx eslint src/components/Overlay.tsx src/components/TelemetryPanel.tsx src/components/ControllerActuatorPanel.tsx src/components/HudComponents.tsx src/components/TelemetryCharts.tsx --max-warnings 0
```
Expected: no errors or warnings.

**Step 3: Vite build**

```bash
cd ui && npm run build 2>&1 | tail -20
```
Expected: `✓ built in` with no errors.

**Step 4: Commit if any lint auto-fixes were applied**

```bash
git add -p
git commit -m "fix(hud): lint fixes from full build check"
```
(Skip this step if nothing changed.)

---

### Task 7: Manual smoke test

**Step 1: Start the dev server**

```bash
make run
```
Navigate to `http://localhost:5173` and switch to **Viewer** mode.

**Step 2: Verify layout**
- TELEMETRY panel appears top-left ✓
- CONTROLLER / ACTUATORS panel appears top-right ✓
- Bottom of screen shows a thin `▲ Charts  POS · ANG · VEL · SOLVE` tab ✓

**Step 3: Verify charts drawer**
- Click the tab → drawer slides up to 224px ✓
- Click again → collapses back to 32px ✓
- Window and visibility toggles work ✓

**Step 4: Verify visuals**
- Mode badge is color-coded (e.g. TRACK = indigo, HOLD = emerald) ✓
- Solver status shows a colored dot + text ✓
- Gate flags show filled circle `●` in green/red ✓
- `HudPanel` corner accents are slightly larger ✓

**Step 5: Start a simulation from Runner mode and return to Viewer**
- Both panels update in real time ✓
- Number columns don't shift layout as values change (tabular-nums) ✓
- Live indicator dot pulses on both panel titles ✓

**Step 6: Final commit**

```bash
git add -A
git commit -m "chore(hud): smoke test confirmed, viewer hud redesign complete"
```

import { useState, useEffect, useMemo } from 'react';
import { Trash2, Save, CheckCircle, Crosshair, Route, Link2, Pause, CircleDot, MapPin } from 'lucide-react';
import { useStudioStore } from './useStudioStore';
import { compileStudioMission } from './compileStudioMission';

function polylineLength(points: [number, number, number][]): number {
  if (!points || points.length < 2) return 0;
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    total += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  }
  return total;
}

function SegmentRow({ index }: { index: number }) {
  const assembly = useStudioStore((s) => s.assembly);
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);
  const paths = useStudioStore((s) => s.paths);
  const holds = useStudioStore((s) => s.holds);
  const wires = useStudioStore((s) => s.wires);
  const obstacles = useStudioStore((s) => s.obstacles);
  const points = useStudioStore((s) => s.points);
  const removePath = useStudioStore((s) => s.removePath);
  const removeHold = useStudioStore((s) => s.removeHold);
  const removeWire = useStudioStore((s) => s.removeWire);
  const removeObstacle = useStudioStore((s) => s.removeObstacle);
  const removePoint = useStudioStore((s) => s.removePoint);
  const setActiveTool = useStudioStore((s) => s.setActiveTool);
  const selectPath = useStudioStore((s) => s.selectPath);
  const setSelectedHandle = useStudioStore((s) => s.setSelectedHandle);
  const setSelectedAssemblyId = useStudioStore((s) => s.setSelectedAssemblyId);

  const item = assembly[index];
  if (!item) return null;
  const isFocused = selectedAssemblyId === item.id;

  let icon: React.ReactNode = <CircleDot size={13} />;
  let label: string = item.type;
  let onRemove: (() => void) | null = null;

  if (item.type === 'place_satellite') {
    icon = <Crosshair size={13} />;
    label = 'Place Satellite';
  }
  if (item.type === 'create_path') {
    icon = <Route size={13} />;
    const path = paths.find((p) => p.id === item.pathId);
    label = `Create Path ${path?.axisSeed ?? ''}`.trim();
    if (path) onRemove = () => removePath(path.id);
  }
  if (item.type === 'connect') {
    icon = <Link2 size={13} />;
    const wire = wires.find((w) => w.id === item.wireId);
    label = wire ? `Connect ${wire.fromNodeId} -> ${wire.toNodeId}` : 'Connect';
    if (wire) onRemove = () => removeWire(wire.id);
  }
  if (item.type === 'hold') {
    icon = <Pause size={13} />;
    const hold = holds.find((h) => h.id === item.holdId);
    label = hold ? `Hold ${hold.duration.toFixed(1)}s @ ${hold.pathId}[${hold.waypointIndex}]` : 'Hold';
    if (hold) onRemove = () => removeHold(hold.id);
  }
  if (item.type === 'obstacle') {
    icon = <CircleDot size={13} />;
    const obs = obstacles.find((o) => o.id === item.obstacleId);
    label = obs ? `Obstacle r=${obs.radius.toFixed(2)}` : 'Obstacle';
    if (obs) onRemove = () => removeObstacle(obs.id);
  }
  if (item.type === 'point') {
    icon = <MapPin size={13} />;
    const point = points.find((p) => p.id === item.pointId);
    label = point
      ? `Point (${point.position[0].toFixed(2)}, ${point.position[1].toFixed(2)}, ${point.position[2].toFixed(2)})`
      : 'Point';
    if (point) onRemove = () => removePoint(point.id);
  }

  const handleSelectSegment = () => {
    setSelectedAssemblyId(isFocused ? null : item.id);
    if (item.type === 'place_satellite') {
      setActiveTool('place_satellite');
      selectPath(null);
      return;
    }
    if (item.type === 'create_path') {
      setActiveTool('create_path');
      if (item.pathId) {
        selectPath(item.pathId);
        setSelectedHandle(item.pathId, null);
      }
      return;
    }
    if (item.type === 'connect') {
      setActiveTool('connect');
      return;
    }
    if (item.type === 'hold') {
      setActiveTool('hold');
      const hold = holds.find((h) => h.id === item.holdId);
      if (hold?.pathId) {
        selectPath(hold.pathId);
        setSelectedHandle(hold.pathId, null);
      }
      return;
    }
    if (item.type === 'obstacle') {
      setActiveTool('obstacle');
      return;
    }
    if (item.type === 'point') {
      setActiveTool('point');
      return;
    }
  };

  return (
    <div
      role="button"
      tabIndex={0}
      onClick={handleSelectSegment}
      onKeyDown={(e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          handleSelectSegment();
        }
      }}
      className={`w-full flex items-center gap-2 px-3 py-2 rounded-lg border group text-left ${
        isFocused
          ? 'border-cyan-600 bg-cyan-900/25'
          : 'border-slate-800 hover:border-cyan-700 bg-slate-900/40'
      }`}
    >
      <span className="text-[10px] text-slate-500 w-5 shrink-0 tabular-nums">{index + 1}</span>
      <span className="text-slate-300">{icon}</span>
      <span className="flex-1 text-xs text-slate-200 font-medium truncate">{label}</span>
      {onRemove && (
        <button
          type="button"
          onClick={(e) => {
            e.stopPropagation();
            onRemove();
          }}
          className="opacity-0 group-hover:opacity-100 text-slate-500 hover:text-red-400 transition-opacity"
        >
          <Trash2 size={12} />
        </button>
      )}
    </div>
  );
}

export function MissionStudioRightPanel() {
  const assembly = useStudioStore((s) => s.assembly);
  const paths = useStudioStore((s) => s.paths);
  const wires = useStudioStore((s) => s.wires);
  const satelliteStart = useStudioStore((s) => s.satelliteStart);
  const points = useStudioStore((s) => s.points);
  const missionName = useStudioStore((s) => s.missionName);
  const setMissionName = useStudioStore((s) => s.setMissionName);
  const validationBusy = useStudioStore((s) => s.validationBusy);
  const setValidationBusy = useStudioStore((s) => s.setValidationBusy);
  const saveBusy = useStudioStore((s) => s.saveBusy);
  const setSaveBusy = useStudioStore((s) => s.setSaveBusy);

  const [saveSuccess, setSaveSuccess] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [validateResult, setValidateResult] = useState<{ ok: boolean; message: string } | null>(null);

  useEffect(() => {
    if (missionName.trim().length > 0) return;
    if (paths.length === 0) return;
    const ts = new Date().toISOString().slice(0, 16).replace(/[-:T]/g, '');
    setMissionName(`Studio_${paths.length}path_${ts}`);
  }, [paths.length, missionName, setMissionName]);

  const totalWaypoints = paths.reduce((acc, p) => acc + p.waypoints.length, 0);
  const totalPathLengthM = useMemo(() => {
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      const manual = (mission.overrides?.manual_path ?? []) as [number, number, number][];
      if (manual.length >= 2) return polylineLength(manual);
    } catch {
      // Route may be incomplete/invalid while authoring; use geometric fallback.
    }

    const authoredPathLength = paths.reduce((acc, p) => acc + polylineLength(p.waypoints), 0);
    const resolveNodePosition = (nodeId: string): [number, number, number] | null => {
      if (nodeId === 'satellite:start') return satelliteStart;
      if (nodeId.startsWith('point:')) {
        const pointId = nodeId.slice('point:'.length);
        return points.find((p) => p.id === pointId)?.position ?? null;
      }
      const parts = nodeId.split(':');
      if (parts.length === 3 && parts[0] === 'path' && (parts[2] === 'start' || parts[2] === 'end')) {
        const path = paths.find((p) => p.id === parts[1]);
        if (!path || path.waypoints.length === 0) return null;
        return parts[2] === 'start' ? path.waypoints[0] : path.waypoints[path.waypoints.length - 1];
      }
      return null;
    };
    const authoredWireLength = wires.reduce((acc, w) => {
      if (w.waypoints && w.waypoints.length >= 2) {
        return acc + polylineLength(w.waypoints);
      }
      const from = resolveNodePosition(w.fromNodeId);
      const to = resolveNodePosition(w.toNodeId);
      if (!from || !to) return acc;
      return acc + Math.hypot(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
    }, 0);
    return authoredPathLength + authoredWireLength;
  }, [paths, wires, satelliteStart, points]);

  const handleValidate = async () => {
    setValidationBusy(true);
    setValidateResult(null);
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      const { unifiedMissionApi } = await import('../../api/unifiedMissionApi');
      const report = await unifiedMissionApi.validateMission(mission);
      setValidateResult({
        ok: report.valid,
        message: report.valid ? 'Validation passed' : `${report.summary?.errors ?? '?'} error(s)`,
      });
    } catch (e) {
      setValidateResult({ ok: false, message: String(e) });
    } finally {
      setValidationBusy(false);
    }
  };

  const handleSave = async () => {
    setSaveBusy(true);
    setSaveSuccess(false);
    setSaveError(null);
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      const { unifiedMissionApi } = await import('../../api/unifiedMissionApi');
      await unifiedMissionApi.saveMission(missionName || 'Untitled Studio Mission', mission);
      setSaveSuccess(true);
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (e) {
      setSaveError(String(e));
    } finally {
      setSaveBusy(false);
    }
  };

  const canSave = assembly.length > 0 && missionName.trim().length > 0;

  return (
    <div className="flex flex-col h-full">
      <div className="px-3 py-2 text-[10px] font-bold uppercase tracking-[0.16em] text-slate-500 border-b border-slate-800/60 flex items-center justify-between">
        <span>Mission Assembly</span>
        <span className="text-slate-600 tabular-nums">
          {assembly.length} seg · {totalWaypoints} pts · {totalPathLengthM.toFixed(1)} m
        </span>
      </div>

      <div className="flex-1 overflow-y-auto p-3 flex flex-col gap-1.5">
        {assembly.length === 0 ? (
          <div className="text-xs text-slate-600 text-center py-8">Add segments using the left panel</div>
        ) : (
          assembly.map((_, i) => <SegmentRow key={i} index={i} />)
        )}
      </div>

      {validateResult && (
        <div className="px-3 py-2 border-t border-slate-800/60">
          <div className={`text-xs font-semibold ${validateResult.ok ? 'text-emerald-400' : 'text-amber-400'}`}>
            {validateResult.ok ? '✓' : '✗'} {validateResult.message}
          </div>
        </div>
      )}
      {saveError && <div className="px-3 py-1 text-[10px] text-red-400">{saveError}</div>}

      <div className="p-3 border-t border-slate-800/60 flex flex-col gap-2">
        <input
          className="w-full bg-black/40 border border-slate-700 rounded-lg px-2.5 py-1.5 text-xs text-slate-100 placeholder:text-slate-600 focus:outline-none focus:border-cyan-700"
          placeholder="Mission name..."
          value={missionName}
          onChange={(e) => setMissionName(e.target.value)}
        />
        <button
          type="button"
          onClick={() => void handleValidate()}
          disabled={validationBusy}
          className="w-full py-2 rounded-lg border border-slate-700 bg-slate-800 text-slate-200 text-xs font-semibold disabled:opacity-50 hover:border-slate-600 transition-all"
        >
          {validationBusy ? 'Validating...' : 'Validate'}
        </button>
        <button
          type="button"
          onClick={() => void handleSave()}
          disabled={!canSave || saveBusy}
          className="w-full py-2 rounded-lg border border-emerald-700 bg-emerald-900/40 text-emerald-100 text-xs font-semibold disabled:opacity-40 flex items-center justify-center gap-1.5 hover:bg-emerald-900/60 transition-all"
        >
          {saveSuccess ? (
            <>
              <CheckCircle size={13} /> Saved!
            </>
          ) : saveBusy ? (
            'Saving...'
          ) : (
            <>
              <Save size={13} /> Save Mission
            </>
          )}
        </button>
      </div>
    </div>
  );
}

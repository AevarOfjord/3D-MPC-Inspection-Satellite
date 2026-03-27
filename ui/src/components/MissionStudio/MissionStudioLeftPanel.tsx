import { useRef, useState, useEffect, useMemo } from 'react';
import { Crosshair, Route, PenSquare, Link2, Pause, CircleDot, MapPin, Trash2 } from 'lucide-react';
import { useStudioStore } from './useStudioStore';
import { useRegenerateWaypoints } from './useRegenerateWaypoints';
import { trajectoryApi } from '../../api/trajectory';
import { studioModelPathToUrl, studioReferenceLabel } from './studioReference';
import { getStudioRouteDiagnostics } from './studioRouteDiagnostics';

function SectionHeader({ label }: { label: string }) {
  return (
    <div className="px-3 py-2 text-[10px] font-bold uppercase tracking-[0.16em] text-slate-500 border-b border-slate-800/60">
      {label}
    </div>
  );
}

function ToolButton({
  icon,
  label,
  description,
  tool,
}: {
  icon: React.ReactNode;
  label: string;
  description: string;
  tool: NonNullable<ReturnType<typeof useStudioStore.getState>['activeTool']>;
}) {
  const activeTool = useStudioStore((s) => s.activeTool);
  const setActiveTool = useStudioStore((s) => s.setActiveTool);
  const setSelectedAssemblyId = useStudioStore((s) => s.setSelectedAssemblyId);
  const active = activeTool === tool;
  return (
    <button
      type="button"
      onClick={() => {
        setSelectedAssemblyId(null);
        setActiveTool(active ? null : tool);
      }}
      className={`w-full flex items-center gap-2 px-3 py-2 rounded-lg border text-xs font-semibold transition-all ${
        active
          ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
          : 'border-slate-700 text-slate-300 hover:border-cyan-700'
      }`}
    >
      <span className="shrink-0">{icon}</span>
      <span className="min-w-0 flex-1 text-left">
        <span className="block">{label}</span>
        <span className="mt-0.5 block text-[10px] font-normal text-slate-500">
          {description}
        </span>
      </span>
      {active ? (
        <span className="rounded-full border border-cyan-500/50 bg-cyan-950/60 px-2 py-0.5 text-[9px] uppercase tracking-[0.14em] text-cyan-100">
          Active
        </span>
      ) : null}
    </button>
  );
}

function NumberField({
  label,
  value,
  onChange,
  step = 0.1,
}: {
  label: string;
  value: number;
  onChange: (next: number) => void;
  step?: number;
}) {
  return (
    <label className="flex items-center justify-between gap-2 text-[11px] text-slate-300">
      <span className="text-slate-500 uppercase tracking-wide">{label}</span>
      <input
        type="number"
        value={Number.isFinite(value) ? value : 0}
        step={step}
        onChange={(e) => onChange(Number(e.target.value))}
        className="w-24 bg-black/40 border border-slate-700 rounded px-2 py-1 text-xs text-slate-100"
      />
    </label>
  );
}

export function MissionStudioLeftPanel() {
  const {
    satelliteStart,
    setSatelliteStart,
    addPath,
    paths,
    selectedPathId,
    selectPath,
    updatePath,
    removePath,
    updatePathCenter,
    updatePathRotation,
    wires,
    assembly,
    removeWire,
    setWireConstraintMode,
    holds,
    updateHold,
    removeHold,
    obstacles,
    addObstacle,
    updateObstacle,
    removeObstacle,
    points,
    addPoint,
    updatePoint,
    removePoint,
    modelUrl,
    referenceObjectPath,
    setModelUrl,
    setReferenceObjectPath,
    pathEditMode,
    setPathEditMode,
    editMode,
    setEditMode,
    setPathWaypointsManual,
  } = useStudioStore();
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);

  const activeTool = useStudioStore((s) => s.activeTool);
  const setSelectedAssemblyId = useStudioStore((s) => s.setSelectedAssemblyId);
  const regenerate = useRegenerateWaypoints();
  const fileRef = useRef<HTMLInputElement>(null);
  const [uploading, setUploading] = useState(false);
  const [uploadError, setUploadError] = useState<string | null>(null);
  const [axisSeed, setAxisSeed] = useState<'X' | 'Y' | 'Z'>('Z');

  const routeDiagnostics = useMemo(
    () =>
      getStudioRouteDiagnostics({
        referenceObjectPath,
        paths,
        wires,
        holds,
        points,
        assembly,
      }),
    [referenceObjectPath, paths, wires, holds, points, assembly]
  );

  const selectedPath = paths.find((p) => p.id === selectedPathId) ?? null;
  const selectedPathCenter = selectedPath
    ? ([
        0.5 * (selectedPath.planeA.position[0] + selectedPath.planeB.position[0]),
        0.5 * (selectedPath.planeA.position[1] + selectedPath.planeB.position[1]),
        0.5 * (selectedPath.planeA.position[2] + selectedPath.planeB.position[2]),
      ] as [number, number, number])
    : null;
  const selectedWireId = selectedAssemblyId
    ? assembly.find((item) => item.id === selectedAssemblyId && item.type === 'connect')?.wireId ?? null
    : null;
  const selectedWire = selectedWireId ? wires.find((w) => w.id === selectedWireId) ?? null : null;

  const resamplePolyline = (points: [number, number, number][], density: number): [number, number, number][] => {
    if (points.length < 2) return points;
    const clamped = Math.max(0.25, Math.min(25, density || 1));
    const outCount = Math.max(2, Math.round(points.length * clamped));
    const segLens: number[] = [];
    let total = 0;
    for (let i = 0; i < points.length - 1; i += 1) {
      const a = points[i];
      const b = points[i + 1];
      const len = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
      segLens.push(len);
      total += len;
    }
    if (total <= 1e-9) return points;
    const sampled: [number, number, number][] = [];
    for (let i = 0; i < outCount; i += 1) {
      const t = i / Math.max(1, outCount - 1);
      let dist = t * total;
      let seg = 0;
      while (seg < segLens.length - 1 && dist > segLens[seg]) {
        dist -= segLens[seg];
        seg += 1;
      }
      const a = points[seg];
      const b = points[Math.min(seg + 1, points.length - 1)];
      const segLen = Math.max(1e-9, segLens[Math.min(seg, segLens.length - 1)] || 1e-9);
      const u = Math.max(0, Math.min(1, dist / segLen));
      sampled.push([
        a[0] + (b[0] - a[0]) * u,
        a[1] + (b[1] - a[1]) * u,
        a[2] + (b[2] - a[2]) * u,
      ]);
    }
    return sampled;
  };

  const resampleSnippet = (
    points: [number, number, number][],
    start: number,
    end: number,
    density: number
  ): { points: [number, number, number][]; newRange: [number, number] } => {
    if (points.length < 2) return { points, newRange: [0, Math.max(0, points.length - 1)] };
    const lo = Math.max(0, Math.min(start, end));
    const hi = Math.min(points.length - 1, Math.max(start, end));
    if (hi - lo < 1) return { points, newRange: [lo, hi] };
    const snippet = points.slice(lo, hi + 1);
    const sampled = resamplePolyline(snippet, density);
    const merged = [...points.slice(0, lo), ...sampled, ...points.slice(hi + 1)];
    return { points: merged, newRange: [lo, lo + sampled.length - 1] };
  };

  useEffect(() => {
    if (activeTool !== 'obstacle') return;
    if (obstacles.length > 0) return;
    addObstacle();
  }, [activeTool, obstacles.length, addObstacle]);

  useEffect(() => {
    if (activeTool !== 'point') return;
    if (points.length > 0) return;
    addPoint();
  }, [activeTool, points.length, addPoint]);

  const handleLoadModel = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    setUploading(true);
    setUploadError(null);
    try {
      const uploaded = await trajectoryApi.uploadObject(file);
      setReferenceObjectPath(uploaded.path);
      setModelUrl(studioModelPathToUrl(uploaded.path));
    } catch (err) {
      setUploadError(err instanceof Error ? err.message : String(err));
    } finally {
      setUploading(false);
      if (fileRef.current) fileRef.current.value = '';
    }
  };

  const activeTaskCopy =
    activeTool === 'place_satellite'
      ? 'Set the LVLH start pose. Use this before routing if the default [0,0,20] start is not right.'
      : activeTool === 'create_path'
        ? 'Author spiral scans. Add a path, then move, rotate, and size it until the target sweep is correct.'
        : activeTool === 'connect'
          ? 'Build one continuous route. Drag from satellite:start or a path endpoint into the next valid target.'
          : activeTool === 'hold'
            ? 'Click any waypoint on a connected path to insert a dwell segment.'
            : activeTool === 'obstacle'
              ? 'Place keep-out spheres where the route must avoid structure or debris.'
              : activeTool === 'point'
                ? 'Insert guide points to bend transfers between paths.'
                : activeTool === 'edit'
                  ? 'Refine the selected path or wire. Stretch, add, delete, or resample locally.'
                  : 'Pick the next authoring task. Studio now exposes route health continuously while you work.';

  return (
    <div className="flex flex-col gap-0">
      <SectionHeader label="Workflow" />
      <div className="p-3 flex flex-col gap-2">
        <div className="rounded-xl border border-slate-800 bg-slate-950/80 p-3">
          <div className="text-[10px] font-bold uppercase tracking-[0.16em] text-slate-500">
            Current Task
          </div>
          <div className="mt-1 text-sm font-semibold text-slate-100">
            {activeTool ? activeTool.replace('_', ' ') : 'Choose a tool'}
          </div>
          <div className="mt-2 text-[11px] leading-5 text-slate-300">
            {activeTaskCopy}
          </div>
        </div>
      </div>

      <SectionHeader label="Add Segment" />
      <div className="p-3 flex flex-col gap-2">
        <ToolButton
          icon={<Crosshair size={13} />}
          label="Place Satellite"
          description="Set the starting LVLH position."
          tool="place_satellite"
        />
        <ToolButton
          icon={<Route size={13} />}
          label="Create Path"
          description="Add and shape a spiral inspection pass."
          tool="create_path"
        />
        <ToolButton
          icon={<Link2 size={13} />}
          label="Connect"
          description="Link start, points, and paths into one route."
          tool="connect"
        />
        <ToolButton
          icon={<Pause size={13} />}
          label="Hold"
          description="Insert dwell time on a connected path."
          tool="hold"
        />
        <ToolButton
          icon={<PenSquare size={13} />}
          label="Edit"
          description="Refine path geometry and wire curvature."
          tool="edit"
        />
        <ToolButton
          icon={<CircleDot size={13} />}
          label="Obstacle"
          description="Place keep-out spheres in the scene."
          tool="obstacle"
        />
        <ToolButton
          icon={<MapPin size={13} />}
          label="Point"
          description="Add guide nodes to shape transfers."
          tool="point"
        />
      </div>

      {activeTool === 'place_satellite' && (
        <>
          <SectionHeader label="Place Satellite" />
          <div className="p-3 flex flex-col gap-2">
            <NumberField label="X" value={satelliteStart[0]} onChange={(v) => setSatelliteStart([v, satelliteStart[1], satelliteStart[2]])} />
            <NumberField label="Y" value={satelliteStart[1]} onChange={(v) => setSatelliteStart([satelliteStart[0], v, satelliteStart[2]])} />
            <NumberField label="Z" value={satelliteStart[2]} onChange={(v) => setSatelliteStart([satelliteStart[0], satelliteStart[1], v])} />
          </div>
        </>
      )}

      {activeTool === 'create_path' && (
        <>
          <SectionHeader label="Create Path" />
          <div className="p-3 flex flex-col gap-3">
            <div className="flex gap-1">
              {(['X', 'Y', 'Z'] as const).map((axis) => (
                <button
                  key={axis}
                  type="button"
                  onClick={() => setAxisSeed(axis)}
                  className={`flex-1 py-1.5 rounded-lg border text-xs font-bold transition-all ${
                    axisSeed === axis
                      ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                      : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                  }`}
                >
                  {axis}
                </button>
              ))}
            </div>
            <button
              type="button"
              onClick={() => {
                const id = addPath(axisSeed);
                selectPath(id);
                setPathEditMode('translate');
                regenerate(id, 0);
              }}
              className="w-full py-2 rounded-lg border border-violet-700 bg-violet-900/30 text-violet-100 text-xs font-semibold"
            >
              Add Path
            </button>
            <div className="rounded-lg border border-slate-800 bg-slate-950/70 px-3 py-2 text-[10px] text-slate-300">
              New paths are created with the current axis seed and default spacing, then focused
              immediately for move/rotate adjustments.
            </div>

            {selectedPath && (
              <>
                <div className="text-[10px] text-slate-500">Selected: {selectedPath.id}</div>
                <div className="flex gap-1">
                  <button
                    type="button"
                    onClick={() => setPathEditMode('translate')}
                    className={`flex-1 py-1.5 rounded-lg border text-[10px] font-bold transition-all ${
                      pathEditMode === 'translate'
                        ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                        : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                    }`}
                  >
                    Move
                  </button>
                  <button
                    type="button"
                    onClick={() => setPathEditMode('rotate')}
                    className={`flex-1 py-1.5 rounded-lg border text-[10px] font-bold transition-all ${
                      pathEditMode === 'rotate'
                        ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                        : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                    }`}
                  >
                    Rotate
                  </button>
                </div>
                <div className="text-[10px] text-slate-500">
                  {pathEditMode === 'translate'
                    ? 'Move: click centerline or plane to toggle controls.'
                    : 'Rotate: use the degree fields below to spin the spiral about the path midpoint.'}
                </div>
                {pathEditMode === 'translate' && selectedPathCenter && (
                  <>
                    <NumberField
                      label="X Position"
                      value={selectedPathCenter[0]}
                      onChange={(v) => {
                        updatePathCenter(selectedPath.id, [v, selectedPathCenter[1], selectedPathCenter[2]]);
                        regenerate(selectedPath.id, 120);
                      }}
                    />
                    <NumberField
                      label="Y Position"
                      value={selectedPathCenter[1]}
                      onChange={(v) => {
                        updatePathCenter(selectedPath.id, [selectedPathCenter[0], v, selectedPathCenter[2]]);
                        regenerate(selectedPath.id, 120);
                      }}
                    />
                    <NumberField
                      label="Z Position"
                      value={selectedPathCenter[2]}
                      onChange={(v) => {
                        updatePathCenter(selectedPath.id, [selectedPathCenter[0], selectedPathCenter[1], v]);
                        regenerate(selectedPath.id, 120);
                      }}
                    />
                  </>
                )}
                {pathEditMode === 'rotate' && (
                  <>
                    <NumberField
                      label="X Rotation °"
                      value={selectedPath.rotationDegrees?.x ?? 0}
                      onChange={(v) => {
                        updatePathRotation(selectedPath.id, { x: v });
                        regenerate(selectedPath.id, 120);
                      }}
                      step={1}
                    />
                    <NumberField
                      label="Y Rotation °"
                      value={selectedPath.rotationDegrees?.y ?? 0}
                      onChange={(v) => {
                        updatePathRotation(selectedPath.id, { y: v });
                        regenerate(selectedPath.id, 120);
                      }}
                      step={1}
                    />
                    <NumberField
                      label="Z Rotation °"
                      value={selectedPath.rotationDegrees?.z ?? 0}
                      onChange={(v) => {
                        updatePathRotation(selectedPath.id, { z: v });
                        regenerate(selectedPath.id, 120);
                      }}
                      step={1}
                    />
                  </>
                )}
                <NumberField
                  label="Level Spacing"
                  value={selectedPath.levelSpacing}
                  onChange={(v) => {
                    updatePath(selectedPath.id, { levelSpacing: Math.max(0.05, v) });
                    regenerate(selectedPath.id, 120);
                  }}
                  step={0.05}
                />
                <NumberField
                  label="Ellipse X"
                  value={selectedPath.ellipse.radiusX}
                  onChange={(v) => {
                    useStudioStore.getState().updatePathEllipse(selectedPath.id, { radiusX: Math.max(0.1, v) });
                    regenerate(selectedPath.id, 120);
                  }}
                />
                <NumberField
                  label="Ellipse Y"
                  value={selectedPath.ellipse.radiusY}
                  onChange={(v) => {
                    useStudioStore.getState().updatePathEllipse(selectedPath.id, { radiusY: Math.max(0.1, v) });
                    regenerate(selectedPath.id, 120);
                  }}
                />
                <button
                  type="button"
                  onClick={() => removePath(selectedPath.id)}
                  className="w-full py-1.5 rounded-lg border border-red-800/80 text-red-300 text-xs"
                >
                  Remove Path
                </button>
              </>
            )}
          </div>
        </>
      )}

      {activeTool === 'connect' && (
        <>
          <SectionHeader label="Connect" />
          <div className="p-3 flex flex-col gap-2">
            <div className="rounded-lg border border-slate-800 bg-slate-950/70 px-3 py-2 text-[10px] text-slate-300">
              Drag from a source node to a valid destination. Green targets are legal, red
              targets would branch, duplicate, or cycle the route.
            </div>
            {wires.map((w) => (
              <div key={w.id} className="text-[10px] border border-slate-800 rounded px-2 py-1 text-slate-300 flex items-center justify-between gap-2">
                <span className="truncate">{w.fromNodeId} → {w.toNodeId}</span>
                <button type="button" onClick={() => removeWire(w.id)} className="text-red-400 hover:text-red-300">
                  <Trash2 size={11} />
                </button>
              </div>
            ))}
          </div>
        </>
      )}

      {activeTool === 'hold' && (
        <>
          <SectionHeader label="Hold" />
          <div className="p-3 flex flex-col gap-2">
            <div className="text-[10px] text-slate-500">Click a waypoint in canvas to add hold.</div>
            {holds.map((h) => (
              <div key={h.id} className="border border-slate-800 rounded p-2 text-[10px] text-slate-300">
                <div>{h.pathId} · wp {h.waypointIndex}</div>
                <div className="flex items-center justify-between mt-1">
                  <input
                    type="number"
                    step={0.5}
                    min={0}
                    value={h.duration}
                    onChange={(e) => updateHold(h.id, { duration: Math.max(0, Number(e.target.value)) })}
                    className="w-20 bg-black/40 border border-slate-700 rounded px-1.5 py-1 text-xs"
                  />
                  <button type="button" onClick={() => removeHold(h.id)} className="text-red-400 hover:text-red-300">
                    <Trash2 size={11} />
                  </button>
                </div>
              </div>
            ))}
          </div>
        </>
      )}

      {activeTool === 'obstacle' && (
        <>
          <SectionHeader label="Obstacle" />
          <div className="p-3 flex flex-col gap-2">
            <button
              type="button"
              onClick={() => addObstacle()}
              className="w-full py-2 rounded-lg border border-red-800 bg-red-900/20 text-red-100 text-xs font-semibold"
            >
              Add Sphere Obstacle
            </button>
            <div className="rounded-lg border border-slate-800 bg-slate-950/70 px-3 py-2 text-[10px] text-slate-300">
              New obstacles appear near the current authoring focus and can be dragged directly in the canvas.
            </div>
            {obstacles.map((o) => (
              <div
                key={o.id}
                className={`border rounded p-2 flex flex-col gap-1 ${
                  selectedAssemblyId && assembly.some((item) => item.id === selectedAssemblyId && item.obstacleId === o.id)
                    ? 'border-red-500/40 bg-red-950/10'
                    : 'border-slate-800'
                }`}
              >
                <NumberField label="X" value={o.position[0]} onChange={(v) => updateObstacle(o.id, { position: [v, o.position[1], o.position[2]] })} />
                <NumberField label="Y" value={o.position[1]} onChange={(v) => updateObstacle(o.id, { position: [o.position[0], v, o.position[2]] })} />
                <NumberField label="Z" value={o.position[2]} onChange={(v) => updateObstacle(o.id, { position: [o.position[0], o.position[1], v] })} />
                <NumberField label="Radius" value={o.radius} onChange={(v) => updateObstacle(o.id, { radius: Math.max(0.05, v) })} />
                <button type="button" onClick={() => removeObstacle(o.id)} className="text-[10px] text-red-400 hover:text-red-300 text-left">
                  Remove
                </button>
              </div>
            ))}
          </div>
        </>
      )}

      {activeTool === 'point' && (
        <>
          <SectionHeader label="Point" />
          <div className="p-3 flex flex-col gap-2">
            <button
              type="button"
              onClick={() => addPoint()}
              className="w-full py-2 rounded-lg border border-cyan-700 bg-cyan-900/20 text-cyan-100 text-xs font-semibold"
            >
              Add Point
            </button>
            <div className="rounded-lg border border-slate-800 bg-slate-950/70 px-3 py-2 text-[10px] text-slate-300">
              New guide points spawn from the current route context and become the active draggable point immediately.
            </div>
            {points.map((point) => (
              <div
                key={point.id}
                className={`border rounded p-2 flex flex-col gap-1 ${
                  selectedAssemblyId && assembly.some((item) => item.id === selectedAssemblyId && item.pointId === point.id)
                    ? 'border-cyan-500/40 bg-cyan-950/10'
                    : 'border-slate-800'
                }`}
              >
                <div className="text-[10px] text-slate-500">{point.id}</div>
                <NumberField
                  label="X"
                  value={point.position[0]}
                  onChange={(v) => updatePoint(point.id, { position: [v, point.position[1], point.position[2]] })}
                />
                <NumberField
                  label="Y"
                  value={point.position[1]}
                  onChange={(v) => updatePoint(point.id, { position: [point.position[0], v, point.position[2]] })}
                />
                <NumberField
                  label="Z"
                  value={point.position[2]}
                  onChange={(v) => updatePoint(point.id, { position: [point.position[0], point.position[1], v] })}
                />
                <button
                  type="button"
                  onClick={() => removePoint(point.id)}
                  className="text-[10px] text-red-400 hover:text-red-300 text-left"
                >
                  Remove
                </button>
              </div>
            ))}
          </div>
        </>
      )}

      {activeTool === 'edit' && (
        <>
          <SectionHeader label="Edit" />
          <div className="p-3 flex flex-col gap-2">
            <div className="grid grid-cols-2 gap-1">
              <button
                type="button"
                onClick={() => setEditMode('stretch')}
                className={`py-1.5 rounded-lg border text-[10px] font-bold transition-all ${
                  editMode === 'stretch'
                    ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                    : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                }`}
              >
                Edit Path
              </button>
              <button
                type="button"
                onClick={() => setEditMode('add')}
                className={`py-1.5 rounded-lg border text-[10px] font-bold transition-all ${
                  editMode === 'add'
                    ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                    : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                }`}
              >
                Add Waypoint
              </button>
              <button
                type="button"
                onClick={() => setEditMode('delete')}
                className={`py-1.5 rounded-lg border text-[10px] font-bold transition-all ${
                  editMode === 'delete'
                    ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                    : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                }`}
              >
                Delete Waypoint
              </button>
              <button
                type="button"
                onClick={() => setEditMode('density')}
                className={`py-1.5 rounded-lg border text-[10px] font-bold transition-all ${
                  editMode === 'density'
                    ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                    : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                }`}
              >
                Change Density
              </button>
            </div>
            <div className="text-[10px] text-slate-500">
              {editMode === 'stretch' && 'Click a waypoint and drag to locally stretch nearby waypoints.'}
              {editMode === 'add' && 'Click a location on the path to insert a waypoint there.'}
              {editMode === 'delete' && 'Click a waypoint to delete it (minimum 2 waypoints kept).'}
              {editMode === 'density' && 'Choose Total Path or Path Snippet, then adjust the density multiplier.'}
            </div>
            {selectedWire && (
              <div className="border border-slate-800 rounded p-2">
                <div className="text-[10px] text-slate-500 mb-1">Constraint Mode</div>
                <div className="grid grid-cols-2 gap-1">
                  <button
                    type="button"
                    onClick={() => setWireConstraintMode(selectedWire.id, 'constrained')}
                    className={`py-1 rounded-lg border text-[10px] font-bold transition-all ${
                      (selectedWire.constraintMode ?? 'constrained') === 'constrained'
                        ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                        : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                    }`}
                  >
                    Constrained
                  </button>
                  <button
                    type="button"
                    onClick={() => setWireConstraintMode(selectedWire.id, 'free')}
                    className={`py-1 rounded-lg border text-[10px] font-bold transition-all ${
                      (selectedWire.constraintMode ?? 'constrained') === 'free'
                        ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
                        : 'border-slate-700 text-slate-400 hover:border-cyan-700'
                    }`}
                  >
                    Free
                  </button>
                </div>
                <div className="text-[10px] text-slate-500 mt-2">
                  {(selectedWire.constraintMode ?? 'constrained') === 'constrained'
                    ? 'Maintains smooth endpoint tangency.'
                    : 'Allows lateral bend and arbitrary curvature.'}
                </div>
              </div>
            )}
            {selectedPath && editMode === 'density' ? (
              <div className="border border-slate-800 rounded p-2">
                <div className="grid grid-cols-2 gap-1 mb-2">
                  <button
                    type="button"
                    onClick={() => updatePath(selectedPath.id, { densityScope: 'total', densitySnippetRange: null })}
                    className={`py-1 rounded-lg border text-[10px] font-bold transition-all ${
                      (selectedPath.densityScope ?? 'total') === 'total'
                        ? 'border-amber-500 bg-amber-900/30 text-amber-100'
                        : 'border-slate-700 text-slate-400 hover:border-amber-700'
                    }`}
                  >
                    Total Path
                  </button>
                  <button
                    type="button"
                    onClick={() => updatePath(selectedPath.id, { densityScope: 'snippet' })}
                    className={`py-1 rounded-lg border text-[10px] font-bold transition-all ${
                      (selectedPath.densityScope ?? 'total') === 'snippet'
                        ? 'border-amber-500 bg-amber-900/30 text-amber-100'
                        : 'border-slate-700 text-slate-400 hover:border-amber-700'
                    }`}
                  >
                    Path Snippet
                  </button>
                </div>
                <div className="flex items-center justify-between text-[10px] text-slate-400 mb-1">
                  <span>Waypoint Density</span>
                  <span className="tabular-nums">{(selectedPath.waypointDensity ?? 1).toFixed(2)}x</span>
                </div>
                <input
                  type="range"
                  min={0.25}
                  max={25}
                  step={0.05}
                  value={selectedPath.waypointDensity ?? 1}
                  onChange={(e) => {
                    const v = Math.max(0.25, Math.min(25, Number(e.target.value) || 1));
                    if ((selectedPath.densityScope ?? 'total') === 'snippet') {
                      const range = selectedPath.densitySnippetRange;
                      if (!range || range[1] - range[0] < 1) return;
                      const snippet = resampleSnippet(selectedPath.waypoints, range[0], range[1], v);
                      updatePath(selectedPath.id, { waypointDensity: v, densitySnippetRange: snippet.newRange });
                      setPathWaypointsManual(selectedPath.id, snippet.points);
                      return;
                    }
                    updatePath(selectedPath.id, { waypointDensity: v, densitySnippetRange: null });
                    setPathWaypointsManual(selectedPath.id, resamplePolyline(selectedPath.waypoints, v));
                  }}
                  className="w-full accent-cyan-500"
                />
                <div className="flex justify-between text-[9px] text-slate-600 mt-1">
                  <span>0.25x</span>
                  <span>25x</span>
                </div>
                {(selectedPath.densityScope ?? 'total') === 'snippet' && (
                  <div className="text-[10px] text-amber-300 mt-2">
                    {selectedPath.densitySnippetRange && selectedPath.densitySnippetRange[1] - selectedPath.densitySnippetRange[0] >= 1
                      ? `Snippet selected: wp ${selectedPath.densitySnippetRange[0]} → ${selectedPath.densitySnippetRange[1]}`
                      : 'Click two waypoints on the path to define the snippet.'}
                  </div>
                )}
              </div>
            ) : editMode === 'density' ? (
              <div className="text-[10px] text-slate-600">Select a path waypoint to edit density.</div>
            ) : null}
          </div>
        </>
      )}

      <SectionHeader label="Model" />
      <div className="p-3 flex flex-col gap-2">
        <button
          type="button"
          onClick={() => fileRef.current?.click()}
          disabled={uploading}
          className="w-full flex items-center justify-center gap-2 px-3 py-2 rounded-lg border border-slate-700 text-xs font-semibold text-slate-300 hover:border-cyan-700 transition-all"
        >
          {uploading ? 'Uploading model...' : modelUrl ? 'Change Model' : 'Load OBJ Model'}
        </button>
        <input ref={fileRef} type="file" accept=".obj" className="hidden" onChange={handleLoadModel} />
        <div className="text-[10px] text-slate-500 px-1">Reference: {studioReferenceLabel(referenceObjectPath)}</div>
        {modelUrl && (
          <button
            type="button"
            onClick={() => {
              setReferenceObjectPath(null);
              setModelUrl(null);
            }}
            className="text-[10px] text-slate-500 hover:text-red-400 text-left px-1"
          >
            ✕ Use no object
          </button>
        )}
        {uploadError && <div className="text-[10px] text-red-400 px-1">{uploadError}</div>}
      </div>
    </div>
  );
}

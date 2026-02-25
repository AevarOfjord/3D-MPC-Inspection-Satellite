import { useRef, useState } from 'react';
import { Crosshair, Route, Link2, Pause, CircleDot, Trash2 } from 'lucide-react';
import { useStudioStore } from './useStudioStore';
import { useRegenerateWaypoints } from './useRegenerateWaypoints';
import { trajectoryApi } from '../../api/trajectory';
import { studioModelPathToUrl, studioReferenceLabel } from './studioReference';

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
  tool,
}: {
  icon: React.ReactNode;
  label: string;
  tool: NonNullable<ReturnType<typeof useStudioStore.getState>['activeTool']>;
}) {
  const activeTool = useStudioStore((s) => s.activeTool);
  const setActiveTool = useStudioStore((s) => s.setActiveTool);
  const active = activeTool === tool;
  return (
    <button
      type="button"
      onClick={() => setActiveTool(active ? null : tool)}
      className={`w-full flex items-center gap-2 px-3 py-2 rounded-lg border text-xs font-semibold transition-all ${
        active
          ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100'
          : 'border-slate-700 text-slate-300 hover:border-cyan-700'
      }`}
    >
      {icon}
      {label}
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
    wires,
    removeWire,
    holds,
    updateHold,
    removeHold,
    obstacles,
    addObstacle,
    updateObstacle,
    removeObstacle,
    modelUrl,
    referenceObjectPath,
    setModelUrl,
    setReferenceObjectPath,
  } = useStudioStore();

  const activeTool = useStudioStore((s) => s.activeTool);
  const regenerate = useRegenerateWaypoints();
  const fileRef = useRef<HTMLInputElement>(null);
  const [uploading, setUploading] = useState(false);
  const [uploadError, setUploadError] = useState<string | null>(null);
  const [axisSeed, setAxisSeed] = useState<'X' | 'Y' | 'Z'>('Z');

  const selectedPath = paths.find((p) => p.id === selectedPathId) ?? null;

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

  return (
    <div className="flex flex-col gap-0">
      <SectionHeader label="Add Segment" />
      <div className="p-3 flex flex-col gap-2">
        <ToolButton icon={<Crosshair size={13} />} label="Place Satellite" tool="place_satellite" />
        <ToolButton icon={<Route size={13} />} label="Create Path" tool="create_path" />
        <ToolButton icon={<Link2 size={13} />} label="Connect" tool="connect" />
        <ToolButton icon={<Pause size={13} />} label="Hold" tool="hold" />
        <ToolButton icon={<CircleDot size={13} />} label="Obstacle" tool="obstacle" />
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
              }}
              className="w-full py-2 rounded-lg border border-violet-700 bg-violet-900/30 text-violet-100 text-xs font-semibold"
            >
              Add Path
            </button>

            {selectedPath && (
              <>
                <div className="text-[10px] text-slate-500">Selected: {selectedPath.id}</div>
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
            <div className="text-[10px] text-slate-500">Drag from one endpoint node to another in canvas.</div>
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
              onClick={addObstacle}
              className="w-full py-2 rounded-lg border border-red-800 bg-red-900/20 text-red-100 text-xs font-semibold"
            >
              Add Sphere Obstacle
            </button>
            {obstacles.map((o) => (
              <div key={o.id} className="border border-slate-800 rounded p-2 flex flex-col gap-1">
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

      <SectionHeader label="Model" />
      <div className="p-3 flex flex-col gap-2">
        <button
          type="button"
          onClick={() => fileRef.current?.click()}
          disabled={uploading}
          className="w-full flex items-center justify-center gap-2 px-3 py-2 rounded-lg border border-slate-700 text-xs font-semibold text-slate-300 hover:border-cyan-700 transition-all"
        >
          {uploading ? 'Uploading model...' : modelUrl ? '⬛ Change Model' : '📂 Load OBJ Model'}
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

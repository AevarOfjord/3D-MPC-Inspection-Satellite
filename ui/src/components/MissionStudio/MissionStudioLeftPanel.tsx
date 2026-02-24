import { useRef } from 'react';
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
  const colorMap: Record<string, string> = {
    cyan: active ? 'border-cyan-600 bg-cyan-900/40 text-cyan-100' : 'border-slate-700 text-slate-300 hover:border-cyan-700',
    violet: active ? 'border-violet-600 bg-violet-900/40 text-violet-100' : 'border-slate-700 text-slate-300 hover:border-violet-700',
    amber: active ? 'border-amber-600 bg-amber-900/40 text-amber-100' : 'border-slate-700 text-slate-300 hover:border-amber-700',
    red: 'border-slate-700 text-slate-300 hover:border-red-700',
  };
  return (
    <button
      type="button"
      onClick={onClick}
      className={`w-full flex items-center gap-2 px-3 py-2 rounded-lg border text-xs font-semibold transition-all ${colorMap[color]}`}
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
          className="w-full flex items-center justify-center gap-2 px-3 py-2 rounded-lg border border-slate-700 text-xs font-semibold text-slate-300 hover:border-cyan-700 transition-all"
        >
          {modelUrl ? '⬛ Model Loaded' : '📂 Load OBJ Model'}
        </button>
        <input ref={fileRef} type="file" accept=".obj" className="hidden" onChange={handleLoadModel} />
        {modelUrl && (
          <button
            type="button"
            onClick={() => setModelUrl(null)}
            className="text-[10px] text-slate-500 hover:text-red-400 text-left px-1"
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
                  className={`flex-1 py-1.5 rounded-lg border text-xs font-bold transition-all ${
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

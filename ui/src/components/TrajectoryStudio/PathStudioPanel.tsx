import { useEffect, useState } from 'react';
import { RefreshCcw, Save, Link2 } from 'lucide-react';
import { HudPanel, HudSection, HudInput, HudButton } from '../HudComponents';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';

interface PathStudioPanelProps {
  builder: ReturnType<typeof useMissionBuilder>;
}

export function PathStudioPanel({ builder }: PathStudioPanelProps) {
  const { state, setters, actions } = builder;
  const [assetName, setAssetName] = useState('');
  const [selectedAssetId, setSelectedAssetId] = useState('');

  useEffect(() => {
    actions.refreshModelList().catch(() => null);
    actions.refreshPathAssets().catch(() => null);
    // Run once on mount.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const handleUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;
    await actions.handleFileUpload(file);
  };

  const handleSave = async () => {
    const saved = await actions.savePathAsset(assetName);
    if (saved) {
      setAssetName('');
    }
  };

  return (
    <HudPanel className="w-80 max-h-[calc(100vh-220px)] overflow-y-auto custom-scrollbar" title="Path Studio">
      <div className="space-y-3">
        <HudSection title="Model" defaultOpen>
          <div className="space-y-2">
            <select
              value={state.config.obj_path}
              onChange={(e) => actions.selectModelPath(e.target.value)}
              className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
            >
              <option value="">Select OBJ...</option>
              {state.availableModels.map((model) => (
                <option key={model.path} value={model.path}>
                  {model.filename}
                </option>
              ))}
            </select>
            <div className="flex items-center gap-2">
              <label className="flex-1 text-[10px] uppercase text-slate-500">Upload OBJ</label>
              <input
                type="file"
                accept=".obj"
                onChange={handleUpload}
                className="text-xs text-slate-400"
              />
            </div>
            {state.config.obj_path && (
              <div className="text-[10px] text-slate-500 font-mono break-all">
                {state.config.obj_path}
              </div>
            )}
          </div>
        </HudSection>

        <HudSection title="Scan Params" defaultOpen>
          <div className="mb-2">
            <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
              Level Mode
            </label>
            <div className="grid grid-cols-2 gap-2">
              <button
                onClick={() => setters.setUseLevelSpacing(false)}
                className={`text-xs py-1.5 rounded border transition-colors uppercase ${
                  !state.useLevelSpacing
                    ? 'bg-cyan-500/20 border-cyan-500 text-cyan-100'
                    : 'bg-slate-900/50 border-slate-700 text-slate-400 hover:bg-slate-800'
                }`}
              >
                Fixed Levels
              </button>
              <button
                onClick={() => setters.setUseLevelSpacing(true)}
                className={`text-xs py-1.5 rounded border transition-colors uppercase ${
                  state.useLevelSpacing
                    ? 'bg-cyan-500/20 border-cyan-500 text-cyan-100'
                    : 'bg-slate-900/50 border-slate-700 text-slate-400 hover:bg-slate-800'
                }`}
              >
                Level Spacing
              </button>
            </div>
            <div className="text-[10px] text-slate-500 mt-1">
              {state.useLevelSpacing
                ? 'Levels are auto-computed from spacing.'
                : 'Fixed levels are used for all axes.'}
            </div>
          </div>
          <div className="grid grid-cols-2 gap-2">
            <HudInput
              label="Standoff (m)"
              value={state.config.standoff}
              type="number"
              step={0.1}
              onChange={(val) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  standoff: Number.isFinite(val) ? val : prev.standoff,
                }))
              }
            />
            <HudInput
              label="Levels"
              value={state.config.levels}
              type="number"
              step={1}
              onChange={(val) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  levels: Number.isFinite(val) ? Math.max(1, Math.floor(val)) : prev.levels,
                }))
              }
            />
            <HudInput
              label="Level Spacing (m)"
              value={state.levelSpacing}
              type="number"
              step={0.1}
              onChange={(val) => setters.setLevelSpacing(Number.isFinite(val) ? val : state.levelSpacing)}
            />
            <HudInput
              label="Points / Ring"
              value={state.config.points_per_circle}
              type="number"
              step={1}
              onChange={(val) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  points_per_circle: Number.isFinite(val) ? Math.max(6, Math.floor(val)) : prev.points_per_circle,
                }))
              }
            />
            <HudInput
              label="Speed Max"
              value={state.config.speed_max}
              type="number"
              step={0.01}
              onChange={(val) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  speed_max: Number.isFinite(val) ? val : prev.speed_max,
                }))
              }
            />
            <HudInput
              label="Speed Min"
              value={state.config.speed_min}
              type="number"
              step={0.01}
              onChange={(val) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  speed_min: Number.isFinite(val) ? val : prev.speed_min,
                }))
              }
            />
            <HudInput
              label="Lat Accel"
              value={state.config.lateral_accel}
              type="number"
              step={0.01}
              onChange={(val) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  lateral_accel: Number.isFinite(val) ? val : prev.lateral_accel,
                }))
              }
            />
            <HudInput
              label="Z Margin"
              value={state.config.z_margin}
              type="number"
              step={0.05}
              onChange={(val) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  z_margin: Number.isFinite(val) ? val : prev.z_margin,
                }))
              }
            />
          </div>
          <div className="mt-2">
            <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
              Scan Axis
            </label>
            <select
              value={state.config.scan_axis}
              onChange={(e) =>
                setters.setConfig((prev) => ({
                  ...prev,
                  scan_axis: e.target.value as 'X' | 'Y' | 'Z',
                }))
              }
              className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
            >
              <option value="X">X</option>
              <option value="Y">Y</option>
              <option value="Z">Z</option>
            </select>
          </div>
        </HudSection>

        <div className="flex gap-2">
          <HudButton
            variant="secondary"
            size="sm"
            className="flex-1"
            onClick={() => actions.handlePreview()}
            disabled={state.loading || !state.config.obj_path}
          >
            <RefreshCcw size={12} className={state.loading ? 'animate-spin' : ''} />
            Preview
          </HudButton>
        </div>

        {state.stats && (
          <div className="text-[10px] text-slate-400 font-mono">
            <div>Points: {state.stats.points}</div>
            <div>Length: {state.stats.length.toFixed(2)} m</div>
            <div>Est. Duration: {state.stats.duration.toFixed(1)} s</div>
          </div>
        )}

        <HudSection title="Edit Path" defaultOpen={false}>
          <div className="flex gap-2">
            <HudButton
              variant="secondary"
              size="sm"
              className="flex-1"
              onClick={() => actions.addWaypoint()}
            >
              Add Waypoint
            </HudButton>
            <HudButton
              variant="danger"
              size="sm"
              className="flex-1"
              onClick={() => actions.removeWaypoint()}
              disabled={state.previewPath.length <= 2}
            >
              Remove Waypoint
            </HudButton>
          </div>
          <div className="text-[10px] text-slate-500 mt-2">
            Select a waypoint in the viewport to insert/remove near it.
          </div>
        </HudSection>

        <HudSection title="Save Path" defaultOpen={false}>
          <div className="space-y-2">
            <HudInput
              label="Path Name"
              value={assetName}
              onChange={(val) => setAssetName(String(val))}
            />
            <HudButton
              variant="primary"
              size="sm"
              className="w-full"
              onClick={handleSave}
              disabled={!assetName.trim() || state.previewPath.length === 0}
            >
              <Save size={12} /> Save Path Asset
            </HudButton>
          </div>
        </HudSection>

        <HudSection title="Library" defaultOpen={false}>
          <div className="space-y-2">
            <select
              value={selectedAssetId}
              onChange={(e) => setSelectedAssetId(e.target.value)}
              className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
            >
              <option value="">Select saved path…</option>
              {state.pathAssets.map((asset) => (
                <option key={asset.id} value={asset.id}>
                  {asset.name}
                </option>
              ))}
            </select>
            <div className="flex gap-2">
              <HudButton
                variant="secondary"
                size="sm"
                className="flex-1"
                onClick={() => selectedAssetId && actions.loadPathAsset(selectedAssetId)}
                disabled={!selectedAssetId}
              >
                Load
              </HudButton>
              <HudButton
                variant="primary"
                size="sm"
                className="flex-1"
                onClick={() => selectedAssetId && actions.applyPathAssetToSegment(selectedAssetId)}
                disabled={!selectedAssetId}
                icon={Link2}
              >
                Use on Scan
              </HudButton>
            </div>
            {selectedAssetId && (
              <div className="text-[10px] text-slate-500">
                Use the selected scan segment in the Generator Stack to attach this path.
              </div>
            )}
          </div>
        </HudSection>
      </div>
    </HudPanel>
  );
}

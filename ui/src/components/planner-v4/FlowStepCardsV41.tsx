import { useMemo, useState } from 'react';
import { ArrowLeftRight, Copy, Plus, Redo2, Save, Sparkles, Trash2, Undo2 } from 'lucide-react';

import { orbitSnapshot } from '../../data/orbitSnapshot';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { FieldRow } from '../ui-v4/FieldRow';
import { InlineBanner } from '../ui-v4/InlineBanner';
import { Panel } from '../ui-v4/Panel';
import { StatusPill } from '../ui-v4/StatusPill';

interface BaseCardProps {
  builder: ReturnType<typeof useMissionBuilder>;
}

function parseNum(raw: string, fallback: number): number {
  const parsed = Number.parseFloat(raw);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function complexityFromPoints(points: number): { label: 'Easy' | 'Medium' | 'Advanced'; tone: 'success' | 'warning' | 'danger' } {
  if (points <= 120) return { label: 'Easy', tone: 'success' };
  if (points <= 350) return { label: 'Medium', tone: 'warning' };
  return { label: 'Advanced', tone: 'danger' };
}

export function PathLibraryStepCardV41({ builder }: BaseCardProps) {
  const { state, actions, setters } = builder;
  const [sourceMode, setSourceMode] = useState<'library' | 'create' | 'duplicate'>('library');
  const [assetId, setAssetId] = useState('');
  const [duplicateName, setDuplicateName] = useState('copied_path_asset');
  const [newBakedName, setNewBakedName] = useState('scan_asset_v4_1');

  const selectedAsset = useMemo(
    () => state.pathAssets.find((asset) => asset.id === assetId) ?? null,
    [state.pathAssets, assetId]
  );
  const hasScanSegment = state.segments.some((segment) => segment.type === 'scan');
  const activeTargetId = state.selectedOrbitTargetId ?? state.startTargetId ?? '';

  return (
    <Panel
      title="Step 1 · Path Library"
      subtitle="Choose target object and path source"
      actions={<StatusPill tone="info">{state.pathAssets.length} Paths</StatusPill>}
    >
      <div id="coachmark-context_panel" className="space-y-3">
        <InlineBanner tone="info" title="What happens here">
          Select the target object and choose one path source. Then use the path on a scan segment.
        </InlineBanner>

        <FieldRow label="Target Object">
          <select
            className="v4-field"
            value={activeTargetId}
            onChange={(event) => {
              const targetId = event.target.value;
              if (!targetId) return;
              const target = orbitSnapshot.objects.find((obj) => obj.id === targetId);
              actions.assignScanTarget(targetId, target?.position_m as [number, number, number] | undefined);
              setters.setStartTargetId(targetId);
              actions.setSelectedOrbitTargetId(targetId);
            }}
          >
            <option value="">Select target...</option>
            {orbitSnapshot.objects.map((obj) => (
              <option key={obj.id} value={obj.id}>
                {obj.name}
              </option>
            ))}
          </select>
        </FieldRow>

        {!hasScanSegment ? (
          <InlineBanner
            tone="warning"
            title="Scan Segment Needed"
            actions={
              <button
                type="button"
                onClick={() => actions.addScanSegment()}
                className="v4-focus v4-button px-2 py-1.5 bg-amber-900/30 border-amber-700 text-amber-50"
              >
                Add Scan Segment
              </button>
            }
          >
            Add at least one scan segment so a path can be attached.
          </InlineBanner>
        ) : null}

        <div id="coachmark-templates" className="grid grid-cols-3 gap-2">
          <button
            type="button"
            onClick={() => setSourceMode('library')}
            className={`v4-focus v4-button px-2 py-2 ${
              sourceMode === 'library'
                ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
            }`}
          >
            Library Path
          </button>
          <button
            type="button"
            onClick={() => setSourceMode('create')}
            className={`v4-focus v4-button px-2 py-2 ${
              sourceMode === 'create'
                ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
            }`}
          >
            Create New
          </button>
          <button
            type="button"
            onClick={() => setSourceMode('duplicate')}
            className={`v4-focus v4-button px-2 py-2 ${
              sourceMode === 'duplicate'
                ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
            }`}
          >
            Duplicate
          </button>
        </div>

        <FieldRow label="Path Asset">
          <select
            className="v4-field"
            value={assetId}
            onChange={(event) => setAssetId(event.target.value)}
          >
            <option value="">Select saved path...</option>
            {state.pathAssets.map((asset) => (
              <option key={asset.id} value={asset.id}>
                {asset.name}
              </option>
            ))}
          </select>
        </FieldRow>

        {selectedAsset ? (
          <div className="v4-subtle-panel p-3 space-y-2">
            <div className="flex items-center justify-between gap-2">
              <div className="text-xs text-[color:var(--v4-text-1)]">{selectedAsset.name}</div>
              <StatusPill tone={complexityFromPoints(selectedAsset.points).tone}>
                {complexityFromPoints(selectedAsset.points).label}
              </StatusPill>
            </div>
            <div className="text-[11px] text-[color:var(--v4-text-3)]">
              points={selectedAsset.points}, length={selectedAsset.path_length.toFixed(1)}m
            </div>
          </div>
        ) : null}

        {sourceMode === 'library' ? (
          <button
            type="button"
            disabled={!assetId || !hasScanSegment}
            onClick={async () => {
              if (!assetId) return;
              await actions.loadPathAsset(assetId);
              actions.applyPathAssetToSegment(assetId);
            }}
            className="v4-focus v4-button w-full px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100"
          >
            Use This Path
          </button>
        ) : null}

        {sourceMode === 'create' ? (
          <div className="space-y-2">
            <InlineBanner tone="info" title="Create New Path">
              Preview the current scan project, then save it as a reusable path asset.
            </InlineBanner>
            <button
              type="button"
              onClick={() => void actions.previewScanProject(100)}
              className="v4-focus v4-button w-full px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
            >
              Preview Current Scan Path
            </button>
            <FieldRow label="New Asset Name">
              <input
                className="v4-field"
                value={newBakedName}
                onChange={(event) => setNewBakedName(event.target.value)}
              />
            </FieldRow>
            <button
              type="button"
              disabled={!newBakedName.trim()}
              onClick={async () => {
                const saved = await actions.saveBakedPathFromCompiled(newBakedName.trim());
                if (saved) {
                  setAssetId(saved.id);
                  setNewBakedName('');
                }
              }}
              className="v4-focus v4-button w-full px-3 py-2 bg-violet-900/35 border-violet-700 text-violet-100"
            >
              <Save size={12} /> Save New Path Asset
            </button>
          </div>
        ) : null}

        {sourceMode === 'duplicate' ? (
          <div className="space-y-2">
            <FieldRow label="Copy Name">
              <input
                className="v4-field"
                value={duplicateName}
                onChange={(event) => setDuplicateName(event.target.value)}
              />
            </FieldRow>
            <button
              type="button"
              disabled={!assetId || !duplicateName.trim()}
              onClick={async () => {
                if (!assetId) return;
                await actions.loadPathAsset(assetId);
                await actions.savePathAsset(duplicateName.trim());
                await actions.refreshPathAssets();
              }}
              className="v4-focus v4-button w-full px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
            >
              <Copy size={12} /> Duplicate Existing Path
            </button>
          </div>
        ) : null}
      </div>
    </Panel>
  );
}

export function StartTransferStepCardV41({ builder }: BaseCardProps) {
  const { state, actions, setters } = builder;
  const transferSegments = state.segments.filter((segment) => segment.type === 'transfer').length;
  const selectedTargetId = state.startTargetId ?? state.selectedOrbitTargetId ?? '';

  return (
    <Panel
      title="Step 2 · Start + Auto Transfer"
      subtitle="Place the satellite and generate transfer path"
      actions={<StatusPill tone="info">{transferSegments} Transfers</StatusPill>}
    >
      <div className="space-y-3">
        <InlineBanner tone="info" title="One-click transfer">
          Set start pose and click Generate Transfer to build a route to the selected scan path.
        </InlineBanner>

        <FieldRow label="Reference Frame">
          <div className="grid grid-cols-2 gap-2">
            {(['ECI', 'LVLH'] as const).map((frame) => (
              <button
                key={frame}
                type="button"
                onClick={() => {
                  setters.setStartFrame(frame);
                  if (frame === 'ECI') setters.setStartTargetId(undefined);
                }}
                className={`v4-focus v4-button px-2 py-1.5 ${
                  state.startFrame === frame
                    ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                    : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
                }`}
              >
                {frame}
              </button>
            ))}
          </div>
        </FieldRow>

        {state.startFrame === 'LVLH' ? (
          <FieldRow label="Relative To">
            <select
              className="v4-field"
              value={selectedTargetId}
              onChange={(event) => setters.setStartTargetId(event.target.value || undefined)}
            >
              <option value="">Select object...</option>
              {orbitSnapshot.objects.map((obj) => (
                <option key={obj.id} value={obj.id}>
                  {obj.name}
                </option>
              ))}
            </select>
          </FieldRow>
        ) : null}

        <FieldRow label="Start Position (m)">
          <div className="grid grid-cols-3 gap-2">
            {[0, 1, 2].map((index) => (
              <input
                key={index}
                className="v4-field"
                value={state.startPosition[index]}
                onChange={(event) => {
                  const next = [...state.startPosition] as [number, number, number];
                  next[index] = parseNum(event.target.value, state.startPosition[index]);
                  setters.setStartPosition(next);
                }}
              />
            ))}
          </div>
        </FieldRow>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => actions.addTransferSegment()}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            + Transfer Segment
          </button>
          <button
            type="button"
            onClick={() => void actions.generateUnifiedPath()}
            className="v4-focus v4-button px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100"
          >
            <Sparkles size={12} /> Generate Transfer
          </button>
        </div>

        {state.stats ? (
          <div className="v4-subtle-panel p-3 grid grid-cols-3 gap-2 text-xs">
            <div>
              <div className="text-[color:var(--v4-text-3)]">Path Length</div>
              <div className="text-[color:var(--v4-text-1)]">{state.stats.length.toFixed(1)} m</div>
            </div>
            <div>
              <div className="text-[color:var(--v4-text-3)]">ETA</div>
              <div className="text-[color:var(--v4-text-1)]">{state.stats.duration.toFixed(1)} s</div>
            </div>
            <div>
              <div className="text-[color:var(--v4-text-3)]">Path Points</div>
              <div className="text-[color:var(--v4-text-1)]">{state.stats.points}</div>
            </div>
          </div>
        ) : null}
      </div>
    </Panel>
  );
}

export function ObstaclesStepCardV41({ builder }: BaseCardProps) {
  const { state, actions } = builder;

  return (
    <Panel
      title="Step 3 · Obstacles"
      subtitle="Add obstacles and recompute around risk areas"
      actions={<StatusPill tone="info">{state.obstacles.length} Obstacles</StatusPill>}
    >
      <div className="space-y-3">
        <InlineBanner tone="info" title="Obstacle presets">
          Add Small / Medium / Large obstacles, then recompute to update the path.
        </InlineBanner>

        <div className="grid grid-cols-3 gap-2">
          <button
            type="button"
            onClick={() => actions.addObstacle(state.startPosition, [5, 0, 0], 0.3)}
            className="v4-focus v4-button px-2 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            Small
          </button>
          <button
            type="button"
            onClick={() => actions.addObstacle(state.startPosition, [6, 0, 0], 0.6)}
            className="v4-focus v4-button px-2 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            Medium
          </button>
          <button
            type="button"
            onClick={() => actions.addObstacle(state.startPosition, [7, 0, 0], 1.0)}
            className="v4-focus v4-button px-2 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            Large
          </button>
        </div>

        {state.obstacles.length === 0 ? (
          <InlineBanner tone="warning" title="No obstacles added">
            Add at least one obstacle if you need collision constraints on transfer/path.
          </InlineBanner>
        ) : (
          <div className="space-y-2 max-h-[18rem] overflow-y-auto custom-scrollbar pr-1">
            {state.obstacles.map((obstacle, index) => (
              <div
                key={`obstacle-${index}`}
                className="rounded-[10px] border border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)] px-3 py-2"
              >
                <div className="flex items-center justify-between">
                  <div className="text-xs text-[color:var(--v4-text-2)]">Obstacle {index + 1}</div>
                  <button
                    type="button"
                    onClick={() => actions.removeObstacle(index)}
                    className="v4-focus v4-button px-2 py-1 bg-red-900/35 border-red-700 text-red-100"
                  >
                    <Trash2 size={11} />
                  </button>
                </div>
                <div className="mt-2 grid grid-cols-4 gap-2">
                  {[0, 1, 2].map((axisIndex) => (
                    <input
                      key={axisIndex}
                      className="v4-field"
                      value={obstacle.position[axisIndex]}
                      onChange={(event) => {
                        const next = [...obstacle.position] as [number, number, number];
                        next[axisIndex] = parseNum(event.target.value, next[axisIndex]);
                        actions.updateObstacle(index, { position: next });
                      }}
                    />
                  ))}
                  <input
                    className="v4-field"
                    value={obstacle.radius}
                    onChange={(event) =>
                      actions.updateObstacle(index, {
                        radius: Math.max(0.05, parseNum(event.target.value, obstacle.radius)),
                      })
                    }
                  />
                </div>
              </div>
            ))}
          </div>
        )}

        <button
          type="button"
          onClick={() => void actions.generateUnifiedPath()}
          className="v4-focus v4-button w-full px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100"
        >
          <ArrowLeftRight size={12} /> Recompute Around Obstacles
        </button>
      </div>
    </Panel>
  );
}

export function PathEditStepCardV41({
  builder,
  onFinishEditing,
}: BaseCardProps & { onFinishEditing: () => void }) {
  const { state, actions } = builder;

  return (
    <Panel
      title="Step 4 · Path Edit"
      subtitle="Edit spline points directly in the viewport"
      actions={<StatusPill tone={state.isManualMode ? 'success' : 'warning'}>{state.isManualMode ? 'Manual' : 'Auto'}</StatusPill>}
    >
      <div id="coachmark-path_edit" className="space-y-3">
        <InlineBanner tone="info" title="Edit controls">
          Click a waypoint to select it. Drag to move. Shift/Alt+click to delete quickly.
        </InlineBanner>

        <div className="v4-subtle-panel p-3 grid grid-cols-3 gap-2 text-xs">
          <div>
            <div className="text-[color:var(--v4-text-3)]">Points</div>
            <div className="text-[color:var(--v4-text-1)]">{state.previewPath.length}</div>
          </div>
          <div>
            <div className="text-[color:var(--v4-text-3)]">Can Undo</div>
            <div className="text-[color:var(--v4-text-1)]">{state.canUndo ? 'Yes' : 'No'}</div>
          </div>
          <div>
            <div className="text-[color:var(--v4-text-3)]">Can Redo</div>
            <div className="text-[color:var(--v4-text-1)]">{state.canRedo ? 'Yes' : 'No'}</div>
          </div>
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => actions.addWaypoint()}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            <Plus size={12} /> Add Point
          </button>
          <button
            type="button"
            onClick={() => actions.removeWaypoint()}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            <Trash2 size={12} /> Delete Selected
          </button>
          <button
            type="button"
            disabled={!state.canUndo}
            onClick={() => actions.undo()}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)] disabled:opacity-50"
          >
            <Undo2 size={12} /> Undo
          </button>
          <button
            type="button"
            disabled={!state.canRedo}
            onClick={() => actions.redo()}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)] disabled:opacity-50"
          >
            <Redo2 size={12} /> Redo
          </button>
        </div>

        <button
          type="button"
          onClick={() => void actions.generateUnifiedPath()}
          className="v4-focus v4-button w-full px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100"
        >
          Recompute from Current Mission
        </button>
        <button
          type="button"
          onClick={onFinishEditing}
          className="v4-focus v4-button w-full px-3 py-2 bg-emerald-900/35 border-emerald-700 text-emerald-100"
        >
          Finish Editing
        </button>
      </div>
    </Panel>
  );
}

import { useEffect, useMemo, useState } from 'react';
import {
  Copy,
  Link2,
  Plus,
  Redo2,
  Save,
  Sparkles,
  Trash2,
  Undo2,
} from 'lucide-react';

import type { TransferSegment } from '../../api/unifiedMission';
import { ORBIT_SCALE, orbitSnapshot } from '../../data/orbitSnapshot';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { makeId } from '../../utils/scanProjectValidation';
import type { ScanDefinition } from '../../types/scanProject';
import { useCameraStore } from '../../store/cameraStore';
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

function gapMeters(scan: ScanDefinition): number {
  const dx = scan.plane_b[0] - scan.plane_a[0];
  const dy = scan.plane_b[1] - scan.plane_a[1];
  const dz = scan.plane_b[2] - scan.plane_a[2];
  return Math.hypot(dx, dy, dz);
}

function guessModelPathForTarget(
  targetId: string,
  availableModels: { name: string; filename: string; path: string }[]
): string | null {
  const query = targetId.trim().toLowerCase();
  if (!query) return null;
  const keywords = query.includes('starlink')
    ? ['starlink']
    : query.includes('iss')
      ? ['iss']
      : [query];
  for (const keyword of keywords) {
    const match = availableModels.find((model) => {
      const haystack = `${model.name} ${model.filename} ${model.path}`.toLowerCase();
      return haystack.includes(keyword);
    });
    if (match) return match.path;
  }
  return null;
}

type EndpointRow = {
  scanId: string;
  endpoint: 'start' | 'end';
  position: [number, number, number];
  label: string;
};

function flattenEndpoints(
  endpoints: Record<string, { start: [number, number, number]; end: [number, number, number] }> | undefined
): EndpointRow[] {
  if (!endpoints) return [];
  return Object.entries(endpoints).flatMap(([scanId, value]) => [
    {
      scanId,
      endpoint: 'start' as const,
      position: value.start,
      label: `${scanId} · Start`,
    },
    {
      scanId,
      endpoint: 'end' as const,
      position: value.end,
      label: `${scanId} · End`,
    },
  ]);
}

function resolveTransferEndpoint(
  builder: ReturnType<typeof useMissionBuilder>
): EndpointRow | null {
  const ref = builder.state.transferTargetRef;
  if (!ref) return null;
  const endpoint = builder.state.compilePreviewState?.endpoints?.[ref.scanId];
  if (!endpoint) return null;
  return {
    scanId: ref.scanId,
    endpoint: ref.endpoint,
    position: endpoint[ref.endpoint],
    label: `${ref.scanId} · ${ref.endpoint === 'start' ? 'Start' : 'End'}`,
  };
}

function summarizePathWarnings(
  path: [number, number, number][],
  obstacles: { position: [number, number, number]; radius: number }[]
) {
  let obstacleIntersections = 0;
  for (const point of path) {
    for (const obstacle of obstacles) {
      const dx = point[0] - obstacle.position[0];
      const dy = point[1] - obstacle.position[1];
      const dz = point[2] - obstacle.position[2];
      if (Math.hypot(dx, dy, dz) <= obstacle.radius) {
        obstacleIntersections += 1;
      }
    }
  }

  let extremeBends = 0;
  for (let i = 1; i < path.length - 1; i += 1) {
    const a = path[i - 1];
    const b = path[i];
    const c = path[i + 1];
    const v1: [number, number, number] = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const v2: [number, number, number] = [c[0] - b[0], c[1] - b[1], c[2] - b[2]];
    const n1 = Math.hypot(v1[0], v1[1], v1[2]);
    const n2 = Math.hypot(v2[0], v2[1], v2[2]);
    if (n1 < 1e-6 || n2 < 1e-6) continue;
    const cosValue = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (n1 * n2);
    const angleDeg = (Math.acos(Math.max(-1, Math.min(1, cosValue))) * 180) / Math.PI;
    if (angleDeg > 75) extremeBends += 1;
  }

  return { obstacleIntersections, extremeBends };
}

export function PathMakerStepCardV42({ builder }: BaseCardProps) {
  const { state, actions, setters } = builder;
  const activeTargetId = state.selectedOrbitTargetId ?? state.startTargetId ?? '';
  const selectedPair =
    state.scanProject.scans.find((scan) => scan.id === state.selectedScanId) ??
    state.scanProject.scans[0] ??
    null;

  const selectedEndpoints = useMemo(() => {
    if (!selectedPair) return [];
    const endpoints = state.compilePreviewState?.endpoints?.[selectedPair.id];
    if (!endpoints) return [];
    return [
      { key: 'start' as const, pos: endpoints.start },
      { key: 'end' as const, pos: endpoints.end },
    ];
  }, [state.compilePreviewState?.endpoints, selectedPair]);

  const duplicateSelectedPair = () => {
    if (!selectedPair) return;
    const duplicate: ScanDefinition = {
      ...selectedPair,
      id: makeId('scan'),
      name: `${selectedPair.name} Copy`,
      key_levels: selectedPair.key_levels.map((level) => ({
        ...level,
        id: makeId('kl'),
      })),
    };
    actions.updateScanProject((prev) => ({ ...prev, scans: [...prev.scans, duplicate] }));
    actions.setSelectedScanId(duplicate.id);
    actions.setSelectedKeyLevelId(duplicate.key_levels[0]?.id ?? null);
  };

  return (
    <Panel
      title="Step 1 · Path Maker"
      subtitle="Create panel pairs, shape spirals, and connect endpoints"
      actions={<StatusPill tone="info">{state.scanProject.scans.length} Pairs</StatusPill>}
    >
      <div id="coachmark-context_panel" className="space-y-3">
        <InlineBanner tone="info" title="Pair + Spiral workflow">
          Set axis + plane gap for each pair, shape the ellipse by dragging 4 handles in the viewport,
          then connect spiral endpoints.
        </InlineBanner>

        <FieldRow label="Target Object">
          <select
            className="v4-field"
            value={activeTargetId}
            onChange={(event) => {
              const targetId = event.target.value;
              if (!targetId) return;
              const target = orbitSnapshot.objects.find((obj) => obj.id === targetId);
              actions.assignScanTarget(
                targetId,
                target?.position_m as [number, number, number] | undefined
              );
              if (target?.position_m) {
                const targetPosition: [number, number, number] = [
                  target.position_m[0],
                  target.position_m[1],
                  target.position_m[2],
                ];
                setters.setReferencePosition([
                  targetPosition[0],
                  targetPosition[1],
                  targetPosition[2],
                ]);
                actions.updateScanProject((prev) => ({
                  ...prev,
                  scans: prev.scans.map((scan) => {
                    const center: [number, number, number] = [
                      0.5 * (scan.plane_a[0] + scan.plane_b[0]),
                      0.5 * (scan.plane_a[1] + scan.plane_b[1]),
                      0.5 * (scan.plane_a[2] + scan.plane_b[2]),
                    ];
                    const delta: [number, number, number] = [
                      targetPosition[0] - center[0],
                      targetPosition[1] - center[1],
                      targetPosition[2] - center[2],
                    ];
                    return {
                      ...scan,
                      plane_a: [
                        scan.plane_a[0] + delta[0],
                        scan.plane_a[1] + delta[1],
                        scan.plane_a[2] + delta[2],
                      ] as [number, number, number],
                      plane_b: [
                        scan.plane_b[0] + delta[0],
                        scan.plane_b[1] + delta[1],
                        scan.plane_b[2] + delta[2],
                      ] as [number, number, number],
                    };
                  }),
                }));
              }
              setters.setStartTargetId(targetId);
              actions.setSelectedOrbitTargetId(targetId);
              if (target?.position_m) {
                const focusDistance = target.real_span_m
                  ? Math.max(target.real_span_m * 5, 10)
                  : 20;
                // Planner viewport uses floating-origin scene coords.
                // Focusing [0,0,0] keeps camera pinned to the selected-object origin after recenter.
                useCameraStore.getState().requestFocus([0, 0, 0], focusDistance * ORBIT_SCALE);
              }
              if (!state.config.obj_path) {
                const suggestedPath = guessModelPathForTarget(targetId, state.availableModels);
                if (suggestedPath) {
                  actions.selectModelPath(suggestedPath);
                }
              }
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

        <FieldRow label="Object Model">
          <select
            className="v4-field"
            value={state.config.obj_path}
            onChange={(event) => actions.selectModelPath(event.target.value)}
          >
            <option value="">Select OBJ model...</option>
            {state.availableModels.map((model) => (
              <option key={model.path} value={model.path}>
                {model.name}
              </option>
            ))}
          </select>
        </FieldRow>

        {!state.config.obj_path ? (
          <InlineBanner tone="warning" title="Model required">
            Choose an OBJ model before building spiral preview.
          </InlineBanner>
        ) : null}

        <div className="grid grid-cols-3 gap-2">
          <button
            type="button"
            onClick={() => actions.addScan()}
            className="v4-focus v4-button px-2 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100"
          >
            <Plus size={12} /> Add Pair
          </button>
          <button
            type="button"
            onClick={duplicateSelectedPair}
            disabled={!selectedPair}
            className="v4-focus v4-button px-2 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)] disabled:opacity-40"
          >
            <Copy size={12} /> Duplicate
          </button>
          <button
            type="button"
            onClick={() => {
              if (!selectedPair) return;
              actions.removeScan(selectedPair.id);
            }}
            disabled={!selectedPair || state.scanProject.scans.length <= 1}
            className="v4-focus v4-button px-2 py-2 bg-red-900/35 border-red-700 text-red-100 disabled:opacity-40"
          >
            <Trash2 size={12} /> Remove
          </button>
        </div>

        <div className="space-y-2 max-h-[14rem] overflow-y-auto custom-scrollbar pr-1">
          {state.scanProject.scans.map((scan) => {
            const isSelected = scan.id === selectedPair?.id;
            return (
              <button
                key={scan.id}
                type="button"
                onClick={() => {
                  actions.setSelectedScanId(scan.id);
                  actions.setSelectedKeyLevelId(scan.key_levels[0]?.id ?? null);
                }}
                className={`v4-focus w-full rounded-[10px] border px-3 py-2 text-left ${
                  isSelected
                    ? 'border-cyan-500/85 bg-cyan-900/25'
                    : 'border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)]'
                }`}
              >
                <div className="text-xs font-semibold text-[color:var(--v4-text-1)]">{scan.name}</div>
                <div className="text-[11px] text-[color:var(--v4-text-3)]">
                  axis={scan.axis}, gap={gapMeters(scan).toFixed(2)}m
                </div>
              </button>
            );
          })}
        </div>

        {selectedPair ? (
          <>
            <FieldRow label="Pair Axis">
              <div className="grid grid-cols-3 gap-2">
                {(['X', 'Y', 'Z'] as const).map((axis) => (
                  <button
                    key={axis}
                    type="button"
                    onClick={() => actions.setScanAxisAligned(selectedPair.id, axis)}
                    className={`v4-focus v4-button px-2 py-1.5 ${
                      selectedPair.axis === axis
                        ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                        : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
                    }`}
                  >
                    {axis}
                  </button>
                ))}
              </div>
            </FieldRow>

            <div className="v4-subtle-panel p-3 space-y-2">
              <div className="text-xs text-[color:var(--v4-text-2)]">
                Plane gap: <span className="text-[color:var(--v4-text-1)]">{gapMeters(selectedPair).toFixed(2)} m</span>
              </div>
              <div className="text-[11px] text-[color:var(--v4-text-3)]">
                Spiral width/length is uniform across both planes. Drag one ellipse handle to update both ends.
              </div>
              <div className="grid grid-cols-2 gap-2">
                <button
                  type="button"
                  onClick={() =>
                    actions.setSelectedProjectScanPlaneHandle({
                      scanId: selectedPair.id,
                      handle: 'a',
                    })
                  }
                  className="v4-focus v4-button px-2 py-1.5 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
                >
                  Select Plane A
                </button>
                <button
                  type="button"
                  onClick={() =>
                    actions.setSelectedProjectScanPlaneHandle({
                      scanId: selectedPair.id,
                      handle: 'b',
                    })
                  }
                  className="v4-focus v4-button px-2 py-1.5 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
                >
                  Select Plane B
                </button>
              </div>
              <FieldRow label="Layer Height (m)">
                <input
                  type="number"
                  min={0.01}
                  step={0.01}
                  className="v4-field"
                  value={selectedPair.level_spacing_m ?? 0.1}
                  onChange={(event) => {
                    const next = Math.max(
                      0.01,
                      parseNum(event.target.value, selectedPair.level_spacing_m ?? 0.1)
                    );
                    actions.updateScan(selectedPair.id, {
                      level_spacing_m: next,
                      turns: undefined,
                    });
                  }}
                />
              </FieldRow>
              <div className="text-[11px] text-[color:var(--v4-text-3)]">
                Drag plane handles and the 4 ellipse handles directly in the viewport.
              </div>
            </div>

            <div className="v4-subtle-panel p-3 space-y-2">
              <div className="flex items-center justify-between">
                <div className="text-xs text-[color:var(--v4-text-2)]">Endpoint Connect</div>
                <button
                  type="button"
                  onClick={() =>
                    state.connectMode ? actions.cancelConnectMode() : actions.startConnectMode()
                  }
                  className={`v4-focus v4-button px-2 py-1.5 ${
                    state.connectMode
                      ? 'bg-amber-900/35 border-amber-700 text-amber-100'
                      : 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                  }`}
                >
                  <Link2 size={12} /> {state.connectMode ? 'Cancel Connect' : 'Connect Endpoints'}
                </button>
              </div>
              {state.connectSourceEndpoint ? (
                <div className="text-[11px] text-[color:var(--v4-text-3)]">
                  Source selected: {state.connectSourceEndpoint.scanId} ·{' '}
                  {state.connectSourceEndpoint.endpoint.toUpperCase()}
                </div>
              ) : (
                <div className="text-[11px] text-[color:var(--v4-text-3)]">
                  Turn on connect mode, then click two endpoints in the viewport.
                </div>
              )}
            </div>

            <button
              type="button"
              onClick={() => void actions.previewScanProject(80)}
              disabled={!state.config.obj_path}
              className="v4-focus v4-button w-full px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100 disabled:opacity-40"
            >
              <Sparkles size={12} /> Build Spiral Preview
            </button>

            {selectedEndpoints.length > 0 ? (
              <div className="v4-subtle-panel p-3 space-y-2">
                <div className="text-xs text-[color:var(--v4-text-2)]">Selected Pair Endpoints</div>
                {selectedEndpoints.map((row) => (
                  <div key={row.key} className="text-[11px] text-[color:var(--v4-text-3)]">
                    {row.key.toUpperCase()}: [{row.pos.map((v) => v.toFixed(2)).join(', ')}]
                  </div>
                ))}
              </div>
            ) : (
              <InlineBanner tone="warning" title="Endpoints not ready">
                Build spiral preview to expose endpoint targets for Step 2 transfer.
              </InlineBanner>
            )}
          </>
        ) : null}
      </div>
    </Panel>
  );
}

export function TransferStepCardV42({ builder }: BaseCardProps) {
  const { state, actions, setters } = builder;
  const [pendingGenerate, setPendingGenerate] = useState(false);
  const selectedTargetId = state.startTargetId ?? state.selectedOrbitTargetId ?? '';
  const endpoints = useMemo(
    () => flattenEndpoints(state.compilePreviewState?.endpoints),
    [state.compilePreviewState?.endpoints]
  );
  const selectedEndpoint = resolveTransferEndpoint(builder);

  useEffect(() => {
    if (!state.transferTargetRef) return;
    if (selectedEndpoint) return;
    actions.setTransferTargetRef(null);
  }, [state.transferTargetRef, selectedEndpoint, actions]);

  useEffect(() => {
    if (!pendingGenerate) return;
    setPendingGenerate(false);
    void actions.generateUnifiedPath();
  }, [pendingGenerate, actions, state.segments]);

  const ensureTransferToSelectedEndpoint = async () => {
    if (!selectedEndpoint) return;
    setters.setSegments((prev) => {
      const existing = prev.find((segment) => segment.type === 'transfer') as TransferSegment | undefined;
      const transfer: TransferSegment = existing
        ? {
            ...existing,
            target_id: existing.target_id ?? state.startTargetId ?? undefined,
            end_pose: {
              ...existing.end_pose,
              frame: existing.end_pose.frame ?? 'ECI',
              position: [...selectedEndpoint.position] as [number, number, number],
            },
          }
        : {
            segment_id: `transfer_${Date.now()}`,
            type: 'transfer',
            title: null,
            notes: null,
            target_id: state.startTargetId ?? undefined,
            end_pose: {
              frame: 'ECI',
              position: [...selectedEndpoint.position] as [number, number, number],
            },
            constraints: {
              speed_max: 0.25,
              accel_max: 0.05,
              angular_rate_max: 0.1,
            },
          };
      const nonTransfer = prev.filter((segment) => segment.type !== 'transfer');
      return [transfer, ...nonTransfer];
    });
    actions.selectSegment(0);

    const scanIndex = state.segments.findIndex((segment) => segment.type === 'scan');
    if (scanIndex >= 0) {
      const scanSegment = state.segments[scanIndex];
      if (scanSegment.type === 'scan') {
        const existingAssetId = scanSegment.path_asset ?? '';
        const shouldAutoBake =
          !existingAssetId || existingAssetId.startsWith('auto_compiled_');
        if (!shouldAutoBake) {
          setPendingGenerate(true);
          return;
        }
        const autoAssetName = existingAssetId || `auto_compiled_${scanSegment.target_id || 'scan'}`;
        const savedAsset = await actions.saveBakedPathFromCompiled(autoAssetName);
        if (savedAsset?.id) {
          actions.applyPathAssetToSegment(savedAsset.id);
        }
      }
    }

    setPendingGenerate(true);
  };

  return (
    <Panel
      title="Step 2 · Transfer"
      subtitle="Set start pose and connect to one spiral endpoint"
      actions={<StatusPill tone="info">{endpoints.length} Endpoints</StatusPill>}
    >
      <div className="space-y-3">
        <InlineBanner tone="info" title="Transfer target">
          Choose one endpoint from Step 1 and generate a transfer spline to that endpoint.
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

        {endpoints.length > 0 ? (
          <FieldRow label="Spline Endpoint">
            <select
              className="v4-field"
              value={
                state.transferTargetRef
                  ? `${state.transferTargetRef.scanId}:${state.transferTargetRef.endpoint}`
                  : ''
              }
              onChange={(event) => {
                const [scanId, endpointRaw] = event.target.value.split(':');
                if (!scanId || (endpointRaw !== 'start' && endpointRaw !== 'end')) {
                  actions.setTransferTargetRef(null);
                  return;
                }
                actions.setTransferTargetRef({ scanId, endpoint: endpointRaw });
              }}
            >
              <option value="">Select endpoint...</option>
              {endpoints.map((endpoint) => (
                <option
                  key={`${endpoint.scanId}:${endpoint.endpoint}`}
                  value={`${endpoint.scanId}:${endpoint.endpoint}`}
                >
                  {endpoint.label}
                </option>
              ))}
            </select>
          </FieldRow>
        ) : (
          <InlineBanner tone="warning" title="No endpoints yet">
            Return to Path Maker and click Build Spiral Preview first.
          </InlineBanner>
        )}

        {selectedEndpoint ? (
          <div className="v4-subtle-panel p-3 text-[11px] text-[color:var(--v4-text-3)]">
            Selected endpoint: {selectedEndpoint.label}
            <div className="mt-1">[{selectedEndpoint.position.map((v) => v.toFixed(2)).join(', ')}]</div>
          </div>
        ) : null}

        <button
          type="button"
          onClick={() => void ensureTransferToSelectedEndpoint()}
          disabled={!selectedEndpoint}
          className="v4-focus v4-button w-full px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100 disabled:opacity-40"
        >
          <Sparkles size={12} /> Generate Transfer
        </button>

        {state.stats ? (
          <div className="v4-subtle-panel p-3 grid grid-cols-3 gap-2 text-xs">
            <div>
              <div className="text-[color:var(--v4-text-3)]">Length</div>
              <div className="text-[color:var(--v4-text-1)]">{state.stats.length.toFixed(1)} m</div>
            </div>
            <div>
              <div className="text-[color:var(--v4-text-3)]">ETA</div>
              <div className="text-[color:var(--v4-text-1)]">{state.stats.duration.toFixed(1)} s</div>
            </div>
            <div>
              <div className="text-[color:var(--v4-text-3)]">Points</div>
              <div className="text-[color:var(--v4-text-1)]">{state.stats.points}</div>
            </div>
          </div>
        ) : null}
      </div>
    </Panel>
  );
}

export function ObstaclesStepCardV42({ builder }: BaseCardProps) {
  const { state, actions } = builder;

  return (
    <Panel
      title="Step 3 · Obstacles"
      subtitle="Add spherical obstacles for visual diagnostics only"
      actions={<StatusPill tone="info">{state.obstacles.length} Obstacles</StatusPill>}
    >
      <div className="space-y-3">
        <InlineBanner tone="info" title="Visual-only in V4.2">
          Obstacles do not auto-reroute paths. Place them here and fix collisions manually in Step 4.
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
            Add obstacles only where you need manual path-edit checks in Step 4.
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
      </div>
    </Panel>
  );
}

export function PathEditStepCardV42({
  builder,
  onFinishEditing,
}: BaseCardProps & { onFinishEditing: () => void }) {
  const { state, actions } = builder;
  const warnings = useMemo(
    () => summarizePathWarnings(state.previewPath, state.obstacles),
    [state.previewPath, state.obstacles]
  );
  const generatedPath = state.compilePreviewState?.combined_path ?? [];
  const manualDiffersFromGenerated =
    state.isManualMode &&
    generatedPath.length > 0 &&
    (generatedPath.length !== state.previewPath.length ||
      generatedPath.some((point, index) => {
        const current = state.previewPath[index];
        if (!current) return true;
        return (
          Math.abs(point[0] - current[0]) > 1e-6 ||
          Math.abs(point[1] - current[1]) > 1e-6 ||
          Math.abs(point[2] - current[2]) > 1e-6
        );
      }));

  return (
    <Panel
      title="Step 4 · Path Edit"
      subtitle="Drag spline points and manually resolve warnings"
      actions={
        <StatusPill tone={state.isManualMode ? 'success' : 'warning'}>
          {state.isManualMode ? 'Manual' : 'Auto'}
        </StatusPill>
      }
    >
      <div id="coachmark-path_edit" className="space-y-3">
        <InlineBanner tone="info" title="Spline editing">
          Click a waypoint to select it, drag points to deform the path, and use undo/redo freely.
        </InlineBanner>

        {manualDiffersFromGenerated ? (
          <InlineBanner tone="warning" title="Unsaved Manual Edits">
            Manual path differs from generated path. Save mission to preserve these edits.
          </InlineBanner>
        ) : null}

        {(warnings.obstacleIntersections > 0 || warnings.extremeBends > 0) ? (
          <div className="v4-subtle-panel p-3 grid grid-cols-2 gap-2 text-xs">
            <div>
              <div className="text-[color:var(--v4-text-3)]">Obstacle Intersections</div>
              <div className="text-amber-200">{warnings.obstacleIntersections}</div>
            </div>
            <div>
              <div className="text-[color:var(--v4-text-3)]">Extreme Bends</div>
              <div className="text-amber-200">{warnings.extremeBends}</div>
            </div>
          </div>
        ) : (
          <InlineBanner tone="success" title="No path warnings">
            Current path has no obstacle intersections or extreme bend warnings.
          </InlineBanner>
        )}

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
          onClick={onFinishEditing}
          className="v4-focus v4-button w-full px-3 py-2 bg-emerald-900/35 border-emerald-700 text-emerald-100"
        >
          Finish Editing
        </button>
      </div>
    </Panel>
  );
}

import { useEffect, useMemo, useRef, useState } from 'react';
import {
  Copy,
  Link2,
  Plus,
  Redo2,
  Save,
  Trash2,
  Undo2,
} from 'lucide-react';

import type { TransferSegment } from '../../api/unifiedMission';
import { ORBIT_SCALE, orbitSnapshot } from '../../data/orbitSnapshot';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import type { TransferTargetRef } from '../../types/plannerUx';
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
  if (!query || availableModels.length === 0) return null;

  const tokenSet = new Set<string>();
  tokenSet.add(query);
  for (const token of query.split(/[^a-z0-9]+/)) {
    if (token) tokenSet.add(token);
  }
  if (query.includes('starlink')) tokenSet.add('starlink');
  if (query.includes('iss')) tokenSet.add('iss');

  const keywords = Array.from(tokenSet).sort((a, b) => b.length - a.length);
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
  return resolveEndpointFromRef(builder.state.transferTargetRef, builder.state.compilePreviewState?.endpoints);
}

function resolveEndpointFromRef(
  ref: TransferTargetRef,
  endpoints: Record<string, { start: [number, number, number]; end: [number, number, number] }> | undefined
): EndpointRow | null {
  if (!ref || !endpoints) return null;
  const endpoint = endpoints[ref.scanId];
  if (!endpoint) return null;
  return {
    scanId: ref.scanId,
    endpoint: ref.endpoint,
    position: endpoint[ref.endpoint],
    label: `${ref.scanId} · ${ref.endpoint === 'start' ? 'Start' : 'End'}`,
  };
}

function resolveCoreTransferIndex(
  segments: ReturnType<typeof useMissionBuilder>['state']['segments']
): number | null {
  const titled = segments.findIndex(
    (segment) => segment.type === 'transfer' && segment.title === 'Transfer To Path'
  );
  if (titled >= 0) return titled;
  const firstTransfer = segments.findIndex((segment) => segment.type === 'transfer');
  return firstTransfer >= 0 ? firstTransfer : null;
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
  const canConnectEndpoints = state.scanProject.scans.length > 1;
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

  useEffect(() => {
    if (!activeTargetId || state.availableModels.length === 0) return;
    const suggestedPath = guessModelPathForTarget(activeTargetId, state.availableModels);
    if (!suggestedPath) return;
    if (state.config.obj_path === suggestedPath) return;
    actions.selectModelPath(suggestedPath);
  }, [
    activeTargetId,
    state.availableModels,
    state.config.obj_path,
    actions,
  ]);

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
              const suggestedPath = guessModelPathForTarget(targetId, state.availableModels);
              if (suggestedPath) {
                actions.selectModelPath(suggestedPath);
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

        {!state.config.obj_path ? (
          <InlineBanner tone="warning" title="Model not mapped">
            No OBJ model is mapped to this target yet. Add a matching model file/name for this target.
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

            {canConnectEndpoints ? (
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
            ) : null}

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
              <InlineBanner tone={state.compilePending ? 'info' : 'warning'} title={state.compilePending ? 'Building endpoints' : 'Endpoints not ready'}>
                Endpoints are generated automatically when the path changes.
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
  const coreAutoSignatureRef = useRef<string | null>(null);
  const optionalEndpointAutoSignatureRef = useRef<string | null>(null);
  const optionalManualAutoSignatureRef = useRef<string | null>(null);
  const optionalManualTimerRef = useRef<number | null>(null);
  const [optionalTransferSourceRefs, setOptionalTransferSourceRefs] = useState<
    Record<string, TransferTargetRef>
  >({});
  const [optionalTransferDestinationMode, setOptionalTransferDestinationMode] = useState<
    Record<string, 'endpoint' | 'manual'>
  >({});
  const composerSelection = state.selectedSegmentIndex;
  const selectedSegmentIndex =
    composerSelection !== null && composerSelection >= 0 && composerSelection < state.segments.length
      ? composerSelection
      : null;
  const selectedSegment = selectedSegmentIndex !== null ? state.segments[selectedSegmentIndex] : null;
  const selectedType =
    composerSelection === -1 ? 'start' : selectedSegment ? selectedSegment.type : null;
  const coreTransferIndex = resolveCoreTransferIndex(state.segments);
  const selectedTransferIndex =
    selectedSegment?.type === 'transfer' && selectedSegmentIndex !== null ? selectedSegmentIndex : null;
  const activeTransferIndex = selectedTransferIndex ?? coreTransferIndex;
  const activeTransferSegment =
    activeTransferIndex !== null && state.segments[activeTransferIndex]?.type === 'transfer'
      ? (state.segments[activeTransferIndex] as TransferSegment)
      : null;
  const selectedTransferSegment =
    selectedType === 'transfer' && selectedSegment?.type === 'transfer' ? selectedSegment : null;
  const selectedTransferSegmentId = selectedTransferSegment?.segment_id ?? null;
  const isCoreTransferSelected =
    selectedType === 'transfer' && selectedSegmentIndex !== null && selectedSegmentIndex === coreTransferIndex;
  const startTargetId = state.startTargetId ?? state.selectedOrbitTargetId ?? '';
  const transferTargetId = selectedTransferSegment?.target_id ?? startTargetId;
  const selectedTarget = useMemo(
    () => orbitSnapshot.objects.find((obj) => obj.id === transferTargetId),
    [transferTargetId]
  );
  const endpoints = useMemo(
    () => flattenEndpoints(state.compilePreviewState?.endpoints),
    [state.compilePreviewState?.endpoints]
  );
  const selectedEndpoint = resolveTransferEndpoint(builder);
  const selectedOptionalSourceRef =
    selectedTransferSegmentId ? optionalTransferSourceRefs[selectedTransferSegmentId] ?? null : null;
  const selectedOptionalSourceEndpoint = useMemo(
    () => resolveEndpointFromRef(selectedOptionalSourceRef, state.compilePreviewState?.endpoints),
    [selectedOptionalSourceRef, state.compilePreviewState?.endpoints]
  );
  const optionalDestinationMode =
    selectedTransferSegmentId
      ? optionalTransferDestinationMode[selectedTransferSegmentId] ?? 'endpoint'
      : 'endpoint';
  const selectedEndpointRelative = useMemo(() => {
    if (!selectedEndpoint || !selectedTarget) return null;
    return [
      selectedEndpoint.position[0] - selectedTarget.position_m[0],
      selectedEndpoint.position[1] - selectedTarget.position_m[1],
      selectedEndpoint.position[2] - selectedTarget.position_m[2],
    ] as [number, number, number];
  }, [selectedEndpoint, selectedTarget]);
  const setTransferTargetRef = actions.setTransferTargetRef;
  const generateUnifiedPath = actions.generateUnifiedPath;
  const selectSegment = actions.selectSegment;
  const saveBakedPathFromCompiled = actions.saveBakedPathFromCompiled;
  const applyPathAssetToSegment = actions.applyPathAssetToSegment;
  const setStartFrame = setters.setStartFrame;
  const setStartTargetId = setters.setStartTargetId;
  const setSegments = setters.setSegments;

  const updateActiveTransfer = (patch: Partial<TransferSegment>) => {
    if (activeTransferIndex === null || !activeTransferSegment) return;
    actions.updateSegment(activeTransferIndex, {
      ...activeTransferSegment,
      ...patch,
    });
  };

  async function ensureTransferToSelectedEndpoint(targetId: string) {
    if (!selectedEndpoint || !selectedEndpointRelative || !targetId) return;
    let nextTransferIndex: number | null = null;
    setSegments((prev) => {
      const preferredIndex =
        selectedTransferIndex !== null && prev[selectedTransferIndex]?.type === 'transfer'
          ? selectedTransferIndex
          : prev.findIndex((segment) => segment.type === 'transfer');
      const existing =
        preferredIndex >= 0 && prev[preferredIndex]?.type === 'transfer'
          ? (prev[preferredIndex] as TransferSegment)
          : undefined;
      const transfer: TransferSegment = existing
        ? {
            ...existing,
            target_id: targetId,
            end_pose: {
              ...existing.end_pose,
              frame: 'LVLH',
              position: [...selectedEndpointRelative] as [number, number, number],
            },
          }
        : {
            segment_id: `transfer_${Date.now()}`,
            type: 'transfer',
            title: 'Transfer To Path',
            notes: null,
            target_id: targetId,
            end_pose: {
              frame: 'LVLH',
              position: [...selectedEndpointRelative] as [number, number, number],
            },
            constraints: {
              speed_max: 0.25,
              accel_max: 0.05,
              angular_rate_max: 0.1,
            },
          };
      if (preferredIndex >= 0) {
        nextTransferIndex = preferredIndex;
        return prev.map((segment, index) => (index === preferredIndex ? transfer : segment));
      }
      nextTransferIndex = 0;
      const nonTransfer = prev.filter((segment) => segment.type !== 'transfer');
      return [transfer, ...nonTransfer];
    });
    selectSegment(nextTransferIndex ?? 0);

    const scanIndex =
      selectedSegment?.type === 'scan' && selectedSegmentIndex !== null
        ? selectedSegmentIndex
        : state.segments.findIndex((segment) => segment.type === 'scan');
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
        const savedAsset = await saveBakedPathFromCompiled(autoAssetName);
        if (savedAsset?.id) {
          applyPathAssetToSegment(savedAsset.id);
        }
      }
    }

    setPendingGenerate(true);
  }

  useEffect(() => {
    if (state.startFrame !== 'LVLH') {
      setStartFrame('LVLH');
    }
    if (!state.startTargetId && state.selectedOrbitTargetId) {
      setStartTargetId(state.selectedOrbitTargetId);
    }
  }, [
    state.startFrame,
    state.startTargetId,
    state.selectedOrbitTargetId,
    setStartFrame,
    setStartTargetId,
  ]);

  useEffect(() => {
    if (!state.transferTargetRef) return;
    if (selectedEndpoint) return;
    setTransferTargetRef(null);
  }, [state.transferTargetRef, selectedEndpoint, setTransferTargetRef]);

  useEffect(() => {
    if (!pendingGenerate) return;
    setPendingGenerate(false);
    void generateUnifiedPath();
  }, [pendingGenerate, generateUnifiedPath]);

  useEffect(() => {
    if (!isCoreTransferSelected) return;
    if (!selectedEndpoint || !selectedEndpointRelative || !startTargetId) return;
    const signature = [
      selectedEndpoint.scanId,
      selectedEndpoint.endpoint,
      startTargetId,
      state.startPosition[0],
      state.startPosition[1],
      state.startPosition[2],
    ].join('|');
    if (coreAutoSignatureRef.current === signature) return;
    coreAutoSignatureRef.current = signature;
    void ensureTransferToSelectedEndpoint(startTargetId);
  }, [
    isCoreTransferSelected,
    selectedEndpoint,
    selectedEndpointRelative,
    startTargetId,
    state.startPosition,
  ]);

  useEffect(() => {
    if (selectedType !== 'transfer' || isCoreTransferSelected) return;
    if (optionalDestinationMode !== 'endpoint') return;
    if (!selectedTransferSegmentId || !selectedEndpoint || !selectedEndpointRelative || !transferTargetId) {
      return;
    }
    const signature = [
      selectedTransferSegmentId,
      transferTargetId,
      selectedEndpoint.scanId,
      selectedEndpoint.endpoint,
    ].join('|');
    if (optionalEndpointAutoSignatureRef.current === signature) return;
    optionalEndpointAutoSignatureRef.current = signature;
    void ensureTransferToSelectedEndpoint(transferTargetId);
  }, [
    selectedType,
    isCoreTransferSelected,
    optionalDestinationMode,
    selectedTransferSegmentId,
    selectedEndpoint,
    selectedEndpointRelative,
    transferTargetId,
  ]);

  useEffect(() => {
    if (selectedType !== 'transfer' || isCoreTransferSelected) return;
    if (optionalDestinationMode !== 'manual') return;
    if (!selectedTransferSegmentId || !transferTargetId || !activeTransferSegment) return;
    const signature = [
      selectedTransferSegmentId,
      transferTargetId,
      activeTransferSegment.end_pose.position[0],
      activeTransferSegment.end_pose.position[1],
      activeTransferSegment.end_pose.position[2],
    ].join('|');
    if (optionalManualAutoSignatureRef.current === signature) return;
    optionalManualAutoSignatureRef.current = signature;
    if (optionalManualTimerRef.current !== null) {
      window.clearTimeout(optionalManualTimerRef.current);
    }
    optionalManualTimerRef.current = window.setTimeout(() => {
      setPendingGenerate(true);
    }, 220);
    return () => {
      if (optionalManualTimerRef.current !== null) {
        window.clearTimeout(optionalManualTimerRef.current);
        optionalManualTimerRef.current = null;
      }
    };
  }, [
    selectedType,
    isCoreTransferSelected,
    optionalDestinationMode,
    selectedTransferSegmentId,
    transferTargetId,
    activeTransferSegment,
  ]);

  const setOptionalSourceFromSelectedEndpoint = () => {
    if (!selectedTransferSegmentId || !state.transferTargetRef) return;
    setOptionalTransferSourceRefs((prev) => ({
      ...prev,
      [selectedTransferSegmentId]: state.transferTargetRef,
    }));
  };

  const setOptionalDestinationMode = (mode: 'endpoint' | 'manual') => {
    if (!selectedTransferSegmentId) return;
    setOptionalTransferDestinationMode((prev) => ({
      ...prev,
      [selectedTransferSegmentId]: mode,
    }));
  };

  return (
    <Panel
      title="Step 2 · Transfer"
      subtitle="Set start pose and connect to one spiral endpoint"
      actions={<StatusPill tone="info">{endpoints.length} Endpoints</StatusPill>}
    >
      <div className="space-y-3">
        <InlineBanner tone="info" title="Segment Composer selection">
          {selectedType === 'start'
            ? 'Editing Start.'
            : selectedType
            ? `Editing ${selectedType.toUpperCase()} segment #${(selectedSegmentIndex ?? 0) + 1}.`
            : 'Select Start or a segment in Segment Composer.'}
        </InlineBanner>

        {selectedType === 'start' ? (
          <>
            <FieldRow label="Relative To">
              <select
                className="v4-field"
                value={startTargetId}
                onChange={(event) => setStartTargetId(event.target.value || undefined)}
              >
                <option value="">Select object...</option>
                {orbitSnapshot.objects.map((obj) => (
                  <option key={obj.id} value={obj.id}>
                    {obj.name}
                  </option>
                ))}
              </select>
            </FieldRow>

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
          </>
        ) : null}

        {selectedType === 'transfer' ? (
          <>
            {isCoreTransferSelected ? (
              <>
                <InlineBanner tone="info" title="Transfer To Path">
                  This transfer always starts from the Start segment position. Select an endpoint to transfer to.
                </InlineBanner>
                <div className="text-[11px] text-[color:var(--v4-text-3)]">
                  Uses Start: LVLH @ {startTargetId || 'Select target in Start'} ·
                  [{state.startPosition.map((v) => v.toFixed(1)).join(', ')}]
                </div>

                {endpoints.length === 0 ? (
                  <InlineBanner tone="warning" title="No endpoints yet">
                    Endpoints are generated automatically in Step 1 when the path updates.
                  </InlineBanner>
                ) : (
                  <InlineBanner tone="info" title="Endpoint required">
                    Click a start/end endpoint marker in the viewport to auto-update transfer.
                  </InlineBanner>
                )}

                {selectedEndpoint ? (
                  <div className="v4-subtle-panel p-3 text-[11px] text-[color:var(--v4-text-3)]">
                    Selected endpoint: {selectedEndpoint.label}
                    <div className="mt-1">
                      {selectedEndpointRelative
                        ? `[${selectedEndpointRelative.map((v) => v.toFixed(2)).join(', ')}]`
                        : 'Select a target object to compute LVLH-relative endpoint.'}
                    </div>
                    {activeTransferSegment ? (
                      <div className="mt-1 text-[color:var(--v4-text-3)]">
                        Editing transfer segment #{(activeTransferIndex ?? 0) + 1}
                      </div>
                    ) : null}
                    <div className="mt-2">
                      <button
                        type="button"
                        onClick={() => setTransferTargetRef(null)}
                        className="v4-focus v4-button px-2 py-1 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
                      >
                        Clear Endpoint
                      </button>
                    </div>
                  </div>
                ) : null}

                <div className="text-[11px] text-[color:var(--v4-text-3)]">
                  Transfer path updates automatically after endpoint selection.
                </div>
              </>
            ) : (
              <>
                <InlineBanner tone="info" title="Transfer">
                  Select a start endpoint, then set destination by endpoint selection or manual XYZ.
                </InlineBanner>

                <FieldRow label="Relative To">
                  <select
                    className="v4-field"
                    value={transferTargetId}
                    onChange={(event) => {
                      const nextTarget = event.target.value || undefined;
                      updateActiveTransfer({ target_id: nextTarget });
                    }}
                  >
                    <option value="">Select object...</option>
                    {orbitSnapshot.objects.map((obj) => (
                      <option key={obj.id} value={obj.id}>
                        {obj.name}
                      </option>
                    ))}
                  </select>
                </FieldRow>

                <div className="v4-subtle-panel p-3 space-y-2">
                  <div className="text-xs text-[color:var(--v4-text-2)]">Start Endpoint</div>
                  <button
                    type="button"
                    onClick={setOptionalSourceFromSelectedEndpoint}
                    disabled={!state.transferTargetRef}
                    className="v4-focus v4-button w-full px-2 py-1.5 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)] disabled:opacity-40"
                  >
                    Use Selected Endpoint as Start
                  </button>
                  <div className="text-[11px] text-[color:var(--v4-text-3)]">
                    {selectedOptionalSourceEndpoint
                      ? `${selectedOptionalSourceEndpoint.label} [${selectedOptionalSourceEndpoint.position
                          .map((value) => value.toFixed(2))
                          .join(', ')}]`
                      : 'Click an endpoint in the viewport, then assign it as start.'}
                  </div>
                </div>

                <FieldRow label="Destination Mode">
                  <div className="grid grid-cols-2 gap-2">
                    <button
                      type="button"
                      onClick={() => setOptionalDestinationMode('endpoint')}
                      className={`v4-focus v4-button px-2 py-1.5 ${
                        optionalDestinationMode === 'endpoint'
                          ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                          : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
                      }`}
                    >
                      Endpoint
                    </button>
                    <button
                      type="button"
                      onClick={() => setOptionalDestinationMode('manual')}
                      className={`v4-focus v4-button px-2 py-1.5 ${
                        optionalDestinationMode === 'manual'
                          ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                          : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
                      }`}
                    >
                      Manual XYZ
                    </button>
                  </div>
                </FieldRow>

                {optionalDestinationMode === 'endpoint' ? (
                  <div className="v4-subtle-panel p-3 space-y-2">
                    <div className="text-[11px] text-[color:var(--v4-text-3)]">
                      {selectedEndpoint
                        ? `${selectedEndpoint.label} ${
                            selectedEndpointRelative
                              ? `[${selectedEndpointRelative.map((v) => v.toFixed(2)).join(', ')}]`
                              : ''
                          }`
                        : 'Click an endpoint in the viewport to choose destination.'}
                    </div>
                    <div className="text-[11px] text-[color:var(--v4-text-3)]">
                      Path updates automatically after destination selection.
                    </div>
                  </div>
                ) : (
                  <FieldRow label="End Position (m)">
                    <div className="grid grid-cols-3 gap-2">
                      {[0, 1, 2].map((index) => (
                        <input
                          key={index}
                          className="v4-field"
                          value={activeTransferSegment?.end_pose.position[index] ?? 0}
                          onChange={(event) => {
                            if (!activeTransferSegment) return;
                            const next = [...activeTransferSegment.end_pose.position] as [number, number, number];
                            next[index] = parseNum(event.target.value, activeTransferSegment.end_pose.position[index]);
                            updateActiveTransfer({
                              end_pose: {
                                ...activeTransferSegment.end_pose,
                                frame: 'LVLH',
                                position: next,
                              },
                            });
                          }}
                        />
                      ))}
                    </div>
                  </FieldRow>
                )}
                <div className="text-[11px] text-[color:var(--v4-text-3)]">
                  Path updates automatically when destination settings change.
                </div>
              </>
            )}
          </>
        ) : null}

        {selectedType === 'scan' ? (
          <>
            <InlineBanner tone="warning" title="Scan is edited in Step 1">
              To edit scan path go back to step 1.
            </InlineBanner>
            <button
              type="button"
              onClick={() => actions.setAuthoringStep('scan_definition')}
              className="v4-focus v4-button w-full px-3 py-2 bg-violet-900/35 border-violet-700 text-violet-100"
            >
              Go to Step 1 · Path Maker
            </button>
          </>
        ) : null}

        {selectedType === 'hold' && selectedSegmentIndex !== null && selectedSegment?.type === 'hold' ? (
          <FieldRow label="Duration (s)">
            <input
              className="v4-field"
              value={selectedSegment.duration}
              onChange={(event) => {
                actions.updateSegment(selectedSegmentIndex, {
                  ...selectedSegment,
                  duration: Math.max(0.1, parseNum(event.target.value, selectedSegment.duration)),
                });
              }}
            />
          </FieldRow>
        ) : null}

        {!selectedType ? (
          <InlineBanner tone="warning" title="No selection">
            Select Start or a segment in Segment Composer to edit it here.
          </InlineBanner>
        ) : null}

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
  const defaultObstacleTargetId =
    state.startTargetId ?? state.selectedOrbitTargetId ?? orbitSnapshot.objects[0]?.id ?? '';
  const [obstacleTargetId, setObstacleTargetId] = useState<string>(defaultObstacleTargetId);
  const [obstaclePosition, setObstaclePosition] = useState<[number, number, number]>([0, 0, 0]);
  const [obstacleRadius, setObstacleRadius] = useState<number>(0.5);

  useEffect(() => {
    if (obstacleTargetId) return;
    if (!defaultObstacleTargetId) return;
    setObstacleTargetId(defaultObstacleTargetId);
  }, [obstacleTargetId, defaultObstacleTargetId]);

  const addObstacleFromForm = () => {
    const missionOriginTargetId = state.startTargetId ?? state.selectedOrbitTargetId ?? '';
    const missionOrigin =
      orbitSnapshot.objects.find((obj) => obj.id === missionOriginTargetId)?.position_m ?? [0, 0, 0];
    const selectedOrigin =
      orbitSnapshot.objects.find((obj) => obj.id === obstacleTargetId)?.position_m ?? missionOrigin;
    const translatedPosition: [number, number, number] = [
      obstaclePosition[0] + (selectedOrigin[0] - missionOrigin[0]),
      obstaclePosition[1] + (selectedOrigin[1] - missionOrigin[1]),
      obstaclePosition[2] + (selectedOrigin[2] - missionOrigin[2]),
    ];
    actions.addObstacle(undefined, translatedPosition, Math.max(0.05, obstacleRadius));
  };

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

        <div className="v4-subtle-panel p-3 space-y-3">
          <FieldRow label="Relative To">
            <select
              className="v4-field"
              value={obstacleTargetId}
              onChange={(event) => setObstacleTargetId(event.target.value)}
            >
              <option value="">Select object...</option>
              {orbitSnapshot.objects.map((obj) => (
                <option key={obj.id} value={obj.id}>
                  {obj.name}
                </option>
              ))}
            </select>
          </FieldRow>

          <FieldRow label="3D Position (m)">
            <div className="grid grid-cols-3 gap-2">
              {[0, 1, 2].map((axisIndex) => (
                <input
                  key={axisIndex}
                  className="v4-field"
                  value={obstaclePosition[axisIndex]}
                  onChange={(event) => {
                    const next = [...obstaclePosition] as [number, number, number];
                    next[axisIndex] = parseNum(event.target.value, next[axisIndex]);
                    setObstaclePosition(next);
                  }}
                />
              ))}
            </div>
          </FieldRow>

          <FieldRow label="Sphere Radius (m)">
            <input
              className="v4-field"
              value={obstacleRadius}
              onChange={(event) =>
                setObstacleRadius(Math.max(0.05, parseNum(event.target.value, obstacleRadius)))
              }
            />
          </FieldRow>

          <button
            type="button"
            onClick={addObstacleFromForm}
            disabled={!obstacleTargetId}
            className="v4-focus v4-button w-full px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100 disabled:opacity-40"
          >
            <Plus size={12} /> Add Obstacle
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

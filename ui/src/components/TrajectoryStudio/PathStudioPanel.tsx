import { useEffect, useMemo, useState } from 'react';
import { Link2, RefreshCcw, Save } from 'lucide-react';
import { HudButton, HudInput, HudPanel, HudSection } from '../HudComponents';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { mapIssuePathToPlannerStep } from '../../utils/plannerValidation';

interface PathStudioPanelProps {
  builder: ReturnType<typeof useMissionBuilder>;
}

export function PathStudioPanel({ builder }: PathStudioPanelProps) {
  const { state, setters, actions } = builder;

  const [selectedAssetId, setSelectedAssetId] = useState('');
  const [bakedPathName, setBakedPathName] = useState('');
  const [scanProjectId, setScanProjectId] = useState('');

  const selectedScan = useMemo(
    () =>
      state.scanProject.scans.find((scan) => scan.id === state.selectedScanId) ??
      state.scanProject.scans[0],
    [state.scanProject.scans, state.selectedScanId]
  );

  const selectedKeyLevel = useMemo(
    () =>
      selectedScan?.key_levels.find((level) => level.id === state.selectedKeyLevelId) ??
      selectedScan?.key_levels[0],
    [selectedScan, state.selectedKeyLevelId]
  );
  const scanDefinitionIssues = useMemo(
    () =>
      (state.validationReport?.issues ?? []).filter(
        (issue) => mapIssuePathToPlannerStep(issue.path) === 'scan_definition'
      ),
    [state.validationReport]
  );

  useEffect(() => {
    actions.refreshModelList().catch(() => null);
    actions.refreshPathAssets().catch(() => null);
    actions.refreshScanProjects().catch(() => null);
    // Run once.
  }, []);

  useEffect(() => {
    if (!selectedScan) return;
    if (!state.selectedScanId || state.selectedScanId !== selectedScan.id) {
      setters.setSelectedScanId(selectedScan.id);
    }
  }, [selectedScan?.id]);

  useEffect(() => {
    if (!selectedScan) return;
    if (
      !state.selectedKeyLevelId ||
      !selectedScan.key_levels.some((level) => level.id === state.selectedKeyLevelId)
    ) {
      setters.setSelectedKeyLevelId(selectedScan.key_levels[0]?.id ?? null);
    }
  }, [selectedScan?.id, selectedScan?.key_levels.length, state.selectedKeyLevelId]);

  const handleUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;
    await actions.handleFileUpload(file);
  };

  return (
    <HudPanel className="w-96 max-h-[calc(100vh-220px)] overflow-y-auto custom-scrollbar" title="Path Studio">
      <div className="space-y-3">
        {scanDefinitionIssues.length > 0 ? (
          <div className="text-[10px] text-amber-200 bg-amber-950/50 border border-amber-700/60 rounded px-2 py-1.5">
            {scanDefinitionIssues.length} scan-definition issue(s). Run Validate and click an issue to jump to the exact field.
          </div>
        ) : null}
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
              <div className="text-[10px] text-slate-500 font-mono break-all">{state.config.obj_path}</div>
            )}
          </div>
        </HudSection>

        <HudSection title="Scans" defaultOpen>
          <div className="space-y-2">
            <div className="grid grid-cols-2 gap-2">
              <HudButton
                variant="secondary"
                size="sm"
                className="w-full"
                onClick={() => actions.addScan()}
              >
                Add Scan
              </HudButton>
              <HudButton
                variant="danger"
                size="sm"
                className="w-full"
                onClick={() => selectedScan && actions.removeScan(selectedScan.id)}
                disabled={!selectedScan || state.scanProject.scans.length <= 1}
              >
                Remove Scan
              </HudButton>
            </div>

            <select
              value={state.selectedScanId ?? ''}
              onChange={(e) => setters.setSelectedScanId(e.target.value || null)}
              className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
            >
              {state.scanProject.scans.map((scan) => (
                <option key={scan.id} value={scan.id}>
                  {scan.name}
                </option>
              ))}
            </select>

            {selectedScan && (
              <div className="rounded border border-slate-700/70 bg-slate-900/50 p-2 space-y-2">
                <HudInput
                  label="Scan Name"
                  value={selectedScan.name}
                  onChange={(val) => actions.updateScan(selectedScan.id, { name: String(val) })}
                />

                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
                      Axis
                    </label>
                    <select
                      value={selectedScan.axis}
                      onChange={(e) => actions.setScanAxisAligned(selectedScan.id, e.target.value as 'X' | 'Y' | 'Z')}
                      className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
                    >
                      <option value="X">Body X</option>
                      <option value="Y">Body Y</option>
                      <option value="Z">Body Z</option>
                    </select>
                  </div>

                  <HudInput
                    label="Level Spacing (m)"
                    type="number"
                    min={0.01}
                    step={0.01}
                    value={selectedScan.level_spacing_m ?? 0.1}
                    onChange={(val) =>
                      actions.updateScan(selectedScan.id, {
                        level_spacing_m: Math.max(0.01, Number(val)),
                      })
                    }
                  />

                  <HudInput
                    label="Densify"
                    type="number"
                    min={1}
                    step={1}
                    value={selectedScan.densify_multiplier}
                    onChange={(val) => actions.updateScan(selectedScan.id, { densify_multiplier: Number(val) })}
                  />

                  <HudInput
                    label="Speed Max"
                    type="number"
                    min={0.01}
                    step={0.01}
                    value={selectedScan.speed_max}
                    onChange={(val) => actions.updateScan(selectedScan.id, { speed_max: Number(val) })}
                  />
                </div>

                <div className="rounded border border-slate-700/60 p-2 space-y-2">
                  <div className="text-[10px] font-bold uppercase tracking-wider text-slate-400">Planes</div>
                  <div className="grid grid-cols-3 gap-2">
                    <HudButton
                      variant={
                        state.selectedProjectScanPlaneHandle?.scanId === selectedScan.id &&
                        state.selectedProjectScanPlaneHandle?.handle === 'a'
                          ? 'primary'
                          : 'secondary'
                      }
                      size="sm"
                      className="w-full"
                      onClick={() =>
                        (() => {
                          const next =
                            state.selectedProjectScanPlaneHandle?.scanId === selectedScan.id &&
                            state.selectedProjectScanPlaneHandle?.handle === 'a'
                              ? null
                              : ({ scanId: selectedScan.id, handle: 'a' } as const);
                          setters.setSelectedProjectScanPlaneHandle(next);
                          setters.setSelectedScanCenterHandle(null);
                          if (next) {
                            setters.setSelectedKeyLevelHandle(null);
                            setters.setSelectedConnectorControl(null);
                          }
                        })()
                      }
                    >
                      Move Plane A
                    </HudButton>
                    <HudButton
                      variant={
                        state.selectedProjectScanPlaneHandle?.scanId === selectedScan.id &&
                        state.selectedProjectScanPlaneHandle?.handle === 'b'
                          ? 'primary'
                          : 'secondary'
                      }
                      size="sm"
                      className="w-full"
                      onClick={() =>
                        (() => {
                          const next =
                            state.selectedProjectScanPlaneHandle?.scanId === selectedScan.id &&
                            state.selectedProjectScanPlaneHandle?.handle === 'b'
                              ? null
                              : ({ scanId: selectedScan.id, handle: 'b' } as const);
                          setters.setSelectedProjectScanPlaneHandle(next);
                          setters.setSelectedScanCenterHandle(null);
                          if (next) {
                            setters.setSelectedKeyLevelHandle(null);
                            setters.setSelectedConnectorControl(null);
                          }
                        })()
                      }
                    >
                      Move Plane B
                    </HudButton>
                    <HudButton
                      variant={
                        state.selectedScanCenterHandle?.scanId === selectedScan.id
                          ? 'primary'
                          : 'secondary'
                      }
                      size="sm"
                      className="w-full"
                      onClick={() =>
                        (() => {
                          const next =
                            state.selectedScanCenterHandle?.scanId === selectedScan.id
                              ? null
                              : ({ scanId: selectedScan.id } as const);
                          setters.setSelectedScanCenterHandle(next);
                          setters.setSelectedProjectScanPlaneHandle(null);
                          if (next) {
                            setters.setSelectedKeyLevelHandle(null);
                            setters.setSelectedConnectorControl(null);
                          }
                        })()
                      }
                    >
                      Move Center
                    </HudButton>
                  </div>
                  <div className="text-[10px] text-slate-400">
                    Centerline is defined by Plane A/B. Use <span className="font-semibold text-slate-200">Move Center</span> to translate the whole scan.
                  </div>

                  <div className="grid grid-cols-2 gap-2">
                    <HudInput
                      label="A X"
                      type="number"
                      step={0.1}
                      value={selectedScan.plane_a[0]}
                      onChange={(val) =>
                        actions.updateScan(selectedScan.id, {
                          plane_a: [Number(val), selectedScan.plane_a[1], selectedScan.plane_a[2]],
                        })
                      }
                    />
                    <HudInput
                      label="A Y"
                      type="number"
                      step={0.1}
                      value={selectedScan.plane_a[1]}
                      onChange={(val) =>
                        actions.updateScan(selectedScan.id, {
                          plane_a: [selectedScan.plane_a[0], Number(val), selectedScan.plane_a[2]],
                        })
                      }
                    />
                    <HudInput
                      label="A Z"
                      type="number"
                      step={0.1}
                      value={selectedScan.plane_a[2]}
                      onChange={(val) =>
                        actions.updateScan(selectedScan.id, {
                          plane_a: [selectedScan.plane_a[0], selectedScan.plane_a[1], Number(val)],
                        })
                      }
                    />
                    <div />
                    <HudInput
                      label="B X"
                      type="number"
                      step={0.1}
                      value={selectedScan.plane_b[0]}
                      onChange={(val) =>
                        actions.updateScan(selectedScan.id, {
                          plane_b: [Number(val), selectedScan.plane_b[1], selectedScan.plane_b[2]],
                        })
                      }
                    />
                    <HudInput
                      label="B Y"
                      type="number"
                      step={0.1}
                      value={selectedScan.plane_b[1]}
                      onChange={(val) =>
                        actions.updateScan(selectedScan.id, {
                          plane_b: [selectedScan.plane_b[0], Number(val), selectedScan.plane_b[2]],
                        })
                      }
                    />
                    <HudInput
                      label="B Z"
                      type="number"
                      step={0.1}
                      value={selectedScan.plane_b[2]}
                      onChange={(val) =>
                        actions.updateScan(selectedScan.id, {
                          plane_b: [selectedScan.plane_b[0], selectedScan.plane_b[1], Number(val)],
                        })
                      }
                    />
                  </div>
                </div>

                <div className="rounded border border-slate-700/60 p-2 space-y-2">
                  <div className="flex items-center justify-between gap-2">
                    <div className="text-[10px] font-bold uppercase tracking-wider text-slate-400">Key Levels</div>
                    <div className="flex gap-2">
                      <HudButton
                        variant="secondary"
                        size="sm"
                        onClick={() => actions.addKeyLevel(selectedScan.id)}
                      >
                        Add
                      </HudButton>
                      <HudButton
                        variant="danger"
                        size="sm"
                        onClick={() =>
                          selectedKeyLevel && actions.removeKeyLevel(selectedScan.id, selectedKeyLevel.id)
                        }
                        disabled={!selectedKeyLevel || selectedScan.key_levels.length <= 2}
                      >
                        Remove
                      </HudButton>
                    </div>
                  </div>

                  <select
                    value={selectedKeyLevel?.id ?? ''}
                    onChange={(e) => setters.setSelectedKeyLevelId(e.target.value || null)}
                    className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
                  >
                    {selectedScan.key_levels
                      .slice()
                      .sort((a, b) => a.t - b.t)
                      .map((level, idx) => (
                        <option key={level.id} value={level.id}>
                          Level {idx + 1} (t={level.t.toFixed(2)})
                        </option>
                      ))}
                  </select>

                  {selectedKeyLevel && (
                    <div className="grid grid-cols-2 gap-2">
                      <HudInput
                        label="t"
                        type="number"
                        min={0}
                        max={1}
                        step={0.01}
                        value={selectedKeyLevel.t}
                        onChange={(val) =>
                          actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, { t: Number(val) })
                        }
                      />
                      <HudInput
                        label="Rotation"
                        type="number"
                        step={1}
                        value={selectedKeyLevel.rotation_deg}
                        onChange={(val) =>
                          actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, {
                            rotation_deg: Number(val),
                          })
                        }
                      />
                      <HudInput
                        label="Radius X"
                        type="number"
                        min={0.01}
                        step={0.05}
                        value={selectedKeyLevel.radius_x}
                        onChange={(val) =>
                          actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, {
                            radius_x: Number(val),
                          })
                        }
                      />
                      <HudInput
                        label="Radius Y"
                        type="number"
                        min={0.01}
                        step={0.05}
                        value={selectedKeyLevel.radius_y}
                        onChange={(val) =>
                          actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, {
                            radius_y: Number(val),
                          })
                        }
                      />
                    </div>
                  )}
                </div>
              </div>
            )}

            <HudButton
              variant="secondary"
              size="sm"
              className="w-full"
              onClick={() => actions.previewScanProject(100)}
              disabled={state.loading || state.compilePending || !state.config.obj_path}
            >
              <RefreshCcw size={12} className={state.loading ? 'animate-spin' : ''} /> Preview Project
            </HudButton>
            {state.scanProjectAutoPreviewEnabled && (
              <div className="text-[10px] text-emerald-300">Live update is on while editing.</div>
            )}
          </div>
        </HudSection>

        <HudSection title="Connect Scans" defaultOpen={false}>
          <div className="space-y-2">
            <div className="grid grid-cols-2 gap-2">
              <HudButton
                variant={state.connectMode ? 'primary' : 'secondary'}
                size="sm"
                className="w-full"
                onClick={() => (state.connectMode ? actions.cancelConnectMode() : actions.startConnectMode())}
              >
                {state.connectMode ? 'Cancel Connect' : 'Connect By Click'}
              </HudButton>
              <HudButton
                variant="secondary"
                size="sm"
                className="w-full"
                onClick={() => actions.previewScanProject(100)}
                disabled={!state.scanProject.scans.length}
              >
                Refresh Endpoints
              </HudButton>
            </div>

            {state.connectMode && (
              <div className="text-[10px] text-cyan-300">
                Click a scan endpoint marker (S/E) in viewport, then click endpoint on another scan.
              </div>
            )}

            {state.connectSourceEndpoint && (
              <div className="text-[10px] text-slate-400">
                Source: <span className="text-slate-200">{state.connectSourceEndpoint.scanId}</span> /{' '}
                <span className="text-slate-200">{state.connectSourceEndpoint.endpoint.toUpperCase()}</span>
              </div>
            )}

            {state.scanProject.connectors.length === 0 ? (
              <div className="text-[10px] text-slate-500">No connectors yet.</div>
            ) : (
              <div className="space-y-2">
                {state.scanProject.connectors.map((connector) => (
                  <div
                    key={connector.id}
                    className="rounded border border-slate-700/70 bg-slate-900/50 p-2 space-y-2"
                  >
                    <div className="text-[10px] font-mono text-slate-300">
                      {connector.from_scan_id}:{connector.from_endpoint} {'->'} {connector.to_scan_id}:{connector.to_endpoint}
                    </div>
                    <div className="flex gap-2">
                      <HudButton
                        variant={
                          state.selectedConnectorControl?.connectorId === connector.id &&
                          state.selectedConnectorControl?.control === 'control1'
                            ? 'primary'
                            : 'secondary'
                        }
                        size="sm"
                        className="flex-1"
                        onClick={() =>
                          setters.setSelectedConnectorControl({
                            connectorId: connector.id,
                            control: 'control1',
                          })
                        }
                      >
                        Control 1
                      </HudButton>
                      <HudButton
                        variant={
                          state.selectedConnectorControl?.connectorId === connector.id &&
                          state.selectedConnectorControl?.control === 'control2'
                            ? 'primary'
                            : 'secondary'
                        }
                        size="sm"
                        className="flex-1"
                        onClick={() =>
                          setters.setSelectedConnectorControl({
                            connectorId: connector.id,
                            control: 'control2',
                          })
                        }
                      >
                        Control 2
                      </HudButton>
                      <HudButton
                        variant="danger"
                        size="sm"
                        onClick={() => actions.removeConnector(connector.id)}
                      >
                        Del
                      </HudButton>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        </HudSection>

        <HudSection title="Diagnostics" defaultOpen={false}>
          {state.compilePreviewState ? (
            <div className="space-y-2 text-[10px] font-mono text-slate-300">
              <div>Path: {state.compilePreviewState.points} pts</div>
              <div>Length: {state.compilePreviewState.path_length.toFixed(2)} m</div>
              <div>Est. Time: {state.compilePreviewState.estimated_duration.toFixed(1)} s</div>
              <div>
                Min Clearance:{' '}
                {state.compilePreviewState.diagnostics.min_clearance_m != null
                  ? `${state.compilePreviewState.diagnostics.min_clearance_m.toFixed(3)} m`
                  : 'n/a'}
              </div>
              <div>
                Collision Points: {state.compilePreviewState.diagnostics.collision_points_count}
              </div>
              {state.compilePreviewState.diagnostics.warnings.length > 0 && (
                <div className="rounded border border-amber-500/40 bg-amber-500/10 p-2 text-amber-200">
                  {state.compilePreviewState.diagnostics.warnings.join(' | ')}
                </div>
              )}
              <div className="text-slate-500">Per scan and connector clearance overlays are shown in viewport.</div>
            </div>
          ) : (
            <div className="text-[10px] text-slate-500">No compile diagnostics yet.</div>
          )}
        </HudSection>

        <HudSection title="Save" defaultOpen={false}>
          <div className="space-y-2">
            <HudInput
              label="Project Name"
              value={state.scanProject.name}
              onChange={(val) =>
                actions.updateScanProject((prev) => ({
                  ...prev,
                  name: String(val),
                }))
              }
            />

            <HudButton
              variant="primary"
              size="sm"
              className="w-full"
              onClick={() => void actions.saveScanProject(state.scanProject.name)}
              disabled={!state.scanProject.name.trim() || !state.scanProject.obj_path}
            >
              <Save size={12} /> Save Project
            </HudButton>

            <HudInput
              label="Baked Path Name"
              value={bakedPathName}
              onChange={(val) => setBakedPathName(String(val))}
            />

            <HudButton
              variant="secondary"
              size="sm"
              className="w-full"
              onClick={async () => {
                const saved = await actions.saveBakedPathFromCompiled(bakedPathName);
                if (saved) setBakedPathName('');
              }}
              disabled={!bakedPathName.trim()}
              icon={Link2}
            >
              Save Baked Path Asset
            </HudButton>
          </div>
        </HudSection>

        <HudSection title="Project Library" defaultOpen={false}>
          <div className="space-y-2">
            <select
              value={scanProjectId}
              onChange={(e) => setScanProjectId(e.target.value)}
              className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5"
            >
              <option value="">Select project…</option>
              {state.scanProjects.map((project) => (
                <option key={project.id} value={project.id}>
                  {project.name}
                </option>
              ))}
            </select>
            <div className="flex gap-2">
              <HudButton
                variant="secondary"
                size="sm"
                className="flex-1"
                onClick={() => scanProjectId && actions.loadScanProjectById(scanProjectId)}
                disabled={!scanProjectId}
              >
                Load Project
              </HudButton>
              <HudButton
                variant="secondary"
                size="sm"
                className="flex-1"
                onClick={() => actions.createDefaultScanProjectState(state.config.obj_path)}
              >
                New Project
              </HudButton>
            </div>
          </div>
        </HudSection>

        <HudSection title="Path Asset Library" defaultOpen={false}>
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
                Load Path
              </HudButton>
              <HudButton
                variant="primary"
                size="sm"
                className="flex-1"
                onClick={() => selectedAssetId && actions.applyPathAssetToSegment(selectedAssetId)}
                disabled={!selectedAssetId}
              >
                Use on Scan
              </HudButton>
            </div>
          </div>
        </HudSection>
      </div>
    </HudPanel>
  );
}

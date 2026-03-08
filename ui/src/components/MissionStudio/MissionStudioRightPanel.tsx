import { useState, useEffect, useMemo, useRef } from 'react';
import { Trash2, Save, CheckCircle, AlertCircle, Crosshair, Route, Link2, Pause, CircleDot, MapPin, ChevronDown, ChevronUp } from 'lucide-react';
import { useStudioStore } from './useStudioStore';
import { compileStudioMission } from './compileStudioMission';
import { getStudioRouteDiagnostics } from './studioRouteDiagnostics';

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
  const holds = useStudioStore((s) => s.holds);
  const satelliteStart = useStudioStore((s) => s.satelliteStart);
  const points = useStudioStore((s) => s.points);
  const referenceObjectPath = useStudioStore((s) => s.referenceObjectPath);
  const missionName = useStudioStore((s) => s.missionName);
  const setMissionName = useStudioStore((s) => s.setMissionName);
  const validationReport = useStudioStore((s) => s.validationReport);
  const setValidationReport = useStudioStore((s) => s.setValidationReport);
  const validationBusy = useStudioStore((s) => s.validationBusy);
  const setValidationBusy = useStudioStore((s) => s.setValidationBusy);
  const saveBusy = useStudioStore((s) => s.saveBusy);
  const setSaveBusy = useStudioStore((s) => s.setSaveBusy);

  const [saveSuccess, setSaveSuccess] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [validateResult, setValidateResult] = useState<{ ok: boolean; message: string } | null>(null);
  const [issuesExpanded, setIssuesExpanded] = useState(false);
  const [validationIssuesExpanded, setValidationIssuesExpanded] = useState(false);

  useEffect(() => {
    if (missionName.trim().length > 0) return;
    if (paths.length === 0) return;
    const ts = new Date()
      .toISOString()
      .slice(0, 16)
      .replaceAll('-', '')
      .replaceAll(':', '')
      .replace('T', '');
    setMissionName(`Studio_${paths.length}path_${ts}`);
  }, [paths.length, missionName, setMissionName]);

  const totalWaypoints = paths.reduce((acc, p) => acc + p.waypoints.length, 0);
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
  const authoringFingerprint = useMemo(
    () =>
      JSON.stringify({
        referenceObjectPath,
        satelliteStart,
        paths,
        wires,
        holds,
        points,
        assembly,
      }),
    [referenceObjectPath, satelliteStart, paths, wires, holds, points, assembly]
  );
  const previousFingerprintRef = useRef(authoringFingerprint);
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

  useEffect(() => {
    if (routeDiagnostics.status !== 'executable') {
      setIssuesExpanded(true);
    }
  }, [routeDiagnostics.status]);

  useEffect(() => {
    if ((validationReport?.issues.length ?? 0) > 0) {
      setValidationIssuesExpanded(true);
    }
  }, [validationReport]);

  useEffect(() => {
    if (previousFingerprintRef.current === authoringFingerprint) return;
    previousFingerprintRef.current = authoringFingerprint;
    if (validationReport !== null) {
      setValidationReport(null);
    }
    if (validateResult !== null) {
      setValidateResult(null);
    }
    if (saveSuccess) {
      setSaveSuccess(false);
    }
    if (saveError) {
      setSaveError(null);
    }
  }, [authoringFingerprint, saveError, saveSuccess, setValidationReport, validateResult, validationReport]);

  const handleValidate = async () => {
    setValidationBusy(true);
    setValidateResult(null);
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      const { unifiedMissionApi } = await import('../../api/unifiedMissionApi');
      const report = await unifiedMissionApi.validateMission(mission);
      setValidationReport(report);
      setValidateResult({
        ok: report.valid,
        message: report.valid ? 'Validation passed' : `${report.summary?.errors ?? '?'} error(s)`,
      });
    } catch (e) {
      setValidationReport(null);
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

  const canSave = routeDiagnostics.executable && missionName.trim().length > 0;
  const canValidate = routeDiagnostics.executable && missionName.trim().length > 0;
  const validationPassed = validationReport?.valid === true;
  const validationIssueCount = validationReport?.issues.length ?? 0;
  const validationStateLabel = validationBusy
    ? 'Validating'
    : validationReport?.valid
      ? 'Validated'
      : validationReport
        ? `${validationIssueCount} issue${validationIssueCount === 1 ? '' : 's'}`
        : 'Not yet validated';
  const validationStateClass = validationBusy
    ? 'border-amber-500/40 bg-amber-950/40 text-amber-200'
    : validationReport?.valid
      ? 'border-emerald-500/40 bg-emerald-950/40 text-emerald-200'
      : validationReport
        ? 'border-red-500/40 bg-red-950/35 text-red-200'
        : 'border-slate-700 bg-slate-900/60 text-slate-300';
  const routeIssueCount =
    routeDiagnostics.invalidPathIds.length +
    routeDiagnostics.invalidWireIds.length +
    routeDiagnostics.branchingSources.length +
    routeDiagnostics.multiIncomingTargets.length +
    routeDiagnostics.disconnectedPathIds.length +
    routeDiagnostics.disconnectedWireIds.length +
    routeDiagnostics.unconnectedHoldIds.length +
    (routeDiagnostics.cycleDetected ? 1 : 0) +
    (routeDiagnostics.routeStartsAtSatellite ? 0 : routeDiagnostics.validPathCount > 0 ? 1 : 0);
  const primaryAction = !missionName.trim()
    ? { label: 'Name Mission First', disabled: true, tone: 'neutral' as const, onClick: () => undefined }
    : !routeDiagnostics.executable
      ? { label: 'Complete Route Before Validation', disabled: true, tone: 'warn' as const, onClick: () => undefined }
      : !validationPassed
        ? { label: validationBusy ? 'Validating...' : 'Validate Mission', disabled: validationBusy, tone: 'info' as const, onClick: () => void handleValidate() }
        : { label: saveBusy ? 'Saving...' : 'Save Mission', disabled: saveBusy, tone: 'good' as const, onClick: () => void handleSave() };
  const primaryActionClass =
    primaryAction.tone === 'good'
      ? 'border-emerald-700 bg-emerald-900/40 text-emerald-100 hover:bg-emerald-900/60'
      : primaryAction.tone === 'info'
        ? 'border-cyan-700 bg-cyan-900/35 text-cyan-100 hover:bg-cyan-900/55'
        : 'border-amber-700/60 bg-amber-950/30 text-amber-100';
  const nextStepLabel = !missionName.trim()
    ? 'Enter a mission name to unlock validation.'
    : !routeDiagnostics.executable
      ? routeDiagnostics.nextAction
      : !validationPassed
        ? 'Route is executable. Run validation before saving.'
        : 'Validation is current. Save this mission when ready.';

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 overflow-y-auto p-3 flex flex-col gap-1.5">
        <div className="rounded-xl border border-slate-800 bg-slate-900/70 p-3 mb-2">
          <div>
            <div className="text-[10px] uppercase tracking-[0.16em] text-slate-500">Status</div>
            <div className="mt-1 text-sm font-semibold text-slate-100 truncate">
              {missionName.trim() || 'Untitled Studio Mission'}
            </div>
          </div>
          <div className="mt-3 grid grid-cols-3 gap-2 text-center">
            <div className="rounded-lg border border-slate-800 bg-black/20 px-2 py-2">
              <div className="text-[7px] uppercase tracking-[0.06em] text-slate-500">Segments</div>
              <div className="mt-1 text-sm font-semibold text-slate-100">{assembly.length}</div>
            </div>
            <div className="rounded-lg border border-slate-800 bg-black/20 px-2 py-2">
              <div className="text-[7px] uppercase tracking-[0.04em] text-slate-500">Waypoints</div>
              <div className="mt-1 text-sm font-semibold text-slate-100">{totalWaypoints}</div>
            </div>
            <div className="rounded-lg border border-slate-800 bg-black/20 px-2 py-2">
              <div className="text-[7px] uppercase tracking-[0.06em] text-slate-500">Path</div>
              <div className="mt-1 text-sm font-semibold text-slate-100">{totalPathLengthM.toFixed(1)} m</div>
            </div>
          </div>
          <div className="mt-3 grid grid-cols-2 gap-2 text-[10px]">
            <div
              className={`rounded-lg border px-2 py-2 ${
                routeDiagnostics.targetMode === 'object'
                  ? 'border-cyan-500/30 bg-cyan-950/20 text-cyan-100'
                  : 'border-slate-800 bg-black/20 text-slate-300'
              }`}
            >
              Target: {routeDiagnostics.targetMode === 'object' ? 'Object selected' : 'Local origin'}
            </div>
            <div
              className={`rounded-lg border px-2 py-2 ${
                routeDiagnostics.executable
                  ? 'border-emerald-500/30 bg-emerald-950/20 text-emerald-100'
                  : routeDiagnostics.status === 'invalid'
                    ? 'border-red-500/30 bg-red-950/20 text-red-100'
                    : 'border-amber-500/30 bg-amber-950/20 text-amber-100'
              }`}
            >
              Route: {routeDiagnostics.executable ? 'Executable' : routeDiagnostics.status}
            </div>
          </div>
          <div className="mt-2 rounded-lg border border-slate-800 bg-slate-950/80 px-3 py-2 text-xs text-slate-300">
            {routeDiagnostics.nextAction}
          </div>
          <div className="mt-3 rounded-lg border border-slate-800 bg-slate-950/80">
            <button
              type="button"
              onClick={() => setIssuesExpanded((expanded) => !expanded)}
              className="flex w-full items-center justify-between gap-3 px-3 py-2 text-left text-xs text-slate-200"
            >
              <span className="font-semibold">
                Route Issues {routeIssueCount > 0 ? `(${routeIssueCount})` : '(0)'}
              </span>
              {issuesExpanded ? <ChevronUp size={14} className="text-slate-400" /> : <ChevronDown size={14} className="text-slate-400" />}
            </button>
            {issuesExpanded ? (
              <div className="border-t border-slate-800 px-3 py-2">
                {routeDiagnostics.detailLines.length === 0 ? (
                  <div className="text-xs text-slate-400">No route issues are currently surfaced.</div>
                ) : (
                  <div className="flex flex-col gap-1.5">
                    {routeDiagnostics.detailLines.map((line, index) => (
                      <div key={`${index}-${line}`} className="flex items-start gap-2 text-xs text-slate-300">
                        <span className="mt-[5px] h-1.5 w-1.5 shrink-0 rounded-full bg-slate-500" />
                        <span>{line}</span>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ) : null}
          </div>
          {validationReport ? (
            <div className="mt-3 rounded-lg border border-slate-800 bg-slate-950/80">
              <button
                type="button"
                onClick={() => setValidationIssuesExpanded((expanded) => !expanded)}
                className="flex w-full items-center justify-between gap-3 px-3 py-2 text-left text-xs text-slate-200"
              >
                <span className="font-semibold">
                  Validation Issues {validationIssueCount > 0 ? `(${validationIssueCount})` : '(0)'}
                </span>
                {validationIssuesExpanded ? (
                  <ChevronUp size={14} className="text-slate-400" />
                ) : (
                  <ChevronDown size={14} className="text-slate-400" />
                )}
              </button>
              {validationIssuesExpanded ? (
                <div className="border-t border-slate-800 px-3 py-2">
                  {validationIssueCount === 0 ? (
                    <div className="text-xs text-emerald-300">No backend validation issues are currently reported.</div>
                  ) : (
                    <div className="flex flex-col gap-2">
                      {validationReport.issues.map((issue, index) => {
                        const severityClass =
                          issue.severity === 'error'
                            ? 'border-red-500/30 bg-red-950/20 text-red-100'
                            : issue.severity === 'warning'
                              ? 'border-amber-500/30 bg-amber-950/20 text-amber-100'
                              : 'border-sky-500/30 bg-sky-950/20 text-sky-100';
                        return (
                          <div
                            key={`${issue.code}-${issue.path}-${index}`}
                            className="rounded-lg border border-slate-800 bg-black/20 px-3 py-2"
                          >
                            <div className="flex items-start justify-between gap-3">
                              <div className="min-w-0">
                                <div className="text-xs font-semibold text-slate-100 break-words">{issue.message}</div>
                                <div className="mt-1 text-[10px] text-slate-400 break-all">
                                  {issue.path || 'mission'} · {issue.code}
                                </div>
                              </div>
                              <div className={`shrink-0 rounded-full border px-2 py-1 text-[10px] font-semibold uppercase tracking-[0.12em] ${severityClass}`}>
                                {issue.severity}
                              </div>
                            </div>
                            {issue.suggestion ? (
                              <div className="mt-2 text-xs text-slate-300">
                                Suggestion: <span className="text-slate-200">{issue.suggestion}</span>
                              </div>
                            ) : null}
                          </div>
                        );
                      })}
                    </div>
                  )}
                </div>
              ) : null}
            </div>
          ) : null}
        </div>
        {assembly.length > 0 ? (
          <div className="px-1 pt-1 text-[10px] font-bold uppercase tracking-[0.16em] text-slate-500">
            Segments
          </div>
        ) : null}
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
        <div className="rounded-xl border border-slate-800 bg-slate-950/85 p-3">
          <div className="flex items-center justify-between text-[10px] uppercase tracking-[0.14em] text-slate-500">
            <span>Next Step</span>
            <span>
              {canSave && validationPassed
                ? 'Save ready'
                : validationPassed
                  ? 'Validated'
                  : canValidate
                    ? 'Validate now'
                    : 'Blocked'}
            </span>
          </div>
          <div className="mt-2 text-xs text-slate-200">{nextStepLabel}</div>
          <button
            type="button"
            onClick={primaryAction.onClick}
            disabled={primaryAction.disabled}
            className={`mt-3 w-full py-2 rounded-lg border text-xs font-semibold transition-all disabled:opacity-45 ${primaryActionClass}`}
          >
            {primaryAction.label}
          </button>
        </div>
        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => void handleValidate()}
            disabled={!canValidate || validationBusy}
            className="w-full py-2 rounded-lg border border-slate-700 bg-slate-800 text-slate-200 text-xs font-semibold disabled:opacity-50 hover:border-slate-600 transition-all"
          >
            {validationBusy ? 'Validating...' : validationPassed ? 'Revalidate' : 'Validate'}
          </button>
          <button
            type="button"
            onClick={() => void handleSave()}
            disabled={!canSave || !validationPassed || saveBusy}
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
    </div>
  );
}

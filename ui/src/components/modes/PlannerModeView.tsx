import { Suspense, useMemo, useState } from 'react';
import { ChevronDown, ChevronUp, Play, Save, Workflow } from 'lucide-react';

import { useCameraStore } from '../../store/cameraStore';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { ORBIT_SCALE, orbitSnapshot } from '../../data/orbitSnapshot';
import { UnifiedViewport } from '../UnifiedViewport';
import { OrbitTargetsPanel } from '../OrbitTargetsPanel';
import { MissionAuthoringStepper } from '../TrajectoryStudio/MissionAuthoringStepper';
import { MissionValidationPanel } from '../TrajectoryStudio/MissionValidationPanel';
import { GeneratorStack } from '../TrajectoryStudio/GeneratorStack';
import { PropertyInspector } from '../TrajectoryStudio/PropertyInspector';
import { PathStudioPanel } from '../TrajectoryStudio/PathStudioPanel';
import { StateTimeline } from '../TrajectoryStudio/StateTimeline';
import { isSaveLaunchReady } from '../../utils/plannerValidation';

interface PlannerModeViewProps {
  viewMode: 'free' | 'chase' | 'top';
  builder: ReturnType<typeof useMissionBuilder>;
}

function SaveLaunchPanel({ builder }: { builder: ReturnType<typeof useMissionBuilder> }) {
  const report = builder.state.validationReport;
  const readyForSaveLaunch = isSaveLaunchReady(report);

  return (
    <div className="w-80 bg-slate-950/90 backdrop-blur-md border border-slate-800 rounded-lg shadow-2xl p-3 space-y-3">
      <div className="flex items-center gap-2 text-slate-100">
        <Workflow size={14} className="text-cyan-300" />
        <h3 className="text-sm font-semibold">Save / Launch</h3>
      </div>
      <div className="text-xs text-slate-300">
        {report
          ? `Validation: ${report.valid ? 'pass' : 'fail'} · errors=${report.summary.errors} · warnings=${report.summary.warnings}`
          : 'Run validation before launch to enable gating.'}
      </div>
      <div className="grid gap-2">
        <button
          type="button"
          onClick={() => void builder.actions.validateUnifiedMission()}
          className="px-3 py-2 text-xs rounded border border-cyan-700 text-cyan-200 hover:bg-cyan-900/30"
        >
          Run Validation
        </button>
        <button
          type="button"
          onClick={() => void builder.actions.generateUnifiedPath()}
          className="px-3 py-2 text-xs rounded border border-slate-700 text-slate-200 hover:bg-slate-800"
        >
          Generate Preview Path
        </button>
        <button
          type="button"
          onClick={() => void builder.actions.handleSaveUnifiedMission()}
          disabled={!readyForSaveLaunch}
          className="px-3 py-2 text-xs rounded border border-emerald-700 text-emerald-200 hover:bg-emerald-900/30 disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2"
        >
          <Save size={13} />
          Save Mission
        </button>
        <button
          type="button"
          disabled={!readyForSaveLaunch}
          onClick={() => void builder.actions.handleRun()}
          className="px-3 py-2 text-xs rounded border border-blue-700 text-blue-100 hover:bg-blue-900/30 disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2"
        >
          <Play size={13} />
          Launch Mission
        </button>
      </div>
      {!readyForSaveLaunch ? (
        <div className="text-[11px] text-amber-300">
          Save and launch are gated until mission validation passes.
        </div>
      ) : null}
    </div>
  );
}

export function PlannerModeView({ viewMode, builder }: PlannerModeViewProps) {
  const [timelineCollapsed, setTimelineCollapsed] = useState(false);

  const plannerStep = builder.state.authoringStep;
  const viewportMode = plannerStep === 'scan_definition' ? 'scan' : 'mission';
  const showOrbitTargets = plannerStep !== 'scan_definition';
  const panelWidthClass = plannerStep === 'scan_definition' ? 'w-[24rem]' : 'w-80';

  const contextPanel = useMemo(() => {
    if (plannerStep === 'scan_definition') {
      return <PathStudioPanel builder={builder} />;
    }
    if (plannerStep === 'validate') {
      return <MissionValidationPanel builder={builder} />;
    }
    if (plannerStep === 'save_launch') {
      return <SaveLaunchPanel builder={builder} />;
    }
    if (plannerStep === 'segments') {
      return <GeneratorStack builder={builder} showExecutionActions={false} />;
    }
    return <PropertyInspector builder={builder} />;
  }, [builder, plannerStep]);

  return (
    <div className="flex-1 relative flex flex-col h-full overflow-hidden">
      <div className="flex-1 relative min-h-0">
        <div className="absolute inset-0 z-0">
          <UnifiedViewport
            mode={viewportMode}
            viewMode={viewMode}
            builderState={builder.state}
            builderActions={builder.actions}
          />
        </div>

        <div className="absolute top-4 left-4 z-30">
          <MissionAuthoringStepper builder={builder} />
        </div>

        <div className={`absolute top-4 right-4 z-30 max-h-[calc(100vh-220px)] overflow-y-auto custom-scrollbar ${panelWidthClass}`}>
          {contextPanel}
        </div>

        {showOrbitTargets ? (
          <div className="absolute bottom-4 right-4 z-20">
            <Suspense fallback={null}>
              <OrbitTargetsPanel
                selectedTargetId={builder.state.selectedOrbitTargetId}
                ownSatellite={{
                  id: 'SATELLITE',
                  name: 'Your Satellite',
                  positionScene: [
                    builder.state.startPosition[0] * ORBIT_SCALE,
                    builder.state.startPosition[1] * ORBIT_SCALE,
                    builder.state.startPosition[2] * ORBIT_SCALE,
                  ],
                  positionMeters: [
                    builder.state.startPosition[0],
                    builder.state.startPosition[1],
                    builder.state.startPosition[2],
                  ],
                }}
                solarBodies={[]}
                onFocusTarget={(targetId, _positionScene, focusDistance) => {
                  const originTargetId =
                    builder.state.selectedOrbitTargetId || builder.state.startTargetId;
                  const originObj = originTargetId
                    ? orbitSnapshot.objects.find((o) => o.id === originTargetId)
                    : null;
                  const targetObj = orbitSnapshot.objects.find((o) => o.id === targetId);
                  if (!targetObj) return;
                  const origin = originObj?.position_m ?? [0, 0, 0];
                  const scenePos: [number, number, number] = [
                    (targetObj.position_m[0] - origin[0]) * ORBIT_SCALE,
                    (targetObj.position_m[1] - origin[1]) * ORBIT_SCALE,
                    (targetObj.position_m[2] - origin[2]) * ORBIT_SCALE,
                  ];
                  useCameraStore.getState().requestFocus(scenePos, focusDistance);
                }}
              />
            </Suspense>
          </div>
        ) : null}

        {builder.state.pendingDraftRestore ? (
          <div className="absolute top-4 left-1/2 -translate-x-1/2 z-40 w-[34rem] rounded-lg border border-amber-500/50 bg-amber-950/90 px-4 py-3 text-amber-50 shadow-2xl backdrop-blur-md">
            <div className="text-sm">
              Restore mission draft from{' '}
              {new Date(builder.state.pendingDraftRestore.savedAt).toLocaleString()}?
            </div>
            <div className="mt-3 flex items-center justify-end gap-2">
              <button
                type="button"
                onClick={() => builder.actions.discardPendingDraft()}
                className="px-3 py-1.5 text-xs rounded border border-amber-200/30 bg-amber-900/20 hover:bg-amber-900/40"
              >
                Discard
              </button>
              <button
                type="button"
                onClick={() => builder.actions.restorePendingDraft()}
                className="px-3 py-1.5 text-xs rounded border border-amber-300/60 bg-amber-700/40 hover:bg-amber-700/60"
              >
                Restore
              </button>
            </div>
          </div>
        ) : null}
      </div>

      <div className={`${timelineCollapsed ? 'h-11' : 'h-48'} z-30 shadow-[0_-5px_20px_rgba(0,0,0,0.45)] bg-slate-950/95 border-t border-slate-800`}>
        <div className="h-11 px-3 flex items-center justify-between">
          <div className="text-[10px] uppercase tracking-[0.14em] text-slate-400">Timeline</div>
          <button
            type="button"
            onClick={() => setTimelineCollapsed((prev) => !prev)}
            className="px-2 py-1 text-[11px] rounded border border-slate-700 text-slate-300 hover:bg-slate-800 flex items-center gap-1"
          >
            {timelineCollapsed ? (
              <>
                <ChevronUp size={12} />
                Expand
              </>
            ) : (
              <>
                <ChevronDown size={12} />
                Collapse
              </>
            )}
          </button>
        </div>
        {!timelineCollapsed ? <StateTimeline builder={builder} /> : null}
      </div>
    </div>
  );
}

import { Suspense, useEffect, useMemo, useRef, useState } from 'react';
import { ChevronDown, ChevronUp } from 'lucide-react';

import { useCameraStore } from '../../store/cameraStore';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { usePlannerWizard } from '../../hooks/usePlannerWizard';
import { usePlannerOnboarding } from '../../hooks/usePlannerOnboarding';
import { ORBIT_SCALE, orbitSnapshot } from '../../data/orbitSnapshot';
import { UnifiedViewport } from '../UnifiedViewport';
import { OrbitTargetsPanel } from '../OrbitTargetsPanel';
import { StateTimeline } from '../TrajectoryStudio/StateTimeline';
import { InlineBanner } from '../ui-v4/InlineBanner';
import { PlannerStepRailV4 } from '../planner-v4/PlannerStepRailV4';
import { SegmentComposerCardV4 } from '../planner-v4/SegmentComposerCardV4';
import { ValidationNavigatorCardV4 } from '../planner-v4/ValidationNavigatorCardV4';
import { SaveLaunchCardV4 } from '../planner-v4/SaveLaunchCardV4';
import { CoachmarkLayer } from '../planner-v4/CoachmarkLayer';
import {
  ObstaclesStepCardV42,
  PathEditStepCardV42,
  PathMakerStepCardV42,
  TransferStepCardV42,
} from '../planner-v4/FlowStepCardsV42';

interface PlannerModeViewV4Props {
  viewMode: 'free' | 'chase' | 'top';
  builder: ReturnType<typeof useMissionBuilder>;
  onOpenRunner?: () => void;
}

function IntroBanner({
  onStart,
  onDismiss,
  onNever,
}: {
  onStart: () => void;
  onDismiss: () => void;
  onNever: () => void;
}) {
  return (
    <InlineBanner
      tone="info"
      title="Welcome to the guided planner"
      actions={
        <>
          <button
            type="button"
            onClick={onStart}
            className="v4-focus v4-button px-2 py-1 bg-cyan-900/45 border-cyan-700 text-cyan-100"
          >
            Take 60s Tour
          </button>
          <button
            type="button"
            onClick={onDismiss}
            className="v4-focus v4-button px-2 py-1 bg-slate-900/80 text-slate-200"
          >
            Dismiss
          </button>
          <button
            type="button"
            onClick={onNever}
            className="v4-focus v4-button px-2 py-1 bg-slate-900/80 text-slate-300"
          >
            Never show again
          </button>
        </>
      }
      className="w-[34rem]"
    >
      Create missions in 5 steps: Path Maker, Transfer, Obstacles, Path Edit, Mission Saver.
    </InlineBanner>
  );
}

export function PlannerModeViewV4({ viewMode, builder, onOpenRunner }: PlannerModeViewV4Props) {
  const [timelineCollapsed, setTimelineCollapsed] = useState(true);
  const hasAutoValidatedSaveStepRef = useRef(false);

  const wizard = usePlannerWizard({
    authoringStep: builder.state.authoringStep,
    setAuthoringStep: builder.actions.setAuthoringStep,
    startFrame: builder.state.startFrame,
    startTargetId: builder.state.startTargetId,
    segments: builder.state.segments,
    validationReport: builder.state.validationReport,
    scanPairCount: builder.state.scanProject.scans.length,
    scanEndpointCount: Object.keys(builder.state.compilePreviewState?.endpoints ?? {}).length * 2,
    transferTargetSelected: Boolean(builder.state.transferTargetRef),
    obstaclesCount: builder.state.obstacles.length,
    previewPathPoints: builder.state.previewPath.length,
    isManualMode: builder.state.isManualMode,
  });

  const onboarding = usePlannerOnboarding();

  const plannerStep = wizard.state.flowStep;
  const viewportMode = plannerStep === 'path_maker' ? 'scan' : 'mission';
  const showOrbitTargets = plannerStep !== 'path_maker';

  useEffect(() => {
    if (plannerStep === 'mission_saver') {
      if (!hasAutoValidatedSaveStepRef.current) {
        hasAutoValidatedSaveStepRef.current = true;
        void builder.actions.validateUnifiedMission();
      }
      return;
    }
    hasAutoValidatedSaveStepRef.current = false;
  }, [plannerStep, builder.actions.validateUnifiedMission]);

  const contextPanel = useMemo(() => {
    if (plannerStep === 'path_maker') {
      return [<PathMakerStepCardV42 key="path-maker" builder={builder} />];
    }
    if (plannerStep === 'transfer') {
      return [
        <TransferStepCardV42 key="transfer" builder={builder} />,
        <SegmentComposerCardV4 key="transfer-segments" builder={builder} />,
      ];
    }
    if (plannerStep === 'obstacles') {
      return [<ObstaclesStepCardV42 key="obstacles" builder={builder} />];
    }
    if (plannerStep === 'path_edit') {
      return [
        <PathEditStepCardV42
          key="path-edit"
          builder={builder}
          onFinishEditing={() => wizard.actions.goToStep('mission_saver')}
        />,
      ];
    }
    return [
      <ValidationNavigatorCardV4 key="validate" builder={builder} />,
      <SaveLaunchCardV4 key="save" builder={builder} onOpenRunner={onOpenRunner} />,
    ];
  }, [plannerStep, builder, wizard.actions, onOpenRunner]);

  return (
    <div className="flex-1 flex flex-col min-h-0 overflow-hidden">
      <div className="flex-1 min-h-0 p-3 flex gap-3 overflow-hidden">
        <aside className="w-[20rem] shrink-0 overflow-y-auto custom-scrollbar pr-1 relative z-30">
          <PlannerStepRailV4 builder={builder} wizard={wizard} />
        </aside>

        <section className="flex-1 min-w-0 relative rounded-[14px] border border-[color:var(--v4-border)] overflow-hidden bg-[color:var(--v4-surface-1)]">
          <div className="absolute inset-0 z-0">
            <UnifiedViewport
              mode={viewportMode}
              viewMode={viewMode}
              builderState={builder.state}
              builderActions={builder.actions}
            />
          </div>

          {showOrbitTargets ? (
            <div className="absolute right-4 bottom-4 z-20">
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
                      ? orbitSnapshot.objects.find((obj) => obj.id === originTargetId)
                      : null;
                    const targetObj = orbitSnapshot.objects.find((obj) => obj.id === targetId);
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

          {onboarding.state.showIntroBanner ? (
            <div className="absolute top-4 left-1/2 -translate-x-1/2 z-30">
              <IntroBanner
                onStart={onboarding.actions.startTour}
                onDismiss={onboarding.actions.dismissIntro}
                onNever={onboarding.actions.setNeverShowAgain}
              />
            </div>
          ) : null}

          {builder.state.pendingDraftRestore ? (
            <div className="absolute top-4 left-1/2 -translate-x-1/2 z-30 w-[34rem] rounded-[12px] border border-amber-500/65 bg-amber-950/90 p-3 shadow-xl">
              <div className="text-sm text-amber-50">
                Restore mission draft from{' '}
                {new Date(builder.state.pendingDraftRestore.savedAt).toLocaleString()}?
              </div>
              <div className="mt-3 flex justify-end gap-2">
                <button
                  type="button"
                  onClick={() => builder.actions.discardPendingDraft()}
                  className="v4-focus v4-button px-3 py-1.5 bg-amber-900/25 border-amber-300/40 text-amber-50"
                >
                  Discard
                </button>
                <button
                  type="button"
                  onClick={() => builder.actions.restorePendingDraft()}
                  className="v4-focus v4-button px-3 py-1.5 bg-amber-700/45 border-amber-300/80 text-amber-50"
                >
                  Restore
                </button>
              </div>
            </div>
          ) : null}

          <CoachmarkLayer
            open={onboarding.state.tourOpen}
            visibleCoachmarks={onboarding.state.visibleCoachmarks}
            onDismiss={onboarding.actions.dismissCoachmark}
            onClose={onboarding.actions.closeTour}
          />
        </section>

        <aside className="w-[28rem] shrink-0 overflow-y-auto custom-scrollbar pr-1 space-y-3 relative z-30">
          {contextPanel}
        </aside>
      </div>

      <div
        className={`${timelineCollapsed ? 'h-11' : 'h-48'} border-t border-[color:var(--v4-border)] bg-[color:var(--v4-surface-2)]/90 shrink-0`}
      >
        <div className="h-11 px-3 flex items-center justify-between">
          <div className="text-[10px] uppercase tracking-[0.14em] text-[color:var(--v4-text-3)]">
            Diagnostics Timeline
          </div>
          <button
            type="button"
            onClick={() => setTimelineCollapsed((prev) => !prev)}
            className="v4-focus v4-button px-2 py-1 bg-[color:var(--v4-surface-1)] text-[color:var(--v4-text-2)] flex items-center gap-1"
          >
            {timelineCollapsed ? (
              <>
                <ChevronUp size={12} /> Expand
              </>
            ) : (
              <>
                <ChevronDown size={12} /> Collapse
              </>
            )}
          </button>
        </div>
        {!timelineCollapsed ? <StateTimeline builder={builder} /> : null}
      </div>
    </div>
  );
}

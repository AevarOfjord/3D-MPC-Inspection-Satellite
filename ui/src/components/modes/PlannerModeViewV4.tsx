import { Suspense, useMemo, useState } from 'react';
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
import { TargetCardV4 } from '../planner-v4/TargetCardV4';
import { SegmentDetailsCardV4 } from '../planner-v4/SegmentDetailsCardV4';
import { ScanBasicsCardV4 } from '../planner-v4/ScanBasicsCardV4';
import { ScanGeometryAdvancedCardV4 } from '../planner-v4/ScanGeometryAdvancedCardV4';
import { ScanAssetsCardV4 } from '../planner-v4/ScanAssetsCardV4';
import { ScanDiagnosticsCardV4 } from '../planner-v4/ScanDiagnosticsCardV4';
import { ConstraintPresetsCardV4 } from '../planner-v4/ConstraintPresetsCardV4';
import { ValidationNavigatorCardV4 } from '../planner-v4/ValidationNavigatorCardV4';
import { SaveLaunchCardV4 } from '../planner-v4/SaveLaunchCardV4';
import { CoachmarkLayer } from '../planner-v4/CoachmarkLayer';

interface PlannerModeViewV4Props {
  viewMode: 'free' | 'chase' | 'top';
  builder: ReturnType<typeof useMissionBuilder>;
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
      Create a mission using step-by-step guidance. You can switch to advanced mode at any time.
    </InlineBanner>
  );
}

export function PlannerModeViewV4({ viewMode, builder }: PlannerModeViewV4Props) {
  const [timelineCollapsed, setTimelineCollapsed] = useState(true);

  const wizard = usePlannerWizard({
    authoringStep: builder.state.authoringStep,
    setAuthoringStep: builder.actions.setAuthoringStep,
    startFrame: builder.state.startFrame,
    startTargetId: builder.state.startTargetId,
    segments: builder.state.segments,
    validationReport: builder.state.validationReport,
  });

  const onboarding = usePlannerOnboarding();

  const plannerStep = builder.state.authoringStep;
  const viewportMode = plannerStep === 'scan_definition' ? 'scan' : 'mission';
  const showOrbitTargets = plannerStep !== 'scan_definition';

  const contextPanel = useMemo(() => {
    if (plannerStep === 'target') {
      return [<TargetCardV4 key="target" builder={builder} />];
    }
    if (plannerStep === 'segments') {
      return [
        <SegmentComposerCardV4 key="segments-list" builder={builder} />,
        <SegmentDetailsCardV4 key="segments-details" builder={builder} />,
      ];
    }
    if (plannerStep === 'scan_definition') {
      return [
        <ScanBasicsCardV4 key="scan-basics" builder={builder} />,
        <ScanGeometryAdvancedCardV4 key="scan-geometry" builder={builder} />,
        <ScanAssetsCardV4 key="scan-assets" builder={builder} />,
        <ScanDiagnosticsCardV4 key="scan-diagnostics" builder={builder} />,
      ];
    }
    if (plannerStep === 'constraints') {
      return [
        <ConstraintPresetsCardV4 key="constraint-presets" builder={builder} />,
        <SegmentComposerCardV4 key="constraint-segments" builder={builder} emphasizeConstraints />,
        <SegmentDetailsCardV4 key="constraint-details" builder={builder} constraintsOnly />,
      ];
    }
    if (plannerStep === 'validate') {
      return [<ValidationNavigatorCardV4 key="validate" builder={builder} />];
    }
    return [<SaveLaunchCardV4 key="save" builder={builder} />];
  }, [plannerStep, builder]);

  return (
    <div className="flex-1 flex flex-col min-h-0 overflow-hidden">
      <div className="flex-1 min-h-0 p-3 flex gap-3 overflow-hidden">
        <aside className="w-[20rem] shrink-0 overflow-y-auto custom-scrollbar pr-1">
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

        <aside className="w-[28rem] shrink-0 overflow-y-auto custom-scrollbar pr-1 space-y-3">
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

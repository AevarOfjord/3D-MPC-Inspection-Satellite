import { Workflow } from 'lucide-react';

import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import type { usePlannerWizard } from '../../hooks/usePlannerWizard';
import { Panel } from '../ui-v4/Panel';
import { StepBadge } from '../ui-v4/StepBadge';
import { StatusPill } from '../ui-v4/StatusPill';
import { PLANNER_FLOW_STEP_ORDER_V5, type PlannerFlowStepV5 } from '../../types/plannerUx';
import { mapFlowStepToInternalStep } from '../../utils/plannerFlowV5';

interface PlannerStepRailV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
  wizard: ReturnType<typeof usePlannerWizard>;
}

const STEP_LABEL: Record<PlannerFlowStepV5, string> = {
  path_maker: 'Path Maker',
  transfer: 'Transfer',
  obstacles: 'Obstacles',
  path_edit: 'Path Edit',
  mission_saver: 'Mission Saver',
};

const STEP_HINT: Record<PlannerFlowStepV5, string> = {
  path_maker: 'Create paired panels + spirals',
  transfer: 'Start pose to spline endpoint',
  obstacles: 'Place spheres and radius',
  path_edit: 'Refine editable spline',
  mission_saver: 'Validate and save mission',
};

export function PlannerStepRailV4({ builder, wizard }: PlannerStepRailV4Props) {
  const { state, actions } = builder;

  const jumpToStep = (step: PlannerFlowStepV5) => {
    const internalStep = mapFlowStepToInternalStep(step);
    if (state.authoringStep !== internalStep) {
      actions.setAuthoringStep(internalStep);
    }
    if (step === 'path_maker') {
      const firstScan = state.segments.findIndex((segment) => segment.type === 'scan');
      if (firstScan >= 0) {
        actions.selectSegment(firstScan);
      }
    }
    if (step === 'path_edit' && state.selectedSegmentIndex === null && state.segments.length > 0) {
      actions.selectSegment(0);
    }
    wizard.actions.goToStep(step);
  };

  const progressPercent = Math.round(
    (wizard.state.completedCount / PLANNER_FLOW_STEP_ORDER_V5.length) * 100
  );

  return (
    <div id="coachmark-step_rail" className="space-y-3">
      <Panel
        title="Mission Planner"
        actions={<StatusPill tone="info">{progressPercent}% Ready</StatusPill>}
        className="w-[20rem]"
      >
        <div className="space-y-3">
          <div className="space-y-1.5">
            {PLANNER_FLOW_STEP_ORDER_V5.map((step, index) => {
              const active = step === wizard.state.flowStep;
              const status = wizard.state.stepStatuses[step];
              const issues = wizard.state.stepIssueCounts[step];
              const disabled = status === 'locked' && step !== 'path_maker';
              return (
                <button
                  key={step}
                  type="button"
                  onClick={() => jumpToStep(step)}
                  disabled={disabled}
                  className={`v4-focus w-full rounded-[10px] border px-3 py-2.5 text-left transition-colors ${
                    active
                      ? 'border-cyan-500/85 bg-cyan-900/30 hover:border-cyan-400'
                      : 'border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)]/90 hover:border-cyan-700/70'
                  } disabled:opacity-45 disabled:cursor-not-allowed relative z-[1]`}
                >
                  <div className="flex items-center justify-between gap-2">
                    <div className="min-w-0">
                      <div className="text-[10px] uppercase tracking-[0.13em] text-[color:var(--v4-text-3)]">
                        Step {index + 1}
                      </div>
                      <div className="text-sm font-semibold text-[color:var(--v4-text-1)] truncate">
                        {STEP_LABEL[step]}
                      </div>
                      <div className="text-[11px] text-[color:var(--v4-text-3)] truncate">
                        {STEP_HINT[step]}
                      </div>
                    </div>
                    <StepBadge status={status} issueCount={issues} />
                  </div>
                </button>
              );
            })}
          </div>
          <div className="pt-1 grid grid-cols-2 gap-2">
            <button
              type="button"
              onClick={() => wizard.actions.goPrevious()}
              className="v4-focus v4-button px-2 py-1.5 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
            >
              Back
            </button>
            <button
              type="button"
              onClick={() => wizard.actions.goNext()}
              className="v4-focus v4-button px-2 py-1.5 bg-cyan-900/35 border-cyan-700 text-cyan-100"
            >
              Next
            </button>
          </div>
          <div className="flex items-center gap-2 text-[10px] text-[color:var(--v4-text-3)]">
            <Workflow size={12} />
            Use `Alt + 1..5` to jump between planner steps.
          </div>
        </div>
      </Panel>
    </div>
  );
}

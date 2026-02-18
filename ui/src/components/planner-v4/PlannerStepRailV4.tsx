import { Layers3, Sparkles, Workflow } from 'lucide-react';

import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import type { usePlannerWizard } from '../../hooks/usePlannerWizard';
import { Panel } from '../ui-v4/Panel';
import { StepBadge } from '../ui-v4/StepBadge';
import { StatusPill } from '../ui-v4/StatusPill';
import { PLANNER_STEP_ORDER } from '../../types/plannerUx';
import type { PlannerStep } from '../../utils/plannerValidation';

interface PlannerStepRailV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
  wizard: ReturnType<typeof usePlannerWizard>;
}

const STEP_LABEL: Record<PlannerStep, string> = {
  target: 'Target',
  segments: 'Segments',
  scan_definition: 'Scan Definition',
  constraints: 'Constraints',
  validate: 'Validate',
  save_launch: 'Save & Launch',
};

export function PlannerStepRailV4({ builder, wizard }: PlannerStepRailV4Props) {
  const { state, actions } = builder;

  const jumpToStep = (step: PlannerStep) => {
    if (step === 'target') {
      actions.selectSegment(-1);
    }
    if (step === 'scan_definition') {
      const firstScan = state.segments.findIndex((segment) => segment.type === 'scan');
      if (firstScan >= 0) {
        actions.selectSegment(firstScan);
      }
    }
    if (step === 'segments' && state.selectedSegmentIndex === null && state.segments.length > 0) {
      actions.selectSegment(0);
    }
    wizard.actions.goToStep(step);
  };

  const progressPercent = Math.round((wizard.state.completedCount / PLANNER_STEP_ORDER.length) * 100);

  return (
    <div id="coachmark-step_rail" className="space-y-3">
      <Panel
        title="Mission Planner"
        subtitle="Guided step flow for mission creation"
        actions={<StatusPill tone="info">{progressPercent}% Ready</StatusPill>}
        className="w-[20rem]"
      >
        <div className="space-y-3">
          <div className="v4-subtle-panel p-1 flex items-center gap-1">
            <button
              type="button"
              onClick={() => wizard.actions.setUxMode('guided')}
              className={`v4-button v4-focus flex-1 px-2 py-1.5 ${
                wizard.state.uxMode === 'guided'
                  ? 'bg-cyan-700/35 border-cyan-500 text-cyan-100'
                  : 'bg-slate-900 text-slate-300'
              }`}
            >
              Guided
            </button>
            <button
              type="button"
              onClick={() => wizard.actions.setUxMode('advanced')}
              className={`v4-button v4-focus flex-1 px-2 py-1.5 ${
                wizard.state.uxMode === 'advanced'
                  ? 'bg-cyan-700/35 border-cyan-500 text-cyan-100'
                  : 'bg-slate-900 text-slate-300'
              }`}
            >
              Advanced
            </button>
          </div>

          <div className="space-y-1.5">
            {PLANNER_STEP_ORDER.map((step, index) => {
              const active = step === state.authoringStep;
              const status = wizard.state.stepStatuses[step];
              const issues = wizard.state.stepIssueCounts[step];
              const disabled = wizard.state.uxMode === 'guided' && status === 'locked';
              return (
                <button
                  key={step}
                  type="button"
                  onClick={() => jumpToStep(step)}
                  disabled={disabled}
                  className={`v4-focus w-full rounded-[10px] border px-3 py-2.5 text-left transition-colors ${
                    active
                      ? 'border-cyan-500/85 bg-cyan-900/30'
                      : 'border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)]/90 hover:border-cyan-700/70'
                  } disabled:opacity-45 disabled:cursor-not-allowed`}
                >
                  <div className="flex items-center justify-between gap-2">
                    <div className="min-w-0">
                      <div className="text-[10px] uppercase tracking-[0.13em] text-[color:var(--v4-text-3)]">
                        Step {index + 1}
                      </div>
                      <div className="text-sm font-semibold text-[color:var(--v4-text-1)] truncate">
                        {STEP_LABEL[step]}
                      </div>
                    </div>
                    <StepBadge status={status} issueCount={issues} />
                  </div>
                </button>
              );
            })}
          </div>
        </div>
      </Panel>

      <Panel
        title={
          <span className="flex items-center gap-2">
            <Sparkles size={14} />
            Templates
          </span>
        }
        subtitle="Start faster with common mission patterns"
        className="w-[20rem]"
      >
        <div id="coachmark-templates" className="grid gap-2">
          <button
            type="button"
            onClick={() => actions.applyMissionTemplate('quick_inspect')}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-1)]"
          >
            Quick Inspect
          </button>
          <button
            type="button"
            onClick={() => actions.applyMissionTemplate('single_target_spiral')}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-1)]"
          >
            Single Target Spiral
          </button>
          <button
            type="button"
            onClick={() => actions.applyMissionTemplate('transfer_scan')}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-1)]"
          >
            Transfer + Scan
          </button>
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
          <div className="pt-1 flex items-center gap-2 text-[10px] text-[color:var(--v4-text-3)]">
            <Layers3 size={12} />
            Guided mode enforces step readiness. Advanced mode allows free navigation.
          </div>
          <div className="flex items-center gap-2 text-[10px] text-[color:var(--v4-text-3)]">
            <Workflow size={12} />
            Use `Alt + 1..6` to jump steps quickly.
          </div>
        </div>
      </Panel>
    </div>
  );
}

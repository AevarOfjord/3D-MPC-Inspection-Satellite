import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { mapIssuePathToPlannerStep } from '../../utils/plannerValidation';

interface MissionAuthoringStepperProps {
  builder: ReturnType<typeof useMissionBuilder>;
}

const STEPS: Array<{
  id:
    | 'target'
    | 'segments'
    | 'scan_definition'
    | 'constraints'
    | 'validate'
    | 'save_launch';
  label: string;
}> = [
  { id: 'target', label: 'Target' },
  { id: 'segments', label: 'Segments' },
  { id: 'scan_definition', label: 'Scan Definition' },
  { id: 'constraints', label: 'Constraints' },
  { id: 'validate', label: 'Validate' },
  { id: 'save_launch', label: 'Save/Launch' },
];

export function MissionAuthoringStepper({ builder }: MissionAuthoringStepperProps) {
  const { state, actions } = builder;
  const stepIssueCounts = STEPS.reduce<Record<string, number>>((acc, step) => {
    acc[step.id] = 0;
    return acc;
  }, {});
  for (const issue of state.validationReport?.issues ?? []) {
    const step = mapIssuePathToPlannerStep(issue.path);
    stepIssueCounts[step] = (stepIssueCounts[step] ?? 0) + 1;
  }

  const goToStep = (step: (typeof STEPS)[number]['id']) => {
    actions.setAuthoringStep(step);
    if (step === 'target') {
      actions.selectSegment(-1);
      return;
    }
    if (step === 'scan_definition') {
      const firstScanIndex = state.segments.findIndex((segment) => segment.type === 'scan');
      if (firstScanIndex >= 0) {
        actions.selectSegment(firstScanIndex);
        return;
      }
    }
    if (
      state.selectedSegmentIndex === null &&
      state.segments.length > 0 &&
      step !== 'validate' &&
      step !== 'save_launch'
    ) {
      actions.selectSegment(0);
    }
  };

  return (
    <div className="w-72 bg-slate-950/90 backdrop-blur-md border border-slate-800 rounded-lg shadow-2xl p-3">
      <div className="text-[10px] uppercase tracking-[0.14em] text-slate-500 mb-2">
        Planner Steps
      </div>
      <div className="space-y-1.5 mb-3">
        {STEPS.map((step) => {
          const active = state.authoringStep === step.id;
          const issueCount = stepIssueCounts[step.id] ?? 0;
          return (
            <button
              key={step.id}
              type="button"
              onClick={() => goToStep(step.id)}
              className={`w-full flex items-center justify-between px-3 py-2 text-xs rounded border transition-colors ${
                active
                  ? 'bg-cyan-500/20 border-cyan-500 text-cyan-100'
                  : 'bg-slate-900 border-slate-700 text-slate-300 hover:bg-slate-800'
              }`}
            >
              <span>{step.label}</span>
              {issueCount > 0 ? (
                <span className="ml-2 rounded-full bg-amber-900/60 border border-amber-600/50 px-1.5 py-0.5 text-[10px] text-amber-100">
                  {issueCount}
                </span>
              ) : null}
            </button>
          );
        })}
      </div>
      <div className="grid gap-2 text-xs">
        <button
          type="button"
          onClick={() => void actions.validateUnifiedMission()}
          className="px-3 py-1.5 border border-cyan-700 rounded text-cyan-200 hover:bg-cyan-900/30"
        >
          Run Validation
        </button>
      </div>
    </div>
  );
}

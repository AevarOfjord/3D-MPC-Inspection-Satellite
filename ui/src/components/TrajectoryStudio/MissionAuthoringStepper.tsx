import type { useMissionBuilder } from '../../hooks/useMissionBuilder';

interface MissionAuthoringStepperProps {
  builder: ReturnType<typeof useMissionBuilder>;
}

const STEPS: Array<{
  id: 'target' | 'segments' | 'constraints' | 'validate' | 'save_launch';
  label: string;
}> = [
  { id: 'target', label: 'Target' },
  { id: 'segments', label: 'Segments' },
  { id: 'constraints', label: 'Constraints' },
  { id: 'validate', label: 'Validate' },
  { id: 'save_launch', label: 'Save/Launch' },
];

export function MissionAuthoringStepper({ builder }: MissionAuthoringStepperProps) {
  const { state, actions } = builder;

  const goToStep = (step: (typeof STEPS)[number]['id']) => {
    actions.setAuthoringStep(step);
    if (step === 'target') {
      actions.selectSegment(-1);
      return;
    }
    if (state.selectedSegmentIndex === null && state.segments.length > 0) {
      actions.selectSegment(0);
    }
  };

  return (
    <div className="w-[44rem] bg-slate-950/90 backdrop-blur-md border border-slate-800 rounded-lg shadow-2xl p-3">
      <div className="flex items-center gap-2 mb-3">
        {STEPS.map((step) => {
          const active = state.authoringStep === step.id;
          return (
            <button
              key={step.id}
              type="button"
              onClick={() => goToStep(step.id)}
              className={`px-3 py-1.5 text-xs rounded border transition-colors ${
                active
                  ? 'bg-cyan-500/20 border-cyan-500 text-cyan-100'
                  : 'bg-slate-900 border-slate-700 text-slate-300 hover:bg-slate-800'
              }`}
            >
              {step.label}
            </button>
          );
        })}
      </div>
      <div className="flex flex-wrap gap-2 text-xs">
        <button
          type="button"
          onClick={() => actions.applyMissionTemplate('quick_inspect')}
          className="px-3 py-1 border border-slate-700 rounded text-slate-300 hover:bg-slate-800"
        >
          Template: Quick Inspect
        </button>
        <button
          type="button"
          onClick={() => actions.applyMissionTemplate('single_target_spiral')}
          className="px-3 py-1 border border-slate-700 rounded text-slate-300 hover:bg-slate-800"
        >
          Template: Single Target Spiral
        </button>
        <button
          type="button"
          onClick={() => actions.applyMissionTemplate('transfer_scan')}
          className="px-3 py-1 border border-slate-700 rounded text-slate-300 hover:bg-slate-800"
        >
          Template: Transfer + Scan
        </button>
        <button
          type="button"
          onClick={() => void actions.validateUnifiedMission()}
          className="px-3 py-1 border border-cyan-700 rounded text-cyan-200 hover:bg-cyan-900/30"
        >
          Run Validation
        </button>
      </div>
    </div>
  );
}

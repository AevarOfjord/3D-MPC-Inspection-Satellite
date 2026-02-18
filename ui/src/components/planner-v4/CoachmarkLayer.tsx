import type { CoachmarkId } from '../../types/plannerUx';

interface CoachmarkLayerProps {
  open: boolean;
  visibleCoachmarks: CoachmarkId[];
  onDismiss: (id: CoachmarkId) => void;
  onClose: () => void;
}

const CARD_COPY: Record<CoachmarkId, { title: string; body: string }> = {
  step_rail: {
    title: 'Step Rail',
    body: 'Use guided mode for first missions. Advanced mode unlocks direct step jumps.',
  },
  templates: {
    title: 'Templates',
    body: 'Start from Quick Inspect or Transfer+Scan to reduce setup time.',
  },
  context_panel: {
    title: 'Context Panel',
    body: 'Right panel changes by step. Keep edits focused to lower validation churn.',
  },
  validation: {
    title: 'Validation Navigator',
    body: 'Click any issue to jump directly to the field that needs fixing.',
  },
  save_launch: {
    title: 'Save / Launch',
    body: 'Preflight checklist must pass before save and launch controls activate.',
  },
};

export function CoachmarkLayer({ open, visibleCoachmarks, onDismiss, onClose }: CoachmarkLayerProps) {
  if (!open) return null;

  return (
    <div className="absolute inset-0 pointer-events-none z-40">
      <div className="absolute top-3 right-4 pointer-events-auto">
        <button
          type="button"
          onClick={onClose}
          className="v4-focus v4-button px-2 py-1 bg-slate-900/90 text-slate-200"
        >
          Close Tour
        </button>
      </div>

      <div className="absolute left-[22rem] top-4 w-72 space-y-2 pointer-events-auto">
        {visibleCoachmarks.map((id) => (
          <div key={id} className="rounded-[10px] border border-cyan-600/70 bg-cyan-950/85 text-cyan-100 px-3 py-2 shadow-xl">
            <div className="text-[11px] uppercase tracking-[0.12em] font-semibold">{CARD_COPY[id].title}</div>
            <div className="text-xs mt-1 text-cyan-50/90">{CARD_COPY[id].body}</div>
            <button
              type="button"
              onClick={() => onDismiss(id)}
              className="mt-2 text-[10px] uppercase tracking-[0.12em] underline"
            >
              Dismiss Tip
            </button>
          </div>
        ))}
      </div>
    </div>
  );
}

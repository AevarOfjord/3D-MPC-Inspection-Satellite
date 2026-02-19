import type { CoachmarkId } from '../../types/plannerUx';

interface CoachmarkLayerProps {
  open: boolean;
  visibleCoachmarks: CoachmarkId[];
  onDismiss: (id: CoachmarkId) => void;
  onClose: () => void;
}

const CARD_COPY: Record<CoachmarkId, { title: string; body: string }> = {
  step_rail: {
    title: '5-Step Rail',
    body: 'Follow the steps in order for the simplest mission-creation flow.',
  },
  templates: {
    title: 'Quick Starts',
    body: 'Apply a starter mission, then tune each step in the guided flow.',
  },
  context_panel: {
    title: 'Step Controls',
    body: 'The right panel only shows controls for the current task to reduce clutter.',
  },
  path_edit: {
    title: 'Path Edit',
    body: 'Drag spline points in the viewport. Use add/remove + undo/redo in the panel.',
  },
  save: {
    title: 'Mission Saver',
    body: 'Validation runs here. Fix issues, save mission, then open Runner to launch.',
  },
  validation: {
    title: 'Validation',
    body: 'Validation issues are grouped and clickable for fast fixes.',
  },
  save_launch: {
    title: 'Save',
    body: 'Final step for preflight and mission save.',
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

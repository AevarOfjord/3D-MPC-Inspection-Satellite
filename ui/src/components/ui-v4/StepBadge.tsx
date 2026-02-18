import type { PlannerStepStatus } from '../../types/plannerUx';

interface StepBadgeProps {
  status: PlannerStepStatus;
  issueCount?: number;
}

const STATUS_CLASS: Record<PlannerStepStatus, string> = {
  locked: 'border-slate-700 text-slate-500 bg-slate-900/70',
  ready: 'border-cyan-600/70 text-cyan-200 bg-cyan-950/30',
  complete: 'border-emerald-600/70 text-emerald-200 bg-emerald-950/30',
  error: 'border-amber-600/70 text-amber-100 bg-amber-950/40',
};

export function StepBadge({ status, issueCount = 0 }: StepBadgeProps) {
  return (
    <span
      className={`inline-flex items-center gap-1 rounded-full border px-2 py-0.5 text-[10px] uppercase tracking-[0.12em] ${STATUS_CLASS[status]}`}
    >
      {status}
      {issueCount > 0 ? <span className="font-semibold">{issueCount}</span> : null}
    </span>
  );
}

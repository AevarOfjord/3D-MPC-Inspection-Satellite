import type { ReactNode } from 'react';

type Tone = 'neutral' | 'info' | 'success' | 'warning' | 'danger';

interface StatusPillProps {
  tone?: Tone;
  children: ReactNode;
}

const TONE_CLASS: Record<Tone, string> = {
  neutral: 'border-slate-700 text-slate-300 bg-slate-900/60',
  info: 'border-cyan-700/60 text-cyan-200 bg-cyan-950/40',
  success: 'border-emerald-700/60 text-emerald-200 bg-emerald-950/40',
  warning: 'border-amber-700/60 text-amber-200 bg-amber-950/40',
  danger: 'border-red-700/60 text-red-200 bg-red-950/40',
};

export function StatusPill({ tone = 'neutral', children }: StatusPillProps) {
  return (
    <span className={`inline-flex items-center rounded-full border px-2 py-0.5 text-[10px] ${TONE_CLASS[tone]}`}>
      {children}
    </span>
  );
}

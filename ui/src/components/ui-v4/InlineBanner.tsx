import type { ReactNode } from 'react';

type BannerTone = 'info' | 'success' | 'warning' | 'danger';

interface InlineBannerProps {
  tone?: BannerTone;
  title?: ReactNode;
  children: ReactNode;
  actions?: ReactNode;
  className?: string;
}

const TONE_CLASS: Record<BannerTone, string> = {
  info: 'border-cyan-700/50 bg-cyan-950/35 text-cyan-100',
  success: 'border-emerald-700/50 bg-emerald-950/35 text-emerald-100',
  warning: 'border-amber-700/50 bg-amber-950/35 text-amber-100',
  danger: 'border-red-700/50 bg-red-950/35 text-red-100',
};

export function InlineBanner({
  tone = 'info',
  title,
  children,
  actions,
  className = '',
}: InlineBannerProps) {
  return (
    <div className={`rounded-[10px] border px-3 py-2 ${TONE_CLASS[tone]} ${className}`}>
      {title ? <div className="text-[11px] uppercase tracking-[0.1em] font-semibold mb-1">{title}</div> : null}
      <div className="text-xs">{children}</div>
      {actions ? <div className="mt-2 flex items-center gap-2">{actions}</div> : null}
    </div>
  );
}

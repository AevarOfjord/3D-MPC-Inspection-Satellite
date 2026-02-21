import type { ReactNode } from 'react';

interface PanelProps {
  title?: ReactNode;
  subtitle?: ReactNode;
  actions?: ReactNode;
  className?: string;
  bodyClassName?: string;
  children: ReactNode;
}

export function Panel({ title, subtitle, actions, className = '', bodyClassName = '', children }: PanelProps) {
  return (
    <section className={`v4-panel overflow-hidden ${className}`}>
      {title || actions ? (
        <header className="px-4 py-3 border-b border-[color:var(--v4-border)]/80 bg-[color:var(--v4-surface-2)]/85 flex items-start justify-between gap-3">
          <div>
            {title ? (
              <h3 className="text-sm font-semibold tracking-wide text-[color:var(--v4-text-1)]">{title}</h3>
            ) : null}
            {subtitle ? (
              <p className="text-xs mt-0.5 text-[color:var(--v4-text-3)]">{subtitle}</p>
            ) : null}
          </div>
          {actions ? <div className="shrink-0">{actions}</div> : null}
        </header>
      ) : null}
      <div className={`p-4 ${bodyClassName}`}>{children}</div>
    </section>
  );
}

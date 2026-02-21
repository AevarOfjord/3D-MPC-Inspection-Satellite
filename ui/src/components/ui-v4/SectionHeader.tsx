import type { ReactNode } from 'react';

interface SectionHeaderProps {
  title: ReactNode;
  description?: ReactNode;
  actions?: ReactNode;
  className?: string;
}

export function SectionHeader({ title, description, actions, className = '' }: SectionHeaderProps) {
  return (
    <div className={`flex items-start justify-between gap-3 ${className}`}>
      <div>
        <h4 className="text-xs uppercase tracking-[0.14em] font-semibold text-[color:var(--v4-text-2)]">
          {title}
        </h4>
        {description ? (
          <p className="text-xs mt-1 text-[color:var(--v4-text-3)]">{description}</p>
        ) : null}
      </div>
      {actions ? <div>{actions}</div> : null}
    </div>
  );
}

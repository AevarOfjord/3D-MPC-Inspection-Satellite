import type { ReactNode } from 'react';

interface FieldRowProps {
  label: ReactNode;
  hint?: ReactNode;
  children: ReactNode;
  className?: string;
}

export function FieldRow({ label, hint, children, className = '' }: FieldRowProps) {
  return (
    <div className={`space-y-1 ${className}`}>
      <div className="flex items-center justify-between gap-2">
        <label className="text-[11px] uppercase tracking-[0.14em] text-[color:var(--v4-text-3)] font-semibold">
          {label}
        </label>
        {hint ? <span className="text-[10px] text-[color:var(--v4-text-3)]">{hint}</span> : null}
      </div>
      {children}
    </div>
  );
}

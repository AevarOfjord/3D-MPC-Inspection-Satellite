import type { ReactNode } from 'react';

interface ActionBarProps {
  children: ReactNode;
  className?: string;
}

export function ActionBar({ children, className = '' }: ActionBarProps) {
  return (
    <div className={`flex items-center gap-2 rounded-[10px] border border-[color:var(--v4-border)] bg-[color:var(--v4-surface-2)]/80 p-2 ${className}`}>
      {children}
    </div>
  );
}

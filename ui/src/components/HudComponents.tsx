import React from 'react';

/**
 * A glassmorphism panel container with a sci-fi border.
 */
export function HudPanel({ children, className = '', title }: { children: React.ReactNode; className?: string; title?: React.ReactNode }) {
  return (
    <div className={`
      relative overflow-hidden
      bg-slate-950/80 backdrop-blur-md
      border border-slate-700/50 rounded-lg
      shadow-xl
      ${className}
    `}>
        {/* Sci-Fi Decorative Corners */}
        <div className="absolute top-0 left-0 w-2 h-2 border-t-2 border-l-2 border-cyan-500/50 rounded-tl-sm pointer-events-none" />
        <div className="absolute top-0 right-0 w-2 h-2 border-t-2 border-r-2 border-cyan-500/50 rounded-tr-sm pointer-events-none" />
        <div className="absolute bottom-0 left-0 w-2 h-2 border-b-2 border-l-2 border-cyan-500/50 rounded-bl-sm pointer-events-none" />
        <div className="absolute bottom-0 right-0 w-2 h-2 border-b-2 border-r-2 border-cyan-500/50 rounded-br-sm pointer-events-none" />

        {title && (
            <div className="px-3 py-2 border-b border-slate-800/50 bg-slate-900/50 flex items-center gap-2">
                <div className="w-1 h-3 bg-cyan-500 rounded-full shadow-[0_0_5px_rgba(6,182,212,0.8)]" />
                <span className="text-xs font-bold uppercase tracking-wider text-cyan-400 text-shadow-glow">
                    {title}
                </span>
            </div>
        )}
      <div className="p-3">
        {children}
      </div>
    </div>
  );
}

import React from 'react';

/**
 * A glassmorphism panel container with a sci-fi border.
 * Pass `live={true}` to pulse the title indicator dot while data is streaming.
 */
export function HudPanel({
  children,
  className = '',
  title,
  live = false,
}: {
  children: React.ReactNode;
  className?: string;
  title?: React.ReactNode;
  live?: boolean;
}) {
  return (
    <div className={`
      relative overflow-hidden
      bg-slate-950/80 backdrop-blur-md
      border border-slate-700/50 rounded-lg
      shadow-xl
      ${className}
    `}>
      {/* Sci-Fi Decorative Corners */}
      <div className="absolute top-0 left-0 w-3 h-3 border-t-2 border-l-2 border-cyan-500/70 rounded-tl-sm pointer-events-none" />
      <div className="absolute top-0 right-0 w-3 h-3 border-t-2 border-r-2 border-cyan-500/70 rounded-tr-sm pointer-events-none" />
      <div className="absolute bottom-0 left-0 w-3 h-3 border-b-2 border-l-2 border-cyan-500/70 rounded-bl-sm pointer-events-none" />
      <div className="absolute bottom-0 right-0 w-3 h-3 border-b-2 border-r-2 border-cyan-500/70 rounded-br-sm pointer-events-none" />

      {title && (
        <div className="px-3 py-2 border-b border-slate-800/50 bg-slate-900/50 flex items-center gap-2">
          <div className={`w-1.5 h-3 bg-cyan-500 rounded-full shadow-[0_0_5px_rgba(6,182,212,0.8)] ${live ? 'animate-pulse' : ''}`} />
          <span className="text-xs font-bold uppercase tracking-wider text-cyan-400">
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

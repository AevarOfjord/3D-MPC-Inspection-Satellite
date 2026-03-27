import React, { useState } from 'react';

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

export function HudSection({
  title,
  defaultOpen = true,
  children,
}: {
  title: React.ReactNode;
  defaultOpen?: boolean;
  children: React.ReactNode;
}) {
  const [open, setOpen] = useState(defaultOpen);

  return (
    <section className="border border-slate-800/70 rounded-md bg-slate-900/30 overflow-hidden">
      <button
        type="button"
        onClick={() => setOpen((prev) => !prev)}
        className="w-full flex items-center justify-between px-3 py-2 text-left text-[10px] font-bold uppercase tracking-wider text-slate-300 bg-slate-900/60 hover:bg-slate-900/80 transition-colors"
      >
        <span>{title}</span>
        <span className="text-slate-500">{open ? '−' : '+'}</span>
      </button>
      {open && <div className="p-3">{children}</div>}
    </section>
  );
}

type HudInputBaseProps = {
  label: React.ReactNode;
  className?: string;
  step?: number | string;
  min?: number | string;
};

type HudNumberInputProps = HudInputBaseProps & {
  type: 'number';
  value: number;
  onChange: (value: number) => void;
};

type HudTextInputProps = HudInputBaseProps & {
  type?: 'text';
  value: string;
  onChange: (value: string) => void;
};

type HudInputProps = HudNumberInputProps | HudTextInputProps;

export function HudInput(props: HudInputProps) {
  const { label, className = '', step, min } = props;

  return (
    <label className={`block ${className}`}>
      <span className="text-[10px] font-bold uppercase tracking-wider text-slate-400 block mb-1">
        {label}
      </span>
      <input
        type={props.type ?? 'text'}
        value={props.value}
        step={step}
        min={min}
        onChange={(event) => {
          if (props.type === 'number') {
            props.onChange(event.currentTarget.valueAsNumber);
            return;
          }
          props.onChange(event.currentTarget.value);
        }}
        className="w-full bg-slate-900/50 border border-slate-700 text-slate-200 text-xs rounded-sm px-2 py-1.5 outline-none focus:border-cyan-500"
      />
    </label>
  );
}

type HudButtonIcon = React.ComponentType<{ size?: number; className?: string }>;

export function HudButton({
  children,
  variant = 'secondary',
  size = 'md',
  className = '',
  icon: Icon,
  ...buttonProps
}: React.ButtonHTMLAttributes<HTMLButtonElement> & {
  variant?: 'primary' | 'secondary' | 'danger';
  size?: 'sm' | 'md';
  icon?: HudButtonIcon;
}) {
  const variantClass =
    variant === 'primary'
      ? 'bg-cyan-500/20 border-cyan-500/60 text-cyan-100 hover:bg-cyan-500/30'
      : variant === 'danger'
        ? 'bg-red-500/10 border-red-500/50 text-red-100 hover:bg-red-500/20'
        : 'bg-slate-900/50 border-slate-700 text-slate-200 hover:bg-slate-800';
  const sizeClass = size === 'sm' ? 'px-3 py-1.5 text-xs' : 'px-3 py-2 text-sm';

  return (
    <button
      type="button"
      {...buttonProps}
      className={`inline-flex items-center justify-center gap-2 rounded border font-semibold transition-colors disabled:opacity-50 disabled:cursor-not-allowed ${variantClass} ${sizeClass} ${className}`}
    >
      {Icon ? <Icon size={12} /> : null}
      {children}
    </button>
  );
}

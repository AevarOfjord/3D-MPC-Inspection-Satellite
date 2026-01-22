import React from 'react';
import { type LucideIcon } from 'lucide-react';

// --- Types ---
type Variant = 'primary' | 'secondary' | 'danger' | 'ghost';
type Size = 'sm' | 'md' | 'lg';

// --- Constants ---
const VARIANTS: Record<Variant, string> = {
  primary: 'bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-200 border-cyan-500/50 shadow-[0_0_10px_rgba(6,182,212,0.15)] hover:shadow-[0_0_15px_rgba(6,182,212,0.3)]',
  secondary: 'bg-slate-800/50 hover:bg-slate-700/50 text-slate-300 border-slate-600/50 hover:border-slate-500',
  danger: 'bg-red-500/20 hover:bg-red-500/30 text-red-200 border-red-500/50 shadow-[0_0_10px_rgba(239,68,68,0.15)]',
  ghost: 'bg-transparent hover:bg-white/5 text-slate-400 hover:text-white border-transparent'
};

const SIZES: Record<Size, string> = {
  sm: 'px-2 py-1 text-xs',
  md: 'px-4 py-2 text-sm',
  lg: 'px-6 py-3 text-base'
};

// --- Components ---

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

/**
 * A stylized button with hover effects and glows.
 */
export function HudButton({ 
    children, 
    onClick, 
    variant = 'primary', 
    size = 'md', 
    className = '', 
    disabled = false,
    icon: Icon
}: { 
    children: React.ReactNode; 
    onClick?: () => void; 
    variant?: Variant; 
    size?: Size; 
    className?: string;
    disabled?: boolean;
    icon?: LucideIcon;
}) {
  return (
    <button 
        onClick={onClick}
        disabled={disabled}
        className={`
            relative group overflow-hidden
            flex items-center justify-center gap-2
            font-mono font-bold uppercase tracking-wider
            border transition-all duration-200
            disabled:opacity-50 disabled:cursor-not-allowed
            rounded-sm
            ${VARIANTS[variant]}
            ${SIZES[size]}
            ${className}
        `}
    >
        {/* Animated Scanline effect on hover */}
        <div className="absolute inset-0 bg-gradient-to-r from-transparent via-white/10 to-transparent -translate-x-full group-hover:translate-x-full transition-transform duration-700 ease-out pointer-events-none" />
        
        {Icon && <Icon size={size === 'sm' ? 14 : 18} />}
        {children}
    </button>
  );
}

/**
 * A styled input field.
 */
export function HudInput({ 
    label, 
    value, 
    onChange, 
    type = "text", 
    step, 
    min, 
    max,
    unit,
    placeholder,
    allowEmpty = false,
}: {
    label?: string;
    value: string | number | null | undefined;
    onChange: (val: any) => void;
    type?: "text" | "number";
    step?: number;
    min?: number;
    max?: number;
    unit?: string;
    placeholder?: string;
    allowEmpty?: boolean;
}) {
    const isNumber = type === 'number';
    const safeValue =
        isNumber && typeof value === 'number' && Number.isNaN(value)
            ? ''
            : value ?? '';
    return (
        <div className="flex flex-col gap-1">
            {label && (
                <label className="text-[10px] font-bold uppercase tracking-wider text-slate-400">
                    {label}
                </label>
            )}
            <div className="relative group">
                <input
                    type={type}
                    value={safeValue}
                    step={step}
                    min={min}
                    max={max}
                    placeholder={placeholder}
                    onChange={(e) => {
                        if (type === 'number') {
                            const raw = e.target.value;
                            if (raw === '') {
                                if (allowEmpty) onChange(null);
                                return;
                            }
                            const parsed = parseFloat(raw);
                            if (!Number.isFinite(parsed)) return;
                            onChange(parsed);
                            return;
                        }
                        onChange(e.target.value);
                    }}
                    className={`
                        w-full bg-slate-900/50 border border-slate-700 
                        focus:border-cyan-500 focus:ring-1 focus:ring-cyan-500/50 
                        text-slate-200 font-mono text-sm
                        rounded-sm px-2 py-1.5 outline-none transition-all
                        placeholder:text-slate-600
                    `}
                />
                {unit && (
                    <span className="absolute right-2 top-1/2 -translate-y-1/2 text-xs text-slate-500 font-mono">
                        {unit}
                    </span>
                )}
            </div>
        </div>
    );
}

/**
 * A collapsible section wrapper.
 */
export function HudSection({ title, children, defaultOpen = false }: { title: string; children: React.ReactNode; defaultOpen?: boolean }) {
    const [isOpen, setIsOpen] = React.useState(defaultOpen);

    return (
        <div className="border border-slate-800/50 rounded-sm overflow-hidden bg-slate-900/20">
            <button 
                onClick={() => setIsOpen(!isOpen)}
                className="w-full px-3 py-2 flex items-center justify-between text-left hover:bg-white/5 transition-colors"
            >
                <span className="text-xs font-bold uppercase tracking-wider text-slate-400 group-hover:text-cyan-400 transition-colors">
                    {title}
                </span>
                <span className={`text-[10px] text-slate-600 transform transition-transform ${isOpen ? 'rotate-180' : ''}`}>
                    ▼
                </span>
            </button>
            {isOpen && (
                <div className="p-3 border-t border-slate-800/50 space-y-3 animate-in fade-in slide-in-from-top-1 duration-200">
                    {children}
                </div>
            )}
        </div>
    );
}

import { useEffect, useMemo, useState } from 'react';

export interface CommandPaletteItem {
  id: string;
  label: string;
  description?: string;
  shortcut?: string;
  keywords?: string[];
  onSelect: () => void;
}

interface CommandPaletteProps {
  open: boolean;
  title?: string;
  items: CommandPaletteItem[];
  onClose: () => void;
}

export function CommandPalette({
  open,
  title = 'Command Palette',
  items,
  onClose,
}: CommandPaletteProps) {
  const [query, setQuery] = useState('');
  const [activeIndex, setActiveIndex] = useState(0);

  useEffect(() => {
    if (!open) return;
    setQuery('');
    setActiveIndex(0);
  }, [open]);

  const filtered = useMemo(() => {
    const normalized = query.trim().toLowerCase();
    if (!normalized) return items;
    return items.filter((item) => {
      const haystack = [
        item.label,
        item.description ?? '',
        ...(item.keywords ?? []),
      ]
        .join(' ')
        .toLowerCase();
      return haystack.includes(normalized);
    });
  }, [items, query]);

  useEffect(() => {
    if (activeIndex < filtered.length) return;
    setActiveIndex(0);
  }, [activeIndex, filtered.length]);

  useEffect(() => {
    if (!open) return;
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        event.preventDefault();
        onClose();
        return;
      }
      if (event.key === 'ArrowDown') {
        event.preventDefault();
        if (filtered.length === 0) return;
        setActiveIndex((idx) => (idx + 1) % filtered.length);
        return;
      }
      if (event.key === 'ArrowUp') {
        event.preventDefault();
        if (filtered.length === 0) return;
        setActiveIndex((idx) => (idx - 1 + filtered.length) % filtered.length);
        return;
      }
      if (event.key === 'Enter') {
        event.preventDefault();
        const active = filtered[activeIndex];
        if (!active) return;
        onClose();
        active.onSelect();
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [activeIndex, filtered, onClose, open]);

  if (!open) return null;

  return (
    <div className="fixed inset-0 z-[160] bg-slate-950/70 backdrop-blur-[1px] p-4 flex items-start justify-center">
      <div className="w-full max-w-2xl mt-20 rounded-lg border border-slate-700 bg-slate-950/95 shadow-2xl overflow-hidden">
        <div className="px-4 py-3 border-b border-slate-800">
          <div className="text-sm font-semibold text-slate-100">{title}</div>
        </div>
        <div className="p-3 border-b border-slate-800">
          <input
            autoFocus
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            placeholder="Search commands..."
            className="w-full bg-slate-900 border border-slate-700 rounded px-3 py-2 text-sm text-slate-100 outline-none focus:border-cyan-500"
          />
        </div>
        <div className="max-h-[24rem] overflow-y-auto custom-scrollbar p-2">
          {filtered.length === 0 ? (
            <div className="text-xs text-slate-500 px-2 py-3">No matching commands.</div>
          ) : (
            filtered.map((item, index) => {
              const active = index === activeIndex;
              return (
                <button
                  key={item.id}
                  type="button"
                  onMouseEnter={() => setActiveIndex(index)}
                  onClick={() => {
                    onClose();
                    item.onSelect();
                  }}
                  className={`w-full text-left px-3 py-2 rounded border transition-colors mb-1 ${
                    active
                      ? 'border-cyan-500/70 bg-cyan-900/25 text-cyan-100'
                      : 'border-slate-800 bg-slate-900/40 text-slate-200 hover:bg-slate-800'
                  }`}
                >
                  <div className="flex items-center justify-between gap-3">
                    <span className="text-sm">{item.label}</span>
                    {item.shortcut ? (
                      <span className="text-[10px] uppercase tracking-wide text-slate-400">
                        {item.shortcut}
                      </span>
                    ) : null}
                  </div>
                  {item.description ? (
                    <div className="text-xs text-slate-400 mt-0.5">{item.description}</div>
                  ) : null}
                </button>
              );
            })
          )}
        </div>
      </div>
    </div>
  );
}

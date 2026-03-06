interface ShortcutItem {
  keys: string;
  description: string;
}

interface ShortcutHelpPanelProps {
  open: boolean;
  onClose: () => void;
}

const SHORTCUTS: ShortcutItem[] = [
  { keys: 'Ctrl/Cmd + K', description: 'Open command palette' },
  { keys: '?', description: 'Open shortcut help' },
  { keys: 'Ctrl/Cmd + 1..5', description: 'Switch top-level mode (Viewer, Studio, Runner, Data, Settings)' },
];

export function ShortcutHelpPanel({ open, onClose }: ShortcutHelpPanelProps) {
  if (!open) return null;

  return (
    <div className="fixed inset-0 z-[155] bg-slate-950/70 backdrop-blur-[1px] p-4 flex items-start justify-center">
      <div className="w-full max-w-2xl mt-20 rounded-lg border border-slate-700 bg-slate-950/95 shadow-2xl overflow-hidden">
        <div className="px-4 py-3 border-b border-slate-800 flex items-center justify-between">
          <h2 className="text-sm font-semibold text-slate-100">Keyboard Shortcuts</h2>
          <button
            type="button"
            onClick={onClose}
            className="px-2 py-1 text-xs border border-slate-700 rounded text-slate-300 hover:bg-slate-800"
          >
            Close
          </button>
        </div>
        <div className="p-3 space-y-2">
          {SHORTCUTS.map((item) => (
            <div
              key={item.keys}
              className="rounded border border-slate-800 bg-slate-900/40 px-3 py-2 flex items-start justify-between gap-4"
            >
              <div className="text-xs text-slate-200">{item.description}</div>
              <div className="text-[10px] uppercase tracking-wide text-slate-400 text-right">
                {item.keys}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

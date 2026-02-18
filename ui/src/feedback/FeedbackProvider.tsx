import {
  useCallback,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from 'react';
import {
  DialogContext,
  ToastContext,
  type DialogContextValue,
  type DialogIntent,
  type DialogResult,
  type ToastContextValue,
  type ToastIntent,
  type ToastTone,
} from './feedbackContext';

type ActiveDialog = {
  intent: DialogIntent;
  resolve: (value: DialogResult) => void;
};

type ToastEntry = ToastIntent & { id: number; tone: ToastTone };

const toastToneClass: Record<ToastTone, string> = {
  info: 'border-cyan-700/50 bg-cyan-950/40 text-cyan-100',
  success: 'border-emerald-700/50 bg-emerald-950/40 text-emerald-100',
  error: 'border-red-700/50 bg-red-950/40 text-red-100',
};

const dialogToneClass: Record<ToastTone, string> = {
  info: 'border-cyan-700/50',
  success: 'border-emerald-700/50',
  error: 'border-red-700/50',
};

export function FeedbackProvider({ children }: { children: ReactNode }) {
  const [toasts, setToasts] = useState<ToastEntry[]>([]);
  const [activeDialog, setActiveDialog] = useState<ActiveDialog | null>(null);
  const [promptValue, setPromptValue] = useState('');
  const [formValues, setFormValues] = useState<Record<string, string>>({});
  const toastIdRef = useRef(0);

  const showToast = useCallback((intent: ToastIntent) => {
    toastIdRef.current += 1;
    const id = toastIdRef.current;
    const entry: ToastEntry = {
      id,
      tone: intent.tone ?? 'info',
      durationMs: intent.durationMs ?? 3200,
      title: intent.title,
      message: intent.message,
      actionLabel: intent.actionLabel,
      onAction: intent.onAction,
    };
    setToasts((prev) => [...prev, entry]);
    window.setTimeout(() => {
      setToasts((prev) => prev.filter((toast) => toast.id !== id));
    }, entry.durationMs);
  }, []);

  const openDialog = useCallback((intent: DialogIntent) => {
    return new Promise<DialogResult>((resolve) => {
      if (intent.type === 'prompt') {
        setPromptValue(intent.defaultValue ?? '');
      }
      if (intent.type === 'form') {
        const initial = intent.fields.reduce<Record<string, string>>((acc, field) => {
          acc[field.id] = field.defaultValue ?? '';
          return acc;
        }, {});
        setFormValues(initial);
      }
      setActiveDialog({ intent, resolve });
    });
  }, []);

  const dialog = useMemo<DialogContextValue>(
    () => ({
      openDialog,
      alert: async (message, options) => {
        await openDialog({
          type: 'alert',
          message,
          ...options,
        });
      },
      confirm: async (message, options) => {
        const result = await openDialog({
          type: 'confirm',
          message,
          ...options,
        });
        return Boolean(result);
      },
      prompt: async (message, options) => {
        const result = await openDialog({
          type: 'prompt',
          message,
          ...options,
        });
        return typeof result === 'string' ? result : null;
      },
      form: async (options) => {
        const result = await openDialog({
          ...options,
          type: 'form',
        });
        if (!result || typeof result !== 'object' || Array.isArray(result)) return null;
        return result;
      },
    }),
    [openDialog]
  );

  const toast = useMemo<ToastContextValue>(
    () => ({
      showToast,
    }),
    [showToast]
  );

  const closeDialog = () => {
    setActiveDialog(null);
  };

  const handleDialogConfirm = () => {
    if (!activeDialog) return;
    if (activeDialog.intent.type === 'prompt') {
      activeDialog.resolve(promptValue);
      closeDialog();
      return;
    }
    if (activeDialog.intent.type === 'form') {
      activeDialog.resolve(formValues);
      closeDialog();
      return;
    }
    activeDialog.resolve(true);
    closeDialog();
  };

  const handleDialogCancel = () => {
    if (!activeDialog) return;
    if (activeDialog.intent.type === 'prompt' || activeDialog.intent.type === 'form') {
      activeDialog.resolve(null);
      closeDialog();
      return;
    }
    activeDialog.resolve(false);
    closeDialog();
  };

  const dialogTone = activeDialog?.intent.tone ?? 'info';
  const dialogTitle =
    activeDialog?.intent.title ??
    (activeDialog?.intent.type === 'confirm'
      ? 'Confirm Action'
      : activeDialog?.intent.type === 'prompt'
      ? 'Enter Value'
      : activeDialog?.intent.type === 'form'
      ? 'Fill Details'
      : 'Notice');

  const dialogCanConfirm = useMemo(() => {
    if (!activeDialog) return false;
    if (activeDialog.intent.type === 'prompt') {
      return !activeDialog.intent.requireNonEmpty || promptValue.trim().length > 0;
    }
    if (activeDialog.intent.type === 'form') {
      return activeDialog.intent.fields.every((field) => {
        if (!field.required) return true;
        return (formValues[field.id] ?? '').trim().length > 0;
      });
    }
    return true;
  }, [activeDialog, promptValue, formValues]);

  return (
    <DialogContext.Provider value={dialog}>
      <ToastContext.Provider value={toast}>
        {children}

        <div className="fixed top-4 right-4 z-[120] space-y-2 pointer-events-none">
          {toasts.map((entry) => (
            <div
              key={entry.id}
              className={`min-w-72 max-w-96 rounded border px-3 py-2 shadow-xl backdrop-blur-sm pointer-events-auto ${toastToneClass[entry.tone]}`}
            >
              {entry.title ? (
                <div className="text-xs font-semibold uppercase tracking-wide mb-0.5">
                  {entry.title}
                </div>
              ) : null}
              <div className="text-xs">{entry.message}</div>
              {entry.actionLabel && entry.onAction ? (
                <button
                  type="button"
                  className="mt-2 px-2 py-1 text-[10px] uppercase tracking-wide rounded border border-current/60"
                  onClick={() => entry.onAction?.()}
                >
                  {entry.actionLabel}
                </button>
              ) : null}
            </div>
          ))}
        </div>

        {activeDialog ? (
          <div className="fixed inset-0 z-[130] bg-slate-950/70 backdrop-blur-[1px] flex items-center justify-center p-4">
            <div
              role="dialog"
              aria-modal="true"
              aria-label={dialogTitle}
              className={`w-full max-w-lg rounded-lg border bg-slate-950 text-slate-200 shadow-2xl ${dialogToneClass[dialogTone]}`}
            >
              <div className="px-4 py-3 border-b border-slate-800">
                <div className="text-sm font-semibold">{dialogTitle}</div>
              </div>
              <div className="px-4 py-4 space-y-3">
                {activeDialog.intent.message ? (
                  <div className="text-sm text-slate-200">{activeDialog.intent.message}</div>
                ) : null}
                {activeDialog.intent.type === 'prompt' ? (
                  <input
                    type="text"
                    value={promptValue}
                    autoFocus
                    placeholder={activeDialog.intent.placeholder}
                    onChange={(event) => setPromptValue(event.target.value)}
                    className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1.5 text-sm outline-none focus:border-cyan-500"
                  />
                ) : null}
                {activeDialog.intent.type === 'form' ? (
                  <div className="space-y-2">
                    {activeDialog.intent.fields.map((field, index) => (
                      <label key={field.id} className="block" htmlFor={`dialog-field-${field.id}`}>
                        <div className="text-[11px] uppercase tracking-[0.12em] text-slate-400 mb-1">
                          {field.label}
                        </div>
                        <input
                          id={`dialog-field-${field.id}`}
                          autoFocus={index === 0}
                          type="text"
                          value={formValues[field.id] ?? ''}
                          placeholder={field.placeholder}
                          onChange={(event) =>
                            setFormValues((prev) => ({
                              ...prev,
                              [field.id]: event.target.value,
                            }))
                          }
                          className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1.5 text-sm outline-none focus:border-cyan-500"
                        />
                      </label>
                    ))}
                  </div>
                ) : null}
              </div>
              <div className="px-4 py-3 border-t border-slate-800 flex items-center justify-end gap-2">
                {activeDialog.intent.type !== 'alert' ? (
                  <button
                    type="button"
                    onClick={handleDialogCancel}
                    className="px-3 py-1.5 text-xs rounded border border-slate-700 bg-slate-900 text-slate-200 hover:bg-slate-800"
                  >
                    {activeDialog.intent.cancelLabel ?? 'Cancel'}
                  </button>
                ) : null}
                <button
                  type="button"
                  onClick={handleDialogConfirm}
                  disabled={!dialogCanConfirm}
                  className="px-3 py-1.5 text-xs rounded border border-cyan-700 bg-cyan-900/40 text-cyan-100 hover:bg-cyan-900/60 disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  {activeDialog.intent.confirmLabel ??
                    (activeDialog.intent.type === 'alert' ? 'OK' : 'Continue')}
                </button>
              </div>
            </div>
          </div>
        ) : null}
      </ToastContext.Provider>
    </DialogContext.Provider>
  );
}

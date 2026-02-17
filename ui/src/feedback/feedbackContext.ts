import { createContext, useContext } from 'react';

export type ToastTone = 'info' | 'success' | 'error';

export interface ToastIntent {
  title?: string;
  message: string;
  tone?: ToastTone;
  durationMs?: number;
}

export type DialogIntent =
  | {
      type: 'alert';
      title?: string;
      message: string;
      tone?: ToastTone;
      confirmLabel?: string;
    }
  | {
      type: 'confirm';
      title?: string;
      message: string;
      tone?: ToastTone;
      confirmLabel?: string;
      cancelLabel?: string;
    }
  | {
      type: 'prompt';
      title?: string;
      message: string;
      tone?: ToastTone;
      confirmLabel?: string;
      cancelLabel?: string;
      placeholder?: string;
      defaultValue?: string;
      requireNonEmpty?: boolean;
    };

export interface ToastContextValue {
  showToast: (intent: ToastIntent) => void;
}

export interface DialogContextValue {
  openDialog: (intent: DialogIntent) => Promise<boolean | string | null>;
  alert: (
    message: string,
    options?: Omit<Extract<DialogIntent, { type: 'alert' }>, 'type' | 'message'>
  ) => Promise<void>;
  confirm: (
    message: string,
    options?: Omit<Extract<DialogIntent, { type: 'confirm' }>, 'type' | 'message'>
  ) => Promise<boolean>;
  prompt: (
    message: string,
    options?: Omit<Extract<DialogIntent, { type: 'prompt' }>, 'type' | 'message'>
  ) => Promise<string | null>;
}

export const ToastContext = createContext<ToastContextValue | null>(null);
export const DialogContext = createContext<DialogContextValue | null>(null);

export function useToast() {
  const context = useContext(ToastContext);
  if (!context) {
    throw new Error('useToast must be used within FeedbackProvider');
  }
  return context;
}

export function useDialog() {
  const context = useContext(DialogContext);
  if (!context) {
    throw new Error('useDialog must be used within FeedbackProvider');
  }
  return context;
}

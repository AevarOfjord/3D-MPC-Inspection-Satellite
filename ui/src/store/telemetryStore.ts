import { create } from 'zustand';
import type { TelemetryData } from '../services/telemetry';

export type EventLevel = 'info' | 'warn' | 'error';

export interface TelemetryEvent {
  id: string;
  level: EventLevel;
  message: string;
  createdAt: number;
  simTime?: number;
}

interface TelemetryState {
  connected: boolean;
  lastReceivedAt: number | null;
  latest: TelemetryData | null;
  history: TelemetryData[];
  events: TelemetryEvent[];
  maxHistory: number;
  maxEvents: number;

  setConnected: (connected: boolean) => void;
  pushSample: (data: TelemetryData) => void;
  addEvent: (level: EventLevel, message: string, simTime?: number) => void;
  clearEvents: () => void;
}

export const useTelemetryStore = create<TelemetryState>((set, get) => ({
  connected: false,
  lastReceivedAt: null,
  latest: null,
  history: [],
  events: [],
  maxHistory: 5000,
  maxEvents: 100,

  setConnected: (connected) => set({ connected }),

  pushSample: (data) => {
    const { maxHistory, history } = get();
    const nextHistory = [...history, data];
    const trimmedHistory =
      nextHistory.length > maxHistory
        ? nextHistory.slice(nextHistory.length - maxHistory)
        : nextHistory;

    set({
      latest: data,
      lastReceivedAt: Date.now(),
      history: trimmedHistory,
    });
  },

  addEvent: (level, message, simTime) => {
    const { events, maxEvents } = get();
    const entry: TelemetryEvent = {
      id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
      level,
      message,
      createdAt: Date.now(),
      simTime,
    };
    const nextEvents = [...events, entry];
    set({
      events:
        nextEvents.length > maxEvents
          ? nextEvents.slice(nextEvents.length - maxEvents)
          : nextEvents,
    });
  },

  clearEvents: () => set({ events: [] }),
}));

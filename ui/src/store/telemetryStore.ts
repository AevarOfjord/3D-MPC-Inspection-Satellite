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

export interface TelemetrySample {
  time: number;
  posError: number;
  angError: number;
  velocity: number;
  solveTime: number;
}

interface TelemetryState {
  connected: boolean;
  lastReceivedAt: number | null;
  latest: TelemetryData | null;
  history: TelemetrySample[];
  events: TelemetryEvent[];
  maxHistory: number;
  maxEvents: number;
  lastSampleTime: number | null;

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
  maxHistory: 1200,
  maxEvents: 100,
  lastSampleTime: null,

  setConnected: (connected) => set({ connected }),

  pushSample: (data) => {
    const { maxHistory, history, lastSampleTime } = get();
    const now = Date.now();

    set({
      latest: data,
      lastReceivedAt: now,
    });

    if (lastSampleTime !== null && Math.abs(data.time - lastSampleTime) < 0.1) {
      return;
    }

    const velocityMag = Math.sqrt(
      data.velocity[0] ** 2 + data.velocity[1] ** 2 + data.velocity[2] ** 2
    );

    const sample: TelemetrySample = {
      time: Number(data.time.toFixed(1)),
      posError: data.pos_error ?? 0,
      angError: (data.ang_error ?? 0) * (180 / Math.PI),
      velocity: velocityMag,
      solveTime: (data.solve_time ?? 0) * 1000,
    };

    const nextHistory = [...history, sample];
    const trimmedHistory =
      nextHistory.length > maxHistory
        ? nextHistory.slice(nextHistory.length - maxHistory)
        : nextHistory;

    set({
      history: trimmedHistory,
      lastSampleTime: data.time,
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

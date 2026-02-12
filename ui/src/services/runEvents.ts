import type { SimulationRun } from '../api/simulations';
import { wsUrl } from '../config/endpoints';

type RunsEventType = 'runs_snapshot' | 'runs_updated';

export interface RunsEvent {
  type: RunsEventType;
  runs: SimulationRun[];
  latest_run_id?: string | null;
  updated_at?: number;
}

type RunsEventCallback = (event: RunsEvent) => void;

class RunsEventsService {
  private subscribers: Set<RunsEventCallback> = new Set();
  private socket: WebSocket | null = null;
  private reconnectTimer: number | null = null;
  private reconnectAttempt = 0;

  subscribe(callback: RunsEventCallback): () => void {
    this.subscribers.add(callback);
    if (this.subscribers.size === 1) {
      this.connect();
    }
    return () => {
      this.subscribers.delete(callback);
      if (this.subscribers.size === 0) {
        this.cleanup();
      }
    };
  }

  private notify(event: RunsEvent): void {
    this.subscribers.forEach((cb) => cb(event));
  }

  private connect(): void {
    if (this.socket || this.subscribers.size === 0) return;
    try {
      this.socket = new WebSocket(wsUrl('/simulations/runs/ws'));
    } catch (err) {
      console.error('Failed to create runs websocket:', err);
      this.scheduleReconnect();
      return;
    }

    this.socket.onopen = () => {
      this.reconnectAttempt = 0;
    };

    this.socket.onmessage = (event) => {
      try {
        const parsed = JSON.parse(event.data) as RunsEvent;
        if (parsed?.type === 'runs_snapshot' || parsed?.type === 'runs_updated') {
          this.notify(parsed);
        }
      } catch (err) {
        console.error('Failed to parse runs websocket message:', err);
      }
    };

    this.socket.onerror = () => {
      this.socket?.close();
    };

    this.socket.onclose = () => {
      this.socket = null;
      this.scheduleReconnect();
    };
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer !== null || this.subscribers.size === 0) {
      return;
    }
    const backoffMs = Math.min(1000 * 2 ** this.reconnectAttempt, 10000);
    this.reconnectAttempt += 1;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, backoffMs);
  }

  private cleanup(): void {
    if (this.reconnectTimer !== null) {
      window.clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    this.reconnectAttempt = 0;
    if (this.socket) {
      this.socket.onclose = null;
      this.socket.close();
      this.socket = null;
    }
  }
}

export const runEvents = new RunsEventsService();


import { WS_URL } from '../config/endpoints';

export interface TelemetryData {
  time: number;
  position: [number, number, number];
  quaternion: [number, number, number, number];
  velocity: [number, number, number];
  angular_velocity: [number, number, number];
  target_position: [number, number, number];
  target_orientation: [number, number, number];
  thrusters: number[];
  rw_torque: number[];
  obstacles: Array<{
    position: [number, number, number];
    radius: number;
  }>;
  solve_time?: number;
  pos_error?: number; // meters
  ang_error?: number; // radians
  planned_path?: [number, number, number][];
  paused?: boolean;
  sim_speed?: number;
}

type TelemetryCallback = (data: TelemetryData) => void;
type ConnectionCallback = (connected: boolean) => void;

class TelemetryService {
  private socket: WebSocket | null = null;
  private subscribers: Set<TelemetryCallback> = new Set();
  private statusSubscribers: Set<ConnectionCallback> = new Set();
  private isConnected: boolean = false;

  public get connected() {
    return this.isConnected;
  }

  connect(url: string = WS_URL) {
    if (this.socket) return;

    this.socket = new WebSocket(url);

    this.socket.onopen = () => {
      console.log("Telemetry Connected");
      this.isConnected = true;
      this.notifyStatus(true);
    };

    this.socket.onmessage = (event) => {
      try {
        const data: TelemetryData = JSON.parse(event.data);
        this.notify(data);
      } catch (e) {
        console.error("Failed to parse telemetry", e);
      }
    };

    this.socket.onclose = () => {
      console.log("Telemetry Disconnected");
      this.isConnected = false;
      this.socket = null;
      this.notifyStatus(false);
      // Reconnect logic could go here
      setTimeout(() => this.connect(url), 1000);
    };
  }

  subscribe(callback: TelemetryCallback) {
    this.subscribers.add(callback);
    return () => this.subscribers.delete(callback);
  }

  subscribeStatus(callback: ConnectionCallback) {
    this.statusSubscribers.add(callback);
    return () => this.statusSubscribers.delete(callback);
  }

  private notify(data: TelemetryData) {
    this.subscribers.forEach((cb) => cb(data));
  }

  private notifyStatus(connected: boolean) {
    this.statusSubscribers.forEach((cb) => cb(connected));
  }
}

export const telemetry = new TelemetryService();

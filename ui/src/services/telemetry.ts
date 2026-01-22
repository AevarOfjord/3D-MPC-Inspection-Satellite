import { WS_URL } from '../config/endpoints';

export interface TelemetryData {
  time: number;
  position: [number, number, number];
  quaternion: [number, number, number, number];
  velocity: [number, number, number];
  angular_velocity: [number, number, number];
  reference_position: [number, number, number];
  reference_orientation: [number, number, number];
  reference_quaternion?: [number, number, number, number];
  scan_object?: {
    type: 'cylinder' | 'starlink' | 'mesh';
    position: [number, number, number];
    orientation: [number, number, number];
    radius: number;
    height: number;
    obj_path?: string;
  };
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
  private manualMode: boolean = false;

  public get connected() {
    return this.isConnected;
  }

  connect(url: string = WS_URL) {
    if (this.socket || this.manualMode) return;

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
      if (!this.manualMode) {
        setTimeout(() => this.connect(url), 1000);
      }
    };
  }

  setManualMode(enabled: boolean) {
    this.manualMode = enabled;
    if (enabled && this.socket) {
      this.socket.close();
      this.socket = null;
      this.isConnected = false;
      this.notifyStatus(false);
    }
  }

  emit(data: TelemetryData) {
    this.notify(data);
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

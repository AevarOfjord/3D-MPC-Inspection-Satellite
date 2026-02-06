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
  target_position?: [number, number, number];
  target_quaternion?: [number, number, number, number];
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
  frame?: 'ECI' | 'LVLH';
  frame_origin?: [number, number, number] | null;
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
        this.notify(this.normalize(data));
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
    this.notify(this.normalize(data));
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

  normalize(data: TelemetryData): TelemetryData {
    if (data.frame !== 'LVLH' || !data.frame_origin) {
      return data;
    }

    const origin = data.frame_origin;
    const add = (p?: [number, number, number]) =>
      p
        ? ([p[0] + origin[0], p[1] + origin[1], p[2] + origin[2]] as [number, number, number])
        : p;

    const addPath = (path?: [number, number, number][]) =>
      path ? path.map((p) => add(p) as [number, number, number]) : path;

    const addObstacles = (obs?: Array<{ position: [number, number, number]; radius: number }>) =>
      obs
        ? obs.map((o) => ({ ...o, position: add(o.position) as [number, number, number] }))
        : obs;

    const scan_object = data.scan_object
      ? {
          ...data.scan_object,
          position: (() => {
            const pos = data.scan_object?.position;
            if (!pos) return pos;
            const sameAsOrigin =
              Math.abs(pos[0] - origin[0]) < 1e-6 &&
              Math.abs(pos[1] - origin[1]) < 1e-6 &&
              Math.abs(pos[2] - origin[2]) < 1e-6;
            // If scan_object position already equals the LVLH origin, don't add it again.
            return sameAsOrigin ? pos : (add(pos) as [number, number, number]);
          })(),
        }
      : data.scan_object;

    return {
      ...data,
      position: add(data.position) as [number, number, number],
      reference_position: add(data.reference_position) as [number, number, number],
      planned_path: addPath(data.planned_path),
      obstacles: addObstacles(data.obstacles) ?? [],
      scan_object,
      frame: 'ECI',
    };
  }

  private notifyStatus(connected: boolean) {
    this.statusSubscribers.forEach((cb) => cb(connected));
  }
}

export const telemetry = new TelemetryService();

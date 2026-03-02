

export interface TelemetryData {
  time: number;
  position: [number, number, number];
  quaternion: [number, number, number, number];
  velocity: [number, number, number];
  angular_velocity: [number, number, number];
  orientation_unwrapped_deg?: [number, number, number];
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
  obstacles?: Array<{
    position: [number, number, number];
    radius: number;
  }>;
  solve_time?: number;
  pos_error?: number; // meters
  ang_error?: number; // radians
  yaw_unwrapped_deg?: number;
  euler_unreliable?: boolean;
  planned_path?: [number, number, number][];
  paused?: boolean;
  sim_speed?: number;
  frame?: 'ECI' | 'LVLH';
  frame_origin?: [number, number, number] | null;
  mode_state?: {
    current_mode: string;
    time_in_mode_s: number;
  } | null;
  completion_gate?: {
    position_ok: boolean;
    angle_ok: boolean;
    velocity_ok: boolean;
    angular_velocity_ok: boolean;
    hold_elapsed_s: number;
    hold_required_s: number;
    last_breach_reason?: string | null;
  } | null;
  solver_health?: {
    status: string;
    fallback_count: number;
    hard_limit_breaches: number;
    fallback_active?: boolean;
    fallback_age_s?: number;
    fallback_scale?: number;
    last_fallback_reason?: string | null;
    fallback_reasons?: Record<string, number>;
  } | null;
  pointing_status?: {
    pointing_context_source?: string | null;
    pointing_axis_world?: [number, number, number];
    z_axis_error_deg?: number;
    x_axis_error_deg?: number;
    pointing_guardrail_breached?: boolean;
    object_visible_side?: '+Y' | '-Y' | null;
    pointing_guardrail_reason?: string | null;
  } | null;
  controller_core?: string;
  controller_profile?: 'hybrid' | 'nonlinear' | 'linear' | string;
  linearization_mode?: 'hybrid_tolerant_stage' | 'nonlinear_exact_stage' | 'linear_frozen_step' | string;
}

type TelemetryCallback = (data: TelemetryData) => void;
type ConnectionCallback = (connected: boolean) => void;

class TelemetryService {
  private subscribers: Set<TelemetryCallback> = new Set();
  private statusSubscribers: Set<ConnectionCallback> = new Set();

  public get connected() {
    // Always return false since we're playback-only (no live websocket)
    return false;
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
    const distance = (a: [number, number, number], b: [number, number, number]) =>
      Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    const norm = (p: [number, number, number]) => Math.hypot(p[0], p[1], p[2]);

    // Decide frame interpretation once per payload to avoid mixing absolute and
    // relative points in a single planned path.
    const originNorm = norm(origin);
    const isAbsoluteLike = (p?: [number, number, number]) =>
      Boolean(
        p &&
          originNorm > 1e5 &&
          norm(p) > 1e5 &&
          distance(p, origin) < 1e5
      );

    const candidates: [number, number, number][] = [];
    const pushCandidate = (p?: [number, number, number]) => {
      if (p) candidates.push(p);
    };
    const pushPathSamples = (path?: [number, number, number][]) => {
      if (!path || path.length === 0) return;
      pushCandidate(path[0]);
      pushCandidate(path[Math.floor(path.length / 2)]);
      pushCandidate(path[path.length - 1]);
    };
    pushCandidate(data.position);
    pushCandidate(data.reference_position);
    pushCandidate(data.target_position);
    pushPathSamples(data.planned_path);
    pushCandidate(data.scan_object?.position);
    if (data.obstacles && data.obstacles.length > 0) {
      pushCandidate(data.obstacles[0].position);
      pushCandidate(data.obstacles[data.obstacles.length - 1].position);
    }

    let absoluteLike = 0;
    let localLike = 0;
    for (const p of candidates) {
      if (isAbsoluteLike(p)) {
        absoluteLike += 1;
      } else if (norm(p) < 1e5) {
        localLike += 1;
      }
    }
    const payloadIsAbsolute = absoluteLike > 0 && absoluteLike >= localLike;

    const add = (p?: [number, number, number]) => {
      if (!p) return p;
      if (payloadIsAbsolute) return p;
      return [p[0] + origin[0], p[1] + origin[1], p[2] + origin[2]] as [number, number, number];
    };

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
      target_position: add(data.target_position) as [number, number, number] | undefined,
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

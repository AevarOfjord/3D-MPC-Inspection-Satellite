export type Frame = 'ECI' | 'LVLH';
export type SpiralDirection = 'CW' | 'CCW';
export type SensorAxis = '+Y' | '-Y';
export type SpiralAxis = '+X' | '-X' | '+Y' | '-Y' | '+Z' | '-Z' | 'custom';

export interface Pose {
  frame: Frame;
  position: [number, number, number];
  orientation?: [number, number, number, number];
}

export interface Constraints {
  speed_max?: number;
  accel_max?: number;
  angular_rate_max?: number;
}

export interface SplineControl {
  position: [number, number, number];
  weight?: number;
}

export interface TransferSegment {
  segment_id: string;
  title?: string | null;
  notes?: string | null;
  type: 'transfer';
  target_id?: string;
  end_pose: Pose;
  constraints?: Constraints;
}

export type SpiralPattern = 'spiral' | 'circles';

export interface ScanKeyLevel {
  id?: string;
  t: number;
  radius_x: number;
  radius_y: number;
  rotation_deg: number;
  offset_x: number;
  offset_y: number;
}

export interface ScanConfig {
  frame: Frame;
  axis: SpiralAxis;
  standoff: number;
  overlap: number;
  fov_deg: number;
  pitch?: number | null;
  revolutions: number;
  direction: SpiralDirection;
  sensor_axis: SensorAxis;
  pattern: SpiralPattern;
  level_spacing_m?: number | null;
  key_levels?: ScanKeyLevel[] | null;
}

export interface ScanSegment {
  segment_id: string;
  title?: string | null;
  notes?: string | null;
  type: 'scan';
  target_id: string;
  target_pose?: Pose;
  scan: ScanConfig;
  path_asset?: string;
  constraints?: Constraints;
}

export interface HoldSegment {
  segment_id: string;
  title?: string | null;
  notes?: string | null;
  type: 'hold';
  duration: number;
  constraints?: Constraints;
}

export type MissionSegment = TransferSegment | ScanSegment | HoldSegment;

export interface MissionOverrides {
  spline_controls?: SplineControl[];
  manual_path?: [number, number, number][];
  hold_schedule?: { path_index: number; duration_s: number }[];
  path_density_multiplier?: number;
}

export interface MissionObstacle {
  position: [number, number, number];
  radius: number;
}

export interface MissionMetadata {
  version: number;
  created_at?: string | null;
  updated_at?: string | null;
  tags?: string[];
}

export interface UnifiedMission {
  schema_version: 2;
  mission_id: string;
  name: string;
  epoch: string;
  start_pose: Pose;
  start_target_id?: string;
  segments: MissionSegment[];
  obstacles?: MissionObstacle[];
  overrides?: MissionOverrides;
  metadata: MissionMetadata;
}

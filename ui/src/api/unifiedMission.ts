export type Frame = 'ECI' | 'LVLH';
export type SegmentType = 'transfer' | 'scan' | 'hold';
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
  type: 'transfer';
  end_pose: Pose;
  constraints?: Constraints;
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
}

export interface ScanSegment {
  type: 'scan';
  target_id: string;
  target_pose?: Pose;
  scan: ScanConfig;
  constraints?: Constraints;
}

export interface HoldSegment {
  type: 'hold';
  duration: number;
  constraints?: Constraints;
}

export type MissionSegment = TransferSegment | ScanSegment | HoldSegment;

export interface MissionOverrides {
  spline_controls?: SplineControl[];
}

export interface MissionObstacle {
  position: [number, number, number];
  radius: number;
}

export interface UnifiedMission {
  epoch: string;
  start_pose: Pose;
  segments: MissionSegment[];
  obstacles?: MissionObstacle[];
  overrides?: MissionOverrides;
}

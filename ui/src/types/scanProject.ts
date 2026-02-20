export type BodyAxis = 'X' | 'Y' | 'Z';
export type EndpointKind = 'start' | 'end';

export interface ScanKeyLevel {
  id: string;
  t: number;
  center_offset: [number, number];
  radius_x: number;
  radius_y: number;
  rotation_deg: number;
}

export interface ScanDefinition {
  id: string;
  name: string;
  axis: BodyAxis;
  plane_a: [number, number, number];
  plane_b: [number, number, number];
  level_spacing_m?: number;
  turns?: number;
  coarse_points_per_turn: number;
  densify_multiplier: number;
  speed_max: number;
  key_levels: ScanKeyLevel[];
}

export interface ScanConnector {
  id: string;
  from_scan_id: string;
  to_scan_id: string;
  from_endpoint: EndpointKind;
  to_endpoint: EndpointKind;
  control1?: [number, number, number] | null;
  control2?: [number, number, number] | null;
  samples: number;
}

export interface ScanProject {
  schema_version: number;
  id?: string | null;
  name: string;
  obj_path: string;
  path_density_multiplier: number;
  scans: ScanDefinition[];
  connectors: ScanConnector[];
  created_at?: string | null;
  updated_at?: string | null;
}

export interface ScanProjectSummary {
  id: string;
  name: string;
  obj_path: string;
  scans: number;
  connectors: number;
  created_at?: string | null;
  updated_at?: string | null;
}

export interface ScanPathDiagnostics {
  id: string;
  kind: 'scan' | 'connector';
  points: number;
  path_length: number;
  path?: [number, number, number][] | null;
  min_clearance_m?: number | null;
  collision_points_count: number;
  clearance_per_point?: number[] | null;
}

export interface ScanCompileDiagnostics {
  min_clearance_m?: number | null;
  collision_points_count: number;
  clearance_threshold_m: number;
  combined_clearance_per_point?: number[] | null;
  warnings: string[];
}

export interface ScanCompileResponse {
  status: string;
  combined_path: [number, number, number][];
  path_length: number;
  estimated_duration: number;
  points: number;
  endpoints: Record<string, { start: [number, number, number]; end: [number, number, number] }>;
  scan_paths: ScanPathDiagnostics[];
  connector_paths: ScanPathDiagnostics[];
  diagnostics: ScanCompileDiagnostics;
}

export interface ScanCompileRequest {
  project: ScanProject;
  quality?: 'preview' | 'final';
  include_collision?: boolean;
  collision_threshold_m?: number;
}

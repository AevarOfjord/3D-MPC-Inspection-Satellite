export type ControllerProfileId =
  | 'cpp_linearized_rti_osqp'
  | 'cpp_hybrid_rti_osqp'
  | 'cpp_nonlinear_rti_osqp'
  | 'cpp_nonlinear_fullnlp_ipopt'
  | 'cpp_nonlinear_rti_hpipm'
  | 'cpp_nonlinear_sqp_hpipm';

export interface MpcSettings {
  prediction_horizon: number;
  control_horizon: number;
  dt: number;
  solver_time_limit: number;
  solver_type: string;
  Q_contour: number;
  Q_progress: number;
  progress_reward: number;
  Q_lag: number;
  Q_lag_default: number;
  Q_velocity_align: number;
  Q_s_anchor: number;
  Q_smooth: number;
  Q_attitude: number;
  Q_axis_align: number;
  Q_terminal_pos: number;
  Q_terminal_s: number;
  q_angular_velocity: number;
  r_thrust: number;
  r_rw_torque: number;
  thrust_l1_weight: number;
  thrust_pair_weight: number;
  thruster_type: 'PWM' | 'CON';
  verbose_mpc: boolean;
  obstacle_margin: number;
  enable_collision_avoidance: boolean;
  path_speed: number;
  path_speed_min: number;
  path_speed_max: number;
  enable_thruster_hysteresis: boolean;
  thruster_hysteresis_on: number;
  thruster_hysteresis_off: number;
  max_linear_velocity: number;
  max_angular_velocity: number;
  enable_delta_u_coupling: boolean;
  enable_gyro_jacobian: boolean;
  enable_auto_state_bounds: boolean;
  [key: string]: unknown;
}

export interface MpcCoreSettings {
  controller_profile: ControllerProfileId;
  solver_backend: 'OSQP';
  [key: string]: unknown;
}

export interface MpcProfileOverrideSettings {
  base_overrides: Partial<MpcSettings>;
  profile_specific: Record<string, unknown>;
}

export interface MpcProfileOverridesSettings {
  cpp_linearized_rti_osqp: MpcProfileOverrideSettings;
  cpp_hybrid_rti_osqp: MpcProfileOverrideSettings;
  cpp_nonlinear_rti_osqp: MpcProfileOverrideSettings;
  cpp_nonlinear_fullnlp_ipopt: MpcProfileOverrideSettings;
  cpp_nonlinear_rti_hpipm: MpcProfileOverrideSettings;
  cpp_nonlinear_sqp_hpipm: MpcProfileOverrideSettings;
}

export interface SharedSettings {
  parameters: boolean;
  profile_parameter_files: Record<ControllerProfileId, string>;
}

export interface SimulationSettings {
  dt: number;
  max_duration: number;
  control_dt: number;
  [key: string]: unknown;
}

export interface SettingsConfig {
  mpc: MpcSettings;
  mpc_core: MpcCoreSettings;
  shared: SharedSettings;
  mpc_profile_overrides: MpcProfileOverridesSettings;
  simulation: SimulationSettings;
  physics?: Record<string, unknown>;
  reference_scheduler?: Record<string, unknown>;
  actuator_policy?: Record<string, unknown>;
  controller_contracts?: Record<string, unknown>;
  input_file_path?: string | null;
}

export interface PresetPayload {
  config: SettingsConfig;
  updated_at?: string;
}

export interface RunnerSystemStatus {
  ready_for_runner: boolean;
  runner_active: boolean;
  checks: Record<string, boolean>;
  dependencies: Record<string, boolean>;
  missing_checks: string[];
  missing_dependencies: string[];
  python?: {
    executable?: string;
    version?: string;
    pid?: number;
  };
}

export interface PackageJobStatus {
  status: 'idle' | 'running' | 'completed' | 'failed' | string;
  running: boolean;
  started_at?: string | null;
  finished_at?: string | null;
  return_code?: number | null;
  archive_path?: string | null;
  error?: string | null;
  log_lines?: string[];
}

export interface WorkspaceInspection {
  schema_version: string;
  bundle: {
    missions: string[];
    presets: string[];
    simulation_runs: string[];
    has_runner_overrides: boolean;
  };
  conflicts: {
    missions: string[];
    presets: string[];
    simulation_runs: string[];
  };
  counts: {
    missions_total: number;
    presets_total: number;
    simulation_runs_total: number;
    mission_conflicts: number;
    preset_conflicts: number;
    simulation_run_conflicts: number;
  };
}

export interface MPCSettingsViewProps {
  onDirtyChange?: (dirty: boolean) => void;
}

export interface SettingReferenceItem {
  key: string;
  label: string;
  description: string;
  impact: string;
}

export interface SettingReferenceSection {
  title: string;
  items: SettingReferenceItem[];
}

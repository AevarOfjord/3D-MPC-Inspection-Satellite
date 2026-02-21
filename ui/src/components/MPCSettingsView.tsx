import React, { useEffect, useMemo, useState } from 'react';
import {
  AlertCircle,
  Check,
  ChevronDown,
  ChevronRight,
  Loader2,
  RotateCcw,
  Save,
} from 'lucide-react';
import { RUNNER_API_URL } from '../config/endpoints';

interface MpcSettings {
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

interface SimulationSettings {
  dt: number;
  max_duration: number;
  control_dt: number;
  [key: string]: unknown;
}

interface SettingsConfig {
  mpc: MpcSettings;
  simulation: SimulationSettings;
  physics?: Record<string, unknown>;
  reference_scheduler?: Record<string, unknown>;
  actuator_policy?: Record<string, unknown>;
  controller_contracts?: Record<string, unknown>;
  input_file_path?: string | null;
}

interface PresetPayload {
  config: SettingsConfig;
  updated_at?: string;
}

interface RunnerSystemStatus {
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

interface PackageJobStatus {
  status: 'idle' | 'running' | 'completed' | 'failed' | string;
  running: boolean;
  started_at?: string | null;
  finished_at?: string | null;
  return_code?: number | null;
  archive_path?: string | null;
  error?: string | null;
  log_lines?: string[];
}

interface WorkspaceInspection {
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

interface MPCSettingsViewProps {
  onDirtyChange?: (dirty: boolean) => void;
}

interface SettingReferenceItem {
  key: string;
  label: string;
  description: string;
  impact: string;
}

interface SettingReferenceSection {
  title: string;
  items: SettingReferenceItem[];
}

const DEFAULT_MPC_SETTINGS: MpcSettings = {
  prediction_horizon: 50,
  control_horizon: 40,
  dt: 0.05,
  solver_time_limit: 0.035,
  solver_type: 'OSQP',
  Q_contour: 2400.0,
  Q_progress: 70.0,
  progress_reward: 0.0,
  Q_lag: 0.0,
  Q_lag_default: 4000.0,
  Q_velocity_align: 120.0,
  Q_s_anchor: 500.0,
  Q_smooth: 20.0,
  Q_attitude: 3500.0,
  Q_axis_align: 3000.0,
  Q_terminal_pos: 0.0,
  Q_terminal_s: 0.0,
  q_angular_velocity: 1200.0,
  r_thrust: 0.02,
  r_rw_torque: 0.003,
  thrust_l1_weight: 0.02,
  thrust_pair_weight: 0.8,
  thruster_type: 'CON',
  verbose_mpc: false,
  obstacle_margin: 0.1,
  enable_collision_avoidance: false,
  path_speed: 0.08,
  path_speed_min: 0.0,
  path_speed_max: 0.08,
  enable_thruster_hysteresis: true,
  thruster_hysteresis_on: 0.015,
  thruster_hysteresis_off: 0.007,
  max_linear_velocity: 0.0,
  max_angular_velocity: 0.0,
  enable_delta_u_coupling: false,
  enable_gyro_jacobian: false,
  enable_auto_state_bounds: false,
};

const DEFAULT_SIMULATION_SETTINGS: SimulationSettings = {
  dt: 0.001,
  max_duration: 0.0,
  control_dt: 0.05,
};
const MPC_CANONICAL_KEYS = new Set(Object.keys(DEFAULT_MPC_SETTINGS));

const SETTING_REFERENCE_SECTIONS: SettingReferenceSection[] = [
  {
    title: 'Basic - Timing and Horizons',
    items: [
      {
        key: 'simulation.max_duration',
        label: 'Simulation Duration (s)',
        description: 'Maximum run time before simulation stop. A value of 0 disables duration-based stopping.',
        impact: 'Higher values allow long missions; lower values stop early for faster iteration.',
      },
      {
        key: 'mpc.dt',
        label: 'Control Step dt (s)',
        description: 'MPC update interval. This is synced to simulation control_dt when saved.',
        impact: 'Smaller dt reacts faster but increases compute load.',
      },
      {
        key: 'mpc.prediction_horizon',
        label: 'Prediction Horizon',
        description: 'Number of future timesteps the optimizer predicts.',
        impact: 'Longer horizon improves foresight but raises solve time.',
      },
      {
        key: 'mpc.control_horizon',
        label: 'Control Horizon',
        description: 'Number of steps with independent control actions (must be <= prediction horizon).',
        impact: 'Lower values smooth commands and reduce optimization complexity.',
      },
      {
        key: 'mpc.solver_time_limit',
        label: 'Solver Time Limit (s)',
        description: 'Maximum per-step optimizer runtime budget.',
        impact: 'Lower values improve real-time reliability but may reduce solution quality.',
      },
    ],
  },
  {
    title: 'Basic - Tracking Weights',
    items: [
      {
        key: 'mpc.Q_contour',
        label: 'Contour Error (Q_contour)',
        description: 'Penalty on distance from the reference path.',
        impact: 'Higher values force tighter path adherence.',
      },
      {
        key: 'mpc.Q_progress',
        label: 'Progress (Q_progress)',
        description: 'Penalty tied to path progress-speed tracking.',
        impact: 'Higher values prioritize consistent forward progress.',
      },
      {
        key: 'mpc.Q_attitude',
        label: 'Attitude (Q_attitude)',
        description: 'Penalty on orientation error relative to path tangent behavior.',
        impact: 'Higher values keep body alignment tighter.',
      },
      {
        key: 'mpc.Q_smooth',
        label: 'Smoothness (Q_smooth)',
        description: 'Penalty on control increments (delta-u).',
        impact: 'Higher values reduce command jitter but can slow response.',
      },
      {
        key: 'mpc.q_angular_velocity',
        label: 'Angular Velocity (q_angular_velocity)',
        description: 'Penalty on rotational rate in the cost function.',
        impact: 'Higher values damp angular motion and reduce spin.',
      },
    ],
  },
  {
    title: 'Basic - Actuation and Path',
    items: [
      {
        key: 'mpc.r_thrust',
        label: 'Thrust Cost (r_thrust)',
        description: 'Penalty on thruster usage.',
        impact: 'Higher values save thrust/fuel but can slow translational response.',
      },
      {
        key: 'mpc.r_rw_torque',
        label: 'RW Torque Cost (r_rw_torque)',
        description: 'Penalty on reaction wheel torque usage.',
        impact: 'Higher values reduce wheel effort but may weaken attitude authority.',
      },
      {
        key: 'mpc.path_speed',
        label: 'Path Speed (m/s)',
        description: 'Nominal desired progress speed along path.',
        impact: 'Higher values speed mission completion but demand more control effort.',
      },
    ],
  },
  {
    title: 'Advanced Settings',
    items: [
      {
        key: 'mpc.Q_lag',
        label: 'Lag Error (Q_lag)',
        description: 'Penalty on along-track lag relative to the path parameterization.',
        impact: 'Higher values reduce behind-path drift.',
      },
      {
        key: 'mpc.Q_lag_default',
        label: 'Lag Default (Q_lag_default)',
        description: 'Fallback lag weight when Q_lag <= 0. Use -1 to keep auto behavior.',
        impact: 'Controls default lag coupling without forcing it to contour weight.',
      },
      {
        key: 'mpc.Q_velocity_align',
        label: 'Velocity Align (Q_velocity_align)',
        description: 'Weight for velocity alignment along path tangent. 0 reuses Q_progress.',
        impact: 'Separates speed-tracking pressure from velocity-direction tracking.',
      },
      {
        key: 'mpc.Q_s_anchor',
        label: 'S Anchor (Q_s_anchor)',
        description: 'Weight anchoring path parameter state to the linearization reference. -1 keeps auto behavior.',
        impact: 'Lets you tune path-parameter anchoring strength explicitly.',
      },
      {
        key: 'mpc.Q_axis_align',
        label: 'Axis Align (Q_axis_align)',
        description: 'Extra alignment weight added on top of Q_attitude.',
        impact: 'Increase to bias stronger body-axis alignment without changing other attitude tuning.',
      },
      {
        key: 'mpc.path_speed_min',
        label: 'Path Speed Min (m/s)',
        description: 'Lower clamp for path progress speed.',
        impact: 'Higher minimum prevents stalling but can reduce precision near hard sections.',
      },
      {
        key: 'mpc.path_speed_max',
        label: 'Path Speed Max (m/s)',
        description: 'Upper clamp for path progress speed.',
        impact: 'Lower maximum limits aggressiveness and compute stress.',
      },
      {
        key: 'mpc.Q_terminal_pos',
        label: 'Terminal Position (Q_terminal_pos)',
        description: 'Terminal-stage position weight. 0 means automatic scaling.',
        impact: 'Higher values enforce stronger endpoint position convergence.',
      },
      {
        key: 'mpc.Q_terminal_s',
        label: 'Terminal Progress (Q_terminal_s)',
        description: 'Terminal-stage progress/path-parameter weight. 0 means automatic scaling.',
        impact: 'Higher values enforce stronger endpoint progress completion.',
      },
      {
        key: 'mpc.progress_reward',
        label: 'Progress Reward',
        description: 'Linear reward term for moving forward along the path.',
        impact: 'Higher values bias toward faster forward motion.',
      },
      {
        key: 'mpc.max_linear_velocity',
        label: 'Max Linear Velocity (m/s)',
        description: 'State bound for translational speed. 0 means automatic bound behavior.',
        impact: 'Lower bounds reduce aggressive motion and increase safety margin.',
      },
      {
        key: 'mpc.max_angular_velocity',
        label: 'Max Angular Velocity (rad/s)',
        description: 'State bound for rotational speed. 0 means automatic bound behavior.',
        impact: 'Lower bounds reduce spin and improve stability.',
      },
      {
        key: 'mpc.obstacle_margin',
        label: 'Obstacle Margin (m)',
        description: 'Extra safety clearance around modeled obstacles.',
        impact: 'Higher margin is safer but may make pathing more conservative.',
      },
      {
        key: 'mpc.enable_auto_state_bounds',
        label: 'Enable Auto State Bounds',
        description: 'Automatically derives velocity bounds when explicit bounds are unset.',
        impact: 'Improves robustness when limits are left at defaults.',
      },
      {
        key: 'mpc.enable_collision_avoidance',
        label: 'Enable Collision Avoidance',
        description: 'Enables online obstacle constraints in MPC.',
        impact: 'Improves safety around obstacles but can increase solver complexity.',
      },
      {
        key: 'mpc.thruster_type',
        label: 'Thruster Type',
        description: 'Thruster actuation mode: CON for continuous or PWM for binary-like behavior.',
        impact: 'Changes control style and can alter smoothness/realism tradeoffs.',
      },
      {
        key: 'mpc.solver_type',
        label: 'Solver',
        description: 'Optimizer backend selector.',
        impact: 'Currently OSQP is the supported runtime path in this stack.',
      },
      {
        key: 'mpc.enable_delta_u_coupling',
        label: 'Enable Delta-U Coupling',
        description: 'Uses full temporal coupling for smoothness cost terms.',
        impact: 'Can improve smoothness fidelity but increases solve load.',
      },
      {
        key: 'mpc.enable_gyro_jacobian',
        label: 'Enable Gyro Jacobian',
        description: 'Includes gyroscopic Jacobian terms in angular linearization.',
        impact: 'Improves high-rate rotational accuracy with extra computation.',
      },
      {
        key: 'mpc.verbose_mpc',
        label: 'Verbose MPC Solver Logs',
        description: 'Enables detailed solver logging output.',
        impact: 'Useful for tuning/debugging; increases log noise.',
      },
    ],
  },
  {
    title: 'Expert Settings',
    items: [
      {
        key: 'mpc.thrust_l1_weight',
        label: 'Thruster L1 Weight',
        description: 'Linear thruster usage penalty (fuel-bias term).',
        impact: 'Higher values promote coasting and sparse thrust usage.',
      },
      {
        key: 'mpc.thrust_pair_weight',
        label: 'Thruster Pair Weight',
        description: 'Penalty on opposing thrusters firing together.',
        impact: 'Higher values discourage wasteful opposing pair usage.',
      },
    ],
  },
];

function asRecord(value: unknown): Record<string, unknown> | null {
  if (value && typeof value === 'object' && !Array.isArray(value)) {
    return value as Record<string, unknown>;
  }
  return null;
}

function stripRemovedMpcFields(
  mpc: Record<string, unknown> | null | undefined
): Record<string, unknown> {
  const source = mpc ?? {};
  const sanitized: Record<string, unknown> = {};
  Object.entries(source).forEach(([key, value]) => {
    if (MPC_CANONICAL_KEYS.has(key)) {
      sanitized[key] = value;
    }
  });
  return sanitized;
}

function normalizeConfig(raw: unknown): SettingsConfig | null {
  const root = asRecord(raw);
  if (!root) return null;

  const appConfig = asRecord(root.app_config);
  const source = appConfig ?? root;
  const mpcCore = asRecord(source.mpc_core);
  const mpc = mpcCore ?? asRecord(source.mpc);
  const simulation = asRecord(source.simulation);
  const physics = asRecord(source.physics);
  const referenceScheduler = asRecord(source.reference_scheduler);
  const actuatorPolicy = asRecord(source.actuator_policy);
  const controllerContracts = asRecord(source.controller_contracts);
  const inputFilePath =
    typeof source.input_file_path === 'string' || source.input_file_path === null
      ? source.input_file_path
      : undefined;

  if (mpc && simulation) {
    const sanitizedMpc = stripRemovedMpcFields(mpc);
    const normalizedMpc = {
      ...DEFAULT_MPC_SETTINGS,
      ...(sanitizedMpc as Partial<MpcSettings>),
    };
    const normalizedSimulation = {
      ...DEFAULT_SIMULATION_SETTINGS,
      ...(simulation as Partial<SimulationSettings>),
    };
    if (typeof normalizedMpc.dt === 'number') {
      normalizedSimulation.control_dt = normalizedMpc.dt;
    }
    if (actuatorPolicy) {
      if (typeof actuatorPolicy.enable_thruster_hysteresis === 'boolean') {
        normalizedMpc.enable_thruster_hysteresis = actuatorPolicy.enable_thruster_hysteresis;
      }
      if (typeof actuatorPolicy.thruster_hysteresis_on === 'number') {
        normalizedMpc.thruster_hysteresis_on = actuatorPolicy.thruster_hysteresis_on;
      }
      if (typeof actuatorPolicy.thruster_hysteresis_off === 'number') {
        normalizedMpc.thruster_hysteresis_off = actuatorPolicy.thruster_hysteresis_off;
      }
    }
    return {
      mpc: normalizedMpc,
      simulation: normalizedSimulation,
      physics: physics ?? undefined,
      reference_scheduler: referenceScheduler ?? undefined,
      actuator_policy: actuatorPolicy ?? undefined,
      controller_contracts: controllerContracts ?? undefined,
      input_file_path: inputFilePath,
    };
  }

  // Legacy shape fallback
  const legacyControl = asRecord(root.control);
  const legacyMpc = asRecord(legacyControl?.mpc);
  const legacyWeights = asRecord(legacyMpc?.weights);
  const legacySettings = asRecord(legacyMpc?.settings);
  const legacyPath = asRecord(legacyMpc?.path_following);
  const legacySim = asRecord(root.sim);

  const normalizedMpc = {
    ...DEFAULT_MPC_SETTINGS,
    ...(stripRemovedMpcFields(legacyMpc) as Partial<MpcSettings>),
  };

  if (legacyWeights) {
    if (typeof legacyWeights.Q_contour === 'number') normalizedMpc.Q_contour = legacyWeights.Q_contour;
    if (typeof legacyWeights.Q_progress === 'number') normalizedMpc.Q_progress = legacyWeights.Q_progress;
    if (typeof legacyWeights.Q_smooth === 'number') normalizedMpc.Q_smooth = legacyWeights.Q_smooth;
    if (typeof legacyWeights.Q_attitude === 'number') normalizedMpc.Q_attitude = legacyWeights.Q_attitude;
    if (typeof legacyWeights.Q_axis_align === 'number') normalizedMpc.Q_axis_align = legacyWeights.Q_axis_align;
    if (typeof legacyWeights.angular_velocity === 'number') normalizedMpc.q_angular_velocity = legacyWeights.angular_velocity;
    if (typeof legacyWeights.thrust === 'number') normalizedMpc.r_thrust = legacyWeights.thrust;
    if (typeof legacyWeights.rw_torque === 'number') normalizedMpc.r_rw_torque = legacyWeights.rw_torque;
  }

  if (legacySettings) {
    if (typeof legacySettings.dt === 'number') normalizedMpc.dt = legacySettings.dt;
    if (typeof legacySettings.max_linear_velocity === 'number') normalizedMpc.max_linear_velocity = legacySettings.max_linear_velocity;
    if (typeof legacySettings.max_angular_velocity === 'number') normalizedMpc.max_angular_velocity = legacySettings.max_angular_velocity;
    if (typeof legacySettings.enable_collision_avoidance === 'boolean') {
      normalizedMpc.enable_collision_avoidance = legacySettings.enable_collision_avoidance;
    }
    if (typeof legacySettings.enable_auto_state_bounds === 'boolean') {
      normalizedMpc.enable_auto_state_bounds = legacySettings.enable_auto_state_bounds;
    }
  }

  if (legacyPath) {
    if (typeof legacyPath.path_speed === 'number') normalizedMpc.path_speed = legacyPath.path_speed;
  }

  const normalizedSimulation: SimulationSettings = {
    ...DEFAULT_SIMULATION_SETTINGS,
    ...(legacySim as Partial<SimulationSettings>),
  };
  if (typeof legacySim?.duration === 'number') {
    normalizedSimulation.max_duration = legacySim.duration;
  }
  normalizedSimulation.control_dt = normalizedMpc.dt;

  return {
    mpc: normalizedMpc,
    simulation: normalizedSimulation,
    physics: undefined,
    reference_scheduler: undefined,
    actuator_policy: undefined,
    controller_contracts: undefined,
    input_file_path: undefined,
  };
}

function buildV3Envelope(config: SettingsConfig): Record<string, unknown> {
  const sanitizedMpc = stripRemovedMpcFields(config.mpc as Record<string, unknown>);
  const actuatorPolicy =
    asRecord(config.actuator_policy) ?? {};
  const mergedActuatorPolicy: Record<string, unknown> = {
    ...actuatorPolicy,
    enable_thruster_hysteresis: config.mpc.enable_thruster_hysteresis,
    thruster_hysteresis_on: config.mpc.thruster_hysteresis_on,
    thruster_hysteresis_off: config.mpc.thruster_hysteresis_off,
  };

  const appConfig: Record<string, unknown> = {
    mpc_core: sanitizedMpc,
    actuator_policy: mergedActuatorPolicy,
    simulation: {
      ...config.simulation,
      control_dt: config.mpc.dt,
    },
  };
  if (config.physics && typeof config.physics === 'object') {
    appConfig.physics = config.physics;
  }
  if (config.reference_scheduler && typeof config.reference_scheduler === 'object') {
    appConfig.reference_scheduler = config.reference_scheduler;
  }
  if (config.controller_contracts && typeof config.controller_contracts === 'object') {
    appConfig.controller_contracts = config.controller_contracts;
  }
  if ('input_file_path' in config) {
    appConfig.input_file_path = config.input_file_path ?? null;
  }
  return {
    schema_version: 'app_config_v3',
    app_config: appConfig,
  };
}

// eslint-disable-next-line react-refresh/only-export-components
export const MPC_SETTINGS_TESTING = {
  normalizeConfig,
  buildV3Envelope,
  stripRemovedMpcFields,
};

function stableSerializeConfig(config: SettingsConfig): string {
  return JSON.stringify(config);
}

function deepCloneConfig(config: SettingsConfig): SettingsConfig {
  return JSON.parse(JSON.stringify(config)) as SettingsConfig;
}

function parseApiErrorText(payload: string): string {
  if (!payload) return '';
  try {
    const parsed = JSON.parse(payload) as Record<string, unknown>;
    return String(parsed.detail ?? parsed.message ?? payload);
  } catch {
    return payload;
  }
}

async function parseApiError(res: Response, fallback: string): Promise<string> {
  const text = await res.text();
  const detail = parseApiErrorText(text);
  return detail
    ? `${fallback} (HTTP ${res.status}): ${detail}`
    : `${fallback} (HTTP ${res.status})`;
}

function isNonNegative(n: number): boolean {
  return Number.isFinite(n) && n >= 0;
}

function validateConfig(config: SettingsConfig): string[] {
  const issues: string[] = [];
  const { mpc, simulation } = config;

  if (mpc.prediction_horizon < 1) issues.push('Prediction horizon must be >= 1.');
  if (mpc.control_horizon < 1) issues.push('Control horizon must be >= 1.');
  if (mpc.control_horizon > mpc.prediction_horizon) {
    issues.push('Control horizon cannot exceed prediction horizon.');
  }
  if (mpc.dt <= 0 || mpc.dt > 1.0) issues.push('Control dt must be in (0, 1.0].');
  if (simulation.dt <= 0 || simulation.dt > 0.1) {
    issues.push('Simulation dt must be in (0, 0.1].');
  }
  if (mpc.dt < simulation.dt) {
    issues.push('Control dt must be >= simulation dt.');
  }
  if (mpc.solver_time_limit <= 0 || mpc.solver_time_limit > 10.0) {
    issues.push('Solver time limit must be in (0, 10].');
  }
  if (mpc.path_speed_min < 0 || mpc.path_speed_min > 1.0) {
    issues.push('Path speed min must be in [0, 1].');
  }
  if (mpc.path_speed_max <= 0 || mpc.path_speed_max > 1.0) {
    issues.push('Path speed max must be in (0, 1].');
  }
  if (mpc.path_speed_min > mpc.path_speed_max) {
    issues.push('Path speed min cannot exceed path speed max.');
  }
  if (mpc.path_speed < mpc.path_speed_min || mpc.path_speed > mpc.path_speed_max) {
    issues.push('Path speed must be within [path speed min, path speed max].');
  }
  if (!isNonNegative(mpc.max_linear_velocity)) issues.push('Max linear velocity must be >= 0.');
  if (!isNonNegative(mpc.max_angular_velocity)) issues.push('Max angular velocity must be >= 0.');
  if (!isNonNegative(mpc.obstacle_margin)) issues.push('Obstacle margin must be >= 0.');
  if (mpc.Q_lag_default < -1) issues.push('Q_lag_default must be >= -1.');
  if (mpc.Q_s_anchor < -1) issues.push('Q_s_anchor must be >= -1.');
  if (mpc.thruster_hysteresis_off < 0 || mpc.thruster_hysteresis_on < 0) {
    issues.push('Thruster hysteresis thresholds must be >= 0.');
  }
  if (mpc.thruster_hysteresis_on <= mpc.thruster_hysteresis_off) {
    issues.push('Thruster hysteresis on-threshold must be greater than off-threshold.');
  }

  const nonNegativeWeights: Array<[string, number]> = [
    ['Q_contour', mpc.Q_contour],
    ['Q_progress', mpc.Q_progress],
    ['Q_lag', mpc.Q_lag],
    ['Q_velocity_align', mpc.Q_velocity_align],
    ['Q_smooth', mpc.Q_smooth],
    ['Q_attitude', mpc.Q_attitude],
    ['Q_axis_align', mpc.Q_axis_align],
    ['Q_terminal_pos', mpc.Q_terminal_pos],
    ['Q_terminal_s', mpc.Q_terminal_s],
    ['q_angular_velocity', mpc.q_angular_velocity],
    ['r_thrust', mpc.r_thrust],
    ['r_rw_torque', mpc.r_rw_torque],
    ['thrust_l1_weight', mpc.thrust_l1_weight],
    ['thrust_pair_weight', mpc.thrust_pair_weight],
  ];
  nonNegativeWeights.forEach(([name, value]) => {
    if (!isNonNegative(value)) issues.push(`${name} must be >= 0.`);
  });

  return issues;
}

export function MPCSettingsView({ onDirtyChange }: MPCSettingsViewProps) {
  const [config, setConfig] = useState<SettingsConfig | null>(null);
  const [savedSnapshot, setSavedSnapshot] = useState<string>('');
  const [removedMpcFieldsWarning, setRemovedMpcFieldsWarning] = useState<string[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [successMsg, setSuccessMsg] = useState<string | null>(null);
  const [presetName, setPresetName] = useState('');
  const [selectedPreset, setSelectedPreset] = useState('');
  const [presets, setPresets] = useState<Record<string, SettingsConfig>>({});
  const [showBasic, setShowBasic] = useState(true);
  const [showAdvanced, setShowAdvanced] = useState(false);
  const [showExpert, setShowExpert] = useState(false);
  const [showReference, setShowReference] = useState(false);
  const [systemStatus, setSystemStatus] = useState<RunnerSystemStatus | null>(null);
  const [statusLoading, setStatusLoading] = useState(false);
  const [quickMissionName, setQuickMissionName] = useState('');
  const [packageStatus, setPackageStatus] = useState<PackageJobStatus | null>(null);
  const [packageLoading, setPackageLoading] = useState(false);
  const [packageStarting, setPackageStarting] = useState(false);
  const [workspaceImportFile, setWorkspaceImportFile] = useState<File | null>(null);
  const [workspaceImporting, setWorkspaceImporting] = useState(false);
  const [workspaceInspecting, setWorkspaceInspecting] = useState(false);
  const [workspaceInspection, setWorkspaceInspection] = useState<WorkspaceInspection | null>(null);
  const [replaceExistingMissions, setReplaceExistingMissions] = useState(true);
  const [replaceExistingPresets, setReplaceExistingPresets] = useState(false);
  const [replaceExistingSimulationRuns, setReplaceExistingSimulationRuns] = useState(false);
  const [applyRunnerConfigOnImport, setApplyRunnerConfigOnImport] = useState(true);
  const [includeSimulationDataExport, setIncludeSimulationDataExport] = useState(false);
  const [missionConflictFilter, setMissionConflictFilter] = useState('');
  const [presetConflictFilter, setPresetConflictFilter] = useState('');
  const [simulationRunConflictFilter, setSimulationRunConflictFilter] = useState('');
  const [overwriteMissionNames, setOverwriteMissionNames] = useState<string[]>([]);
  const [overwritePresetNames, setOverwritePresetNames] = useState<string[]>([]);
  const [overwriteSimulationRunNames, setOverwriteSimulationRunNames] = useState<string[]>([]);
  const validationErrors = useMemo(() => (config ? validateConfig(config) : []), [config]);
  const isDirty = useMemo(
    () => (config ? stableSerializeConfig(config) !== savedSnapshot : false),
    [config, savedSnapshot]
  );

  useEffect(() => {
    void fetchConfig();
    void fetchPresets();
    void fetchSystemStatus();
    void fetchPackageStatus();
  }, []);

  useEffect(() => {
    onDirtyChange?.(isDirty);
  }, [isDirty, onDirtyChange]);

  useEffect(() => {
    const onBeforeUnload = (e: BeforeUnloadEvent) => {
      if (!isDirty) return;
      e.preventDefault();
      e.returnValue = '';
    };
    window.addEventListener('beforeunload', onBeforeUnload);
    return () => window.removeEventListener('beforeunload', onBeforeUnload);
  }, [isDirty]);

  useEffect(() => {
    if (!packageStatus?.running) return;
    const timer = window.setInterval(() => {
      void fetchPackageStatus();
    }, 2000);
    return () => window.clearInterval(timer);
  }, [packageStatus?.running]);

  const fetchConfig = async () => {
    setIsLoading(true);
    setError(null);
    try {
      const res = await fetch(`${RUNNER_API_URL}/config`);
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to fetch config'));
      const data = await res.json();
      const normalized = normalizeConfig(data);
      if (!normalized) throw new Error('Backend returned invalid config format');
      const configMeta = asRecord(asRecord(data)?.config_meta);
      const deprecations = asRecord(configMeta?.deprecations);
      const removedFieldValue = deprecations?.removed_mpc_fields_seen;
      const removedFields = Array.isArray(removedFieldValue)
        ? removedFieldValue.filter(
            (value): value is string => typeof value === 'string' && value.length > 0
          )
        : [];
      setRemovedMpcFieldsWarning(removedFields);
      setConfig(normalized);
      setSavedSnapshot(stableSerializeConfig(normalized));
    } catch (err) {
      setError(String(err));
    } finally {
      setIsLoading(false);
    }
  };

  const fetchPresets = async () => {
    try {
      const res = await fetch(`${RUNNER_API_URL}/presets`);
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to fetch presets'));
      const data = (await res.json()) as { presets?: Record<string, PresetPayload> };
      const next: Record<string, SettingsConfig> = {};
      const presetsMap = data.presets ?? {};
      Object.entries(presetsMap).forEach(([name, payload]) => {
        const normalized = normalizeConfig(payload?.config);
        if (normalized) next[name] = normalized;
      });
      setPresets(next);
      if (selectedPreset && !next[selectedPreset]) {
        setSelectedPreset('');
      }
    } catch (err) {
      setError(`Failed to load presets: ${String(err)}`);
    }
  };

  const fetchSystemStatus = async () => {
    setStatusLoading(true);
    try {
      const res = await fetch(`${RUNNER_API_URL}/system_status`);
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to fetch system status'));
      const data = (await res.json()) as RunnerSystemStatus;
      setSystemStatus(data);
    } catch (err) {
      setError(`Failed to load system status: ${String(err)}`);
    } finally {
      setStatusLoading(false);
    }
  };

  const fetchPackageStatus = async () => {
    setPackageLoading(true);
    try {
      const res = await fetch(`${RUNNER_API_URL}/package_app/status`);
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to fetch package status'));
      const data = (await res.json()) as PackageJobStatus;
      setPackageStatus(data);
    } catch (err) {
      setError(`Failed to load package status: ${String(err)}`);
    } finally {
      setPackageLoading(false);
    }
  };

  const handleQuickRunnerStart = async () => {
    setError(null);
    try {
      const payload = quickMissionName.trim()
        ? { mission_name: quickMissionName.trim() }
        : {};
      const res = await fetch(`${RUNNER_API_URL}/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to start runner'));
      setSuccessMsg('Runner start command sent.');
      setTimeout(() => setSuccessMsg(null), 2000);
      await fetchSystemStatus();
    } catch (err) {
      setError(`Failed to start runner: ${String(err)}`);
    }
  };

  const handleQuickRunnerStop = async () => {
    setError(null);
    try {
      const res = await fetch(`${RUNNER_API_URL}/stop`, {
        method: 'POST',
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to stop runner'));
      setSuccessMsg('Runner stop command sent.');
      setTimeout(() => setSuccessMsg(null), 2000);
      await fetchSystemStatus();
    } catch (err) {
      setError(`Failed to stop runner: ${String(err)}`);
    }
  };

  const handleStartPackaging = async () => {
    setPackageStarting(true);
    setError(null);
    try {
      const res = await fetch(`${RUNNER_API_URL}/package_app/start`, {
        method: 'POST',
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to start packaging job'));
      setSuccessMsg('Packaging started.');
      setTimeout(() => setSuccessMsg(null), 2000);
      await fetchPackageStatus();
    } catch (err) {
      setError(`Failed to start packaging: ${String(err)}`);
    } finally {
      setPackageStarting(false);
    }
  };

  const handleImportWorkspace = async () => {
    if (!workspaceImportFile) {
      setError('Select a workspace .zip file to import.');
      return;
    }

    setWorkspaceImporting(true);
    setError(null);
    try {
      const formData = new FormData();
      formData.append('file', workspaceImportFile);
      formData.append('replace_existing_missions', String(replaceExistingMissions));
      formData.append('replace_existing_presets', String(replaceExistingPresets));
      formData.append('replace_existing_simulation_runs', String(replaceExistingSimulationRuns));
      formData.append('apply_runner_config', String(applyRunnerConfigOnImport));
      formData.append('overwrite_missions_json', JSON.stringify(overwriteMissionNames));
      formData.append('overwrite_presets_json', JSON.stringify(overwritePresetNames));
      formData.append('overwrite_simulation_runs_json', JSON.stringify(overwriteSimulationRunNames));

      const res = await fetch(`${RUNNER_API_URL}/workspace/import`, {
        method: 'POST',
        body: formData,
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to import workspace'));

      const data = await res.json() as {
        missions_imported?: number;
        missions_skipped?: number;
        presets_imported?: number;
        presets_skipped?: number;
        simulation_runs_imported?: number;
        simulation_runs_skipped?: number;
        config_imported?: boolean;
      };

      setSuccessMsg(
        `Workspace imported: missions=${data.missions_imported ?? 0} (skipped ${data.missions_skipped ?? 0}), presets=${data.presets_imported ?? 0} (skipped ${data.presets_skipped ?? 0}), runs=${data.simulation_runs_imported ?? 0} (skipped ${data.simulation_runs_skipped ?? 0}), config=${data.config_imported ? 'yes' : 'no'}.`
      );
      setTimeout(() => setSuccessMsg(null), 4000);
      setWorkspaceImportFile(null);
      setWorkspaceInspection(null);
      setMissionConflictFilter('');
      setPresetConflictFilter('');
      setSimulationRunConflictFilter('');
      setOverwriteMissionNames([]);
      setOverwritePresetNames([]);
      setOverwriteSimulationRunNames([]);
      await Promise.all([
        fetchConfig(),
        fetchPresets(),
        fetchSystemStatus(),
        fetchPackageStatus(),
      ]);
    } catch (err) {
      setError(`Failed to import workspace: ${String(err)}`);
    } finally {
      setWorkspaceImporting(false);
    }
  };

  const handleInspectWorkspace = async () => {
    if (!workspaceImportFile) {
      setError('Select a workspace .zip file first.');
      return;
    }
    setWorkspaceInspecting(true);
    setError(null);
    try {
      const formData = new FormData();
      formData.append('file', workspaceImportFile);
      const res = await fetch(`${RUNNER_API_URL}/workspace/inspect`, {
        method: 'POST',
        body: formData,
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to inspect workspace'));
      const data = (await res.json()) as WorkspaceInspection;
      setWorkspaceInspection(data);
      setOverwriteMissionNames([]);
      setOverwritePresetNames([]);
      setOverwriteSimulationRunNames([]);
      setSuccessMsg('Workspace inspection complete.');
      setTimeout(() => setSuccessMsg(null), 2000);
    } catch (err) {
      setError(`Failed to inspect workspace: ${String(err)}`);
    } finally {
      setWorkspaceInspecting(false);
    }
  };

  const toggleNameSelection = (
    name: string,
    selected: string[],
    setSelected: (value: string[]) => void
  ) => {
    if (selected.includes(name)) {
      setSelected(selected.filter((item) => item !== name));
    } else {
      setSelected([...selected, name]);
    }
  };

  const filteredMissionConflicts = workspaceInspection
    ? (missionConflictFilter.trim()
      ? workspaceInspection.conflicts.missions.filter((name) =>
          name.toLowerCase().includes(missionConflictFilter.trim().toLowerCase())
        )
      : workspaceInspection.conflicts.missions)
    : [];
  const filteredPresetConflicts = workspaceInspection
    ? (presetConflictFilter.trim()
      ? workspaceInspection.conflicts.presets.filter((name) =>
          name.toLowerCase().includes(presetConflictFilter.trim().toLowerCase())
        )
      : workspaceInspection.conflicts.presets)
    : [];
  const filteredSimulationRunConflicts = workspaceInspection
    ? (simulationRunConflictFilter.trim()
      ? workspaceInspection.conflicts.simulation_runs.filter((name) =>
          name.toLowerCase().includes(simulationRunConflictFilter.trim().toLowerCase())
        )
      : workspaceInspection.conflicts.simulation_runs)
    : [];

  const handleReset = async () => {
    setIsLoading(true);
    setError(null);
    setSuccessMsg(null);
    try {
      const res = await fetch(`${RUNNER_API_URL}/config/reset`, {
        method: 'POST',
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to reset config'));
      await fetchConfig();
      setSuccessMsg('Configuration reset to defaults.');
      setTimeout(() => setSuccessMsg(null), 2500);
    } catch (err) {
      setError(`Failed to reset: ${String(err)}`);
      setIsLoading(false);
    }
  };

  const handleSave = async () => {
    if (!config) return;
    if (validationErrors.length > 0) {
      setError('Please fix validation errors before saving.');
      return;
    }
    setIsSaving(true);
    setError(null);
    setSuccessMsg(null);
    try {
      const overrides = buildV3Envelope(config);

      const res = await fetch(`${RUNNER_API_URL}/config`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(overrides),
      });

      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to save config'));

      setSavedSnapshot(stableSerializeConfig(config));
      setSuccessMsg('Configuration saved successfully. Next run will use these settings.');
      setTimeout(() => setSuccessMsg(null), 3000);
    } catch (err) {
      setError(`Failed to save: ${String(err)}`);
    } finally {
      setIsSaving(false);
    }
  };

  const updateConfig = (path: string, value: unknown) => {
    if (!config) return;

    const newConfig = JSON.parse(JSON.stringify(config)) as Record<string, unknown>;
    const parts = path.split('.');
    let current = newConfig;

    for (let i = 0; i < parts.length - 1; i++) {
      const next = current[parts[i]];
      if (!next || typeof next !== 'object') {
        current[parts[i]] = {};
      }
      current = current[parts[i]] as Record<string, unknown>;
    }

    const leaf = parts[parts.length - 1];
    const previous = current[leaf];

    if (typeof previous === 'number') {
      const parsed = typeof value === 'string' ? parseFloat(value) : Number(value);
      if (!Number.isNaN(parsed)) {
        current[leaf] = parsed;
      }
    } else if (typeof previous === 'boolean') {
      current[leaf] = Boolean(value);
    } else {
      current[leaf] = value;
    }

    // Keep simulation control_dt synchronized with mpc.dt
    if (path === 'mpc.dt') {
      const mpcObj = asRecord(newConfig.mpc);
      const simObj = asRecord(newConfig.simulation);
      if (mpcObj && simObj && typeof mpcObj.dt === 'number') {
        simObj.control_dt = mpcObj.dt;
      }
    }

    setConfig(newConfig as unknown as SettingsConfig);
  };

  const handleSavePreset = async () => {
    if (!config) return;
    const name = presetName.trim();
    if (!name) {
      setError('Preset name is required.');
      return;
    }
    setError(null);
    try {
      const res = await fetch(`${RUNNER_API_URL}/presets`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name,
          config: buildV3Envelope(deepCloneConfig(config)),
        }),
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to save preset'));
      await fetchPresets();
      setSelectedPreset(name);
      setSuccessMsg(`Preset "${name}" saved.`);
      setTimeout(() => setSuccessMsg(null), 2000);
    } catch (err) {
      setError(`Failed to save preset: ${String(err)}`);
    }
  };

  const handleLoadPreset = () => {
    if (!selectedPreset) {
      setError('Select a preset to load.');
      return;
    }
    const preset = presets[selectedPreset];
    if (!preset) {
      setError(`Preset "${selectedPreset}" not found.`);
      return;
    }
    setConfig(deepCloneConfig(preset));
    setSuccessMsg(`Preset "${selectedPreset}" loaded (not saved to backend yet).`);
    setTimeout(() => setSuccessMsg(null), 2500);
  };

  const handleDeletePreset = async () => {
    if (!selectedPreset) return;
    setError(null);
    try {
      const res = await fetch(`${RUNNER_API_URL}/presets/${encodeURIComponent(selectedPreset)}`, {
        method: 'DELETE',
      });
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to delete preset'));
      const deleted = selectedPreset;
      await fetchPresets();
      setSelectedPreset('');
      setSuccessMsg(`Preset "${deleted}" deleted.`);
      setTimeout(() => setSuccessMsg(null), 1500);
    } catch (err) {
      setError(`Failed to delete preset: ${String(err)}`);
    }
  };

  if (isLoading) {
    return (
      <div className="h-full flex items-center justify-center text-slate-400">
        <Loader2 className="animate-spin mr-2" /> Loading configuration...
      </div>
    );
  }

  if (error && !config) {
    return (
      <div className="h-full flex flex-col items-center justify-center text-red-400">
        <AlertCircle className="mb-2" size={32} />
        <p>{error}</p>
        <button
          onClick={() => void fetchConfig()}
          className="mt-4 px-4 py-2 bg-slate-800 rounded hover:bg-slate-700 transition"
        >
          Retry
        </button>
      </div>
    );
  }

  return (
    <div className="h-full flex flex-col bg-slate-950 text-slate-200 overflow-hidden">
      <div className="flex-none p-4 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
        <div>
          <h2 className="text-lg font-bold text-white">MPC Settings</h2>
          <p className="text-xs text-slate-300">Controller tuning, bounds, and solver options</p>
          <p className={`text-[11px] mt-1 ${isDirty ? 'text-amber-300' : 'text-emerald-300'}`}>
            {isDirty ? 'Unsaved changes' : 'All changes saved'}
          </p>
        </div>

        <div className="flex gap-2 items-center">
          <input
            type="text"
            value={presetName}
            onChange={(e) => setPresetName(e.target.value)}
            placeholder="Preset name"
            aria-label="Preset name"
            className="bg-slate-900 border border-slate-700 rounded px-2 py-1.5 text-xs text-white w-32 focus:outline-none focus:border-blue-500"
          />
          <button
            onClick={handleSavePreset}
            className="px-2.5 py-1.5 rounded bg-slate-700 hover:bg-slate-600 text-xs text-slate-100"
            aria-label="Save preset"
          >
            Save Preset
          </button>
          <select
            value={selectedPreset}
            onChange={(e) => setSelectedPreset(e.target.value)}
            aria-label="Preset selection"
            className="bg-slate-900 border border-slate-700 rounded px-2 py-1.5 text-xs text-white w-40 focus:outline-none focus:border-blue-500"
          >
            <option value="">Load preset...</option>
            {Object.keys(presets)
              .sort()
              .map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
          </select>
          <button
            onClick={handleLoadPreset}
            className="px-2.5 py-1.5 rounded bg-slate-700 hover:bg-slate-600 text-xs text-slate-100 disabled:opacity-40"
            disabled={!selectedPreset}
            aria-label="Load selected preset"
          >
            Load
          </button>
          <button
            onClick={handleDeletePreset}
            className="px-2.5 py-1.5 rounded bg-slate-700 hover:bg-slate-600 text-xs text-slate-100 disabled:opacity-40"
            disabled={!selectedPreset}
            aria-label="Delete selected preset"
          >
            Delete
          </button>
          <button
            onClick={() => void handleReset()}
            className="flex items-center gap-2 px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 text-sm transition"
            aria-label="Reset configuration to defaults"
          >
            <RotateCcw size={14} /> Reset
          </button>
          <button
            onClick={() => void handleSave()}
            disabled={isSaving || validationErrors.length > 0}
            className="flex items-center gap-2 px-4 py-1.5 rounded bg-blue-600 hover:bg-blue-500 text-white font-semibold text-sm transition shadow-sm disabled:opacity-50"
            aria-label="Save settings"
          >
            {isSaving ? <Loader2 size={14} className="animate-spin" /> : <Save size={14} />}
            Save Changes
          </button>
        </div>
      </div>

      <div className="flex-1 overflow-y-auto p-6">
        <div className="max-w-6xl mx-auto space-y-8">
          {error && (
            <div className="p-3 bg-red-900/20 border border-red-800 rounded text-red-200 text-sm flex items-center gap-2">
              <AlertCircle size={16} /> {error}
            </div>
          )}
          {successMsg && (
            <div className="p-3 bg-green-900/20 border border-green-800 rounded text-green-200 text-sm flex items-center gap-2">
              <Check size={16} /> {successMsg}
            </div>
          )}
          {removedMpcFieldsWarning.length > 0 && (
            <div className="p-3 bg-amber-900/20 border border-amber-700 rounded text-amber-200 text-sm">
              <p className="font-semibold">Deprecated MPC fields were dropped by backend:</p>
              <p className="text-xs mt-1">{removedMpcFieldsWarning.join(', ')}</p>
            </div>
          )}
          {validationErrors.length > 0 && (
            <div className="p-3 bg-amber-900/20 border border-amber-700 rounded text-amber-200 text-sm">
              <p className="font-semibold mb-1">Validation issues ({validationErrors.length}):</p>
              <ul className="list-disc ml-5 space-y-0.5">
                {validationErrors.map((msg) => (
                  <li key={msg}>{msg}</li>
                ))}
              </ul>
            </div>
          )}

          <section className="rounded border border-slate-800 bg-slate-900/60 p-4">
            <div className="flex items-center justify-between gap-3">
              <div>
                <h3 className="text-sm uppercase tracking-wider text-cyan-400 font-bold">
                  System Readiness
                </h3>
                <p className="text-xs text-slate-400 mt-1">
                  Verifies this machine can run from web interface without rebuild.
                </p>
              </div>
              <button
                onClick={() => void fetchSystemStatus()}
                className="px-2.5 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-xs text-slate-200"
                aria-label="Refresh system status"
                disabled={statusLoading}
              >
                {statusLoading ? 'Refreshing...' : 'Refresh'}
              </button>
            </div>

            {systemStatus ? (
              <div className="mt-4 space-y-3">
                <div className="flex items-center gap-2 text-sm">
                  {systemStatus.ready_for_runner ? (
                    <Check size={14} className="text-emerald-400" />
                  ) : (
                    <AlertCircle size={14} className="text-amber-400" />
                  )}
                  <span className={systemStatus.ready_for_runner ? 'text-emerald-300' : 'text-amber-300'}>
                    {systemStatus.ready_for_runner ? 'Runner ready' : 'Runner not ready'}
                  </span>
                  <span className="text-slate-500">|</span>
                  <span className={systemStatus.runner_active ? 'text-indigo-300' : 'text-slate-400'}>
                    {systemStatus.runner_active ? 'Runner active' : 'Runner idle'}
                  </span>
                </div>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-3 text-xs">
                  <div className="rounded border border-slate-800 bg-slate-950/60 p-3">
                    <p className="text-slate-400 mb-2 uppercase tracking-wide">Path Checks</p>
                    {Object.entries(systemStatus.checks).map(([name, ok]) => (
                      <div key={name} className="flex items-center justify-between py-0.5">
                        <span className="text-slate-300 font-mono">{name}</span>
                        <span className={ok ? 'text-emerald-300' : 'text-red-300'}>
                          {ok ? 'ok' : 'missing'}
                        </span>
                      </div>
                    ))}
                  </div>

                  <div className="rounded border border-slate-800 bg-slate-950/60 p-3">
                    <p className="text-slate-400 mb-2 uppercase tracking-wide">Dependencies</p>
                    {Object.entries(systemStatus.dependencies).map(([name, ok]) => (
                      <div key={name} className="flex items-center justify-between py-0.5">
                        <span className="text-slate-300 font-mono">{name}</span>
                        <span className={ok ? 'text-emerald-300' : 'text-red-300'}>
                          {ok ? 'ok' : 'missing'}
                        </span>
                      </div>
                    ))}
                  </div>
                </div>

                {(systemStatus.missing_checks.length > 0 || systemStatus.missing_dependencies.length > 0) && (
                  <div className="rounded border border-amber-800/60 bg-amber-900/20 p-3 text-xs">
                    {systemStatus.missing_checks.length > 0 && (
                      <p className="text-amber-200">
                        Missing checks: <span className="font-mono">{systemStatus.missing_checks.join(', ')}</span>
                      </p>
                    )}
                    {systemStatus.missing_dependencies.length > 0 && (
                      <p className="text-amber-200 mt-1">
                        Missing dependencies:{' '}
                        <span className="font-mono">{systemStatus.missing_dependencies.join(', ')}</span>
                      </p>
                    )}
                  </div>
                )}

                {systemStatus.python && (
                  <p className="text-[11px] text-slate-500 font-mono">
                    python={systemStatus.python.version ?? 'n/a'} pid={String(systemStatus.python.pid ?? 'n/a')}
                  </p>
                )}

                <div className="pt-2 border-t border-slate-800">
                  <p className="text-xs uppercase tracking-wide text-slate-400 mb-2">Runner Controls</p>
                  <div className="flex flex-wrap items-center gap-2">
                    <input
                      type="text"
                      value={quickMissionName}
                      onChange={(e) => setQuickMissionName(e.target.value)}
                      placeholder="Mission name (optional)"
                      aria-label="Mission name for quick start"
                      className="bg-slate-900 border border-slate-700 rounded px-2 py-1.5 text-xs text-white w-48 focus:outline-none focus:border-blue-500"
                    />
                    <button
                      onClick={() => void handleQuickRunnerStart()}
                      className="px-2.5 py-1.5 rounded bg-indigo-700 hover:bg-indigo-600 text-xs text-white"
                    >
                      Start Runner
                    </button>
                    <button
                      onClick={() => void handleQuickRunnerStop()}
                      className="px-2.5 py-1.5 rounded bg-slate-700 hover:bg-slate-600 text-xs text-white"
                    >
                      Stop Runner
                    </button>
                  </div>
                </div>
              </div>
            ) : (
              <div className="mt-3 text-xs text-slate-500">No status loaded yet.</div>
            )}
          </section>

          <section className="rounded border border-slate-800 bg-slate-900/60 p-4">
            <div className="flex items-center justify-between gap-3">
              <div>
                <h3 className="text-sm uppercase tracking-wider text-emerald-400 font-bold">
                  Build & Package
                </h3>
                <p className="text-xs text-slate-400 mt-1">
                  Runs <span className="font-mono">make package-app</span> from the backend.
                </p>
              </div>
              <div className="flex items-center gap-2">
                <button
                  onClick={() => void fetchPackageStatus()}
                  className="px-2.5 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-xs text-slate-200"
                  disabled={packageLoading}
                >
                  {packageLoading ? 'Refreshing...' : 'Refresh'}
                </button>
                <button
                  onClick={() => void handleStartPackaging()}
                  className="px-2.5 py-1.5 rounded bg-emerald-700 hover:bg-emerald-600 text-xs text-white disabled:opacity-50"
                  disabled={packageStarting || Boolean(packageStatus?.running)}
                >
                  {packageStarting ? 'Starting...' : 'Start Packaging'}
                </button>
                <a
                  href={`${RUNNER_API_URL}/package_app/download_latest`}
                  className="px-2.5 py-1.5 rounded bg-cyan-700 hover:bg-cyan-600 text-xs text-white"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  Download Latest Archive
                </a>
                <button
                  onClick={() => {
                    const url = `${RUNNER_API_URL}/workspace/export?include_simulation_data=${includeSimulationDataExport ? 'true' : 'false'}`;
                    window.open(url, '_blank', 'noopener,noreferrer');
                  }}
                  className="px-2.5 py-1.5 rounded bg-blue-700 hover:bg-blue-600 text-xs text-white"
                >
                  Export Workspace
                </button>
              </div>
            </div>

            {packageStatus ? (
              <div className="mt-3 space-y-2 text-xs">
                <div className="flex items-center gap-2">
                  <span className="text-slate-400">Status:</span>
                  <span
                    className={
                      packageStatus.status === 'completed'
                        ? 'text-emerald-300'
                        : packageStatus.status === 'failed'
                          ? 'text-red-300'
                          : packageStatus.status === 'running'
                            ? 'text-cyan-300'
                            : 'text-slate-300'
                    }
                  >
                    {packageStatus.status}
                  </span>
                  {typeof packageStatus.return_code === 'number' && (
                    <span className="text-slate-500 font-mono">rc={packageStatus.return_code}</span>
                  )}
                </div>
                {packageStatus.archive_path && (
                  <p className="text-slate-300">
                    Archive: <span className="font-mono text-emerald-300">{packageStatus.archive_path}</span>
                  </p>
                )}
                {packageStatus.error && (
                  <p className="text-red-300">Error: {packageStatus.error}</p>
                )}
                <div className="rounded border border-slate-800 bg-slate-950/70 p-2 max-h-40 overflow-y-auto font-mono text-[11px] text-slate-300 whitespace-pre-wrap">
                  {(packageStatus.log_lines && packageStatus.log_lines.length > 0)
                    ? packageStatus.log_lines.slice(-40).join('\n')
                    : 'No packaging logs yet.'}
                </div>

                <div className="pt-2 border-t border-slate-800">
                  <label className="flex items-center gap-2 text-[11px] text-slate-300 mb-2">
                    <input
                      type="checkbox"
                      checked={includeSimulationDataExport}
                      onChange={(e) => setIncludeSimulationDataExport(e.target.checked)}
                    />
                    Include simulation run data in export (can be large)
                  </label>
                  <p className="text-slate-400 uppercase tracking-wide mb-2">Import Workspace</p>
                  <div className="flex flex-wrap items-center gap-2">
                    <input
                      type="file"
                      accept=".zip,application/zip"
                      onChange={(e) => {
                        setWorkspaceImportFile(e.target.files?.[0] ?? null);
                        setWorkspaceInspection(null);
                        setMissionConflictFilter('');
                        setPresetConflictFilter('');
                        setSimulationRunConflictFilter('');
                        setOverwriteMissionNames([]);
                        setOverwritePresetNames([]);
                        setOverwriteSimulationRunNames([]);
                      }}
                      className="text-xs text-slate-300 file:mr-2 file:rounded file:border-0 file:bg-slate-700 file:px-2 file:py-1 file:text-xs file:text-white hover:file:bg-slate-600"
                    />
                    <button
                      onClick={() => void handleInspectWorkspace()}
                      className="px-2.5 py-1.5 rounded bg-slate-700 hover:bg-slate-600 text-xs text-white disabled:opacity-50"
                      disabled={workspaceInspecting || !workspaceImportFile}
                    >
                      {workspaceInspecting ? 'Inspecting...' : 'Inspect Workspace'}
                    </button>
                    <button
                      onClick={() => void handleImportWorkspace()}
                      className="px-2.5 py-1.5 rounded bg-violet-700 hover:bg-violet-600 text-xs text-white disabled:opacity-50"
                      disabled={workspaceImporting || !workspaceImportFile}
                    >
                      {workspaceImporting ? 'Importing...' : 'Import Workspace'}
                    </button>
                  </div>
                  {workspaceImportFile && (
                    <p className="text-[11px] text-slate-500 mt-1">Selected: {workspaceImportFile.name}</p>
                  )}
                  <div className="w-full grid grid-cols-1 md:grid-cols-4 gap-2 mt-2">
                    <label className="flex items-center gap-2 text-[11px] text-slate-300">
                      <input
                        type="checkbox"
                        checked={replaceExistingMissions}
                        onChange={(e) => setReplaceExistingMissions(e.target.checked)}
                      />
                      Replace existing missions
                    </label>
                    <label className="flex items-center gap-2 text-[11px] text-slate-300">
                      <input
                        type="checkbox"
                        checked={replaceExistingPresets}
                        onChange={(e) => setReplaceExistingPresets(e.target.checked)}
                      />
                      Replace existing presets
                    </label>
                    <label className="flex items-center gap-2 text-[11px] text-slate-300">
                      <input
                        type="checkbox"
                        checked={replaceExistingSimulationRuns}
                        onChange={(e) => setReplaceExistingSimulationRuns(e.target.checked)}
                      />
                      Replace existing simulation runs
                    </label>
                    <label className="flex items-center gap-2 text-[11px] text-slate-300">
                      <input
                        type="checkbox"
                        checked={applyRunnerConfigOnImport}
                        onChange={(e) => setApplyRunnerConfigOnImport(e.target.checked)}
                      />
                      Apply runner config overrides
                    </label>
                  </div>
                  {workspaceInspection && (
                    <div className="w-full rounded border border-slate-800 bg-slate-950/70 p-2 mt-2 text-[11px]">
                      <p className="text-slate-300">
                        Bundle: missions={workspaceInspection.counts.missions_total}, presets={workspaceInspection.counts.presets_total}, runs={workspaceInspection.counts.simulation_runs_total}, config={workspaceInspection.bundle.has_runner_overrides ? 'yes' : 'no'}
                      </p>
                      <p className="text-amber-300 mt-1">
                        Conflicts: missions={workspaceInspection.counts.mission_conflicts}, presets={workspaceInspection.counts.preset_conflicts}, runs={workspaceInspection.counts.simulation_run_conflicts}
                      </p>
                      {workspaceInspection.conflicts.missions.length > 0 && (
                        <div className="mt-2">
                          <p className="text-slate-400">
                            Mission conflicts ({workspaceInspection.conflicts.missions.length})
                          </p>
                          <input
                            type="text"
                            value={missionConflictFilter}
                            onChange={(e) => setMissionConflictFilter(e.target.value)}
                            placeholder="Filter mission conflicts..."
                            className="mt-1 w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-[11px] text-slate-100 focus:outline-none focus:border-blue-500"
                          />
                          <div className="mt-1 max-h-20 overflow-y-auto rounded border border-slate-800 bg-slate-900/60 p-1 font-mono text-[10px] text-amber-200">
                            {filteredMissionConflicts.length > 0 ? (
                              filteredMissionConflicts.map((name) => (
                                <label key={name} className="flex items-center gap-1">
                                  <input
                                    type="checkbox"
                                    checked={overwriteMissionNames.includes(name)}
                                    onChange={() =>
                                      toggleNameSelection(
                                        name,
                                        overwriteMissionNames,
                                        setOverwriteMissionNames
                                      )
                                    }
                                  />
                                  <span>{name}</span>
                                </label>
                              ))
                            ) : (
                              <span>No matches for current filter.</span>
                            )}
                          </div>
                          <p className="text-[10px] text-slate-500 mt-1">
                            Selected mission overwrites: {overwriteMissionNames.length}
                          </p>
                        </div>
                      )}
                      {workspaceInspection.conflicts.presets.length > 0 && (
                        <div className="mt-2">
                          <p className="text-slate-400">
                            Preset conflicts ({workspaceInspection.conflicts.presets.length})
                          </p>
                          <input
                            type="text"
                            value={presetConflictFilter}
                            onChange={(e) => setPresetConflictFilter(e.target.value)}
                            placeholder="Filter preset conflicts..."
                            className="mt-1 w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-[11px] text-slate-100 focus:outline-none focus:border-blue-500"
                          />
                          <div className="mt-1 max-h-20 overflow-y-auto rounded border border-slate-800 bg-slate-900/60 p-1 font-mono text-[10px] text-amber-200">
                            {filteredPresetConflicts.length > 0 ? (
                              filteredPresetConflicts.map((name) => (
                                <label key={name} className="flex items-center gap-1">
                                  <input
                                    type="checkbox"
                                    checked={overwritePresetNames.includes(name)}
                                    onChange={() =>
                                      toggleNameSelection(
                                        name,
                                        overwritePresetNames,
                                        setOverwritePresetNames
                                      )
                                    }
                                  />
                                  <span>{name}</span>
                                </label>
                              ))
                            ) : (
                              <span>No matches for current filter.</span>
                            )}
                          </div>
                          <p className="text-[10px] text-slate-500 mt-1">
                            Selected preset overwrites: {overwritePresetNames.length}
                          </p>
                        </div>
                      )}
                      {workspaceInspection.conflicts.simulation_runs.length > 0 && (
                        <div className="mt-2">
                          <p className="text-slate-400">
                            Simulation run conflicts ({workspaceInspection.conflicts.simulation_runs.length})
                          </p>
                          <input
                            type="text"
                            value={simulationRunConflictFilter}
                            onChange={(e) => setSimulationRunConflictFilter(e.target.value)}
                            placeholder="Filter simulation run conflicts..."
                            className="mt-1 w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-[11px] text-slate-100 focus:outline-none focus:border-blue-500"
                          />
                          <div className="mt-1 max-h-20 overflow-y-auto rounded border border-slate-800 bg-slate-900/60 p-1 font-mono text-[10px] text-amber-200">
                            {filteredSimulationRunConflicts.length > 0 ? (
                              filteredSimulationRunConflicts.map((name) => (
                                <label key={name} className="flex items-center gap-1">
                                  <input
                                    type="checkbox"
                                    checked={overwriteSimulationRunNames.includes(name)}
                                    onChange={() =>
                                      toggleNameSelection(
                                        name,
                                        overwriteSimulationRunNames,
                                        setOverwriteSimulationRunNames
                                      )
                                    }
                                  />
                                  <span>{name}</span>
                                </label>
                              ))
                            ) : (
                              <span>No matches for current filter.</span>
                            )}
                          </div>
                          <p className="text-[10px] text-slate-500 mt-1">
                            Selected simulation run overwrites: {overwriteSimulationRunNames.length}
                          </p>
                        </div>
                      )}
                    </div>
                  )}
                </div>
              </div>
            ) : (
              <div className="mt-3 text-xs text-slate-500">No package status loaded yet.</div>
            )}
          </section>

          <section>
            <button
              onClick={() => setShowBasic((v) => !v)}
              className="w-full flex items-center justify-between p-3 rounded border border-slate-800 bg-slate-900 hover:bg-slate-800 transition-colors"
            >
              <span className="text-sm uppercase tracking-wider text-blue-400 font-bold">
                Basic Settings
              </span>
              {showBasic ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
            </button>

            {showBasic && (
              <div className="mt-4 space-y-8">
                <section>
                  <h3 className="text-sm uppercase tracking-wider text-slate-500 font-bold mb-4 border-b border-slate-800 pb-1">
                    Basic - Timing and Horizons
                  </h3>
                  <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                    <ConfigField
                      label="Simulation Duration (s)"
                      value={config?.simulation.max_duration}
                      onChange={(v) => updateConfig('simulation.max_duration', v)}
                      isNumber
                      step={1}
                      desc="0 = no hard duration limit"
                    />
                    <ConfigField
                      label="Control Step dt (s)"
                      value={config?.mpc.dt}
                      onChange={(v) => updateConfig('mpc.dt', v)}
                      isNumber
                      step={0.001}
                    />
                    <ConfigField
                      label="Prediction Horizon"
                      value={config?.mpc.prediction_horizon}
                      onChange={(v) => updateConfig('mpc.prediction_horizon', v)}
                      isNumber
                      step={1}
                    />
                    <ConfigField
                      label="Control Horizon"
                      value={config?.mpc.control_horizon}
                      onChange={(v) => updateConfig('mpc.control_horizon', v)}
                      isNumber
                      step={1}
                    />
                    <ConfigField
                      label="Solver Time Limit (s)"
                      value={config?.mpc.solver_time_limit}
                      onChange={(v) => updateConfig('mpc.solver_time_limit', v)}
                      isNumber
                      step={0.001}
                    />
                  </div>
                </section>

                <section>
                  <h3 className="text-sm uppercase tracking-wider text-blue-400 font-bold mb-4 border-b border-blue-900/30 pb-1">
                    Basic - Tracking Weights
                  </h3>
                  <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                    <ConfigField
                      label="Contour Error (Q_contour)"
                      value={config?.mpc.Q_contour}
                      onChange={(v) => updateConfig('mpc.Q_contour', v)}
                      isNumber
                    />
                    <ConfigField
                      label="Progress (Q_progress)"
                      value={config?.mpc.Q_progress}
                      onChange={(v) => updateConfig('mpc.Q_progress', v)}
                      isNumber
                    />
                    <ConfigField
                      label="Attitude (Q_attitude)"
                      value={config?.mpc.Q_attitude}
                      onChange={(v) => updateConfig('mpc.Q_attitude', v)}
                      isNumber
                    />
                    <ConfigField
                      label="Smoothness (Q_smooth)"
                      value={config?.mpc.Q_smooth}
                      onChange={(v) => updateConfig('mpc.Q_smooth', v)}
                      isNumber
                    />
                    <ConfigField
                      label="Angular Velocity (q_angular_velocity)"
                      value={config?.mpc.q_angular_velocity}
                      onChange={(v) => updateConfig('mpc.q_angular_velocity', v)}
                      isNumber
                    />
                  </div>
                </section>

                <section>
                  <h3 className="text-sm uppercase tracking-wider text-slate-500 font-bold mb-4 border-b border-slate-800 pb-1">
                    Basic - Actuation and Path
                  </h3>
                  <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                    <ConfigField
                      label="Thrust Cost (r_thrust)"
                      value={config?.mpc.r_thrust}
                      onChange={(v) => updateConfig('mpc.r_thrust', v)}
                      isNumber
                    />
                    <ConfigField
                      label="RW Torque Cost (r_rw_torque)"
                      value={config?.mpc.r_rw_torque}
                      onChange={(v) => updateConfig('mpc.r_rw_torque', v)}
                      isNumber
                    />
                    <ConfigField
                      label="Path Speed (m/s)"
                      value={config?.mpc.path_speed}
                      onChange={(v) => updateConfig('mpc.path_speed', v)}
                      isNumber
                      step={0.001}
                    />
                  </div>
                </section>

              </div>
            )}
          </section>

          <section>
            <button
              onClick={() => setShowAdvanced((v) => !v)}
              className="w-full flex items-center justify-between p-3 rounded border border-slate-800 bg-slate-900 hover:bg-slate-800 transition-colors"
            >
              <span className="text-sm uppercase tracking-wider text-cyan-400 font-bold">
                Advanced Settings
              </span>
              {showAdvanced ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
            </button>

            {showAdvanced && (
              <div className="mt-4 grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                <ConfigField
                  label="Lag Error (Q_lag)"
                  value={config?.mpc.Q_lag}
                  onChange={(v) => updateConfig('mpc.Q_lag', v)}
                  isNumber
                />
                <ConfigField
                  label="Lag Default (Q_lag_default)"
                  value={config?.mpc.Q_lag_default}
                  onChange={(v) => updateConfig('mpc.Q_lag_default', v)}
                  isNumber
                  desc="-1 = auto fallback"
                />
                <ConfigField
                  label="Velocity Align (Q_velocity_align)"
                  value={config?.mpc.Q_velocity_align}
                  onChange={(v) => updateConfig('mpc.Q_velocity_align', v)}
                  isNumber
                  desc="0 = reuse Q_progress"
                />
                <ConfigField
                  label="S Anchor (Q_s_anchor)"
                  value={config?.mpc.Q_s_anchor}
                  onChange={(v) => updateConfig('mpc.Q_s_anchor', v)}
                  isNumber
                  desc="-1 = auto fallback"
                />
                <ConfigField
                  label="Axis Align (Q_axis_align)"
                  value={config?.mpc.Q_axis_align}
                  onChange={(v) => updateConfig('mpc.Q_axis_align', v)}
                  isNumber
                  desc="extra attitude alignment weight"
                />
                <ConfigField
                  label="Path Speed Min (m/s)"
                  value={config?.mpc.path_speed_min}
                  onChange={(v) => updateConfig('mpc.path_speed_min', v)}
                  isNumber
                  step={0.001}
                />
                <ConfigField
                  label="Path Speed Max (m/s)"
                  value={config?.mpc.path_speed_max}
                  onChange={(v) => updateConfig('mpc.path_speed_max', v)}
                  isNumber
                  step={0.001}
                />
                <ConfigField
                  label="Terminal Position (Q_terminal_pos)"
                  value={config?.mpc.Q_terminal_pos}
                  onChange={(v) => updateConfig('mpc.Q_terminal_pos', v)}
                  isNumber
                  desc="0 = auto"
                />
                <ConfigField
                  label="Terminal Progress (Q_terminal_s)"
                  value={config?.mpc.Q_terminal_s}
                  onChange={(v) => updateConfig('mpc.Q_terminal_s', v)}
                  isNumber
                  desc="0 = auto"
                />
                <ConfigField
                  label="Progress Reward"
                  value={config?.mpc.progress_reward}
                  onChange={(v) => updateConfig('mpc.progress_reward', v)}
                  isNumber
                />
                <ConfigField
                  label="Max Linear Velocity (m/s)"
                  value={config?.mpc.max_linear_velocity}
                  onChange={(v) => updateConfig('mpc.max_linear_velocity', v)}
                  isNumber
                  desc="0 = auto bound"
                />
                <ConfigField
                  label="Max Angular Velocity (rad/s)"
                  value={config?.mpc.max_angular_velocity}
                  onChange={(v) => updateConfig('mpc.max_angular_velocity', v)}
                  isNumber
                  desc="0 = auto bound"
                />
                <ConfigField
                  label="Obstacle Margin (m)"
                  value={config?.mpc.obstacle_margin}
                  onChange={(v) => updateConfig('mpc.obstacle_margin', v)}
                  isNumber
                  step={0.01}
                />
                <ToggleField
                  label="Enable Auto State Bounds"
                  checked={Boolean(config?.mpc.enable_auto_state_bounds)}
                  onChange={(checked) => updateConfig('mpc.enable_auto_state_bounds', checked)}
                />
                <ToggleField
                  label="Enable Collision Avoidance"
                  checked={Boolean(config?.mpc.enable_collision_avoidance)}
                  onChange={(checked) => updateConfig('mpc.enable_collision_avoidance', checked)}
                />
                <SelectField
                  label="Thruster Type"
                  value={String(config?.mpc.thruster_type ?? 'CON')}
                  onChange={(v) => updateConfig('mpc.thruster_type', v)}
                  options={[
                    { label: 'Continuous (CON)', value: 'CON' },
                    { label: 'PWM', value: 'PWM' },
                  ]}
                />
                <SelectField
                  label="Solver"
                  value={String(config?.mpc.solver_type ?? 'OSQP')}
                  onChange={(v) => updateConfig('mpc.solver_type', v)}
                  options={[{ label: 'OSQP', value: 'OSQP' }]}
                />
                <ToggleField
                  label="Enable Delta-U Coupling"
                  checked={Boolean(config?.mpc.enable_delta_u_coupling)}
                  onChange={(checked) => updateConfig('mpc.enable_delta_u_coupling', checked)}
                />
                <ToggleField
                  label="Enable Gyro Jacobian"
                  checked={Boolean(config?.mpc.enable_gyro_jacobian)}
                  onChange={(checked) => updateConfig('mpc.enable_gyro_jacobian', checked)}
                />
                <ToggleField
                  label="Verbose MPC Solver Logs"
                  checked={Boolean(config?.mpc.verbose_mpc)}
                  onChange={(checked) => updateConfig('mpc.verbose_mpc', checked)}
                />
              </div>
            )}
          </section>

          <section>
            <button
              onClick={() => setShowExpert((v) => !v)}
              className="w-full flex items-center justify-between p-3 rounded border border-slate-800 bg-slate-900 hover:bg-slate-800 transition-colors"
            >
              <span className="text-sm uppercase tracking-wider text-orange-400 font-bold">
                Expert Settings
              </span>
              {showExpert ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
            </button>

            {showExpert && (
              <div className="mt-4 grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                <ConfigField
                  label="Thruster L1 Weight"
                  value={config?.mpc.thrust_l1_weight}
                  onChange={(v) => updateConfig('mpc.thrust_l1_weight', v)}
                  isNumber
                />
                <ConfigField
                  label="Thruster Pair Weight"
                  value={config?.mpc.thrust_pair_weight}
                  onChange={(v) => updateConfig('mpc.thrust_pair_weight', v)}
                  isNumber
                />
              </div>
            )}
          </section>

          <section>
            <button
              onClick={() => setShowReference((v) => !v)}
              className="w-full flex items-center justify-between p-3 rounded border border-slate-800 bg-slate-900 hover:bg-slate-800 transition-colors"
            >
              <span className="text-sm uppercase tracking-wider text-emerald-400 font-bold">
                Settings Reference
              </span>
              {showReference ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
            </button>

            {showReference && (
              <div className="mt-4 space-y-4">
                {SETTING_REFERENCE_SECTIONS.map((section) => (
                  <div key={section.title} className="rounded border border-slate-800 bg-slate-900/70 p-4">
                    <h4 className="text-xs uppercase tracking-wider text-slate-400 font-bold mb-3">
                      {section.title}
                    </h4>
                    <div className="grid grid-cols-1 lg:grid-cols-2 gap-3">
                      {section.items.map((item) => (
                        <div key={item.key} className="rounded border border-slate-800 bg-slate-950/60 p-3">
                          <div className="flex items-center justify-between gap-2 mb-1">
                            <p className="text-sm font-semibold text-slate-200">{item.label}</p>
                            <span className="text-[10px] text-slate-500 font-mono">{item.key}</span>
                          </div>
                          <p className="text-xs text-slate-300 mb-1">{item.description}</p>
                          <p className="text-[11px] text-emerald-300/90">{item.impact}</p>
                        </div>
                      ))}
                    </div>
                  </div>
                ))}
              </div>
            )}
          </section>
        </div>
      </div>
    </div>
  );
}

interface ConfigFieldProps {
  label: string;
  value: unknown;
  onChange: (value: string) => void;
  isNumber?: boolean;
  desc?: string;
  step?: number;
}

function ConfigField({ label, value, onChange, isNumber, desc, step }: ConfigFieldProps) {
  const inputValue =
    typeof value === 'string' || typeof value === 'number' ? value : '';
  const inputId = `cfg-${label.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`;

  return (
    <div className="flex flex-col gap-1">
      <label htmlFor={inputId} className="text-xs font-semibold text-slate-300 uppercase">{label}</label>
      <input
        id={inputId}
        aria-label={label}
        type={isNumber ? 'number' : 'text'}
        step={step ?? (isNumber ? 1 : undefined)}
        value={inputValue}
        onChange={(e) => onChange(e.target.value)}
        className="bg-slate-900 border border-slate-600 rounded px-3 py-2 text-sm text-white focus:outline-none focus:border-blue-500 transition-colors"
      />
      {desc && <span className="text-[10px] text-slate-400">{desc}</span>}
    </div>
  );
}

interface ToggleFieldProps {
  label: string;
  checked: boolean;
  onChange: (checked: boolean) => void;
}

function ToggleField({ label, checked, onChange }: ToggleFieldProps) {
  const inputId = `toggle-${label.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`;
  return (
    <div className="flex items-center justify-between p-3 bg-slate-900 rounded border border-slate-800">
      <label htmlFor={inputId} className="text-sm font-medium text-slate-200">
        {label}
      </label>
      <input
        id={inputId}
        aria-label={label}
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        className="w-5 h-5 rounded border-slate-600 bg-slate-700 text-blue-600 focus:ring-offset-slate-900"
      />
    </div>
  );
}

interface SelectFieldOption {
  label: string;
  value: string;
}

interface SelectFieldProps {
  label: string;
  value: string;
  onChange: (value: string) => void;
  options: SelectFieldOption[];
  desc?: string;
}

function SelectField({ label, value, onChange, options, desc }: SelectFieldProps) {
  const selectId = `sel-${label.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`;
  return (
    <div className="flex flex-col gap-1">
      <label htmlFor={selectId} className="text-xs font-semibold text-slate-300 uppercase">{label}</label>
      <select
        id={selectId}
        aria-label={label}
        value={value}
        onChange={(e) => onChange(e.target.value)}
        className="bg-slate-900 border border-slate-600 rounded px-3 py-2 text-sm text-white focus:outline-none focus:border-blue-500 transition-colors"
      >
        {options.map((option) => (
          <option key={option.value} value={option.value}>
            {option.label}
          </option>
        ))}
      </select>
      {desc && <span className="text-[10px] text-slate-400">{desc}</span>}
    </div>
  );
}

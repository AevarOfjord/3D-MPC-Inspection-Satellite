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
  Q_terminal_pos: number;
  Q_terminal_s: number;
  q_angular_velocity: number;
  r_thrust: number;
  r_rw_torque: number;
  thrust_l1_weight: number;
  thrust_pair_weight: number;
  coast_pos_tolerance: number;
  coast_vel_tolerance: number;
  coast_min_speed: number;
  thruster_type: 'PWM' | 'CON';
  verbose_mpc: boolean;
  obstacle_margin: number;
  enable_collision_avoidance: boolean;
  path_speed: number;
  path_speed_min: number;
  path_speed_max: number;
  progress_taper_distance: number;
  progress_slowdown_distance: number;
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
}

interface PresetPayload {
  config: SettingsConfig;
  updated_at?: string;
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
  control_horizon: 50,
  dt: 0.05,
  solver_time_limit: 0.04,
  solver_type: 'OSQP',
  Q_contour: 100000.0,
  Q_progress: 100.0,
  progress_reward: 0.0,
  Q_lag: 0.0,
  Q_lag_default: -1.0,
  Q_velocity_align: 0.0,
  Q_s_anchor: -1.0,
  Q_smooth: 10.0,
  Q_attitude: 5000.0,
  Q_terminal_pos: 0.0,
  Q_terminal_s: 0.0,
  q_angular_velocity: 1000.0,
  r_thrust: 0.1,
  r_rw_torque: 0.01,
  thrust_l1_weight: 0.1,
  thrust_pair_weight: 2.0,
  coast_pos_tolerance: 0.1,
  coast_vel_tolerance: 0.02,
  coast_min_speed: 0.02,
  thruster_type: 'CON',
  verbose_mpc: false,
  obstacle_margin: 0.1,
  enable_collision_avoidance: false,
  path_speed: 0.1,
  path_speed_min: 0.01,
  path_speed_max: 0.1,
  progress_taper_distance: 0.0,
  progress_slowdown_distance: 0.0,
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
        key: 'mpc.progress_taper_distance',
        label: 'Progress Taper Distance (m)',
        description: 'Distance-to-end region where path speed can taper. 0 uses auto behavior.',
        impact: 'Higher values start slowing earlier near endpoint.',
      },
      {
        key: 'mpc.progress_slowdown_distance',
        label: 'Progress Slowdown Distance (m)',
        description: 'Error-trigger distance used to reduce progress when tracking quality degrades.',
        impact: 'Higher values trigger slowdown earlier for safer tracking.',
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
      {
        key: 'mpc.coast_pos_tolerance',
        label: 'Coast Position Tol. (m)',
        description: 'Position-error band where coasting logic may engage.',
        impact: 'Higher tolerance enters coast mode more easily.',
      },
      {
        key: 'mpc.coast_vel_tolerance',
        label: 'Coast Velocity Tol. (m/s)',
        description: 'Lateral velocity threshold for coasting behavior.',
        impact: 'Higher tolerance allows coasting at larger residual velocity.',
      },
      {
        key: 'mpc.coast_min_speed',
        label: 'Coast Min Speed (m/s)',
        description: 'Minimum forward speed maintained when coasting logic is active.',
        impact: 'Higher values keep progress moving even in coast mode.',
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

function normalizeConfig(raw: unknown): SettingsConfig | null {
  const root = asRecord(raw);
  if (!root) return null;

  const mpc = asRecord(root.mpc);
  const simulation = asRecord(root.simulation);

  if (mpc && simulation) {
    const normalizedMpc = {
      ...DEFAULT_MPC_SETTINGS,
      ...(mpc as Partial<MpcSettings>),
    };
    const normalizedSimulation = {
      ...DEFAULT_SIMULATION_SETTINGS,
      ...(simulation as Partial<SimulationSettings>),
    };
    if (typeof normalizedMpc.dt === 'number') {
      normalizedSimulation.control_dt = normalizedMpc.dt;
    }
    return {
      mpc: normalizedMpc,
      simulation: normalizedSimulation,
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
    ...(legacyMpc as Partial<MpcSettings>),
  };

  if (legacyWeights) {
    if (typeof legacyWeights.Q_contour === 'number') normalizedMpc.Q_contour = legacyWeights.Q_contour;
    if (typeof legacyWeights.Q_progress === 'number') normalizedMpc.Q_progress = legacyWeights.Q_progress;
    if (typeof legacyWeights.Q_smooth === 'number') normalizedMpc.Q_smooth = legacyWeights.Q_smooth;
    if (typeof legacyWeights.Q_attitude === 'number') normalizedMpc.Q_attitude = legacyWeights.Q_attitude;
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
  };
}

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

  const nonNegativeWeights: Array<[string, number]> = [
    ['Q_contour', mpc.Q_contour],
    ['Q_progress', mpc.Q_progress],
    ['Q_lag', mpc.Q_lag],
    ['Q_velocity_align', mpc.Q_velocity_align],
    ['Q_smooth', mpc.Q_smooth],
    ['Q_attitude', mpc.Q_attitude],
    ['Q_terminal_pos', mpc.Q_terminal_pos],
    ['Q_terminal_s', mpc.Q_terminal_s],
    ['q_angular_velocity', mpc.q_angular_velocity],
    ['r_thrust', mpc.r_thrust],
    ['r_rw_torque', mpc.r_rw_torque],
    ['thrust_l1_weight', mpc.thrust_l1_weight],
    ['thrust_pair_weight', mpc.thrust_pair_weight],
    ['coast_pos_tolerance', mpc.coast_pos_tolerance],
    ['coast_vel_tolerance', mpc.coast_vel_tolerance],
    ['coast_min_speed', mpc.coast_min_speed],
  ];
  nonNegativeWeights.forEach(([name, value]) => {
    if (!isNonNegative(value)) issues.push(`${name} must be >= 0.`);
  });

  return issues;
}

export function MPCSettingsView({ onDirtyChange }: MPCSettingsViewProps) {
  const [config, setConfig] = useState<SettingsConfig | null>(null);
  const [savedSnapshot, setSavedSnapshot] = useState<string>('');
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
  const validationErrors = useMemo(() => (config ? validateConfig(config) : []), [config]);
  const isDirty = useMemo(
    () => (config ? stableSerializeConfig(config) !== savedSnapshot : false),
    [config, savedSnapshot]
  );

  useEffect(() => {
    void fetchConfig();
    void fetchPresets();
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

  const fetchConfig = async () => {
    setIsLoading(true);
    setError(null);
    try {
      const res = await fetch(`${RUNNER_API_URL}/config`);
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to fetch config'));
      const data = await res.json();
      const normalized = normalizeConfig(data);
      if (!normalized) throw new Error('Backend returned invalid config format');
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
      const overrides = {
        mpc: config.mpc,
        simulation: {
          ...config.simulation,
          control_dt: config.mpc.dt,
        },
      };

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
          config: deepCloneConfig(config),
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
                  label="Progress Taper Distance (m)"
                  value={config?.mpc.progress_taper_distance}
                  onChange={(v) => updateConfig('mpc.progress_taper_distance', v)}
                  isNumber
                  step={0.01}
                  desc="0 = auto"
                />
                <ConfigField
                  label="Progress Slowdown Distance (m)"
                  value={config?.mpc.progress_slowdown_distance}
                  onChange={(v) => updateConfig('mpc.progress_slowdown_distance', v)}
                  isNumber
                  step={0.01}
                  desc="0 = auto"
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
                <ConfigField
                  label="Coast Position Tol. (m)"
                  value={config?.mpc.coast_pos_tolerance}
                  onChange={(v) => updateConfig('mpc.coast_pos_tolerance', v)}
                  isNumber
                />
                <ConfigField
                  label="Coast Velocity Tol. (m/s)"
                  value={config?.mpc.coast_vel_tolerance}
                  onChange={(v) => updateConfig('mpc.coast_vel_tolerance', v)}
                  isNumber
                />
                <ConfigField
                  label="Coast Min Speed (m/s)"
                  value={config?.mpc.coast_min_speed}
                  onChange={(v) => updateConfig('mpc.coast_min_speed', v)}
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

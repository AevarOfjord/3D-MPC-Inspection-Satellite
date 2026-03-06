import type {
  ControllerProfileId,
  MpcCoreSettings,
  MpcProfileOverrideSettings,
  MpcProfileOverridesSettings,
  MpcSettings,
  SharedSettings,
  SimulationSettings,
  SettingsConfig,
} from './mpcSettingsTypes';
import {
  CONTROLLER_PROFILE_IDS,
  DEFAULT_MPC_CORE_SETTINGS,
  DEFAULT_MPC_PROFILE_OVERRIDES,
  DEFAULT_MPC_SETTINGS,
  DEFAULT_SHARED_SETTINGS,
  DEFAULT_SIMULATION_SETTINGS,
  MPC_CANONICAL_KEYS,
} from './mpcSettingsDefaults';

export function asRecord(value: unknown): Record<string, unknown> | null {
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

function normalizeControllerProfileValue(
  mpcCore: Record<string, unknown> | null | undefined
): MpcCoreSettings['controller_profile'] {
  const profile = mpcCore?.controller_profile;
  if (typeof profile === 'string') {
    if ((CONTROLLER_PROFILE_IDS as string[]).includes(profile)) {
      return profile as ControllerProfileId;
    }
    if (profile === 'hybrid') return 'cpp_hybrid_rti_osqp';
    if (profile === 'nonlinear') return 'cpp_nonlinear_rti_osqp';
    if (profile === 'linear') return 'cpp_linearized_rti_osqp';
  }
  const legacyBackend = mpcCore?.controller_backend;
  if (legacyBackend === 'v1') return 'cpp_linearized_rti_osqp';
  return 'cpp_hybrid_rti_osqp';
}

function sanitizeMpcCore(
  mpcCore: Record<string, unknown> | null | undefined
): MpcCoreSettings {
  const normalized: MpcCoreSettings = {
    ...DEFAULT_MPC_CORE_SETTINGS,
    ...(mpcCore as Partial<MpcCoreSettings>),
    controller_profile: normalizeControllerProfileValue(mpcCore),
  };
  // Deprecated legacy backend field must never survive normalization/emission.
  delete (normalized as Record<string, unknown>).controller_backend;
  return normalized;
}

function sanitizeProfileOverrideEntry(
  entry: unknown,
  _fallback: MpcProfileOverrideSettings
): MpcProfileOverrideSettings {
  const entryObj = asRecord(entry);
  const rawBase = asRecord(entryObj?.base_overrides);
  const rawSpecific = asRecord(entryObj?.profile_specific);
  const base_overrides: Partial<MpcSettings> = {};
  Object.entries(rawBase ?? {}).forEach(([key, value]) => {
    if (MPC_CANONICAL_KEYS.has(key)) {
      (base_overrides as Record<string, unknown>)[key] = value;
    }
  });
  return {
    base_overrides,
    profile_specific: { ...(rawSpecific ?? {}) },
  };
}

function normalizeProfileOverrideKey(key: string): ControllerProfileId | null {
  if ((CONTROLLER_PROFILE_IDS as string[]).includes(key)) {
    return key as ControllerProfileId;
  }
  if (key === 'hybrid') return 'cpp_hybrid_rti_osqp';
  if (key === 'nonlinear') return 'cpp_nonlinear_rti_osqp';
  if (key === 'linear') return 'cpp_linearized_rti_osqp';
  return null;
}

function sanitizeMpcProfileOverrides(
  profileOverrides: Record<string, unknown> | null | undefined
): MpcProfileOverridesSettings {
  const normalized = JSON.parse(
    JSON.stringify(DEFAULT_MPC_PROFILE_OVERRIDES)
  ) as MpcProfileOverridesSettings;

  Object.entries(profileOverrides ?? {}).forEach(([rawKey, entry]) => {
    const key = normalizeProfileOverrideKey(rawKey);
    if (!key) return;
    normalized[key] = sanitizeProfileOverrideEntry(
      entry,
      DEFAULT_MPC_PROFILE_OVERRIDES[key]
    );
  });

  return normalized;
}

function sanitizeSharedSettings(
  shared: Record<string, unknown> | null | undefined
): SharedSettings {
  const profileFiles = asRecord(shared?.profile_parameter_files);
  const normalizedFiles = {
    ...DEFAULT_SHARED_SETTINGS.profile_parameter_files,
  } as Record<ControllerProfileId, string>;
  Object.entries(profileFiles ?? {}).forEach(([rawKey, value]) => {
    const key = normalizeProfileOverrideKey(rawKey);
    if (!key || typeof value !== 'string') return;
    normalizedFiles[key] = value;
  });

  return {
    parameters:
      typeof shared?.parameters === 'boolean'
        ? shared.parameters
        : DEFAULT_SHARED_SETTINGS.parameters,
    profile_parameter_files: normalizedFiles,
  };
}

export function normalizeConfig(raw: unknown): SettingsConfig | null {
  const root = asRecord(raw);
  if (!root) return null;

  const appConfig = asRecord(root.app_config);
  const source = appConfig ?? root;
  const mpcCore = asRecord(source.mpc_core);
  const mpc = asRecord(source.mpc) ?? mpcCore;
  const shared = asRecord(source.shared);
  const simulation = asRecord(source.simulation);
  const mpcProfileOverrides = asRecord(source.mpc_profile_overrides);
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
    const normalizedMpcCore = sanitizeMpcCore(mpcCore);
    const normalizedProfileOverrides = sanitizeMpcProfileOverrides(mpcProfileOverrides);
    const normalizedShared = sanitizeSharedSettings(shared);
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
      mpc_core: normalizedMpcCore,
      shared: normalizedShared,
      mpc_profile_overrides: normalizedProfileOverrides,
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
    mpc_core: sanitizeMpcCore(legacyMpc),
    shared: sanitizeSharedSettings(undefined),
    mpc_profile_overrides: sanitizeMpcProfileOverrides(undefined),
    simulation: normalizedSimulation,
    physics: undefined,
    reference_scheduler: undefined,
    actuator_policy: undefined,
    controller_contracts: undefined,
    input_file_path: undefined,
  };
}

export function buildV3Envelope(config: SettingsConfig): Record<string, unknown> {
  const sanitizedMpc = stripRemovedMpcFields(config.mpc as Record<string, unknown>);
  const mpcCore = sanitizeMpcCore(asRecord(config.mpc_core));
  const shared = sanitizeSharedSettings(asRecord(config.shared));
  const profileOverrides = sanitizeMpcProfileOverrides(asRecord(config.mpc_profile_overrides));
  const actuatorPolicy =
    asRecord(config.actuator_policy) ?? {};
  const mergedActuatorPolicy: Record<string, unknown> = {
    ...actuatorPolicy,
    enable_thruster_hysteresis: config.mpc.enable_thruster_hysteresis,
    thruster_hysteresis_on: config.mpc.thruster_hysteresis_on,
    thruster_hysteresis_off: config.mpc.thruster_hysteresis_off,
  };

  const appConfig: Record<string, unknown> = {
    mpc: sanitizedMpc,
    mpc_core: mpcCore,
    shared,
    mpc_profile_overrides: profileOverrides,
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

export const MPC_SETTINGS_TESTING = {
  normalizeConfig,
  buildV3Envelope,
  stripRemovedMpcFields,
};

export function stableSerializeConfig(config: SettingsConfig): string {
  return JSON.stringify(config);
}

export function deepCloneConfig(config: SettingsConfig): SettingsConfig {
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

export async function parseApiError(res: Response, fallback: string): Promise<string> {
  const text = await res.text();
  const detail = parseApiErrorText(text);
  return detail
    ? `${fallback} (HTTP ${res.status}): ${detail}`
    : `${fallback} (HTTP ${res.status})`;
}

function isNonNegative(n: number): boolean {
  return Number.isFinite(n) && n >= 0;
}

export function validateConfig(config: SettingsConfig): string[] {
  const issues: string[] = [];
  const { mpc, simulation, shared } = config;
  const mpcCore = config.mpc_core;

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
  if (!(CONTROLLER_PROFILE_IDS as string[]).includes(mpcCore.controller_profile)) {
    issues.push(
      'Controller profile must be one of the six canonical profile identifiers.'
    );
  }
  const profileOverrides = sanitizeMpcProfileOverrides(asRecord(config.mpc_profile_overrides));
  const selectedProfile = mpcCore.controller_profile;
  const selectedProfileSpecific = profileOverrides[selectedProfile].profile_specific;
  const sqpMaxIter = Number(selectedProfileSpecific.sqp_max_iter);
  if (Number.isFinite(sqpMaxIter) && sqpMaxIter < 1) {
    issues.push('Profile-specific sqp_max_iter must be >= 1.');
  }
  const freezeInterval = Number(selectedProfileSpecific.freeze_refresh_interval_steps);
  if (Number.isFinite(freezeInterval) && freezeInterval < 1) {
    issues.push('Profile-specific freeze_refresh_interval_steps must be >= 1.');
  }
  const ipoptMaxIter = Number(selectedProfileSpecific.ipopt_max_iter);
  if (Number.isFinite(ipoptMaxIter) && ipoptMaxIter < 1) {
    issues.push('Profile-specific ipopt_max_iter must be >= 1.');
  }
  const acadosMaxIter = Number(selectedProfileSpecific.acados_max_iter);
  if (Number.isFinite(acadosMaxIter) && acadosMaxIter < 1) {
    issues.push('Profile-specific acados_max_iter must be >= 1.');
  }
  ['acados_tol_stat', 'acados_tol_eq', 'acados_tol_ineq'].forEach((key) => {
    const value = Number(selectedProfileSpecific[key]);
    if (Number.isFinite(value) && value <= 0) {
      issues.push(`Profile-specific ${key} must be > 0.`);
    }
  });

  if (shared.parameters) {
    CONTROLLER_PROFILE_IDS.forEach((profileId) => {
      const entry = profileOverrides[profileId];
      if (Object.keys(entry.base_overrides).length > 0) {
        issues.push(
          `shared.parameters=true requires ${profileId}.base_overrides to be empty.`
        );
      }
      if (Object.keys(entry.profile_specific).length > 0) {
        issues.push(
          `shared.parameters=true requires ${profileId}.profile_specific to be empty.`
        );
      }
    });
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

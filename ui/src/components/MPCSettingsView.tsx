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

import type {
  ControllerProfileId,
  MpcSettings,
  SettingsConfig,
  RunnerSystemStatus,
  PackageJobStatus,
  WorkspaceInspection,
  MPCSettingsViewProps,
  PresetPayload,
} from './mpc-settings/mpcSettingsTypes';
import {
  CONTROLLER_PROFILE_IDS,
  CONTROLLER_PROFILE_LABELS,
  DEFAULT_MPC_PROFILE_OVERRIDES,
  SETTING_REFERENCE_SECTIONS,
} from './mpc-settings/mpcSettingsDefaults';
import {
  asRecord,
  normalizeConfig,
  buildV3Envelope,
  stableSerializeConfig,
  deepCloneConfig,
  parseApiError,
  validateConfig,
} from './mpc-settings/mpcSettingsUtils';

// Re-export for tests
export { MPC_SETTINGS_TESTING } from './mpc-settings/mpcSettingsUtils';

type SettingsSection = 'mpc' | 'general';

const SETTINGS_SECTION_STORAGE_KEY = 'mission_control_settings_section_v1';

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
  const [settingsSection, setSettingsSection] = useState<SettingsSection>(() => {
    try {
      const stored = window.sessionStorage.getItem(SETTINGS_SECTION_STORAGE_KEY);
      return stored === 'general' ? 'general' : 'mpc';
    } catch {
      return 'mpc';
    }
  });
  const validationErrors = useMemo(() => (config ? validateConfig(config) : []), [config]);
  const isDirty = useMemo(
    () => (config ? stableSerializeConfig(config) !== savedSnapshot : false),
    [config, savedSnapshot]
  );
  const activeControllerProfile = useMemo<ControllerProfileId>(
    () => {
      const profile = String(config?.mpc_core.controller_profile ?? 'cpp_hybrid_rti_osqp');
      if ((CONTROLLER_PROFILE_IDS as string[]).includes(profile)) {
        return profile as ControllerProfileId;
      }
      return 'cpp_hybrid_rti_osqp';
    },
    [config?.mpc_core.controller_profile]
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
    try {
      window.sessionStorage.setItem(SETTINGS_SECTION_STORAGE_KEY, settingsSection);
    } catch {
      // no-op
    }
  }, [settingsSection]);

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

  const ensureProfileOverrides = (
    root: Record<string, unknown>
  ): Record<ControllerProfileId, Record<string, unknown>> => {
    const existing = asRecord(root.mpc_profile_overrides);
    const normalized = {} as Record<ControllerProfileId, Record<string, unknown>>;
    CONTROLLER_PROFILE_IDS.forEach((profileId) => {
      normalized[profileId] = {
        ...DEFAULT_MPC_PROFILE_OVERRIDES[profileId],
        ...(asRecord(existing?.[profileId]) ?? {}),
      };
    });
    root.mpc_profile_overrides = normalized;
    return normalized;
  };

  const updateSelectedProfileBaseOverride = (
    field: keyof MpcSettings,
    rawValue: string
  ) => {
    if (!config || sharedParametersEnabled) return;
    const newConfig = JSON.parse(JSON.stringify(config)) as Record<string, unknown>;
    const overrides = ensureProfileOverrides(newConfig);
    const selected = overrides[activeControllerProfile];
    const base = asRecord(selected.base_overrides) ?? {};
    const baseline = asRecord(newConfig.mpc)?.[String(field)];
    const trimmed = rawValue.trim();
    if (trimmed === '') {
      delete base[String(field)];
    } else if (typeof baseline === 'number') {
      const parsed = Number(trimmed);
      if (!Number.isFinite(parsed)) return;
      base[String(field)] = parsed;
    } else if (typeof baseline === 'boolean') {
      base[String(field)] = trimmed.toLowerCase() === 'true';
    } else {
      base[String(field)] = trimmed;
    }
    selected.base_overrides = base;
    setConfig(newConfig as unknown as SettingsConfig);
  };

  const updateSelectedProfileSpecific = (
    key: string,
    rawValue: string | boolean
  ) => {
    if (!config || sharedParametersEnabled) return;
    const newConfig = JSON.parse(JSON.stringify(config)) as Record<string, unknown>;
    const overrides = ensureProfileOverrides(newConfig);
    const selected = overrides[activeControllerProfile];
    const profileSpecific = asRecord(selected.profile_specific) ?? {};
    if (typeof rawValue === 'boolean') {
      profileSpecific[key] = rawValue;
    } else {
      const trimmed = rawValue.trim();
      if (trimmed === '') {
        delete profileSpecific[key];
      } else {
        const numeric = Number(trimmed);
        profileSpecific[key] = Number.isFinite(numeric) ? numeric : trimmed;
      }
    }
    selected.profile_specific = profileSpecific;
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

  const selectedProfileOverrideEntry = (
    asRecord(config?.mpc_profile_overrides)?.[activeControllerProfile] ??
    DEFAULT_MPC_PROFILE_OVERRIDES[activeControllerProfile]
  ) as unknown;
  const sharedParametersEnabled = Boolean(config?.shared.parameters);
  const selectedBaseOverrides =
    asRecord(asRecord(selectedProfileOverrideEntry)?.base_overrides) ?? {};
  const selectedProfileSpecific =
    asRecord(asRecord(selectedProfileOverrideEntry)?.profile_specific) ?? {};
  const selectedOverrideDiff = Object.entries(selectedBaseOverrides)
    .filter(([key, value]) => asRecord(config?.mpc)?.[key] !== value)
    .sort(([a], [b]) => a.localeCompare(b));
  const settingsSubtitle =
    settingsSection === 'mpc'
      ? 'Tune shared baseline, profile overrides, presets, and run-shaping controller settings.'
      : 'Manage readiness checks, runner utilities, packaging, and workspace import/export.';

  return (
    <div className="h-full w-full flex flex-col bg-slate-950 text-slate-200 overflow-hidden">
      <div className="flex-none border-b border-slate-800 bg-slate-900/50 p-4">
        <div className="flex flex-col gap-4 xl:flex-row xl:items-start xl:justify-between">
          <div className="space-y-3">
            <div>
              <h2 className="text-lg font-bold text-white">Settings</h2>
              <p className="text-xs text-slate-300">{settingsSubtitle}</p>
              <p className={`mt-1 text-[11px] ${isDirty ? 'text-amber-300' : 'text-emerald-300'}`}>
                {isDirty ? 'Unsaved changes' : 'All changes saved'}
              </p>
            </div>
            <div className="inline-flex rounded-xl border border-slate-700 bg-slate-950/70 p-1">
              <button
                type="button"
                onClick={() => setSettingsSection('mpc')}
                className={`rounded-lg px-3 py-1.5 text-xs font-semibold transition ${
                  settingsSection === 'mpc'
                    ? 'bg-cyan-950/60 text-cyan-100'
                    : 'text-slate-300 hover:bg-slate-800 hover:text-white'
                }`}
              >
                MPC Settings
              </button>
              <button
                type="button"
                onClick={() => setSettingsSection('general')}
                className={`rounded-lg px-3 py-1.5 text-xs font-semibold transition ${
                  settingsSection === 'general'
                    ? 'bg-slate-800 text-white'
                    : 'text-slate-300 hover:bg-slate-800 hover:text-white'
                }`}
              >
                General Settings
              </button>
            </div>
          </div>

          {settingsSection === 'mpc' ? (
            <div className="flex flex-wrap items-center gap-2">
              <input
                type="text"
                value={presetName}
                onChange={(e) => setPresetName(e.target.value)}
                placeholder="Preset name"
                aria-label="Preset name"
                className="w-32 rounded border border-slate-700 bg-slate-900 px-2 py-1.5 text-xs text-white focus:border-blue-500 focus:outline-none"
              />
              <button
                onClick={handleSavePreset}
                className="rounded bg-slate-700 px-2.5 py-1.5 text-xs text-slate-100 hover:bg-slate-600"
                aria-label="Save preset"
              >
                Save Preset
              </button>
              <select
                value={selectedPreset}
                onChange={(e) => setSelectedPreset(e.target.value)}
                aria-label="Preset selection"
                className="w-40 rounded border border-slate-700 bg-slate-900 px-2 py-1.5 text-xs text-white focus:border-blue-500 focus:outline-none"
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
                className="rounded bg-slate-700 px-2.5 py-1.5 text-xs text-slate-100 disabled:opacity-40 hover:bg-slate-600"
                disabled={!selectedPreset}
                aria-label="Load selected preset"
              >
                Load
              </button>
              <button
                onClick={handleDeletePreset}
                className="rounded bg-slate-700 px-2.5 py-1.5 text-xs text-slate-100 disabled:opacity-40 hover:bg-slate-600"
                disabled={!selectedPreset}
                aria-label="Delete selected preset"
              >
                Delete
              </button>
              <button
                onClick={() => void handleReset()}
                className="flex items-center gap-2 rounded bg-slate-800 px-3 py-1.5 text-sm text-slate-300 transition hover:bg-slate-700"
                aria-label="Reset configuration to defaults"
              >
                <RotateCcw size={14} /> Reset
              </button>
              <button
                onClick={() => void handleSave()}
                disabled={isSaving || validationErrors.length > 0}
                className="flex items-center gap-2 rounded bg-blue-600 px-4 py-1.5 text-sm font-semibold text-white shadow-sm transition hover:bg-blue-500 disabled:opacity-50"
                aria-label="Save settings"
              >
                {isSaving ? <Loader2 size={14} className="animate-spin" /> : <Save size={14} />}
                Save Changes
              </button>
            </div>
          ) : null}
        </div>
      </div>

      <div className="flex-1 overflow-y-auto p-6">
        <div className="w-full space-y-8">
          {error && (
            <div className="flex items-center gap-2 rounded border border-red-800 bg-red-900/20 p-3 text-sm text-red-200">
              <AlertCircle size={16} /> {error}
            </div>
          )}
          {successMsg && (
            <div className="flex items-center gap-2 rounded border border-green-800 bg-green-900/20 p-3 text-sm text-green-200">
              <Check size={16} /> {successMsg}
            </div>
          )}
          {settingsSection === 'mpc' && removedMpcFieldsWarning.length > 0 && (
            <div className="rounded border border-amber-700 bg-amber-900/20 p-3 text-sm text-amber-200">
              <p className="font-semibold">Deprecated MPC fields were dropped by backend:</p>
              <p className="mt-1 text-xs">{removedMpcFieldsWarning.join(', ')}</p>
            </div>
          )}
          {settingsSection === 'mpc' && validationErrors.length > 0 && (
            <div className="rounded border border-amber-700 bg-amber-900/20 p-3 text-sm text-amber-200">
              <p className="mb-1 font-semibold">Validation issues ({validationErrors.length}):</p>
              <ul className="ml-5 list-disc space-y-0.5">
                {validationErrors.map((msg) => (
                  <li key={msg}>{msg}</li>
                ))}
              </ul>
            </div>
          )}
          {settingsSection === 'mpc' ? (
            <>
              <section>
                <button
                  onClick={() => setShowBasic((v) => !v)}
                  className="w-full flex items-center justify-between rounded border border-slate-800 bg-slate-900 p-3 transition-colors hover:bg-slate-800"
                >
                  <span className="text-sm font-bold uppercase tracking-wider text-blue-400">
                    Basic Settings
                  </span>
                  {showBasic ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
                </button>

                {showBasic && (
                  <div className="mt-4 space-y-8">
                    <section>
                      <h3 className="mb-4 border-b border-emerald-900/30 pb-1 text-sm font-bold uppercase tracking-wider text-emerald-400">
                        Parameter Source Policy
                      </h3>
                      <div className="grid grid-cols-1 gap-6 md:grid-cols-2">
                        <ToggleField
                          label="Use Shared Parameters For All Profiles"
                          checked={sharedParametersEnabled}
                          onChange={(checked) => updateConfig('shared.parameters', checked)}
                        />
                        <div className="rounded border border-slate-800 bg-slate-950/60 p-3">
                          <p className="text-xs text-slate-300">
                            {sharedParametersEnabled
                              ? 'Fair-comparison mode is active. All six controllers use the shared baseline in app_config.mpc.'
                              : 'Per-profile tuning mode is active. Only the selected controller profile can apply delta overrides and an external profile parameter file.'}
                          </p>
                        </div>
                      </div>
                      <div className="mt-5 grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
                        {CONTROLLER_PROFILE_IDS.map((profileId) => (
                          <ConfigField
                            key={profileId}
                            label={`${CONTROLLER_PROFILE_LABELS[profileId]} File`}
                            value={config?.shared.profile_parameter_files[profileId] ?? ''}
                            onChange={(v) =>
                              updateConfig(`shared.profile_parameter_files.${profileId}`, v)
                            }
                            desc={
                              profileId === activeControllerProfile
                                ? 'Applied only when shared mode is off and this profile is selected.'
                                : 'Stored for this profile; inactive unless that profile is selected.'
                            }
                          />
                        ))}
                      </div>
                    </section>

                    <section>
                      <h3 className="mb-4 border-b border-slate-800 pb-1 text-sm font-bold uppercase tracking-wider text-slate-500">
                        Basic - Timing and Horizons
                      </h3>
                      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
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
                        <SelectField
                          label="Controller Profile"
                          value={String(config?.mpc_core.controller_profile ?? 'cpp_hybrid_rti_osqp')}
                          onChange={(v) => updateConfig('mpc_core.controller_profile', v)}
                          options={CONTROLLER_PROFILE_IDS.map((profileId) => ({
                            label: CONTROLLER_PROFILE_LABELS[profileId],
                            value: profileId,
                          }))}
                          desc="In per-profile mode, this decides which profile delta file and override block are active."
                        />
                      </div>
                    </section>

                    <section>
                      <h3 className="mb-4 border-b border-blue-900/30 pb-1 text-sm font-bold uppercase tracking-wider text-blue-400">
                        Selected Profile Overrides ({CONTROLLER_PROFILE_LABELS[activeControllerProfile]})
                      </h3>
                      {sharedParametersEnabled && (
                        <div className="mb-4 rounded border border-amber-900/40 bg-amber-950/40 p-3 text-xs text-amber-200">
                          Per-profile overrides are preserved for later tuning, but inactive while shared fair-comparison mode is enabled.
                        </div>
                      )}
                      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
                        <ConfigField
                          label="Override Prediction Horizon"
                          value={selectedBaseOverrides.prediction_horizon ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('prediction_horizon', v)}
                          isNumber
                          step={1}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                        <ConfigField
                          label="Override Control Horizon"
                          value={selectedBaseOverrides.control_horizon ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('control_horizon', v)}
                          isNumber
                          step={1}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                        <ConfigField
                          label="Override Solver Time Limit (s)"
                          value={selectedBaseOverrides.solver_time_limit ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('solver_time_limit', v)}
                          isNumber
                          step={0.001}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                        <ConfigField
                          label="Override Q_contour"
                          value={selectedBaseOverrides.Q_contour ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('Q_contour', v)}
                          isNumber
                          step={1}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                        <ConfigField
                          label="Override Q_progress"
                          value={selectedBaseOverrides.Q_progress ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('Q_progress', v)}
                          isNumber
                          step={1}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                        <ConfigField
                          label="Override Q_attitude"
                          value={selectedBaseOverrides.Q_attitude ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('Q_attitude', v)}
                          isNumber
                          step={1}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                        <ConfigField
                          label="Override Q_smooth"
                          value={selectedBaseOverrides.Q_smooth ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('Q_smooth', v)}
                          isNumber
                          step={1}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                        <ConfigField
                          label="Override Path Speed"
                          value={selectedBaseOverrides.path_speed ?? ''}
                          onChange={(v) => updateSelectedProfileBaseOverride('path_speed', v)}
                          isNumber
                          step={0.001}
                          desc="Blank = inherit shared baseline"
                          disabled={sharedParametersEnabled}
                        />
                      </div>
                      <div className="mt-5 grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-3">
                        {activeControllerProfile === 'cpp_hybrid_rti_osqp' && (
                          <ToggleField
                            label="allow_stale_stage_reuse"
                            checked={Boolean(selectedProfileSpecific.allow_stale_stage_reuse ?? true)}
                            onChange={(checked) =>
                              updateSelectedProfileSpecific('allow_stale_stage_reuse', checked)
                            }
                            disabled={sharedParametersEnabled}
                          />
                        )}
                        {activeControllerProfile === 'cpp_nonlinear_rti_osqp' && (
                          <>
                            <ConfigField
                              label="sqp_max_iter"
                              value={selectedProfileSpecific.sqp_max_iter ?? 2}
                              onChange={(v) => updateSelectedProfileSpecific('sqp_max_iter', v)}
                              isNumber
                              step={1}
                              disabled={sharedParametersEnabled}
                            />
                            <ConfigField
                              label="sqp_tol"
                              value={selectedProfileSpecific.sqp_tol ?? 0.0001}
                              onChange={(v) => updateSelectedProfileSpecific('sqp_tol', v)}
                              isNumber
                              step={0.0001}
                              disabled={sharedParametersEnabled}
                            />
                            <ToggleField
                              label="strict_integrity"
                              checked={Boolean(selectedProfileSpecific.strict_integrity ?? true)}
                              onChange={(checked) =>
                                updateSelectedProfileSpecific('strict_integrity', checked)
                              }
                              disabled={sharedParametersEnabled}
                            />
                          </>
                        )}
                        {activeControllerProfile === 'cpp_linearized_rti_osqp' && (
                          <ConfigField
                            label="freeze_refresh_interval_steps"
                            value={selectedProfileSpecific.freeze_refresh_interval_steps ?? 1}
                            onChange={(v) =>
                              updateSelectedProfileSpecific('freeze_refresh_interval_steps', v)
                            }
                            isNumber
                            step={1}
                            disabled={sharedParametersEnabled}
                          />
                        )}
                        {activeControllerProfile === 'cpp_nonlinear_fullnlp_ipopt' && (
                          <ConfigField
                            label="ipopt_max_iter"
                            value={selectedProfileSpecific.ipopt_max_iter ?? 3000}
                            onChange={(v) => updateSelectedProfileSpecific('ipopt_max_iter', v)}
                            isNumber
                            step={1}
                            disabled={sharedParametersEnabled}
                          />
                        )}
                        {activeControllerProfile === 'cpp_nonlinear_rti_hpipm' && (
                          <>
                            <ConfigField
                              label="acados_max_iter"
                              value={selectedProfileSpecific.acados_max_iter ?? 1}
                              onChange={(v) => updateSelectedProfileSpecific('acados_max_iter', v)}
                              isNumber
                              step={1}
                              disabled={sharedParametersEnabled}
                            />
                            <ConfigField
                              label="acados_tol_stat"
                              value={selectedProfileSpecific.acados_tol_stat ?? 0.01}
                              onChange={(v) => updateSelectedProfileSpecific('acados_tol_stat', v)}
                              isNumber
                              step={0.001}
                              disabled={sharedParametersEnabled}
                            />
                            <ConfigField
                              label="acados_tol_eq"
                              value={selectedProfileSpecific.acados_tol_eq ?? 0.01}
                              onChange={(v) => updateSelectedProfileSpecific('acados_tol_eq', v)}
                              isNumber
                              step={0.001}
                              disabled={sharedParametersEnabled}
                            />
                            <ConfigField
                              label="acados_tol_ineq"
                              value={selectedProfileSpecific.acados_tol_ineq ?? 0.01}
                              onChange={(v) => updateSelectedProfileSpecific('acados_tol_ineq', v)}
                              isNumber
                              step={0.001}
                              disabled={sharedParametersEnabled}
                            />
                          </>
                        )}
                        {activeControllerProfile === 'cpp_nonlinear_sqp_hpipm' && (
                          <>
                            <ConfigField
                              label="acados_max_iter"
                              value={selectedProfileSpecific.acados_max_iter ?? 50}
                              onChange={(v) => updateSelectedProfileSpecific('acados_max_iter', v)}
                              isNumber
                              step={1}
                              disabled={sharedParametersEnabled}
                            />
                            <ConfigField
                              label="acados_tol_stat"
                              value={selectedProfileSpecific.acados_tol_stat ?? 0.01}
                              onChange={(v) => updateSelectedProfileSpecific('acados_tol_stat', v)}
                              isNumber
                              step={0.001}
                              disabled={sharedParametersEnabled}
                            />
                            <ConfigField
                              label="acados_tol_eq"
                              value={selectedProfileSpecific.acados_tol_eq ?? 0.01}
                              onChange={(v) => updateSelectedProfileSpecific('acados_tol_eq', v)}
                              isNumber
                              step={0.001}
                              disabled={sharedParametersEnabled}
                            />
                            <ConfigField
                              label="acados_tol_ineq"
                              value={selectedProfileSpecific.acados_tol_ineq ?? 0.01}
                              onChange={(v) => updateSelectedProfileSpecific('acados_tol_ineq', v)}
                              isNumber
                              step={0.001}
                              disabled={sharedParametersEnabled}
                            />
                          </>
                        )}
                      </div>
                      <div className="mt-4 rounded border border-slate-800 bg-slate-950/60 p-3">
                        <p className="mb-2 text-xs uppercase tracking-wider text-slate-400">
                          Override Diff Preview
                        </p>
                        {selectedOverrideDiff.length === 0 &&
                        Object.keys(selectedProfileSpecific).length === 0 ? (
                          <p className="text-xs text-slate-500">
                            No profile-specific deltas. This profile inherits shared baseline.
                          </p>
                        ) : (
                          <div className="space-y-1 font-mono text-xs text-emerald-300">
                            {selectedOverrideDiff.map(([key, value]) => (
                              <p key={key}>
                                {key}: {String(value)}
                              </p>
                            ))}
                            {Object.entries(selectedProfileSpecific)
                              .sort(([a], [b]) => a.localeCompare(b))
                              .map(([key, value]) => (
                                <p key={key}>profile_specific.{key}: {String(value)}</p>
                              ))}
                          </div>
                        )}
                      </div>
                    </section>

                    <section>
                      <h3 className="mb-4 border-b border-blue-900/30 pb-1 text-sm font-bold uppercase tracking-wider text-blue-400">
                        Basic - Tracking Weights
                      </h3>
                      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
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
                      <h3 className="mb-4 border-b border-slate-800 pb-1 text-sm font-bold uppercase tracking-wider text-slate-500">
                        Basic - Actuation and Path
                      </h3>
                      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
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
                  className="w-full flex items-center justify-between rounded border border-slate-800 bg-slate-900 p-3 transition-colors hover:bg-slate-800"
                >
                  <span className="text-sm font-bold uppercase tracking-wider text-cyan-400">
                    Advanced Settings
                  </span>
                  {showAdvanced ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
                </button>

                {showAdvanced && (
                  <div className="mt-4 grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
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
                  className="w-full flex items-center justify-between rounded border border-slate-800 bg-slate-900 p-3 transition-colors hover:bg-slate-800"
                >
                  <span className="text-sm font-bold uppercase tracking-wider text-orange-400">
                    Expert Settings
                  </span>
                  {showExpert ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
                </button>

                {showExpert && (
                  <div className="mt-4 grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
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
                  className="w-full flex items-center justify-between rounded border border-slate-800 bg-slate-900 p-3 transition-colors hover:bg-slate-800"
                >
                  <span className="text-sm font-bold uppercase tracking-wider text-emerald-400">
                    Settings Reference
                  </span>
                  {showReference ? <ChevronDown size={16} /> : <ChevronRight size={16} />}
                </button>

                {showReference && (
                  <div className="mt-4 space-y-4">
                    {SETTING_REFERENCE_SECTIONS.map((section) => (
                      <div
                        key={section.title}
                        className="rounded border border-slate-800 bg-slate-900/70 p-4"
                      >
                        <h4 className="mb-3 text-xs font-bold uppercase tracking-wider text-slate-400">
                          {section.title}
                        </h4>
                        <div className="grid grid-cols-1 gap-3 lg:grid-cols-2">
                          {section.items.map((item) => (
                            <div
                              key={item.key}
                              className="rounded border border-slate-800 bg-slate-950/60 p-3"
                            >
                              <div className="mb-1 flex items-center justify-between gap-2">
                                <p className="text-sm font-semibold text-slate-200">{item.label}</p>
                                <span className="font-mono text-[10px] text-slate-500">
                                  {item.key}
                                </span>
                              </div>
                              <p className="mb-1 text-xs text-slate-300">{item.description}</p>
                              <p className="text-[11px] text-emerald-300/90">{item.impact}</p>
                            </div>
                          ))}
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </section>
            </>
          ) : (
            <>
              <section className="rounded border border-slate-800 bg-slate-900/60 p-4">
                <div className="flex items-center justify-between gap-3">
                  <div>
                    <h3 className="text-sm uppercase tracking-wider text-cyan-400 font-bold">
                      System Readiness
                    </h3>
                    <p className="mt-1 text-xs text-slate-400">
                      Verifies this machine can run from web interface without rebuild.
                    </p>
                  </div>
                  <button
                    onClick={() => void fetchSystemStatus()}
                    className="rounded bg-slate-800 px-2.5 py-1.5 text-xs text-slate-200 hover:bg-slate-700"
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
                      <span
                        className={
                          systemStatus.ready_for_runner ? 'text-emerald-300' : 'text-amber-300'
                        }
                      >
                        {systemStatus.ready_for_runner ? 'Runner ready' : 'Runner not ready'}
                      </span>
                      <span className="text-slate-500">|</span>
                      <span
                        className={systemStatus.runner_active ? 'text-indigo-300' : 'text-slate-400'}
                      >
                        {systemStatus.runner_active ? 'Runner active' : 'Runner idle'}
                      </span>
                    </div>

                    <div className="grid grid-cols-1 gap-3 text-xs md:grid-cols-2">
                      <div className="rounded border border-slate-800 bg-slate-950/60 p-3">
                        <p className="mb-2 uppercase tracking-wide text-slate-400">Path Checks</p>
                        {Object.entries(systemStatus.checks).map(([name, ok]) => (
                          <div key={name} className="flex items-center justify-between py-0.5">
                            <span className="font-mono text-slate-300">{name}</span>
                            <span className={ok ? 'text-emerald-300' : 'text-red-300'}>
                              {ok ? 'ok' : 'missing'}
                            </span>
                          </div>
                        ))}
                      </div>

                      <div className="rounded border border-slate-800 bg-slate-950/60 p-3">
                        <p className="mb-2 uppercase tracking-wide text-slate-400">Dependencies</p>
                        {Object.entries(systemStatus.dependencies).map(([name, ok]) => (
                          <div key={name} className="flex items-center justify-between py-0.5">
                            <span className="font-mono text-slate-300">{name}</span>
                            <span className={ok ? 'text-emerald-300' : 'text-red-300'}>
                              {ok ? 'ok' : 'missing'}
                            </span>
                          </div>
                        ))}
                      </div>
                    </div>

                    {(systemStatus.missing_checks.length > 0 ||
                      systemStatus.missing_dependencies.length > 0) && (
                      <div className="rounded border border-amber-800/60 bg-amber-900/20 p-3 text-xs">
                        {systemStatus.missing_checks.length > 0 && (
                          <p className="text-amber-200">
                            Missing checks:{' '}
                            <span className="font-mono">
                              {systemStatus.missing_checks.join(', ')}
                            </span>
                          </p>
                        )}
                        {systemStatus.missing_dependencies.length > 0 && (
                          <p className="mt-1 text-amber-200">
                            Missing dependencies:{' '}
                            <span className="font-mono">
                              {systemStatus.missing_dependencies.join(', ')}
                            </span>
                          </p>
                        )}
                      </div>
                    )}

                    {systemStatus.python && (
                      <p className="font-mono text-[11px] text-slate-500">
                        python={systemStatus.python.version ?? 'n/a'} pid=
                        {String(systemStatus.python.pid ?? 'n/a')}
                      </p>
                    )}

                    <div className="border-t border-slate-800 pt-2">
                      <p className="mb-2 text-xs uppercase tracking-wide text-slate-400">
                        Runner Controls
                      </p>
                      <div className="flex flex-wrap items-center gap-2">
                        <input
                          type="text"
                          value={quickMissionName}
                          onChange={(e) => setQuickMissionName(e.target.value)}
                          placeholder="Mission name (optional)"
                          aria-label="Mission name for quick start"
                          className="w-48 rounded border border-slate-700 bg-slate-900 px-2 py-1.5 text-xs text-white focus:border-blue-500 focus:outline-none"
                        />
                        <button
                          onClick={() => void handleQuickRunnerStart()}
                          className="rounded bg-indigo-700 px-2.5 py-1.5 text-xs text-white hover:bg-indigo-600"
                        >
                          Start Runner
                        </button>
                        <button
                          onClick={() => void handleQuickRunnerStop()}
                          className="rounded bg-slate-700 px-2.5 py-1.5 text-xs text-white hover:bg-slate-600"
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
                    <p className="mt-1 text-xs text-slate-400">
                      Runs <span className="font-mono">make package-app</span> from the backend.
                    </p>
                  </div>
                  <div className="flex items-center gap-2">
                    <button
                      onClick={() => void fetchPackageStatus()}
                      className="rounded bg-slate-800 px-2.5 py-1.5 text-xs text-slate-200 hover:bg-slate-700"
                      disabled={packageLoading}
                    >
                      {packageLoading ? 'Refreshing...' : 'Refresh'}
                    </button>
                    <button
                      onClick={() => void handleStartPackaging()}
                      className="rounded bg-emerald-700 px-2.5 py-1.5 text-xs text-white hover:bg-emerald-600 disabled:opacity-50"
                      disabled={packageStarting || Boolean(packageStatus?.running)}
                    >
                      {packageStarting ? 'Starting...' : 'Start Packaging'}
                    </button>
                    <a
                      href={`${RUNNER_API_URL}/package_app/download_latest`}
                      className="rounded bg-cyan-700 px-2.5 py-1.5 text-xs text-white hover:bg-cyan-600"
                      target="_blank"
                      rel="noopener noreferrer"
                    >
                      Download Latest Archive
                    </a>
                    <button
                      onClick={() => {
                        const url = `${RUNNER_API_URL}/workspace/export?include_simulation_data=${
                          includeSimulationDataExport ? 'true' : 'false'
                        }`;
                        window.open(url, '_blank', 'noopener,noreferrer');
                      }}
                      className="rounded bg-blue-700 px-2.5 py-1.5 text-xs text-white hover:bg-blue-600"
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
                        <span className="font-mono text-slate-500">rc={packageStatus.return_code}</span>
                      )}
                    </div>
                    {packageStatus.archive_path && (
                      <p className="text-slate-300">
                        Archive:{' '}
                        <span className="font-mono text-emerald-300">
                          {packageStatus.archive_path}
                        </span>
                      </p>
                    )}
                    {packageStatus.error && (
                      <p className="text-red-300">Error: {packageStatus.error}</p>
                    )}
                    <div className="max-h-40 overflow-y-auto whitespace-pre-wrap rounded border border-slate-800 bg-slate-950/70 p-2 font-mono text-[11px] text-slate-300">
                      {packageStatus.log_lines && packageStatus.log_lines.length > 0
                        ? packageStatus.log_lines.slice(-40).join('\n')
                        : 'No packaging logs yet.'}
                    </div>

                    <div className="border-t border-slate-800 pt-2">
                      <label className="mb-2 flex items-center gap-2 text-[11px] text-slate-300">
                        <input
                          type="checkbox"
                          checked={includeSimulationDataExport}
                          onChange={(e) => setIncludeSimulationDataExport(e.target.checked)}
                        />
                        Include simulation run data in export (can be large)
                      </label>
                      <p className="mb-2 uppercase tracking-wide text-slate-400">
                        Import Workspace
                      </p>
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
                          className="rounded bg-slate-700 px-2.5 py-1.5 text-xs text-white hover:bg-slate-600 disabled:opacity-50"
                          disabled={workspaceInspecting || !workspaceImportFile}
                        >
                          {workspaceInspecting ? 'Inspecting...' : 'Inspect Workspace'}
                        </button>
                        <button
                          onClick={() => void handleImportWorkspace()}
                          className="rounded bg-violet-700 px-2.5 py-1.5 text-xs text-white hover:bg-violet-600 disabled:opacity-50"
                          disabled={workspaceImporting || !workspaceImportFile}
                        >
                          {workspaceImporting ? 'Importing...' : 'Import Workspace'}
                        </button>
                      </div>
                      {workspaceImportFile && (
                        <p className="mt-1 text-[11px] text-slate-500">
                          Selected: {workspaceImportFile.name}
                        </p>
                      )}
                      <div className="mt-2 grid w-full grid-cols-1 gap-2 md:grid-cols-4">
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
                        <div className="mt-2 w-full rounded border border-slate-800 bg-slate-950/70 p-2 text-[11px]">
                          <p className="text-slate-300">
                            Bundle: missions={workspaceInspection.counts.missions_total}, presets=
                            {workspaceInspection.counts.presets_total}, runs=
                            {workspaceInspection.counts.simulation_runs_total}, config=
                            {workspaceInspection.bundle.has_runner_overrides ? 'yes' : 'no'}
                          </p>
                          <p className="mt-1 text-amber-300">
                            Conflicts: missions={workspaceInspection.counts.mission_conflicts},
                            presets={workspaceInspection.counts.preset_conflicts}, runs=
                            {workspaceInspection.counts.simulation_run_conflicts}
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
                                className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-2 py-1 text-[11px] text-slate-100 focus:border-blue-500 focus:outline-none"
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
                              <p className="mt-1 text-[10px] text-slate-500">
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
                                className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-2 py-1 text-[11px] text-slate-100 focus:border-blue-500 focus:outline-none"
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
                              <p className="mt-1 text-[10px] text-slate-500">
                                Selected preset overwrites: {overwritePresetNames.length}
                              </p>
                            </div>
                          )}
                          {workspaceInspection.conflicts.simulation_runs.length > 0 && (
                            <div className="mt-2">
                              <p className="text-slate-400">
                                Simulation run conflicts (
                                {workspaceInspection.conflicts.simulation_runs.length})
                              </p>
                              <input
                                type="text"
                                value={simulationRunConflictFilter}
                                onChange={(e) => setSimulationRunConflictFilter(e.target.value)}
                                placeholder="Filter simulation run conflicts..."
                                className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-2 py-1 text-[11px] text-slate-100 focus:border-blue-500 focus:outline-none"
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
                              <p className="mt-1 text-[10px] text-slate-500">
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
            </>
          )}
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
  disabled?: boolean;
}

function ConfigField({ label, value, onChange, isNumber, desc, step, disabled }: ConfigFieldProps) {
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
        disabled={disabled}
        onChange={(e) => onChange(e.target.value)}
        className={`bg-slate-900 border border-slate-600 rounded px-3 py-2 text-sm text-white focus:outline-none focus:border-blue-500 transition-colors ${
          disabled ? 'opacity-50 cursor-not-allowed' : ''
        }`}
      />
      {desc && <span className="text-[10px] text-slate-400">{desc}</span>}
    </div>
  );
}

interface ToggleFieldProps {
  label: string;
  checked: boolean;
  onChange: (checked: boolean) => void;
  disabled?: boolean;
}

function ToggleField({ label, checked, onChange, disabled }: ToggleFieldProps) {
  const inputId = `toggle-${label.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`;
  return (
    <div
      className={`flex items-center justify-between p-3 bg-slate-900 rounded border border-slate-800 ${
        disabled ? 'opacity-50' : ''
      }`}
    >
      <label htmlFor={inputId} className="text-sm font-medium text-slate-200">
        {label}
      </label>
      <input
        id={inputId}
        aria-label={label}
        type="checkbox"
        checked={checked}
        disabled={disabled}
        onChange={(e) => onChange(e.target.checked)}
        className={`w-5 h-5 rounded border-slate-600 bg-slate-700 text-blue-600 focus:ring-offset-slate-900 ${
          disabled ? 'cursor-not-allowed' : ''
        }`}
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
  disabled?: boolean;
}

function SelectField({ label, value, onChange, options, desc, disabled }: SelectFieldProps) {
  const selectId = `sel-${label.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`;
  return (
    <div className="flex flex-col gap-1">
      <label htmlFor={selectId} className="text-xs font-semibold text-slate-300 uppercase">{label}</label>
      <select
        id={selectId}
        aria-label={label}
        value={value}
        disabled={disabled}
        onChange={(e) => onChange(e.target.value)}
        className={`bg-slate-900 border border-slate-600 rounded px-3 py-2 text-sm text-white focus:outline-none focus:border-blue-500 transition-colors ${
          disabled ? 'opacity-50 cursor-not-allowed' : ''
        }`}
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

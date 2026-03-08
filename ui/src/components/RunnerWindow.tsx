import React, { useEffect, useMemo, useRef, useState } from 'react';
import {
  AlertCircle,
  CheckCircle2,
  Play,
  RefreshCw,
  Square,
  Terminal,
  Trash2,
} from 'lucide-react';
import { MISSIONS_API_URL, RUNNER_API_URL, RUNNER_WS_URL } from '../config/endpoints';
import { useDialog } from '../feedback/feedbackContext';
import { InlineBanner } from './ui-v4/InlineBanner';
import { Panel } from './ui-v4/Panel';
import { StatusPill } from './ui-v4/StatusPill';
import {
  CONTROLLER_PROFILE_IDS,
  CONTROLLER_PROFILE_LABELS,
} from './mpc-settings/mpcSettingsDefaults';
import type {
  ControllerProfileId,
  RunnerSystemStatus,
  SettingsConfig,
} from './mpc-settings/mpcSettingsTypes';
import { normalizeConfig } from './mpc-settings/mpcSettingsUtils';

interface RunnerConfigMeta {
  config_hash: string;
  config_version?: string;
  overrides_active: boolean;
  generated_at: string;
}

interface RunnerConfigResponse {
  config_meta?: RunnerConfigMeta;
}

interface RunnerViewProps {
  hasUnsavedSettings?: boolean;
}

async function parseApiError(res: Response, fallback: string): Promise<string> {
  try {
    const text = await res.text();
    if (!text) return `${fallback} (HTTP ${res.status})`;
    try {
      const json = JSON.parse(text) as Record<string, unknown>;
      const detail = String(json.detail ?? json.message ?? text);
      return `${fallback} (HTTP ${res.status}): ${detail}`;
    } catch {
      return `${fallback} (HTTP ${res.status}): ${text}`;
    }
  } catch {
    return `${fallback} (HTTP ${res.status})`;
  }
}

function isControllerProfileId(value: string): value is ControllerProfileId {
  return (CONTROLLER_PROFILE_IDS as string[]).includes(value);
}

function formatDateTime(value?: string | null): string {
  if (!value) return '--';
  const parsed = new Date(value);
  if (Number.isNaN(parsed.getTime())) return value;
  return parsed.toLocaleString();
}

function classifyLogTone(log: string): string {
  const normalized = log.toLowerCase();
  if (normalized.includes('error') || normalized.includes('failed') || normalized.includes('traceback')) {
    return 'text-red-200';
  }
  if (normalized.includes('warning') || normalized.includes('warn')) {
    return 'text-amber-200';
  }
  if (normalized.includes('simulation finished') || normalized.includes('simulation stopped')) {
    return 'text-emerald-200';
  }
  if (normalized.includes('process started') || normalized.startsWith('>>>')) {
    return 'text-cyan-200';
  }
  return 'text-slate-200';
}

export const RunnerView: React.FC<RunnerViewProps> = ({ hasUnsavedSettings = false }) => {
  const dialog = useDialog();
  const [logs, setLogs] = useState<string[]>([]);
  const [isConnected, setIsConnected] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [isStarting, setIsStarting] = useState(false);
  const [isStopping, setIsStopping] = useState(false);
  const [isRefreshingControlPlane, setIsRefreshingControlPlane] = useState(false);
  const [isUpdatingProfile, setIsUpdatingProfile] = useState(false);
  const [missions, setMissions] = useState<string[]>([]);
  const [selectedMission, setSelectedMission] = useState<string>('');
  const [selectedProfile, setSelectedProfile] = useState<ControllerProfileId>('cpp_hybrid_rti_osqp');
  const [configMeta, setConfigMeta] = useState<RunnerConfigMeta | null>(null);
  const [runnerConfig, setRunnerConfig] = useState<SettingsConfig | null>(null);
  const [systemStatus, setSystemStatus] = useState<RunnerSystemStatus | null>(null);
  const bottomRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [logs]);

  const fetchControlPlane = async () => {
    const configRes = await fetch(`${RUNNER_API_URL}/config`);
    if (!configRes.ok) {
      throw new Error(await parseApiError(configRes, 'Failed to fetch runner config'));
    }
    const configData = (await configRes.json()) as RunnerConfigResponse;
    const normalizedConfig = normalizeConfig(configData);
    if (!normalizedConfig) {
      throw new Error('Runner config returned an unsupported shape');
    }
    setRunnerConfig(normalizedConfig);
    setSelectedProfile(normalizedConfig.mpc_core.controller_profile);
    setConfigMeta(configData.config_meta ?? null);

    const systemRes = await fetch(`${RUNNER_API_URL}/system_status`);
    if (!systemRes.ok) {
      throw new Error(await parseApiError(systemRes, 'Failed to fetch system status'));
    }
    const systemData = (await systemRes.json()) as RunnerSystemStatus;
    setSystemStatus(systemData);
  };

  const fetchMissions = async () => {
    try {
      const res = await fetch(MISSIONS_API_URL);
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to fetch missions'));
      const data = (await res.json()) as { missions?: string[] };
      const missionList = data.missions || [];
      setMissions(missionList);
      if (missionList.length > 0 && !selectedMission) {
        setSelectedMission(missionList[0]);
      }
      if (selectedMission && !missionList.includes(selectedMission)) {
        setSelectedMission(missionList[0] ?? '');
      }
    } catch (err) {
      setLogs((prev) => [...prev, `>>> ${String(err)}\n`]);
    }
  };

  const refreshRunnerState = async () => {
    setIsRefreshingControlPlane(true);
    try {
      await Promise.all([fetchMissions(), fetchControlPlane()]);
    } catch (err) {
      setLogs((prev) => [...prev, `>>> ${String(err)}\n`]);
    } finally {
      setIsRefreshingControlPlane(false);
    }
  };

  useEffect(() => {
    void refreshRunnerState();
  }, []);

  useEffect(() => {
    let socket: WebSocket | null = null;
    let mounted = true;
    const buffer: string[] = [];

    const flushInterval = setInterval(() => {
      if (buffer.length === 0) return;
      const chunk = [...buffer];
      buffer.length = 0;
      setLogs((prev) => [...prev, ...chunk].slice(-2000));
    }, 500);

    const connect = () => {
      socket = new WebSocket(RUNNER_WS_URL);

      socket.onopen = () => {
        if (!mounted) {
          socket?.close();
          return;
        }
        setIsConnected(true);
        buffer.push('>>> Connected to backend\n');
      };

      socket.onclose = () => {
        if (!mounted) return;
        setIsConnected(false);
        setIsRunning(false);
        setIsStarting(false);
        setIsStopping(false);
        buffer.push('>>> Disconnected\n');
      };

      socket.onerror = () => {
        if (!mounted) return;
        buffer.push('>>> WebSocket error\n');
      };

      socket.onmessage = (event) => {
        if (!mounted) return;
        const message = String(event.data);
        buffer.push(message);
        if (message.includes('Process started')) {
          setIsRunning(true);
          setIsStarting(false);
          void fetchControlPlane().catch(() => undefined);
        }
        if (message.includes('Simulation finished') || message.includes('Simulation stopped')) {
          setIsRunning(false);
          setIsStopping(false);
          void fetchControlPlane().catch(() => undefined);
        }
      };
    };

    connect();

    return () => {
      mounted = false;
      clearInterval(flushInterval);
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.close();
      }
    };
  }, []);

  const handleProfileChange = async (nextProfile: ControllerProfileId) => {
    const previous = selectedProfile;
    setSelectedProfile(nextProfile);
    setIsUpdatingProfile(true);
    try {
      const res = await fetch(`${RUNNER_API_URL}/config`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          schema_version: 'app_config_v3',
          app_config: {
            mpc_core: {
              controller_profile: nextProfile,
            },
          },
        }),
      });
      if (!res.ok) {
        throw new Error(await parseApiError(res, 'Failed to update controller profile'));
      }
      setLogs((prev) => [
        ...prev,
        `>>> Active controller profile set to ${CONTROLLER_PROFILE_LABELS[nextProfile]}\n`,
      ]);
      await fetchControlPlane();
    } catch (err) {
      setSelectedProfile(previous);
      setLogs((prev) => [...prev, `>>> ${String(err)}\n`]);
    } finally {
      setIsUpdatingProfile(false);
    }
  };

  const handleStart = async () => {
    if (!selectedMission) {
      setLogs((prev) => [...prev, '>>> Please select a mission first.\n']);
      return;
    }

    if (hasUnsavedSettings) {
      const proceed = await dialog.confirm(
        'You have unsaved Settings changes. Run simulation with last saved settings?'
      );
      if (!proceed) return;
    }

    setIsStarting(true);
    try {
      const res = await fetch(`${RUNNER_API_URL}/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mission_name: selectedMission }),
      });
      if (!res.ok) {
        throw new Error(await parseApiError(res, 'Error starting simulation'));
      }
      setIsRunning(true);
      setLogs((prev) => [
        ...prev,
        `>>> Start requested for mission "${selectedMission}" using ${CONTROLLER_PROFILE_LABELS[selectedProfile]}\n`,
      ]);
      await fetchControlPlane();
    } catch (e) {
      setLogs((prev) => [...prev, `>>> ${String(e)}\n`]);
      setIsStarting(false);
    }
  };

  const handleStop = async () => {
    setIsStopping(true);
    try {
      const res = await fetch(`${RUNNER_API_URL}/stop`, { method: 'POST' });
      if (!res.ok) {
        throw new Error(await parseApiError(res, 'Error stopping simulation'));
      }
      setLogs((prev) => [...prev, '>>> Stop requested\n']);
    } catch (e) {
      setLogs((prev) => [...prev, `>>> ${String(e)}\n`]);
      setIsStopping(false);
    }
  };

  const handleClear = () => {
    setLogs([]);
  };

  const controlsDisabled = isRunning || isStarting || isStopping || isUpdatingProfile;
  const logSummary = useMemo(() => {
    let errorCount = 0;
    let warningCount = 0;
    let lastStatus = 'Idle';
    for (const log of logs) {
      const normalized = log.toLowerCase();
      if (normalized.includes('error') || normalized.includes('failed')) {
        errorCount += 1;
      }
      if (normalized.includes('warning') || normalized.includes('warn')) {
        warningCount += 1;
      }
      if (normalized.includes('process started')) {
        lastStatus = 'Process started';
      } else if (normalized.includes('simulation finished')) {
        lastStatus = 'Simulation finished';
      } else if (normalized.includes('simulation stopped')) {
        lastStatus = 'Simulation stopped';
      } else if (normalized.startsWith('>>> connected')) {
        lastStatus = 'Backend connected';
      }
    }
    return { errorCount, warningCount, lastStatus };
  }, [logs]);

  const activeProfileLabel = CONTROLLER_PROFILE_LABELS[selectedProfile];
  const fairnessMode = runnerConfig?.shared.parameters ?? true;
  const systemReady = systemStatus?.ready_for_runner ?? false;

  return (
    <div className="h-full w-full overflow-hidden bg-[color:var(--v4-bg)] text-[color:var(--v4-text-1)]">
      <div className="flex h-full flex-col gap-4 overflow-hidden p-4">
        {hasUnsavedSettings ? (
          <InlineBanner tone="warning" title="Settings Pending Save" className="shrink-0">
            The active runner still uses the last saved MPC configuration until Settings changes
            are committed.
          </InlineBanner>
        ) : null}

        {!systemReady && systemStatus ? (
          <InlineBanner tone="warning" title="Runner Not Ready" className="shrink-0">
            Missing checks: {systemStatus.missing_checks.join(', ') || 'none'}.
            {systemStatus.missing_dependencies.length > 0
              ? ` Missing dependencies: ${systemStatus.missing_dependencies.join(', ')}.`
              : ''}
          </InlineBanner>
        ) : null}

        <div className="grid shrink-0 gap-4 xl:grid-cols-[minmax(0,1.45fr)_minmax(320px,0.9fr)]">
          <Panel className="min-h-0">
            <div className="grid gap-4 lg:grid-cols-[minmax(0,1fr)_minmax(0,1fr)_auto]">
              <label className="space-y-2">
                <div className="text-[11px] font-semibold uppercase tracking-[0.12em] text-[color:var(--v4-text-3)]">
                  Mission
                </div>
                <select
                  title="Select Mission"
                  aria-label="Select mission"
                  value={selectedMission}
                  onChange={(e) => setSelectedMission(e.target.value)}
                  className="w-full rounded-xl border border-[color:var(--v4-border)] bg-[color:var(--v4-surface-2)] px-3 py-2 text-sm text-[color:var(--v4-text-1)] focus:border-cyan-500 focus:outline-none"
                  disabled={controlsDisabled}
                >
                  <option value="" disabled>
                    Select a mission...
                  </option>
                  {missions.map((mission) => (
                    <option key={mission} value={mission}>
                      {mission}
                    </option>
                  ))}
                </select>
                <div className="text-xs text-[color:var(--v4-text-3)]">
                  Mission selection feeds the headless run only. Playback stays in Viewer.
                </div>
              </label>

              <label className="space-y-2">
                <div className="text-[11px] font-semibold uppercase tracking-[0.12em] text-[color:var(--v4-text-3)]">
                  Controller Profile
                </div>
                <select
                  title="Select Controller Profile"
                  aria-label="Select controller profile"
                  value={selectedProfile}
                  onChange={(e) => {
                    const nextValue = e.target.value;
                    if (isControllerProfileId(nextValue)) {
                      void handleProfileChange(nextValue);
                    }
                  }}
                  className="w-full rounded-xl border border-[color:var(--v4-border)] bg-[color:var(--v4-surface-2)] py-2 pl-3 pr-3 text-sm text-[color:var(--v4-text-1)] focus:border-cyan-500 focus:outline-none"
                  disabled={controlsDisabled}
                >
                  {CONTROLLER_PROFILE_IDS.map((profileId) => (
                    <option key={profileId} value={profileId}>
                      {CONTROLLER_PROFILE_LABELS[profileId]}
                    </option>
                  ))}
                </select>
                <div className="text-xs text-[color:var(--v4-text-3)]">
                  Active solver profile is stored in runner config and reused until you change it.
                </div>
              </label>

              <div className="flex flex-col justify-end gap-2 lg:items-end">
                {!isRunning ? (
                  <button
                    onClick={() => void handleStart()}
                    disabled={!isConnected || !selectedMission || isStarting || !systemReady}
                    aria-label="Run simulation"
                    className={`inline-flex items-center justify-center gap-2 rounded-xl px-5 py-2.5 text-sm font-semibold text-white transition-colors ${
                      isConnected && selectedMission && !isStarting && systemReady
                        ? 'bg-emerald-600 hover:bg-emerald-500'
                        : 'cursor-not-allowed bg-slate-700 text-slate-400'
                    }`}
                  >
                    <Play size={16} fill="currentColor" />
                    {isStarting ? 'Starting...' : 'Run Simulation'}
                  </button>
                ) : (
                  <button
                    onClick={() => void handleStop()}
                    aria-label="Stop simulation"
                    disabled={isStopping}
                    className="inline-flex items-center justify-center gap-2 rounded-xl bg-red-600 px-5 py-2.5 text-sm font-semibold text-white transition-colors hover:bg-red-500 disabled:opacity-60"
                  >
                    <Square size={16} fill="currentColor" />
                    {isStopping ? 'Stopping...' : 'Stop Run'}
                  </button>
                )}
                <button
                  type="button"
                  onClick={() => void refreshRunnerState()}
                  className="inline-flex items-center gap-2 rounded-lg border border-[color:var(--v4-border)] px-3 py-1.5 text-xs text-[color:var(--v4-text-2)] hover:border-cyan-500 hover:text-cyan-200 disabled:opacity-50"
                  disabled={isRefreshingControlPlane}
                >
                  <RefreshCw size={14} className={isRefreshingControlPlane ? 'animate-spin' : ''} />
                  Refresh
                </button>
              </div>
            </div>
          </Panel>

          <Panel
            title="Run Context"
            subtitle="Execution state, config provenance, and tuning metadata for the active headless run."
            className="min-h-0"
          >
            <div className="space-y-4">
              <dl className="grid gap-3 sm:grid-cols-2">
                <div className="rounded-xl border border-[color:var(--v4-border)]/80 bg-[color:var(--v4-surface-2)]/70 p-3">
                  <dt className="text-[10px] uppercase tracking-[0.14em] text-[color:var(--v4-text-3)]">
                    Active Profile
                  </dt>
                  <dd className="mt-1 text-sm font-semibold text-[color:var(--v4-text-1)]">
                    {activeProfileLabel}
                  </dd>
                </div>
                <div className="rounded-xl border border-[color:var(--v4-border)]/80 bg-[color:var(--v4-surface-2)]/70 p-3">
                  <dt className="text-[10px] uppercase tracking-[0.14em] text-[color:var(--v4-text-3)]">
                    Config Hash
                  </dt>
                  <dd className="mt-1 font-mono text-sm text-cyan-200">
                    {configMeta?.config_hash ?? 'unknown'}
                  </dd>
                </div>
                <div className="rounded-xl border border-[color:var(--v4-border)]/80 bg-[color:var(--v4-surface-2)]/70 p-3">
                  <dt className="text-[10px] uppercase tracking-[0.14em] text-[color:var(--v4-text-3)]">
                    Generated
                  </dt>
                  <dd className="mt-1 text-sm text-[color:var(--v4-text-2)]">
                    {formatDateTime(configMeta?.generated_at)}
                  </dd>
                </div>
                <div className="rounded-xl border border-[color:var(--v4-border)]/80 bg-[color:var(--v4-surface-2)]/70 p-3">
                  <dt className="text-[10px] uppercase tracking-[0.14em] text-[color:var(--v4-text-3)]">
                    Parameter Mode
                  </dt>
                  <dd className="mt-1 text-sm text-[color:var(--v4-text-2)]">
                    {fairnessMode ? 'Shared Parameters' : 'Individual Parameters'}
                  </dd>
                </div>
              </dl>
            </div>
          </Panel>
        </div>

        <Panel
          title="Execution Console"
          subtitle="Structured terminal output from the simulation runner. This is the execution surface, not a live viewport."
          actions={
            <div className="flex items-center gap-2">
              <StatusPill tone={logSummary.errorCount > 0 ? 'danger' : 'neutral'}>
                Errors {logSummary.errorCount}
              </StatusPill>
              <StatusPill tone={logSummary.warningCount > 0 ? 'warning' : 'neutral'}>
                Warnings {logSummary.warningCount}
              </StatusPill>
              <button
                onClick={handleClear}
                className="inline-flex items-center gap-2 rounded-lg border border-[color:var(--v4-border)] px-3 py-1.5 text-xs text-[color:var(--v4-text-2)] hover:border-cyan-500 hover:text-cyan-200"
                aria-label="Clear console output"
              >
                <Trash2 size={14} />
                Clear
              </button>
            </div>
          }
          className="min-h-0 flex-1"
          bodyClassName="flex h-full min-h-0 flex-col p-0"
        >
          <div className="flex flex-wrap items-center justify-between gap-3 border-b border-[color:var(--v4-border)]/80 bg-[color:var(--v4-surface-2)]/70 px-4 py-3 text-xs">
            <div className="flex flex-wrap items-center gap-2">
              <StatusPill tone={isConnected ? 'success' : 'danger'}>
                {isConnected ? 'Socket Live' : 'Socket Offline'}
              </StatusPill>
              <StatusPill tone={isConnected ? 'success' : 'danger'}>
                {isConnected ? 'Backend Connected' : 'Backend Offline'}
              </StatusPill>
            </div>
          </div>

          <div
            className="flex-1 overflow-y-auto bg-[#050816] px-4 py-4 font-mono text-sm leading-6"
            role="log"
            aria-live="polite"
          >
            {logs.length === 0 ? (
              <div className="flex h-full flex-col items-center justify-center gap-4 text-center text-slate-400">
                <Terminal size={48} className="opacity-20" />
                <div>
                  <div className="text-sm font-semibold text-slate-200">Headless execution console</div>
                  <div className="mt-1 text-xs text-slate-500">
                    Select a mission, confirm the controller profile, then launch the run. Use
                    Viewer for playback after completion.
                  </div>
                </div>
              </div>
            ) : (
              logs.map((log, index) => (
                <div
                  key={`${index}-${log.slice(0, 24)}`}
                  className={`whitespace-pre-wrap break-words ${classifyLogTone(log)}`}
                >
                  {log}
                </div>
              ))
            )}
            <div ref={bottomRef} />
          </div>

          <div className="flex flex-wrap items-center justify-between gap-3 border-t border-[color:var(--v4-border)]/80 bg-[color:var(--v4-surface-2)]/80 px-4 py-2 text-[11px] uppercase tracking-[0.12em] text-[color:var(--v4-text-3)]">
            <div className="flex items-center gap-2">
              {systemReady ? (
                <CheckCircle2 size={14} className="text-emerald-400" />
              ) : (
                <AlertCircle size={14} className="text-amber-400" />
              )}
              {systemReady ? 'Runner Ready' : 'Runner Needs Attention'}
            </div>
            <div>{isRunning ? 'Run In Progress' : 'Idle'}</div>
          </div>
        </Panel>
      </div>
    </div>
  );
};

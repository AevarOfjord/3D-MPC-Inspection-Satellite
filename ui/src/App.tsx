import { lazy, Suspense, useState, useEffect, useRef, useMemo } from 'react';
import { TelemetryBridge } from './components/TelemetryBridge';
const EventLog = lazy(() =>
  import('./components/EventLog').then((m) => ({ default: m.EventLog }))
);
import { useTelemetryStore } from './store/telemetryStore';
import { useCameraStore } from './store/cameraStore';
const PlaybackSelector = lazy(() =>
  import('./components/PlaybackSelector').then((m) => ({ default: m.PlaybackSelector }))
);
const FocusButton = lazy(() =>
  import('./components/FocusButton').then((m) => ({ default: m.FocusButton }))
);
const RunnerView = lazy(() =>
  import('./components/RunnerWindow').then((m) => ({ default: m.RunnerView }))
);
import { useMissionBuilder } from './hooks/useMissionBuilder';
import { Monitor, Terminal, Database, FileText, Settings, Keyboard } from 'lucide-react';
import { useDialog } from './feedback/feedbackContext';
import { parseStoredAppMode, type AppMode } from './utils/appMode';
import { CommandPalette, type CommandPaletteItem } from './components/CommandPalette';
import { ShortcutHelpPanel } from './components/ShortcutHelpPanel';
const SimulationDataView = lazy(() =>
  import('./components/SimulationDataView').then((m) => ({ default: m.SimulationDataView }))
);
const MPCSettingsView = lazy(() =>
  import('./components/MPCSettingsView').then((m) => ({ default: m.MPCSettingsView }))
);
const ViewerModeView = lazy(() =>
  import('./components/modes/ViewerModeView').then((m) => ({ default: m.ViewerModeView }))
);
const MissionStudioLayout = lazy(() =>
  import('./components/MissionStudio/MissionStudioLayout').then((m) => ({ default: m.MissionStudioLayout }))
);
import { ORBIT_SCALE } from './data/orbitSnapshot';

const APP_MODE_STORAGE_KEY = 'mission_control_app_mode_v1';

function isEditableEventTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName;
  return (
    target.isContentEditable ||
    tag === 'INPUT' ||
    tag === 'TEXTAREA' ||
    tag === 'SELECT'
  );
}

function getInitialAppMode(): AppMode {
  try {
    const raw = window.localStorage.getItem(APP_MODE_STORAGE_KEY);
    const parsed = parseStoredAppMode(raw);
    if (parsed) return parsed;
  } catch {
    // no-op: fallback below
  }
  return 'viewer';
}

const TOOL_META: Record<
  AppMode,
  { label: string; shortLabel: string; icon: typeof Monitor; description: string; themeClass: string; tone: 'info' | 'neutral' | 'warning' | 'success' }
> = {
  viewer: {
    label: 'Viewer',
    shortLabel: 'Playback',
    icon: Monitor,
    description: 'Inspect completed simulation runs and playback results.',
    themeClass: 'border-cyan-500/60 bg-cyan-950/35 text-cyan-100',
    tone: 'info',
  },
  studio: {
    label: 'Studio',
    shortLabel: 'Mission Creator',
    icon: FileText,
    description: 'Create and edit missions before execution.',
    themeClass: 'border-fuchsia-500/60 bg-fuchsia-950/35 text-fuchsia-100',
    tone: 'info',
  },
  runner: {
    label: 'Runner',
    shortLabel: 'Execution',
    icon: Terminal,
    description: 'Launch headless simulations and monitor terminal-style output.',
    themeClass: 'border-indigo-500/60 bg-indigo-950/35 text-indigo-100',
    tone: 'warning',
  },
  data: {
    label: 'Data',
    shortLabel: 'Results',
    icon: Database,
    description: 'Browse exported files, metrics, and artifacts.',
    themeClass: 'border-amber-500/60 bg-amber-950/35 text-amber-100',
    tone: 'warning',
  },
  settings: {
    label: 'Settings',
    shortLabel: 'Controller Tuning',
    icon: Settings,
    description: 'Edit expert MPC and runtime configuration.',
    themeClass: 'border-slate-500/60 bg-slate-900/80 text-slate-100',
    tone: 'neutral',
  },
};

function App() {
  const dialog = useDialog();
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>(() =>
    getInitialAppMode() === 'viewer' ? 'chase' : 'free'
  );
  const [appMode, setAppMode] = useState<AppMode>(() => getInitialAppMode());
  const [settingsDirty, setSettingsDirty] = useState(false);
  const [eventLogOpen, setEventLogOpen] = useState(false);
  const [commandPaletteOpen, setCommandPaletteOpen] = useState(false);
  const [shortcutHelpOpen, setShortcutHelpOpen] = useState(false);
  const eventCount = useTelemetryStore(s => s.events.length);
  const latestTelemetry = useTelemetryStore(s => s.latest);
  const scanFocusRef = useRef<string>('');
  const activeToolMeta = TOOL_META[appMode];

  // Builder Hook (Hoisted State)
  const builder = useMissionBuilder();
  const is3DPrefetchedRef = useRef(false);

  const preload3DModules = () => {
    if (is3DPrefetchedRef.current) return;
    is3DPrefetchedRef.current = true;
    void Promise.all([
      import('./components/MissionStudio/MissionStudioLayout'),
      import('./components/modes/ViewerModeView'),
    ]);
  };

  const ensureCanLeaveSettings = async (): Promise<boolean> => {
    if (appMode === 'settings' && settingsDirty) {
      return dialog.confirm(
        'You have unsaved settings changes. Leave Settings and discard unsaved edits?'
      );
    }
    return true;
  };

  // Mode Switch Handlers
  const switchToViewer = () => {
      preload3DModules();
      void ensureCanLeaveSettings().then((canLeave) => {
        if (!canLeave) return;
        setAppMode('viewer');
        setViewMode('chase'); // Default to chase in viewer
      });
  };

  const switchToStudio = () => {
    preload3DModules();
    void ensureCanLeaveSettings().then((canLeave) => {
      if (!canLeave) return;
      setAppMode('studio');
      setViewMode('free');
    });
  };

  const switchToRunner = () => {
      void ensureCanLeaveSettings().then((canLeave) => {
        if (!canLeave) return;
        setAppMode('runner');
      });
  };

  const switchToDataView = () => {
      void ensureCanLeaveSettings().then((canLeave) => {
        if (!canLeave) return;
        setAppMode('data');
      });
  };

  const switchToSettings = () => {
      setAppMode('settings');
  };

  const commandItems = useMemo<CommandPaletteItem[]>(() => {
    return [
      {
        id: 'mode-viewer',
        label: 'Open Playback Viewer',
        shortcut: 'Ctrl/Cmd+1',
        description: 'Inspect completed simulation runs and playback results.',
        keywords: ['viewer', 'playback', 'results'],
        onSelect: switchToViewer,
      },
      {
        id: 'mode-studio',
        label: 'Open Mission Studio',
        shortcut: 'Ctrl/Cmd+2',
        description: 'Create and edit missions before execution.',
        keywords: ['studio', 'mission', 'author'],
        onSelect: switchToStudio,
      },
      {
        id: 'mode-runner',
        label: 'Open Simulation Runner',
        shortcut: 'Ctrl/Cmd+3',
        description: 'Configure and launch headless simulations.',
        keywords: ['runner', 'simulation', 'execution'],
        onSelect: switchToRunner,
      },
      {
        id: 'mode-data',
        label: 'Open Results Data',
        shortcut: 'Ctrl/Cmd+4',
        description: 'Browse saved runs, exported files, and metrics.',
        keywords: ['data', 'results', 'files'],
        onSelect: switchToDataView,
      },
      {
        id: 'mode-settings',
        label: 'Open Controller Settings',
        shortcut: 'Ctrl/Cmd+5',
        description: 'Tune MPC parameters and expert runtime configuration.',
        keywords: ['settings', 'mpc', 'controller'],
        onSelect: switchToSettings,
      },
    ];
  }, [switchToDataView, switchToStudio, switchToRunner, switchToSettings, switchToViewer]);

  useEffect(() => {
    try {
      window.localStorage.setItem(APP_MODE_STORAGE_KEY, appMode);
    } catch {
      // no-op
    }
  }, [appMode]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      const mod = event.metaKey || event.ctrlKey;

      if (mod && key === 'k') {
        event.preventDefault();
        setShortcutHelpOpen(false);
        setCommandPaletteOpen(true);
        return;
      }

      if (event.key === '?') {
        event.preventDefault();
        setCommandPaletteOpen(false);
        setShortcutHelpOpen(true);
        return;
      }

      if (event.key === 'Escape') {
        if (shortcutHelpOpen) {
          setShortcutHelpOpen(false);
        }
        return;
      }

      if (isEditableEventTarget(event.target)) return;

      if (mod && event.key >= '1' && event.key <= '5') {
        event.preventDefault();
        if (event.key === '1') switchToViewer();
        if (event.key === '2') switchToStudio();
        if (event.key === '3') switchToRunner();
        if (event.key === '4') switchToDataView();
        if (event.key === '5') switchToSettings();
        return;
      }
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [
    appMode,
    shortcutHelpOpen,
    switchToDataView,
    switchToStudio,
    switchToRunner,
    switchToSettings,
    switchToViewer,
  ]);

  useEffect(() => {
    if (appMode !== 'studio') return;
    const selectedObjectId = builder.state.selectedObjectId;
    const editingWaypoint = Boolean(
      selectedObjectId &&
        (selectedObjectId.startsWith('waypoint-') || selectedObjectId.startsWith('spline-'))
    );
    const editingScanControls =
      editingWaypoint ||
      builder.state.centerDragActive ||
      !!builder.state.selectedProjectScanPlaneHandle ||
      !!builder.state.selectedScanCenterHandle ||
      !!builder.state.selectedKeyLevelHandle ||
      !!builder.state.selectedConnectorControl;
    if (editingScanControls) return;
    const path = builder.state.previewPath;
    if (!path || path.length === 0) return;
    let minX = path[0][0];
    let minY = path[0][1];
    let minZ = path[0][2];
    let maxX = path[0][0];
    let maxY = path[0][1];
    let maxZ = path[0][2];
    for (const p of path) {
      minX = Math.min(minX, p[0]);
      minY = Math.min(minY, p[1]);
      minZ = Math.min(minZ, p[2]);
      maxX = Math.max(maxX, p[0]);
      maxY = Math.max(maxY, p[1]);
      maxZ = Math.max(maxZ, p[2]);
    }
    const center: [number, number, number] = [
      (minX + maxX) / 2,
      (minY + maxY) / 2,
      (minZ + maxZ) / 2,
    ];
    const extent = Math.max(maxX - minX, maxY - minY, maxZ - minZ);
    const key = `${path.length}:${center.map(v => v.toFixed(3)).join(',')}:${extent.toFixed(3)}`;
    if (scanFocusRef.current === key) return;
    scanFocusRef.current = key;
    const distance = Math.max(extent * 2.5, 5);
    useCameraStore.getState().requestFocus(
      [center[0] * ORBIT_SCALE, center[1] * ORBIT_SCALE, center[2] * ORBIT_SCALE],
      distance * ORBIT_SCALE
    );
  }, [
    appMode,
    builder.state.previewPath,
    builder.state.selectedObjectId,
    builder.state.centerDragActive,
    builder.state.selectedProjectScanPlaneHandle,
    builder.state.selectedScanCenterHandle,
    builder.state.selectedKeyLevelHandle,
    builder.state.selectedConnectorControl,
  ]);

  return (
    <div className="flex flex-col h-screen bg-[color:var(--v4-bg)] text-[color:var(--v4-text-1)]">
      <TelemetryBridge />

      <header className="flex-none border-b border-[color:var(--v4-border)]/80 select-none z-50 bg-[color:var(--v4-surface-1)]/96 backdrop-blur-xl shadow-[0_18px_48px_rgba(2,6,23,0.35)]">
        <div className="px-5 py-3 flex items-center justify-between gap-4">
          <div className="flex items-center gap-5 min-w-0">
            <div className="flex rounded-2xl border border-[color:var(--v4-border)]/70 bg-[color:var(--v4-surface-2)]/72 p-1 gap-1 shadow-[inset_0_1px_0_rgba(255,255,255,0.04)]">
              {(
                [
                  ['viewer', switchToViewer, preload3DModules],
                  ['studio', switchToStudio, preload3DModules],
                  ['runner', switchToRunner, undefined],
                  ['data', switchToDataView, undefined],
                  ['settings', switchToSettings, undefined],
                ] as const
              ).map(([mode, onClick, prefetch]) => {
                const meta = TOOL_META[mode];
                const Icon = meta.icon;
                const active = appMode === mode;
                return (
                  <button
                    key={mode}
                    onClick={onClick}
                    onMouseEnter={prefetch}
                    onFocus={prefetch}
                    className={`flex items-center gap-2 px-3 py-2 rounded-xl text-xs font-semibold transition-all duration-200 ${
                      active
                        ? `${meta.themeClass} shadow-[0_8px_22px_rgba(2,6,23,0.22)]`
                        : 'text-[color:var(--v4-text-2)] hover:text-white hover:bg-white/5'
                    }`}
                  >
                    <Icon size={14} />
                    {meta.label.toUpperCase()}
                  </button>
                );
              })}
            </div>
          </div>
          <div className="hidden lg:flex min-w-0 flex-1 items-center justify-center px-4">
            <div className="truncate text-center text-[11px] text-[color:var(--v4-text-3)]">
              {activeToolMeta.description}
            </div>
          </div>
          <div className="flex items-center gap-2">
            <button
              type="button"
              onClick={() => {
                setShortcutHelpOpen(false);
                setCommandPaletteOpen(true);
              }}
              className="px-2.5 py-1.5 text-[10px] uppercase rounded-lg border border-[color:var(--v4-border)] text-[color:var(--v4-text-2)] hover:border-cyan-500 hover:text-cyan-200"
            >
              Command Palette <span className="text-slate-500 ml-1">Ctrl/Cmd+K</span>
            </button>
            <button
              type="button"
              onClick={() => {
                setCommandPaletteOpen(false);
                setShortcutHelpOpen(true);
              }}
              className="px-2.5 py-1.5 text-[10px] uppercase rounded-lg border border-[color:var(--v4-border)] text-[color:var(--v4-text-2)] hover:border-cyan-500 hover:text-cyan-200 flex items-center gap-1"
            >
              <Keyboard size={11} />
              Shortcuts
            </button>
          </div>
        </div>

        {appMode === 'viewer' || (appMode === 'settings' && settingsDirty) ? (
          <div className="px-5 py-3 border-t border-[color:var(--v4-border)]/70 bg-[color:var(--v4-surface-2)]/70 flex items-center justify-between gap-4">
            {appMode === 'viewer' ? (
              <div className="flex items-center gap-3">
                <div className="flex rounded-xl border border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)]/65 p-1 gap-1 items-center">
                  <Suspense fallback={null}>
                    <FocusButton />
                  </Suspense>
                  <button
                    onClick={() => setViewMode(viewMode === 'chase' ? 'free' : 'chase')}
                    className={`px-2 py-1 text-[10px] uppercase rounded-lg border transition-colors ${
                      viewMode === 'chase'
                        ? 'border-blue-500 bg-blue-900/30 text-blue-200'
                        : 'border-slate-700 text-slate-300 hover:border-blue-500'
                    }`}
                  >
                    Chase Sat
                  </button>
                  <div className="w-px h-4 bg-slate-700 mx-1" />
                  <button
                    onClick={() => {
                      if (viewMode === 'chase') {
                        useCameraStore.getState().adjustChaseDistance(1.15);
                        return;
                      }
                      useCameraStore.getState().zoomOut();
                    }}
                    className="px-2 py-1 text-[10px] uppercase rounded-lg border border-slate-700 text-slate-300 hover:border-blue-500"
                  >
                    -
                  </button>
                  <button
                    onClick={() => {
                      if (viewMode === 'chase') {
                        useCameraStore.getState().adjustChaseDistance(0.85);
                        return;
                      }
                      useCameraStore.getState().zoomIn();
                    }}
                    className="px-2 py-1 text-[10px] uppercase rounded-lg border border-slate-700 text-slate-300 hover:border-blue-500"
                  >
                    +
                  </button>
                </div>

                <Suspense fallback={null}>
                  <PlaybackSelector />
                </Suspense>

                <div className="relative">
                  <button
                    onClick={() => setEventLogOpen((open) => !open)}
                    className={`px-2 py-1 text-[10px] uppercase rounded-lg border ${
                      eventLogOpen ? 'border-blue-500 text-blue-300' : 'border-slate-700 text-slate-300 hover:border-blue-500'
                    }`}
                  >
                    Event Log
                    {eventCount > 0 && (
                      <span className="ml-2 px-1.5 py-0.5 rounded bg-slate-800 text-[10px] text-slate-300">
                        {eventCount}
                      </span>
                    )}
                  </button>
                  <Suspense fallback={null}>
                    <EventLog open={eventLogOpen} onClose={() => setEventLogOpen(false)} />
                  </Suspense>
                </div>
              </div>
            ) : null}
            {appMode === 'settings' && settingsDirty ? (
              <div className="ml-auto inline-flex items-center rounded-full border border-amber-700/60 bg-amber-950/40 px-2 py-0.5 text-[10px] text-amber-200">
                Unsaved Settings
              </div>
            ) : null}
          </div>
        ) : null}
      </header>

      {/* Main Layout Area */}
      <main className="flex-1 relative flex overflow-hidden bg-[color:var(--v4-bg)]">

        {appMode === 'studio' && (
          <Suspense fallback={<ModeLoading label="Loading Studio..." />}>
            <MissionStudioLayout />
          </Suspense>
        )}
        {appMode === 'runner' && (
            <div className="flex-1 relative bg-[color:var(--v4-bg)]">
                <Suspense fallback={<ModeLoading label="Loading Runner..." />}>
                  <RunnerView hasUnsavedSettings={settingsDirty} />
                </Suspense>
            </div>
        )}

        {appMode === 'data' && (
          <Suspense fallback={<ModeLoading label="Loading Data View..." />}>
            <SimulationDataView />
          </Suspense>
        )}
        {appMode === 'settings' && (
          <Suspense fallback={<ModeLoading label="Loading Settings..." />}>
            <MPCSettingsView onDirtyChange={setSettingsDirty} />
          </Suspense>
        )}

        {appMode === 'viewer' && (
          <Suspense fallback={<ModeLoading label="Loading Viewer..." />}>
            <ViewerModeView viewMode={viewMode} builder={builder} latestTelemetry={latestTelemetry} />
          </Suspense>
        )}
      </main>
      <CommandPalette
        open={commandPaletteOpen}
        onClose={() => setCommandPaletteOpen(false)}
        items={commandItems}
      />
      <ShortcutHelpPanel
        open={shortcutHelpOpen}
        onClose={() => setShortcutHelpOpen(false)}
      />
    </div>
  );
}

function ModeLoading({ label }: { label: string }) {
  return (
    <div className="h-full w-full flex items-center justify-center text-[color:var(--v4-text-3)] text-sm">
      {label}
    </div>
  );
}

export default App;

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
import { Monitor, Terminal, Rocket, Database, FileText, Settings, Keyboard } from 'lucide-react';
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
  return 'studio';
}

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
        label: 'Switch to Viewer',
        shortcut: 'Ctrl/Cmd+1',
        keywords: ['mode', 'viewer'],
        onSelect: switchToViewer,
      },
      {
        id: 'mode-studio',
        label: 'Switch to Mission Studio',
        shortcut: 'Ctrl/Cmd+2',
        keywords: ['mode', 'studio', 'mission'],
        onSelect: switchToStudio,
      },
      {
        id: 'mode-runner',
        label: 'Switch to Runner',
        shortcut: 'Ctrl/Cmd+3',
        keywords: ['mode', 'runner'],
        onSelect: switchToRunner,
      },
      {
        id: 'mode-data',
        label: 'Switch to Data',
        shortcut: 'Ctrl/Cmd+4',
        keywords: ['mode', 'data'],
        onSelect: switchToDataView,
      },
      {
        id: 'mode-settings',
        label: 'Switch to Settings',
        shortcut: 'Ctrl/Cmd+5',
        keywords: ['mode', 'settings'],
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
    <div className="flex flex-col h-screen bg-slate-950 text-slate-200">
      <TelemetryBridge />

      {/* Header */}
      <header className="flex-none glass-panel border-b border-white/10 select-none z-50 shadow-lg">
        <div className="h-14 px-6 flex items-center justify-between">
          <div className="flex items-center gap-8">
            <div className="flex items-center gap-3">
              <div className="p-1.5 bg-gradient-to-br from-cyan-500 to-fuchsia-600 rounded-lg shadow-[0_0_15px_rgba(6,182,212,0.4)]">
                <Rocket className="text-white" size={18} />
              </div>
              <span className="font-bold tracking-widest text-transparent bg-clip-text bg-gradient-to-r from-cyan-100 to-white">ORBITAL INSPECTOR</span>
            </div>

            <div className="flex bg-slate-950/60 rounded-lg p-1 gap-1 border border-white/5 shadow-inner">
              <button
                onClick={switchToViewer}
                onMouseEnter={preload3DModules}
                onFocus={preload3DModules}
                className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-xs font-semibold transition-all duration-300 ${
                  appMode === 'viewer'
                    ? 'bg-cyan-600/90 text-white shadow-[0_0_10px_rgba(6,182,212,0.3)]'
                    : 'text-slate-400 hover:text-white hover:bg-white/5'
                }`}
              >
                <Monitor size={14} />
                VIEWER
              </button>
              <button
                onClick={switchToStudio}
                onMouseEnter={preload3DModules}
                onFocus={preload3DModules}
                className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-xs font-semibold transition-all duration-300 ${
                  appMode === 'studio'
                    ? 'bg-fuchsia-600/90 text-white shadow-[0_0_10px_rgba(217,70,239,0.3)]'
                    : 'text-slate-400 hover:text-white hover:bg-white/5'
                }`}
              >
                <FileText size={14} />
                STUDIO
              </button>
              <button
                onClick={switchToRunner}
                className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-xs font-semibold transition-all duration-300 ${
                  appMode === 'runner'
                    ? 'bg-indigo-600/90 text-white shadow-[0_0_10px_rgba(79,70,229,0.3)]'
                    : 'text-slate-400 hover:text-white hover:bg-white/5'
                }`}
              >
                <Terminal size={14} />
                RUNNER
              </button>
              <button
                onClick={switchToDataView}
                className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-xs font-semibold transition-all duration-300 ${
                  appMode === 'data'
                    ? 'bg-orange-600/90 text-white shadow-[0_0_10px_rgba(234,88,12,0.3)]'
                    : 'text-slate-400 hover:text-white hover:bg-white/5'
                }`}
              >
                <Database size={14} />
                DATA
              </button>
              <button
                onClick={switchToSettings}
                className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-xs font-semibold transition-all duration-300 ${
                  appMode === 'settings'
                    ? 'bg-slate-600/90 text-white shadow-[0_0_10px_rgba(71,85,105,0.3)]'
                    : 'text-slate-400 hover:text-white hover:bg-white/5'
                }`}
              >
                <Settings size={14} />
                SETTINGS
              </button>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <button
              type="button"
              onClick={() => {
                setShortcutHelpOpen(false);
                setCommandPaletteOpen(true);
              }}
              className="px-2 py-1 text-[10px] uppercase rounded border border-slate-700 text-slate-300 hover:border-cyan-500 hover:text-cyan-200"
            >
              Command Palette <span className="text-slate-500 ml-1">Ctrl/Cmd+K</span>
            </button>
            <button
              type="button"
              onClick={() => {
                setCommandPaletteOpen(false);
                setShortcutHelpOpen(true);
              }}
              className="px-2 py-1 text-[10px] uppercase rounded border border-slate-700 text-slate-300 hover:border-cyan-500 hover:text-cyan-200 flex items-center gap-1"
            >
              <Keyboard size={11} />
              Shortcuts
            </button>
          </div>
        </div>

        {appMode === 'viewer' && (
          <div className="h-10 px-4 border-t border-slate-800/80 bg-slate-950/70 flex items-center justify-between">
            <div className="flex items-center gap-3">
              <div className="flex bg-slate-900 rounded p-1 gap-1 items-center border border-slate-800">
                <Suspense fallback={null}>
                  <FocusButton />
                </Suspense>
                <button
                  onClick={() => setViewMode(viewMode === 'chase' ? 'free' : 'chase')}
                  className={`px-2 py-1 text-[10px] uppercase rounded border transition-colors ${
                    viewMode === 'chase'
                      ? 'border-blue-500 bg-blue-900/30 text-blue-200'
                      : 'border-slate-700 text-slate-300 hover:border-blue-500'
                  }`}
                >
                  Chase Sat
                </button>
                <div className="w-px h-4 bg-slate-700 mx-1" />
                <button
                  onClick={() => useCameraStore.getState().zoomOut()}
                  className="px-2 py-1 text-[10px] uppercase rounded border border-slate-700 text-slate-300 hover:border-blue-500"
                >
                  -
                </button>
                <button
                  onClick={() => useCameraStore.getState().zoomIn()}
                  className="px-2 py-1 text-[10px] uppercase rounded border border-slate-700 text-slate-300 hover:border-blue-500"
                >
                  +
                </button>
              </div>

              <Suspense fallback={null}>
                <PlaybackSelector />
              </Suspense>
            </div>

            <div className="relative">
              <button
                onClick={() => setEventLogOpen((open) => !open)}
                className={`px-2 py-1 text-[10px] uppercase rounded border ${
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
        )}
      </header>

      {/* Main Layout Area */}
      <main className="flex-1 relative flex overflow-hidden">

        {appMode === 'studio' && (
          <Suspense fallback={<ModeLoading label="Loading Studio..." />}>
            <MissionStudioLayout />
          </Suspense>
        )}
        {appMode === 'runner' && (
            <div className="flex-1 relative bg-slate-900">
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
    <div className="h-full w-full flex items-center justify-center text-slate-400 text-sm">
      {label}
    </div>
  );
}

export default App;

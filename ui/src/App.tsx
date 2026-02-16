import { lazy, Suspense, useState, useEffect, useRef } from 'react';
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
import { Monitor, Terminal, Rocket, Database, FileText, Target, Settings } from 'lucide-react';
const SimulationDataView = lazy(() =>
  import('./components/SimulationDataView').then((m) => ({ default: m.SimulationDataView }))
);
const MPCSettingsView = lazy(() =>
  import('./components/MPCSettingsView').then((m) => ({ default: m.MPCSettingsView }))
);
const MissionModeView = lazy(() =>
  import('./components/modes/MissionModeView').then((m) => ({ default: m.MissionModeView }))
);
const ScanModeView = lazy(() =>
  import('./components/modes/ScanModeView').then((m) => ({ default: m.ScanModeView }))
);
const ViewerModeView = lazy(() =>
  import('./components/modes/ViewerModeView').then((m) => ({ default: m.ViewerModeView }))
);
import { ORBIT_SCALE } from './data/orbitSnapshot';

type AppMode = 'viewer' | 'mission' | 'scan' | 'runner' | 'data' | 'settings';
const APP_MODE_STORAGE_KEY = 'mission_control_app_mode_v1';

function parseAppMode(value: unknown): AppMode | null {
  if (
    value === 'viewer' ||
    value === 'mission' ||
    value === 'scan' ||
    value === 'runner' ||
    value === 'data' ||
    value === 'settings'
  ) {
    return value;
  }
  return null;
}

function getInitialAppMode(): AppMode {
  try {
    const raw = window.localStorage.getItem(APP_MODE_STORAGE_KEY);
    const parsed = parseAppMode(raw);
    if (parsed) return parsed;
  } catch {
    // no-op: fallback below
  }
  // Default to non-3D startup mode to avoid pulling 3D stack on first load.
  return 'runner';
}

function App() {
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>(() =>
    getInitialAppMode() === 'viewer' ? 'chase' : 'free'
  );
  const [appMode, setAppMode] = useState<AppMode>(() => getInitialAppMode());
  const [settingsDirty, setSettingsDirty] = useState(false);
  const [eventLogOpen, setEventLogOpen] = useState(false);
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
      import('./components/modes/MissionModeView'),
      import('./components/modes/ScanModeView'),
      import('./components/modes/ViewerModeView'),
    ]);
  };

  const ensureCanLeaveSettings = (): boolean => {
    if (appMode === 'settings' && settingsDirty) {
      return window.confirm(
        'You have unsaved settings changes. Leave Settings and discard unsaved edits?'
      );
    }
    return true;
  };

  // Mode Switch Handlers
  const switchToViewer = () => {
      preload3DModules();
      if (!ensureCanLeaveSettings()) return;
      setAppMode('viewer');
      setViewMode('chase'); // Default to chase in viewer
  };

  const switchToMissionPlanner = () => {
      preload3DModules();
      if (!ensureCanLeaveSettings()) return;
      setAppMode('mission');
      setViewMode('free'); // Free cam for planning
  };

  const switchToScanPlanner = () => {
      preload3DModules();
      if (!ensureCanLeaveSettings()) return;
      setAppMode('scan');
      setViewMode('free'); // Free cam for scan planning
  };

  const switchToRunner = () => {
      if (!ensureCanLeaveSettings()) return;
      setAppMode('runner');
  };

  const switchToDataView = () => {
      if (!ensureCanLeaveSettings()) return;
      setAppMode('data');
  };

  const switchToSettings = () => {
      setAppMode('settings');
  };

  useEffect(() => {
    try {
      window.localStorage.setItem(APP_MODE_STORAGE_KEY, appMode);
    } catch {
      // no-op
    }
  }, [appMode]);

  useEffect(() => {
    if (appMode !== 'scan') return;
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
  }, [appMode, builder.state.previewPath]);

  return (
    <div className="flex flex-col h-screen bg-slate-950 text-slate-200">
      <TelemetryBridge />

      {/* Header */}
      <header className="flex-none bg-slate-900 border-b border-slate-800 select-none z-50">
        <div className="h-12 px-4 flex items-center justify-between">
          <div className="flex items-center gap-6">
            <div className="flex items-center gap-2">
              <Rocket className="text-blue-500" size={20} />
              <span className="font-bold tracking-wide text-blue-100">MISSION CONTROL</span>
            </div>

            <div className="flex bg-slate-800 rounded-md p-1 gap-1">
              <button
                onClick={switchToViewer}
                onMouseEnter={preload3DModules}
                onFocus={preload3DModules}
                className={`flex items-center gap-2 px-3 py-1 rounded text-xs font-semibold transition-all ${
                  appMode === 'viewer' 
                    ? 'bg-blue-600 text-white shadow-sm' 
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-700'
                }`}
              >
                <Monitor size={14} />
                VIEWER
              </button>
              <button
                onClick={switchToMissionPlanner}
                onMouseEnter={preload3DModules}
                onFocus={preload3DModules}
                className={`flex items-center gap-2 px-3 py-1 rounded text-xs font-semibold transition-all ${
                  appMode === 'mission' 
                    ? 'bg-purple-600 text-white shadow-sm' 
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-700'
                }`}
              >
                <FileText size={14} />
                MISSION PLANNER
              </button>
              <button
                onClick={switchToScanPlanner}
                onMouseEnter={preload3DModules}
                onFocus={preload3DModules}
                className={`flex items-center gap-2 px-3 py-1 rounded text-xs font-semibold transition-all ${
                  appMode === 'scan' 
                    ? 'bg-emerald-600 text-white shadow-sm' 
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-700'
                }`}
              >
                <Target size={14} />
                SCAN PLANNER
              </button>
              <button
                onClick={switchToRunner}
                className={`flex items-center gap-2 px-3 py-1 rounded text-xs font-semibold transition-all ${
                  appMode === 'runner' 
                    ? 'bg-indigo-600 text-white shadow-sm' 
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-700'
                }`}
              >
                <Terminal size={14} />
                RUNNER
              </button>
              <button
                onClick={switchToDataView}
                className={`flex items-center gap-2 px-3 py-1 rounded text-xs font-semibold transition-all ${
                  appMode === 'data' 
                    ? 'bg-orange-600 text-white shadow-sm' 
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-700'
                }`}
              >
                <Database size={14} />
                DATA
              </button>
              <button
                onClick={switchToSettings}
                className={`flex items-center gap-2 px-3 py-1 rounded text-xs font-semibold transition-all ${
                  appMode === 'settings' 
                    ? 'bg-slate-600 text-white shadow-sm' 
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-700'
                }`}
              >
                <Settings size={14} />
                SETTINGS
              </button>
            </div>
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
        
        {appMode === 'mission' && (
            <Suspense fallback={<ModeLoading label="Loading Mission Planner..." />}>
              <MissionModeView viewMode={viewMode} builder={builder} />
            </Suspense>
        )}

        {appMode === 'scan' && (
            <Suspense fallback={<ModeLoading label="Loading Scan Planner..." />}>
              <ScanModeView viewMode={viewMode} builder={builder} />
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

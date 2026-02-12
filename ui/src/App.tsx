import { useState, useEffect, useRef } from 'react';
import { UnifiedViewport } from './components/UnifiedViewport';
import { Overlay } from './components/Overlay';
import { TelemetryCharts } from './components/TelemetryCharts';
import { TelemetryBridge } from './components/TelemetryBridge';
import { EventLog } from './components/EventLog';
import { useTelemetryStore } from './store/telemetryStore';
import { useCameraStore } from './store/cameraStore';
import { PlaybackSelector } from './components/PlaybackSelector';
import { FocusButton } from './components/FocusButton';
import { RunnerView } from './components/RunnerWindow'; 
import { TrajectoryStudioLayout } from './components/TrajectoryStudio/TrajectoryStudioLayout';
import { useMissionBuilder } from './hooks/useMissionBuilder';
import { Monitor, Calculator, ScanLine, Terminal, Rocket, Database, FileText, Target, Settings } from 'lucide-react';
import { OrbitTargetsPanel } from './components/OrbitTargetsPanel';
import { SimulationDataView } from './components/SimulationDataView';
import { MPCSettingsView } from './components/MPCSettingsView';
import { ORBIT_SCALE, orbitSnapshot } from './data/orbitSnapshot';

function App() {
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>('free');
  const [appMode, setAppMode] = useState<'viewer' | 'mission' | 'scan' | 'runner' | 'data' | 'settings'>('viewer');
  const [eventLogOpen, setEventLogOpen] = useState(false);
  const eventCount = useTelemetryStore(s => s.events.length);
  const latestTelemetry = useTelemetryStore(s => s.latest);
  const scanFocusRef = useRef<string>('');
  
  // Builder Hook (Hoisted State)
  const builder = useMissionBuilder();

  // Mode Switch Handlers
  const switchToViewer = () => {
      setAppMode('viewer');
      setViewMode('chase'); // Default to chase in viewer
  };

  const switchToMissionPlanner = () => {
      setAppMode('mission');
      setViewMode('free'); // Free cam for planning
  };

  const switchToScanPlanner = () => {
      setAppMode('scan');
      setViewMode('free'); // Free cam for scan planning
  };

  const switchToRunner = () => {
      setAppMode('runner');
  };

  const switchToDataView = () => {
      setAppMode('data');
  };

  const switchToSettings = () => {
      setAppMode('settings');
  };

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
      <header className="flex-none h-12 bg-slate-900 border-b border-slate-800 flex items-center px-4 justify-between select-none z-50">
        <div className="flex items-center gap-6">
          <div className="flex items-center gap-2">
            <Rocket className="text-blue-500" size={20} />
            <span className="font-bold tracking-wide text-blue-100">MISSION CONTROL</span>
          </div>
          
          <div className="flex bg-slate-800 rounded-md p-1 gap-1">
            <button
              onClick={switchToViewer}
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

        <div className="flex gap-4 items-center">
             {appMode === 'viewer' && (
                <>
                  {/* Viewer Controls */}
                  <div className="flex bg-slate-900 rounded p-1 gap-1 items-center border border-slate-800">
                      <FocusButton />
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

                  <PlaybackSelector />

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
                    <EventLog open={eventLogOpen} onClose={() => setEventLogOpen(false)} />
                  </div>
                </>
             )}
        </div>
      </header>
      
      {/* Main Layout Area */}
      <main className="flex-1 relative flex overflow-hidden">
        
        {appMode === 'mission' && (
            <TrajectoryStudioLayout 
                builder={builder}
                showPathStudio={false}
                showGeneratorStack={true}
                showTimeline={true}
                showInspector={true}
                viewport={
                    <div className="absolute inset-0 z-0">
                         <UnifiedViewport 
                            mode={appMode} 
                            viewMode={viewMode} 
                            builderState={builder.state}
                            builderActions={builder.actions}
                        />
                        <OrbitTargetsPanel
                            selectedTargetId={builder.state.selectedOrbitTargetId}
                            ownSatellite={{
                              id: 'SATELLITE',
                              name: 'Your Satellite',
                              positionScene: [
                                builder.state.startPosition[0] * ORBIT_SCALE,
                                builder.state.startPosition[1] * ORBIT_SCALE,
                                builder.state.startPosition[2] * ORBIT_SCALE,
                              ],
                              positionMeters: [
                                builder.state.startPosition[0],
                                builder.state.startPosition[1],
                                builder.state.startPosition[2],
                              ],
                            }}
                            solarBodies={[]}
                            onFocusTarget={(targetId, _positionScene, focusDistance) => {
                              // Match UnifiedViewport floating origin when focusing in Mission Planner.
                              const originTargetId = builder.state.selectedOrbitTargetId || builder.state.startTargetId;
                              const originObj = originTargetId
                                ? orbitSnapshot.objects.find(o => o.id === originTargetId)
                                : null;
                              const targetObj = orbitSnapshot.objects.find(o => o.id === targetId);
                              if (!targetObj) return;
                              const origin = originObj?.position_m ?? [0, 0, 0];
                              const scenePos: [number, number, number] = [
                                (targetObj.position_m[0] - origin[0]) * ORBIT_SCALE,
                                (targetObj.position_m[1] - origin[1]) * ORBIT_SCALE,
                                (targetObj.position_m[2] - origin[2]) * ORBIT_SCALE,
                              ];
                              useCameraStore.getState().requestFocus(scenePos, focusDistance);
                            }}
                          />
                    </div>
                }
            />
        )}

        {appMode === 'scan' && (
            <TrajectoryStudioLayout 
                builder={builder}
                showPathStudio={true}
                showGeneratorStack={false}
                showTimeline={false}
                showInspector={false}
                viewport={
                    <div className="absolute inset-0 z-0">
                         <UnifiedViewport 
                            mode={appMode} 
                            viewMode={viewMode} 
                            builderState={builder.state}
                            builderActions={builder.actions}
                        />
                    </div>
                }
            />
        )}

        {appMode === 'runner' && (
            <div className="flex-1 relative bg-slate-900">
                <RunnerView />
            </div>
        )}

        {appMode === 'data' && <SimulationDataView />}
        {appMode === 'settings' && <MPCSettingsView />}

        {appMode === 'viewer' && (
            <div className="flex-1 relative">
                <UnifiedViewport 
                    mode={appMode} 
                    viewMode={viewMode} 
                    builderState={builder.state}
                    builderActions={builder.actions}
                />
                <OrbitTargetsPanel
                    className="fixed right-6 top-1/2 -translate-y-1/2"
                    selectedTargetId={null}
                    ownSatellite={{
                      id: 'SATELLITE',
                      name: 'Your Satellite',
                      positionScene: latestTelemetry
                        ? [
                            latestTelemetry.position[0] * ORBIT_SCALE,
                            latestTelemetry.position[1] * ORBIT_SCALE,
                            latestTelemetry.position[2] * ORBIT_SCALE,
                          ]
                        : [0, 0, 0],
                      positionMeters: latestTelemetry?.position,
                    }}
                    onFocusTarget={(targetId, positionScene, focusDistance) => {
                      useCameraStore.getState().requestFocus(positionScene, focusDistance);
                    }}
                />
                
                {/* Overlay (Viewer Mode Only) */}
                <Overlay />
                <TelemetryCharts />
            </div>
        )}
      </main>
    </div>
  );
}

export default App;

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
import { TrajectoryStudioLayout } from './components/TrajectoryStudio/TrajectoryStudioLayout';
import { useMissionBuilder } from './hooks/useMissionBuilder';
import { Monitor, Calculator, ScanLine } from 'lucide-react';
import { OrbitTargetsPanel } from './components/OrbitTargetsPanel';
import { ORBIT_SCALE } from './data/orbitSnapshot';
import { solarSystemBodies, getSolarBodyPosition, SOLAR_SCALE } from './data/solarSystemSnapshot';

function App() {
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>('free');
  const [appMode, setAppMode] = useState<'viewer' | 'mission' | 'scan'>('viewer');
  const [eventLogOpen, setEventLogOpen] = useState(false);
  const [orbitVisibility, setOrbitVisibility] = useState<Record<string, boolean>>({});
  const eventCount = useTelemetryStore(s => s.events.length);
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

  const toggleOrbitVisibility = (targetId: string) => {
      setOrbitVisibility(prev => ({ ...prev, [targetId]: !prev[targetId] }));
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
    <div className="flex flex-col h-screen w-screen bg-black text-white overflow-hidden">
      <TelemetryBridge />

      {/* Header */}
      <header className="h-14 bg-slate-950/90 border-b border-slate-800 flex justify-between items-center px-4 shrink-0 z-30">
        <div className="flex items-center gap-6">
            <h1 className="text-xl font-bold flex items-center gap-2 tracking-wider text-cyan-400 text-shadow-glow">
              <span>🛰️</span> MISSION CONTROL
            </h1>
            
            {/* Mode Tabs */}
            <div className="flex bg-slate-900 rounded p-1 border border-slate-800">
                <button
                    onClick={switchToViewer}
                    className={`flex items-center gap-2 px-4 py-1.5 rounded text-xs font-bold uppercase transition-all ${appMode === 'viewer' ? 'bg-cyan-500/20 text-cyan-400 shadow-[0_0_10px_rgba(6,182,212,0.2)]' : 'text-slate-500 hover:text-white'}`}
                >
                    <Monitor size={14} /> VIEWER
                </button>
                <button
                    onClick={switchToMissionPlanner}
                    className={`flex items-center gap-2 px-4 py-1.5 rounded text-xs font-bold uppercase transition-all ${appMode === 'mission' ? 'bg-orange-500/20 text-orange-400 shadow-[0_0_10px_rgba(249,115,22,0.2)]' : 'text-slate-500 hover:text-white'}`}
                >
                    <Calculator size={14} /> MISSION PLANNER
                </button>
                <button
                    onClick={switchToScanPlanner}
                    className={`flex items-center gap-2 px-4 py-1.5 rounded text-xs font-bold uppercase transition-all ${appMode === 'scan' ? 'bg-emerald-500/20 text-emerald-400 shadow-[0_0_10px_rgba(16,185,129,0.2)]' : 'text-slate-500 hover:text-white'}`}
                >
                    <ScanLine size={14} /> SCAN PLANNER
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
        
        {appMode === 'mission' ? (
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
                            orbitVisibility={orbitVisibility}
                        />
                        <OrbitTargetsPanel
                            selectedTargetId={builder.state.selectedOrbitTargetId}
                            orbitVisibility={orbitVisibility}
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
                            onSelectTarget={(targetId, positionMeters) => {
                              builder.actions.assignScanTarget(targetId, positionMeters);
                            }}
                            onFocusTarget={(_id, positionScene, focusDistance) => {
                              useCameraStore.getState().requestFocus(positionScene, focusDistance);
                            }}
                            onToggleOrbit={toggleOrbitVisibility}
                          />
                    </div>
                }
            />
        ) : appMode === 'scan' ? (
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
                            orbitVisibility={orbitVisibility}
                        />
                    </div>
                }
            />
        ) : (
            <div className="flex-1 relative">
                <UnifiedViewport 
                    mode={appMode} 
                    viewMode={viewMode} 
                    builderState={builder.state}
                    builderActions={builder.actions}
                    orbitVisibility={orbitVisibility}
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

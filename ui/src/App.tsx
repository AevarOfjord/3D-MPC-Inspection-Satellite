import { useState } from 'react';
import { UnifiedViewport } from './components/UnifiedViewport';
import { Overlay } from './components/Overlay';
import { TelemetryCharts } from './components/TelemetryCharts';
import { TelemetryBridge } from './components/TelemetryBridge';
import { EventLog } from './components/EventLog';
import { useTelemetryStore } from './store/telemetryStore';
import { useCameraStore } from './store/cameraStore';
import { PlaybackSelector } from './components/PlaybackSelector';
import { FocusButton } from './components/FocusButton';
import { BuilderSidebar } from './components/BuilderSidebar';
import { useMissionBuilder } from './hooks/useMissionBuilder';
import { Monitor, Calculator } from 'lucide-react';

function App() {
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>('free');
  const [appMode, setAppMode] = useState<'monitor' | 'plan'>('monitor');
  const [eventLogOpen, setEventLogOpen] = useState(false);
  const eventCount = useTelemetryStore(s => s.events.length);
  
  // Builder Hook (Hoisted State)
  const builder = useMissionBuilder();

  // Mode Switch Handlers
  const switchToMonitor = () => {
      setAppMode('monitor');
      setViewMode('chase'); // Default to chase in monitor
  };

  const switchToPlan = () => {
      setAppMode('plan');
      setViewMode('free'); // Free cam for planning
  };

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
                    onClick={switchToMonitor}
                    className={`flex items-center gap-2 px-4 py-1.5 rounded text-xs font-bold uppercase transition-all ${appMode === 'monitor' ? 'bg-cyan-500/20 text-cyan-400 shadow-[0_0_10px_rgba(6,182,212,0.2)]' : 'text-slate-500 hover:text-white'}`}
                >
                    <Monitor size={14} /> MONITOR
                </button>
                <button
                    onClick={switchToPlan}
                    className={`flex items-center gap-2 px-4 py-1.5 rounded text-xs font-bold uppercase transition-all ${appMode === 'plan' ? 'bg-orange-500/20 text-orange-400 shadow-[0_0_10px_rgba(249,115,22,0.2)]' : 'text-slate-500 hover:text-white'}`}
                >
                    <Calculator size={14} /> PLAN
                </button>
            </div>
        </div>

        <div className="flex gap-4 items-center">
             {/* View Controls (Only useful if not fully managed by modes, but keeping for overrides) */}
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
        </div>
      </header>
      
      {/* Main Layout Area */}
      <main className="flex-1 relative flex overflow-hidden">
        
        {/* Sidebar (Plan Mode Only) */}
        {appMode === 'plan' && (
            <div className="relative z-20 h-full animate-in slide-in-from-left duration-300">
                <BuilderSidebar builder={builder} onExit={switchToMonitor} />
            </div>
        )}

        {/* Viewport Area */}
        <div className="flex-1 relative">
            <UnifiedViewport 
                mode={appMode} 
                viewMode={viewMode} 
                builderState={builder.state}
                builderActions={builder.actions}
            />

            {/* Overlay (Monitor Mode Only) */}
            {appMode === 'monitor' && <Overlay />}
            
            {/* Charts (Always or Monitor Only? Usually Monitor) */}
            {appMode === 'monitor' && <TelemetryCharts />}
        </div>
      </main>
    </div>
  );
}

export default App;

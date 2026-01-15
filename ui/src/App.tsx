import { useEffect, useState } from 'react';
import { Viewport } from './components/Viewport';
import { telemetry } from './services/telemetry';
import { Overlay } from './components/Overlay';
import { Editor } from './components/Editor';
import { TelemetryCharts } from './components/TelemetryCharts';
import { TelemetryBridge } from './components/TelemetryBridge';
import { SystemStatus } from './components/SystemStatus';
import { EventLog } from './components/EventLog';
import { ErrorBanner } from './components/ErrorBanner';
import { useTelemetryStore } from './store/telemetryStore';
import { useCameraStore } from './store/cameraStore';
import { useUiStore } from './store/uiStore';
import { ViewControls } from './components/ViewControls';
import { controlApi } from './api/control';

function App() {
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>('free');
  const [eventLogOpen, setEventLogOpen] = useState(false);
  const latest = useTelemetryStore(s => s.latest);
  const eventCount = useTelemetryStore(s => s.events.length);
  const requestFocus = useCameraStore(s => s.requestFocus);
  const setMissionError = useUiStore(s => s.setMissionError);

  const focusOn = (target?: [number, number, number]) => {
    if (!target) return;
    setViewMode('free');
    requestFocus(target);
  };

  const handleReset = async () => {
    try {
      setMissionError(null);
      await controlApi.reset();
    } catch (error) {
      console.error(error);
      setMissionError('Failed to reset simulation');
    }
  };

  useEffect(() => {
    // Connect to backend
    telemetry.connect();
  }, []);

  return (
    <div className="flex flex-col h-screen w-screen bg-gray-900 text-white">
      <TelemetryBridge />
      <header className="p-4 bg-gray-800 border-b border-gray-700 flex justify-between items-center z-10 relative">
        <h1 className="text-xl font-bold flex items-center gap-2">
          <span>🛰️</span> Mission Control
        </h1>
        <div className="flex gap-4">
             {/* View Mode Controls */}
             <div className="flex bg-gray-900 rounded p-1 gap-1">
                {(['free', 'chase', 'top'] as const).map(mode => (
                  <button
                    key={mode}
                    onClick={() => setViewMode(mode)}
                    className={`px-3 py-1 text-xs rounded uppercase font-bold transition-colors ${
                      viewMode === mode ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white'
                    }`}
                  >
                    {mode}
                  </button>
                ))}
             </div>
             <div className="flex items-center gap-2">
               <button
                 onClick={() => focusOn(latest?.position)}
                 className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
               >
                 Focus Sat
               </button>
               <button
                 onClick={() => focusOn(latest?.target_position)}
                 className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
               >
                 Focus Target
               </button>
               <button
                 onClick={handleReset}
                 className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-red-500"
               >
                 Reset
               </button>
             </div>

             <div className="relative">
               <button
                 onClick={() => setEventLogOpen((open) => !open)}
                 className={`px-2 py-1 text-[10px] uppercase rounded border ${
                   eventLogOpen ? 'border-blue-500 text-blue-300' : 'border-gray-700 text-gray-300 hover:border-blue-500'
                 }`}
               >
                 Event Log
                 {eventCount > 0 && (
                   <span className="ml-2 px-1.5 py-0.5 rounded bg-gray-900 text-[10px] text-gray-300">
                     {eventCount}
                   </span>
                 )}
               </button>
               <EventLog open={eventLogOpen} onClose={() => setEventLogOpen(false)} />
             </div>

             <SystemStatus />
        </div>
      </header>
      <ErrorBanner />
      
      <main className="flex-1 relative overflow-hidden">
        <div className="absolute top-4 right-24 z-20">
          <ViewControls onRequestFree={() => setViewMode('free')} />
        </div>
        <Viewport viewMode={viewMode} />
        <Overlay />
        <TelemetryCharts />
        <Editor />
      </main>
    </div>
  );
}

export default App;

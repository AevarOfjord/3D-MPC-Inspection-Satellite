import { useState } from 'react';
import { Viewport } from './components/Viewport';
import { Overlay } from './components/Overlay';
import { TelemetryCharts } from './components/TelemetryCharts';
import { TelemetryBridge } from './components/TelemetryBridge';
import { EventLog } from './components/EventLog';
import { useTelemetryStore } from './store/telemetryStore';
import { useCameraStore } from './store/cameraStore';
import { PlaybackSelector } from './components/PlaybackSelector';
import { FocusButton } from './components/FocusButton';
import { TrajectoryBuilder } from './pages/TrajectoryBuilder';

function App() {
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>('free');
  const [eventLogOpen, setEventLogOpen] = useState(false);
  const [currentPage, setCurrentPage] = useState<'dashboard' | 'builder'>('dashboard');
  
  // latest removed to prevent re-renders
  const eventCount = useTelemetryStore(s => s.events.length);
  
  if (currentPage === 'builder') {
      return <TrajectoryBuilder onBack={() => setCurrentPage('dashboard')} />;
  }
  
  return (
    <div className="flex flex-col h-screen w-screen bg-gray-900 text-white">
      <TelemetryBridge />
      <header className="p-4 bg-gray-800 border-b border-gray-700 flex justify-between items-center z-10 relative">
        <h1 className="text-xl font-bold flex items-center gap-2">
          <span>🛰️</span> Mission Control
        </h1>
        <div className="flex gap-4 items-center">
             <button
               onClick={() => setCurrentPage('builder')}
               className="px-3 py-1 bg-blue-600 hover:bg-blue-500 rounded text-xs font-bold uppercase transition-colors"
             >
               + Trajectory Builder
             </button>

             <div className="w-px h-6 bg-gray-600 mx-2" />

             {/* Simplified View Controls */}
             <div className="flex bg-gray-900 rounded p-1 gap-1 items-center">
                 <FocusButton />
                 <button
                    onClick={() => setViewMode(viewMode === 'chase' ? 'free' : 'chase')}
                    className={`px-2 py-1 text-[10px] uppercase rounded border transition-colors ${
                      viewMode === 'chase' 
                        ? 'border-blue-500 bg-blue-900/30 text-blue-200' 
                        : 'border-gray-700 text-gray-300 hover:border-blue-500'
                    }`}
                  >
                    Chase Sat
                  </button>
                  <div className="w-px h-4 bg-gray-700 mx-1" />
                  <button
                    onClick={() => useCameraStore.getState().zoomOut()}
                    className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
                    title="Zoom Out"
                  >
                    -
                  </button>
                  <button
                    onClick={() => useCameraStore.getState().zoomIn()}
                    className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
                    title="Zoom In"
                  >
                    +
                  </button>
             </div>

             <PlaybackSelector />

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

        </div>
      </header>
      
      <main className="flex-1 relative overflow-hidden">

        <Viewport viewMode={viewMode} />
        <Overlay />
        <TelemetryCharts />
      </main>
    </div>
  );
}

export default App;

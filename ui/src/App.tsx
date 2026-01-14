import { useEffect, useState } from 'react';
import { Viewport } from './components/Viewport';
import { telemetry } from './services/telemetry';
import { Overlay } from './components/Overlay';
import { Editor } from './components/Editor';
import { TelemetryCharts } from './components/TelemetryCharts';

function App() {
  const [connected] = useState(true); // TODO: implement real connection status
  const [viewMode, setViewMode] = useState<'free' | 'chase' | 'top'>('free');

  useEffect(() => {
    // Connect to backend
    telemetry.connect();
  }, []);

  return (
    <div className="flex flex-col h-screen w-screen bg-gray-900 text-white">
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

             <div className="flex items-center gap-2 border-l border-gray-600 pl-4">
                <div className={`w-3 h-3 rounded-full ${connected ? 'bg-green-500' : 'bg-green-500 animate-pulse'}`}></div>
                <span className="text-sm text-gray-400">System Active</span>
             </div>
        </div>
      </header>
      
      <main className="flex-1 relative overflow-hidden">
        <Viewport viewMode={viewMode} />
        <Overlay />
        <TelemetryCharts />
        <Editor />
      </main>
    </div>
  );
}

export default App;

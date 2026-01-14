import { useEffect, useState } from 'react';
import { Viewport } from './components/Viewport';
import { telemetry } from './services/telemetry';
import { Overlay } from './components/Overlay';
import { Editor } from './components/Editor';

function App() {
  const [connected] = useState(true); // TODO: implement real connection status

  useEffect(() => {
    // Connect to backend
    telemetry.connect();
    
    // Simple poller to check connection status for UI (or expose observable from service)
    // For MVP, just assume connected if no error
  }, []);

  return (
    <div className="flex flex-col h-screen w-screen bg-gray-900 text-white">
      <header className="p-4 bg-gray-800 border-b border-gray-700 flex justify-between items-center z-10 relative">
        <h1 className="text-xl font-bold flex items-center gap-2">
          <span>🛰️</span> Mission Control
        </h1>
        <div className="flex gap-4">
             <div className="flex items-center gap-2">
                <div className={`w-3 h-3 rounded-full ${connected ? 'bg-green-500' : 'bg-green-500 animate-pulse'}`}></div>
                <span className="text-sm text-gray-400">System Active</span>
             </div>
        </div>
      </header>
      
      <main className="flex-1 relative overflow-hidden">
        <Viewport />
        <Overlay />
        <Editor />
      </main>
    </div>
  );
}

export default App;

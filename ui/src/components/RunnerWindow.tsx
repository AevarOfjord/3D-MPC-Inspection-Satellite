import React, { useEffect, useRef, useState } from 'react';
import { Terminal, Play, Square, Trash2, RefreshCw } from 'lucide-react';

const RUNNER_WS_URL = 'ws://localhost:8000/runner/ws';
const RUNNER_API_URL = 'http://localhost:8000/runner';
const MISSIONS_API_URL = 'http://localhost:8000/saved_missions_v2'; 

export const RunnerView: React.FC = () => {
  const [logs, setLogs] = useState<string[]>([]);
  const [isConnected, setIsConnected] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [missions, setMissions] = useState<string[]>([]);
  const [selectedMission, setSelectedMission] = useState<string>('');
  const bottomRef = useRef<HTMLDivElement>(null);
  const ws = useRef<WebSocket | null>(null);

  // Auto-scroll 
  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [logs]);

  // Fetch missions on mount
  useEffect(() => {
    fetch(MISSIONS_API_URL)
        .then(res => res.json())
        .then(data => {
            const missionList = data.missions || [];
            setMissions(missionList);
            if (missionList.length > 0 && !selectedMission) {
                setSelectedMission(missionList[0]);
            }
        })
        .catch(err => console.error("Failed to fetch missions:", err));
  }, []); // Run once on mount


  // Connect WebSocket
  useEffect(() => {
    let socket: WebSocket | null = null;
    let mounted = true;
    const buffer: string[] = [];
    
    // Flush buffer every 500ms to avoid React render thrashing
    const flushInterval = setInterval(() => {
        if (buffer.length > 0) {
            const chunk = [...buffer]; // Capture current buffer
            buffer.length = 0; // Clear buffer immediately
            
            setLogs(prev => {
                const newLogs = [...prev, ...chunk];
                // Limit to last 2000 lines to prevent memory issues
                return newLogs.slice(-2000); 
            });
        }
    }, 500);

    const connect = () => {
      socket = new WebSocket(RUNNER_WS_URL);

      socket.onopen = () => {
        if (!mounted) {
            socket?.close();
            return;
        }
        setIsConnected(true);
        buffer.push('>>> Connected to backend\n');
      };

      socket.onclose = () => {
        if (!mounted) return;
        setIsConnected(false);
        setIsRunning(false);
        buffer.push('>>> Disconnected\n');
        ws.current = null;
      };

      socket.onerror = (error) => {
        if (!mounted) return;
        console.error("Runner WS Error:", error);
      };

      socket.onmessage = (event) => {
        if (!mounted) return;
        buffer.push(event.data);
        if (event.data.includes("Process started")) setIsRunning(true);
        if (event.data.includes("Simulation finished") || event.data.includes("Simulation stopped")) setIsRunning(false);
      };

      ws.current = socket;
    }
    
    connect();

    return () => {
      mounted = false;
      clearInterval(flushInterval);
      if (socket) {
        if (socket.readyState === WebSocket.OPEN) {
             socket.close();
        } else if (socket.readyState === WebSocket.CONNECTING) {
             // Do not close immediately to avoid console error.
        }
      }
      ws.current = null;
    };
  }, []);

  const handleStart = async () => {
    if (!selectedMission) {
        setLogs(prev => [...prev, '>>> Please select a mission first.\n']);
        return;
    }
    try {
      await fetch(`${RUNNER_API_URL}/start`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ mission_name: selectedMission })
      });
      setIsRunning(true); 
    } catch (e) {
      setLogs(prev => [...prev, `>>> Error starting: ${e}\n`]);
    }
  };

  const handleStop = async () => {
    try {
      await fetch(`${RUNNER_API_URL}/stop`, { method: 'POST' });
    } catch (e) {
      setLogs(prev => [...prev, `>>> Error stopping: ${e}\n`]);
    }
  };

  const handleClear = () => {
    setLogs([]);
  };

  return (
    <div className="flex flex-col h-full w-full bg-slate-900 font-mono text-sm">
      {/* Header / Controls */}
      <div className="bg-slate-800 p-4 flex items-center gap-4 border-b border-slate-700">
        <div className="flex items-center gap-2 text-slate-200 font-bold mr-4">
          <Terminal size={18} />
          <span>SIMULATION RUNNER</span>
          <span className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
        </div>

        <div className="flex items-center gap-2 flex-1 max-w-2xl">
            <span className="text-slate-400 text-xs text-nowrap">MISSION:</span>
            <select 
                title="Select Mission"
                value={selectedMission} 
                onChange={(e) => setSelectedMission(e.target.value)}
                className="bg-slate-900 border border-slate-600 text-slate-200 text-sm py-1.5 px-3 rounded flex-1 focus:outline-none focus:border-cyan-500"
                disabled={isRunning}
            >
                <option value="" disabled>Select a Mission...</option>
                {missions.map(m => (
                    <option key={m} value={m}>{m}</option>
                ))}
            </select>
            <button 
                title="Refresh Missions"
                className="text-slate-400 hover:text-white p-2 hover:bg-slate-700 rounded"
                onClick={() => {
                     fetch(MISSIONS_API_URL)
                        .then(res => res.json())
                        .then(data => setMissions(data.missions || []));
                }}
            >
                <RefreshCw size={14} />
            </button>
        </div>

        <div className="h-6 w-px bg-slate-700 mx-2"></div>

        {!isRunning ? (
          <button
            onClick={handleStart}
            disabled={!isConnected || !selectedMission}
            className={`flex items-center gap-2 px-6 py-2 rounded text-white font-bold transition-colors shadow-lg
              ${isConnected && selectedMission ? 'bg-emerald-600 hover:bg-emerald-500 shadow-emerald-900/20' : 'bg-slate-700 text-slate-500 cursor-not-allowed'}
            `}
          >
            <Play size={16} fill="currentColor" />
            <span>RUN SIMULATION</span>
          </button>
        ) : (
          <button
            onClick={handleStop}
            className="flex items-center gap-2 px-6 py-2 rounded bg-red-600 hover:bg-red-500 text-white font-bold transition-colors shadow-lg shadow-red-900/20"
          >
            <Square size={16} fill="currentColor" />
            <span>STOP</span>
          </button>
        )}
        
        <button
          onClick={handleClear}
          className="ml-auto text-slate-400 hover:text-white flex items-center gap-2 px-3 py-1 hover:bg-slate-800 rounded transition-colors"
        >
          <Trash2 size={16} />
          <span>Clear Console</span>
        </button>
      </div>

      {/* Terminal Output */}
      <div className="flex-1 bg-black p-6 overflow-y-auto font-mono text-sm leading-6">
        {logs.length === 0 && (
          <div className="text-slate-500 italic flex flex-col items-center justify-center h-full gap-4">
             <Terminal size={48} className="opacity-20" />
             <p>Select a mission and click 'Run Simulation' to start.</p>
          </div>
        )}
        {logs.map((log, i) => (
          <div key={i} className="whitespace-pre-wrap text-slate-300 break-words">
            {log}
          </div>
        ))}
        <div ref={bottomRef} />
      </div>

      {/* Footer Status */}
      <div className="bg-slate-950 border-t border-slate-800 px-4 py-1.5 flex justify-between text-xs text-slate-500 uppercase tracking-wider font-semibold">
        <span>{isConnected ? 'Connected to backend' : 'Disconnected'}</span>
        <span>{isRunning ? 'Running...' : 'Idle'}</span>
      </div>
    </div>
  );
};

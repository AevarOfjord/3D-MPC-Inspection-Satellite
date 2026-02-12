import React, { useEffect, useRef, useState } from 'react';
import { Terminal, Play, Square, Trash2, RefreshCw } from 'lucide-react';
import { MISSIONS_API_URL, RUNNER_API_URL, RUNNER_WS_URL } from '../config/endpoints';

interface RunnerConfigMeta {
  config_hash: string;
  config_version?: string;
  overrides_active: boolean;
  generated_at: string;
}

interface RunnerConfigResponse {
  config_meta?: RunnerConfigMeta;
}

interface RunnerViewProps {
  hasUnsavedSettings?: boolean;
}

async function parseApiError(res: Response, fallback: string): Promise<string> {
  try {
    const text = await res.text();
    if (!text) return `${fallback} (HTTP ${res.status})`;
    try {
      const json = JSON.parse(text) as Record<string, unknown>;
      const detail = String(json.detail ?? json.message ?? text);
      return `${fallback} (HTTP ${res.status}): ${detail}`;
    } catch {
      return `${fallback} (HTTP ${res.status}): ${text}`;
    }
  } catch {
    return `${fallback} (HTTP ${res.status})`;
  }
}

export const RunnerView: React.FC<RunnerViewProps> = ({ hasUnsavedSettings = false }) => {
  const [logs, setLogs] = useState<string[]>([]);
  const [isConnected, setIsConnected] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [isStarting, setIsStarting] = useState(false);
  const [isStopping, setIsStopping] = useState(false);
  const [missions, setMissions] = useState<string[]>([]);
  const [selectedMission, setSelectedMission] = useState<string>('');
  const [configMeta, setConfigMeta] = useState<RunnerConfigMeta | null>(null);
  const bottomRef = useRef<HTMLDivElement>(null);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [logs]);

  const fetchConfigMeta = async () => {
    try {
      const res = await fetch(`${RUNNER_API_URL}/config`);
      if (!res.ok) return;
      const data = (await res.json()) as RunnerConfigResponse;
      if (data.config_meta) {
        setConfigMeta(data.config_meta);
      }
    } catch {
      // Non-fatal; metadata is secondary UI.
    }
  };

  const fetchMissions = async () => {
    try {
      const res = await fetch(MISSIONS_API_URL);
      if (!res.ok) throw new Error(await parseApiError(res, 'Failed to fetch missions'));
      const data = (await res.json()) as { missions?: string[] };
      const missionList = data.missions || [];
      setMissions(missionList);
      if (missionList.length > 0 && !selectedMission) {
        setSelectedMission(missionList[0]);
      }
    } catch (err) {
      setLogs((prev) => [...prev, `>>> ${String(err)}\n`]);
    }
  };

  useEffect(() => {
    void fetchMissions();
    void fetchConfigMeta();
  }, []);

  useEffect(() => {
    let socket: WebSocket | null = null;
    let mounted = true;
    const buffer: string[] = [];

    const flushInterval = setInterval(() => {
      if (buffer.length > 0) {
        const chunk = [...buffer];
        buffer.length = 0;

        setLogs((prev) => {
          const newLogs = [...prev, ...chunk];
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
        setIsStarting(false);
        setIsStopping(false);
        buffer.push('>>> Disconnected\n');
        ws.current = null;
      };

      socket.onerror = () => {
        if (!mounted) return;
        buffer.push('>>> WebSocket error\n');
      };

      socket.onmessage = (event) => {
        if (!mounted) return;
        const message = String(event.data);
        buffer.push(message);
        if (message.includes('Process started')) {
          setIsRunning(true);
          setIsStarting(false);
        }
        if (message.includes('Simulation finished') || message.includes('Simulation stopped')) {
          setIsRunning(false);
          setIsStopping(false);
          void fetchConfigMeta();
        }
      };

      ws.current = socket;
    };

    connect();

    return () => {
      mounted = false;
      clearInterval(flushInterval);
      if (socket) {
        if (socket.readyState === WebSocket.OPEN) {
          socket.close();
        }
      }
      ws.current = null;
    };
  }, []);

  const handleStart = async () => {
    if (!selectedMission) {
      setLogs((prev) => [...prev, '>>> Please select a mission first.\n']);
      return;
    }

    if (hasUnsavedSettings) {
      const proceed = window.confirm(
        'You have unsaved Settings changes. Run simulation with last saved settings?'
      );
      if (!proceed) return;
    }

    setIsStarting(true);
    try {
      const res = await fetch(`${RUNNER_API_URL}/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mission_name: selectedMission }),
      });
      if (!res.ok) {
        throw new Error(await parseApiError(res, 'Error starting simulation'));
      }
      setIsRunning(true);
      await fetchConfigMeta();
    } catch (e) {
      setLogs((prev) => [...prev, `>>> ${String(e)}\n`]);
      setIsStarting(false);
    }
  };

  const handleStop = async () => {
    setIsStopping(true);
    try {
      const res = await fetch(`${RUNNER_API_URL}/stop`, { method: 'POST' });
      if (!res.ok) {
        throw new Error(await parseApiError(res, 'Error stopping simulation'));
      }
    } catch (e) {
      setLogs((prev) => [...prev, `>>> ${String(e)}\n`]);
      setIsStopping(false);
    }
  };

  const handleClear = () => {
    setLogs([]);
  };

  const controlsDisabled = isRunning || isStarting || isStopping;

  return (
    <div className="flex flex-col h-full w-full bg-slate-900 font-mono text-sm">
      <div className="bg-slate-800 p-4 flex items-center gap-4 border-b border-slate-700">
        <div className="flex items-center gap-2 text-slate-200 font-bold mr-4">
          <Terminal size={18} />
          <span>SIMULATION RUNNER</span>
          <span
            aria-label={isConnected ? 'Backend connected' : 'Backend disconnected'}
            className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`}
          />
        </div>

        <div className="flex items-center gap-2 flex-1 max-w-3xl">
          <span className="text-slate-300 text-xs text-nowrap">MISSION:</span>
          <select
            title="Select Mission"
            aria-label="Select mission"
            value={selectedMission}
            onChange={(e) => setSelectedMission(e.target.value)}
            className="bg-slate-900 border border-slate-600 text-slate-100 text-sm py-1.5 px-3 rounded flex-1 focus:outline-none focus:border-cyan-500"
            disabled={controlsDisabled}
          >
            <option value="" disabled>Select a Mission...</option>
            {missions.map((m) => (
              <option key={m} value={m}>{m}</option>
            ))}
          </select>
          <button
            title="Refresh Missions"
            aria-label="Refresh mission list"
            className="text-slate-300 hover:text-white p-2 hover:bg-slate-700 rounded disabled:opacity-40"
            onClick={() => void fetchMissions()}
            disabled={controlsDisabled}
          >
            <RefreshCw size={14} />
          </button>
        </div>

        <div className="h-6 w-px bg-slate-700 mx-2" />

        {!isRunning ? (
          <button
            onClick={() => void handleStart()}
            disabled={!isConnected || !selectedMission || isStarting}
            aria-label="Run simulation"
            className={`flex items-center gap-2 px-6 py-2 rounded text-white font-bold transition-colors shadow-lg ${
              isConnected && selectedMission && !isStarting
                ? 'bg-emerald-600 hover:bg-emerald-500 shadow-emerald-900/20'
                : 'bg-slate-700 text-slate-400 cursor-not-allowed'
            }`}
          >
            <Play size={16} fill="currentColor" />
            <span>{isStarting ? 'STARTING...' : 'RUN SIMULATION'}</span>
          </button>
        ) : (
          <button
            onClick={() => void handleStop()}
            aria-label="Stop simulation"
            disabled={isStopping}
            className="flex items-center gap-2 px-6 py-2 rounded bg-red-600 hover:bg-red-500 text-white font-bold transition-colors shadow-lg shadow-red-900/20 disabled:opacity-60"
          >
            <Square size={16} fill="currentColor" />
            <span>{isStopping ? 'STOPPING...' : 'STOP'}</span>
          </button>
        )}

        <button
          onClick={handleClear}
          className="ml-auto text-slate-300 hover:text-white flex items-center gap-2 px-3 py-1 hover:bg-slate-800 rounded transition-colors"
          aria-label="Clear console output"
        >
          <Trash2 size={16} />
          <span>Clear Console</span>
        </button>
      </div>

      <div className="bg-slate-900/80 border-b border-slate-800 px-4 py-2 text-xs flex justify-between">
        <span className="text-slate-300">
          Config Hash: <code className="text-cyan-300">{configMeta?.config_hash ?? 'unknown'}</code>
          {configMeta?.config_version && (
            <span className="ml-3 text-slate-400">Version: {configMeta.config_version}</span>
          )}
        </span>
        <span className={configMeta?.overrides_active ? 'text-amber-300' : 'text-emerald-300'}>
          {configMeta?.overrides_active ? 'Overrides Active' : 'Defaults Active'}
        </span>
      </div>

      <div className="flex-1 bg-black p-6 overflow-y-auto font-mono text-sm leading-6" role="log" aria-live="polite">
        {logs.length === 0 && (
          <div className="text-slate-400 italic flex flex-col items-center justify-center h-full gap-4">
            <Terminal size={48} className="opacity-20" />
            <p>Select a mission and click "Run Simulation" to start.</p>
          </div>
        )}
        {logs.map((log, i) => (
          <div key={i} className="whitespace-pre-wrap text-slate-200 break-words">
            {log}
          </div>
        ))}
        <div ref={bottomRef} />
      </div>

      <div className="bg-slate-950 border-t border-slate-800 px-4 py-1.5 flex justify-between text-xs text-slate-400 uppercase tracking-wider font-semibold">
        <span>{isConnected ? 'Connected to backend' : 'Disconnected'}</span>
        <span>{isRunning ? 'Running...' : 'Idle'}</span>
      </div>
    </div>
  );
};

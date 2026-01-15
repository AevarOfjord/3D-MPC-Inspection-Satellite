import { useEffect, useState } from 'react';
import { useTelemetryStore } from '../store/telemetryStore';

export function SystemStatus() {
  const connected = useTelemetryStore(s => s.connected);
  const latest = useTelemetryStore(s => s.latest);
  const lastReceivedAt = useTelemetryStore(s => s.lastReceivedAt);
  const [since, setSince] = useState(0);

  useEffect(() => {
    const interval = window.setInterval(() => {
      if (lastReceivedAt) {
        setSince((Date.now() - lastReceivedAt) / 1000);
      }
    }, 500);
    return () => window.clearInterval(interval);
  }, [lastReceivedAt]);

  const paused = latest?.paused ?? false;
  const simTime = latest?.time ?? 0;

  return (
    <div className="flex items-center gap-3 text-xs text-gray-400">
      <div className="flex items-center gap-2">
        <div className={`w-2.5 h-2.5 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500 animate-pulse'}`} />
        <span>{connected ? 'Telemetry Live' : 'Disconnected'}</span>
      </div>
      <div className="flex items-center gap-2 border-l border-gray-700 pl-3">
        <span className={paused ? 'text-yellow-300' : 'text-green-300'}>
          {paused ? 'Paused' : 'Running'}
        </span>
        <span className="text-gray-500">t={simTime.toFixed(1)}s</span>
      </div>
      <div className="flex items-center gap-2 border-l border-gray-700 pl-3">
        <span>Last update</span>
        <span className="text-blue-300">{since.toFixed(1)}s</span>
      </div>
    </div>
  );
}

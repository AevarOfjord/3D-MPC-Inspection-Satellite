import { useEffect, useState } from 'react';
import { useTelemetryStore } from '../store/telemetryStore';

interface EventLogProps {
  open: boolean;
  onClose: () => void;
}

export function EventLog({ open, onClose }: EventLogProps) {
  const events = useTelemetryStore(s => s.events);
  const clearEvents = useTelemetryStore(s => s.clearEvents);
  const [now, setNow] = useState(Date.now());

  useEffect(() => {
    const interval = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(interval);
  }, []);

  if (!open) return null;

  const recent = events.slice(-6);

  return (
    <div className="absolute top-full right-0 mt-2 w-72 bg-black/80 border border-white/10 rounded-lg backdrop-blur-md p-3 text-xs text-gray-300 z-20">
      <div className="flex items-center justify-between mb-2">
        <span className="uppercase tracking-wider text-[10px] text-gray-400">Event Log</span>
        <div className="flex items-center gap-2">
          <button
            onClick={clearEvents}
            className="text-[10px] uppercase text-gray-500 hover:text-gray-300"
          >
            Clear
          </button>
          <button
            onClick={onClose}
            className="text-[10px] uppercase text-gray-500 hover:text-gray-300"
          >
            Close
          </button>
        </div>
      </div>
      {events.length === 0 ? (
        <div className="text-gray-500 text-[11px] italic">No events yet.</div>
      ) : (
        <div className="space-y-1">
          {recent.map((event) => (
            <div key={event.id} className="flex items-start gap-2">
              <span
                className={`mt-0.5 h-2 w-2 rounded-full ${
                  event.level === 'error'
                    ? 'bg-red-500'
                    : event.level === 'warn'
                    ? 'bg-yellow-500'
                    : 'bg-green-500'
                }`}
              />
              <div className="flex-1">
                <div className="text-gray-200">{event.message}</div>
                <div className="text-[10px] text-gray-500">
                  {((now - event.createdAt) / 1000).toFixed(0)}s ago
                  {typeof event.simTime === 'number' ? ` | t=${event.simTime.toFixed(1)}s` : ''}
                </div>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

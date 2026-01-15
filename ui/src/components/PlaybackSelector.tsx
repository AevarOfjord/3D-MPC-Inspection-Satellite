import { useCallback, useEffect, useRef, useState } from 'react';
import { simulationsApi, type SimulationRun } from '../api/simulations';
import { telemetry, type TelemetryData } from '../services/telemetry';
import { useTelemetryStore } from '../store/telemetryStore';

const MAX_PLAYBACK_SAMPLES = 4000;

export function PlaybackSelector() {
  const [runs, setRuns] = useState<SimulationRun[]>([]);
  const [selectedId, setSelectedId] = useState('');
  const [loading, setLoading] = useState(false);
  const [playing, setPlaying] = useState(false);
  const resetTelemetry = useTelemetryStore(s => s.reset);

  const timerRef = useRef<number | null>(null);
  const dataRef = useRef<TelemetryData[]>([]);
  const indexRef = useRef(0);

  const stopPlayback = useCallback(() => {
    if (timerRef.current !== null) {
      window.clearTimeout(timerRef.current);
      timerRef.current = null;
    }
    setPlaying(false);
  }, []);

  const tick = useCallback(() => {
    const data = dataRef.current;
    const idx = indexRef.current;
    if (!data.length || idx >= data.length) {
      stopPlayback();
      return;
    }

    const current = data[idx];
    telemetry.emit(current);
    indexRef.current = idx + 1;

    if (indexRef.current >= data.length) {
      stopPlayback();
      return;
    }

    const next = data[indexRef.current];
    const dt = Math.max(0.01, next.time - current.time);
    timerRef.current = window.setTimeout(tick, dt * 1000);
  }, [stopPlayback]);

  const startPlayback = useCallback(() => {
    if (!dataRef.current.length) {
      return;
    }
    stopPlayback();
    indexRef.current = 0;
    setPlaying(true);
    tick();
  }, [stopPlayback, tick]);

  useEffect(() => {
    let mounted = true;
    simulationsApi
      .list()
      .then((response) => {
        if (!mounted) return;
        setRuns(response.runs.filter(run => run.has_physics));
      })
      .catch((error) => {
        console.error(error);
      });
    return () => {
      mounted = false;
    };
  }, []);

  useEffect(() => {
    return () => {
      stopPlayback();
    };
  }, [stopPlayback]);

  const handleSelect = async (value: string) => {
    setSelectedId(value);
    stopPlayback();
    resetTelemetry();

    if (!value) {
      telemetry.setManualMode(false);
      return;
    }

    telemetry.setManualMode(true);
    setLoading(true);
    try {
      const run = runs.find(r => r.id === value);
      const steps = run?.steps ?? 0;
      const stride =
        steps > MAX_PLAYBACK_SAMPLES
          ? Math.ceil(steps / MAX_PLAYBACK_SAMPLES)
          : 1;

      const response = await simulationsApi.loadTelemetry(value, stride);
      dataRef.current = response.telemetry;
      startPlayback();
    } catch (error) {
      console.error(error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="flex items-center gap-2">
      <span className="text-[10px] uppercase text-gray-400">Playback</span>
      <select
        className="bg-gray-900 text-gray-200 text-[11px] px-2 py-1 rounded border border-gray-700 focus:outline-none"
        value={selectedId}
        onChange={(event) => handleSelect(event.target.value)}
      >
        <option value="">Select run...</option>
        {runs.map((run) => (
          <option key={run.id} value={run.id}>
            {run.id}
          </option>
        ))}
      </select>
      <button
        type="button"
        onClick={stopPlayback}
        disabled={!playing}
        className={`px-2 py-1 text-[10px] uppercase rounded border ${
          playing
            ? 'border-gray-500 text-gray-200 hover:border-blue-500'
            : 'border-gray-800 text-gray-600'
        }`}
      >
        Stop
      </button>
      {loading && (
        <span className="text-[10px] uppercase text-gray-400">Loading</span>
      )}
    </div>
  );
}

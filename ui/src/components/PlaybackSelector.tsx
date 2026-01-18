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
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [uiIndex, setUiIndex] = useState(0);
  const [duration, setDuration] = useState(0);
  const resetTelemetry = useTelemetryStore(s => s.reset);

  const rafRef = useRef<number | null>(null);
  const startWallRef = useRef(0);
  const startSimRef = useRef(0);
  const lastEmitRef = useRef<number | null>(null);
  const dataRef = useRef<TelemetryData[]>([]);
  const indexRef = useRef(0);

  const stopPlayback = useCallback(() => {
    if (rafRef.current !== null) {
      window.cancelAnimationFrame(rafRef.current);
      rafRef.current = null;
    }
    setPlaying(false);
  }, []);

  const findIndexForTime = (
    data: TelemetryData[],
    startIdx: number,
    targetTime: number
  ) => {
    if (!data.length) return 0;
    if (targetTime <= data[0].time) return 0;

    let lo = Math.max(0, Math.min(startIdx, data.length - 1));
    let hi = data.length - 1;
    if (data[lo].time > targetTime) {
      lo = 0;
    }

    let result = lo;
    while (lo <= hi) {
      const mid = Math.floor((lo + hi) / 2);
      if (data[mid].time <= targetTime) {
        result = mid;
        lo = mid + 1;
      } else {
        hi = mid - 1;
      }
    }
    return result;
  };

  const tick = useCallback((now?: number) => {
    const data = dataRef.current;
    if (!data.length) {
      stopPlayback();
      return;
    }

    const timestamp = typeof now === 'number' ? now : performance.now();
    const elapsed = (timestamp - startWallRef.current) / 1000;
    const targetTime = startSimRef.current + elapsed * playbackSpeed;
    const lastIndex = data.length - 1;

    if (targetTime >= data[lastIndex].time) {
      telemetry.emit(data[lastIndex]);
      setUiIndex(lastIndex);
      stopPlayback();
      return;
    }

    const startIdx = Math.min(indexRef.current, lastIndex);
    const idx = findIndexForTime(data, startIdx, targetTime);

    if (lastEmitRef.current !== idx) {
      telemetry.emit(data[idx]);
      setUiIndex(idx);
      lastEmitRef.current = idx;
    }
    indexRef.current = Math.min(idx + 1, lastIndex);
    rafRef.current = window.requestAnimationFrame(tick);
  }, [playbackSpeed, stopPlayback]);

  const startPlayback = useCallback(() => {
    if (!dataRef.current.length) {
      return;
    }
    stopPlayback();
    if (indexRef.current >= dataRef.current.length) {
      indexRef.current = 0;
      setUiIndex(0);
    }
    lastEmitRef.current = null;
    startSimRef.current = dataRef.current[indexRef.current]?.time ?? 0;
    startWallRef.current = performance.now();
    setPlaying(true);
    telemetry.emit(dataRef.current[indexRef.current]);
    setUiIndex(indexRef.current);
    lastEmitRef.current = indexRef.current;
    rafRef.current = window.requestAnimationFrame(tick);
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
    useTelemetryStore.getState().setPlaybackFinalState(null);

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
      
      // Set final state for visualization
      if (response.telemetry.length > 0) {
        useTelemetryStore.getState().setPlaybackFinalState(response.telemetry[response.telemetry.length - 1]);
      } else {
        useTelemetryStore.getState().setPlaybackFinalState(null);
      }

      indexRef.current = 0;
      setUiIndex(0);
      setDuration(response.telemetry.at(-1)?.time ?? 0);
      if (response.telemetry.length) {
        telemetry.emit(response.telemetry[0]);
        lastEmitRef.current = 0;
      }
    } catch (error) {
      console.error(error);
    } finally {
      setLoading(false);
    }
  };

  const handleReplay = () => {
    if (!dataRef.current.length) return;
    indexRef.current = 0;
    setUiIndex(0);
    startPlayback();
  };

  const handleSliderChange = (value: number) => {
    stopPlayback();
    const data = dataRef.current;
    const idx = Math.min(Math.max(value, 0), Math.max(data.length - 1, 0));
    indexRef.current = idx;
    setUiIndex(idx);
    if (data[idx]) {
      telemetry.emit(data[idx]);
      lastEmitRef.current = idx;
    }
  };

  const currentTime = dataRef.current[uiIndex]?.time ?? 0;

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
      <div className="flex items-center gap-1">
        <button
          type="button"
          onClick={startPlayback}
          disabled={!dataRef.current.length}
          className={`px-2 py-1 text-[10px] uppercase rounded border ${
            dataRef.current.length
              ? 'border-gray-500 text-gray-200 hover:border-blue-500'
              : 'border-gray-800 text-gray-600'
          }`}
        >
          Start
        </button>
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
        <button
          type="button"
          onClick={() => simulationsApi.downloadVideo(selectedId)}
          disabled={!selectedId}
          className={`px-2 py-1 text-[10px] uppercase rounded border ${
            selectedId
              ? 'border-gray-500 text-gray-200 hover:border-blue-500'
              : 'border-gray-800 text-gray-600'
          }`}
        >
          Save
        </button>
        <button
          type="button"
          onClick={handleReplay}
          disabled={!dataRef.current.length}
          className={`px-2 py-1 text-[10px] uppercase rounded border ${
            dataRef.current.length
              ? 'border-gray-500 text-gray-200 hover:border-blue-500'
              : 'border-gray-800 text-gray-600'
          }`}
        >
          Replay
        </button>
      </div>
      <div className="flex items-center gap-2">
        <input
          type="range"
          min={0}
          max={Math.max(dataRef.current.length - 1, 0)}
          value={uiIndex}
          onChange={(event) => handleSliderChange(Number(event.target.value))}
          className="w-32 accent-blue-500"
          disabled={!dataRef.current.length}
        />
        <span className="text-[10px] text-gray-400">
          {currentTime.toFixed(1)}s / {duration.toFixed(1)}s
        </span>
      </div>
      <select
        className="bg-gray-900 text-gray-200 text-[11px] px-2 py-1 rounded border border-gray-700 focus:outline-none"
        value={playbackSpeed}
        onChange={(event) => setPlaybackSpeed(Number(event.target.value))}
      >
        <option value={0.25}>0.25x</option>
        <option value={0.5}>0.5x</option>
        <option value={1}>1x</option>
        <option value={2}>2x</option>
        <option value={5}>5x</option>
        <option value={10}>10x</option>
      </select>
      {loading && (
        <span className="text-[10px] uppercase text-gray-400">Loading</span>
      )}
    </div>
  );
}

import { useCallback, useEffect, useRef, useState } from 'react';
import { RefreshCcw } from 'lucide-react';
import { simulationsApi, type SimulationRun } from '../api/simulations';
import { telemetry, type TelemetryData } from '../services/telemetry';
import { runEvents } from '../services/runEvents';
import { useTelemetryStore } from '../store/telemetryStore';
import { useCameraStore } from '../store/cameraStore';
import { useViewportStore } from '../store/viewportStore';
import { ORBIT_SCALE } from '../data/orbitSnapshot';
import {
  buildRecordingFilename,
  selectRecordingFormat,
  type RecordingFormat,
} from '../utils/viewerRecording';

const MAX_PLAYBACK_SAMPLES = 1000000;
const RUN_LIST_REFRESH_MS = 5000;
const RECORDING_FPS = 30;

export function PlaybackSelector() {
  const [runs, setRuns] = useState<SimulationRun[]>([]);
  const [selectedId, setSelectedId] = useState('');
  const [loading, setLoading] = useState(false);
  const [runsError, setRunsError] = useState<string | null>(null);
  const [refreshingRuns, setRefreshingRuns] = useState(false);
  const [lastRunsRefreshAt, setLastRunsRefreshAt] = useState<number | null>(null);
  const [recordingError, setRecordingError] = useState<string | null>(null);
  const [playing, setPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [uiIndex, setUiIndex] = useState(0);
  const [duration, setDuration] = useState(0);
  const [isRecording, setIsRecording] = useState(false);
  const resetTelemetry = useTelemetryStore((s) => s.reset);
  const canvas = useViewportStore((s) => s.canvas);
  const setViewportRecording = useViewportStore((s) => s.setRecording);
  const effectiveCanvas =
    canvas ??
    (typeof document !== 'undefined'
      ? (document.querySelector('canvas') as HTMLCanvasElement | null)
      : null);

  const rafRef = useRef<number | null>(null);
  const startWallRef = useRef(0);
  const startSimRef = useRef(0);
  const lastEmitRef = useRef<number | null>(null);
  const dataRef = useRef<TelemetryData[]>([]);
  const indexRef = useRef(0);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const recordingFormatRef = useRef<RecordingFormat | null>(null);
  const recordingChunksRef = useRef<BlobPart[]>([]);
  const recordingStopPromiseRef = useRef<Promise<void> | null>(null);
  const recordingObjectUrlRef = useRef<string | null>(null);
  const recordingHasDataRef = useRef(false);

  const canRecord = Boolean(selectedId);

  const cleanupRecordingUrl = useCallback(() => {
    if (recordingObjectUrlRef.current) {
      URL.revokeObjectURL(recordingObjectUrlRef.current);
      recordingObjectUrlRef.current = null;
    }
  }, []);

  const downloadRecording = useCallback(() => {
    const format = recordingFormatRef.current;
    if (!format || !selectedId || !recordingChunksRef.current.length) return;
    const blob = new Blob(recordingChunksRef.current, { type: format.mimeType });
    if (blob.size <= 0) return;
    cleanupRecordingUrl();
    const url = URL.createObjectURL(blob);
    recordingObjectUrlRef.current = url;
    const link = document.createElement('a');
    link.href = url;
    link.download = buildRecordingFilename(selectedId, format.extension);
    document.body.appendChild(link);
    link.click();
    link.remove();
  }, [cleanupRecordingUrl, selectedId]);

  const finalizeRecording = useCallback(async () => {
    const recorder = mediaRecorderRef.current;
    if (!recorder || recorder.state === 'inactive') {
      return;
    }

    const stopPromise =
      recordingStopPromiseRef.current ??
      new Promise<void>((resolve) => {
        const activeRecorder = mediaRecorderRef.current;
        if (!activeRecorder || activeRecorder.state === 'inactive') {
          resolve();
          return;
        }
        activeRecorder.addEventListener('stop', () => resolve(), { once: true });
      });
    recordingStopPromiseRef.current = stopPromise;
    recorder.stop();
    await stopPromise;
    recordingStopPromiseRef.current = null;
  }, []);

  const stopPlayback = useCallback(() => {
    if (rafRef.current !== null) {
      window.cancelAnimationFrame(rafRef.current);
      rafRef.current = null;
    }
    setPlaying(false);
  }, []);

  const applyRunsUpdate = useCallback(
    (incomingRuns: SimulationRun[]) => {
      const nextRuns = incomingRuns.filter((run) => run.has_physics);
      setRuns(nextRuns);
      setLastRunsRefreshAt(Date.now());
      if (selectedId && !nextRuns.some((run) => run.id === selectedId)) {
        void finalizeRecording();
        stopPlayback();
        dataRef.current = [];
        indexRef.current = 0;
        setUiIndex(0);
        setDuration(0);
        resetTelemetry();
        useTelemetryStore.getState().setPlaybackFinalState(null);
        setSelectedId('');
      }
    },
    [finalizeRecording, resetTelemetry, selectedId, stopPlayback]
  );

  const refreshRuns = useCallback(async () => {
    setRefreshingRuns(true);
    try {
      const response = await simulationsApi.list();
      setRunsError(null);
      applyRunsUpdate(response.runs);
    } catch (error) {
      console.error(error);
      setRunsError('Failed to load runs — is the backend running?');
    } finally {
      setRefreshingRuns(false);
    }
  }, [applyRunsUpdate]);

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

  const tick = useCallback(
    (now?: number) => {
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
        if (mediaRecorderRef.current && mediaRecorderRef.current.state !== 'inactive') {
          void finalizeRecording();
        }
        return;
      }

      const startIdx = Math.min(indexRef.current, lastIndex);
      const idx = findIndexForTime(data, startIdx, targetTime);

      if (lastEmitRef.current !== idx) {
        telemetry.emit(data[idx]);
        setUiIndex(idx);
        useTelemetryStore.getState().setPlaybackIndex(idx);
        lastEmitRef.current = idx;
      }
      indexRef.current = Math.min(idx + 1, lastIndex);
      rafRef.current = window.requestAnimationFrame(tick);
    },
    [finalizeRecording, playbackSpeed, stopPlayback]
  );

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

  const startRecording = useCallback(() => {
    setRecordingError(null);
    if (!selectedId) {
      setRecordingError('Load a playback run before recording.');
      return;
    }
    if (!effectiveCanvas) {
      setRecordingError('Viewer canvas is not ready yet.');
      return;
    }
    const resolvedFormat =
      typeof window !== 'undefined' && 'MediaRecorder' in window
        ? selectRecordingFormat(window.MediaRecorder.isTypeSupported?.bind(window.MediaRecorder))
        : null;
    if (!resolvedFormat) {
      setRecordingError('This browser cannot record the viewer playback format.');
      return;
    }

    try {
      cleanupRecordingUrl();
      recordingChunksRef.current = [];
      recordingHasDataRef.current = false;
      recordingFormatRef.current = resolvedFormat;
      const stream = effectiveCanvas.captureStream(RECORDING_FPS);
      const recorder = new MediaRecorder(stream, { mimeType: resolvedFormat.mimeType });
      mediaRecorderRef.current = recorder;
      setIsRecording(true);
      setViewportRecording(true);

      recorder.ondataavailable = (event) => {
        if (event.data && event.data.size > 0) {
          recordingChunksRef.current.push(event.data);
          recordingHasDataRef.current = true;
        }
      };

      recorder.onerror = () => {
        setRecordingError('Viewer recording failed.');
      };

      recorder.onstop = () => {
        setIsRecording(false);
        setViewportRecording(false);
        if (recordingHasDataRef.current) {
          downloadRecording();
        } else {
          setRecordingError('Recording was too short to save.');
        }
        mediaRecorderRef.current = null;
        recordingFormatRef.current = null;
        recordingChunksRef.current = [];
        recordingHasDataRef.current = false;
      };

      recorder.start(250);
    } catch (error) {
      console.error(error);
      mediaRecorderRef.current = null;
      setIsRecording(false);
      setViewportRecording(false);
      setRecordingError('Failed to start viewer recording.');
    }
  }, [cleanupRecordingUrl, downloadRecording, effectiveCanvas, selectedId, setViewportRecording]);

  const handleRecordToggle = useCallback(() => {
    if (isRecording) {
      void finalizeRecording();
      return;
    }
    startRecording();
  }, [finalizeRecording, isRecording, startRecording]);

  useEffect(() => {
    const unsubscribe = runEvents.subscribe((event) => {
      applyRunsUpdate(event.runs);
    });
    return () => unsubscribe();
  }, [applyRunsUpdate]);

  useEffect(() => {
    void refreshRuns();
    const timer = window.setInterval(() => {
      void refreshRuns();
    }, RUN_LIST_REFRESH_MS);
    const onFocus = () => {
      void refreshRuns();
    };
    window.addEventListener('focus', onFocus);
    return () => {
      window.clearInterval(timer);
      window.removeEventListener('focus', onFocus);
    };
  }, [refreshRuns]);

  useEffect(() => {
    return () => {
      stopPlayback();
      setViewportRecording(false);
      void finalizeRecording();
      cleanupRecordingUrl();
    };
  }, [cleanupRecordingUrl, finalizeRecording, setViewportRecording, stopPlayback]);

  const handleSelect = async (value: string) => {
    await finalizeRecording();
    setRecordingError(null);
    setSelectedId(value);
    stopPlayback();
    resetTelemetry();
    useTelemetryStore.getState().setPlaybackFinalState(null);

    if (!value) {
      return;
    }

    setLoading(true);
    try {
      const run = runs.find((r) => r.id === value);
      const steps = run?.steps ?? 0;
      const stride = steps > MAX_PLAYBACK_SAMPLES ? Math.ceil(steps / MAX_PLAYBACK_SAMPLES) : 1;

      const response = await simulationsApi.loadTelemetry(value, stride);
      const plannedPath = response.planned_path;
      const normalized = response.telemetry.map((sample, idx) => {
        const withPath =
          plannedPath && plannedPath.length > 0 && idx === 0
            ? { ...sample, planned_path: plannedPath }
            : sample;
        return telemetry.normalize(withPath);
      });
      dataRef.current = normalized;
      useTelemetryStore.getState().setPlaybackData(normalized);

      if (normalized.length > 0) {
        useTelemetryStore.getState().setPlaybackFinalState(normalized[normalized.length - 1]);
      } else {
        useTelemetryStore.getState().setPlaybackFinalState(null);
      }

      indexRef.current = 0;
      setUiIndex(0);
      useTelemetryStore.getState().setPlaybackIndex(0);
      setDuration(normalized.at(-1)?.time ?? 0);
      if (normalized.length) {
        telemetry.emit(normalized[0]);
        lastEmitRef.current = 0;
        const firstFrame = normalized[0];
        const focusTarget = firstFrame.reference_position ?? firstFrame.position;
        useCameraStore
          .getState()
          .requestFocus(
            [focusTarget[0], focusTarget[1], focusTarget[2]],
            Math.max(8 * ORBIT_SCALE, 4)
          );
      }
    } catch (error) {
      console.error(error);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    if (selectedId || loading || runs.length !== 1) return;
    void handleSelect(runs[0].id);
  }, [loading, runs, selectedId]);

  const handleReplay = useCallback(async () => {
    if (!dataRef.current.length) return;
    await finalizeRecording();
    indexRef.current = 0;
    setUiIndex(0);
    startPlayback();
  }, [finalizeRecording, startPlayback]);

  const handleSliderChange = (value: number) => {
    stopPlayback();
    const data = dataRef.current;
    const idx = Math.min(Math.max(value, 0), Math.max(data.length - 1, 0));
    indexRef.current = idx;
    setUiIndex(idx);
    useTelemetryStore.getState().setPlaybackIndex(idx);
    if (data[idx]) {
      telemetry.emit(data[idx]);
      lastEmitRef.current = idx;
    }
  };

  const currentTime = dataRef.current[uiIndex]?.time ?? 0;
  const lastRefreshLabel = lastRunsRefreshAt
    ? new Date(lastRunsRefreshAt).toLocaleTimeString()
    : '--:--:--';

  return (
    <div className="flex items-center gap-2">
      <span className="text-[10px] uppercase text-gray-400">Playback</span>
      {runsError ? (
        <span className="text-[10px] text-red-400" title={runsError}>
          ⚠ {runsError}
        </span>
      ) : (
        <span className="text-[10px] text-gray-500">
          {refreshingRuns ? 'Refreshing...' : `Updated ${lastRefreshLabel}`}
        </span>
      )}
      <select
        aria-label="Select playback run"
        className="bg-gray-900 text-gray-200 text-[11px] px-2 py-1 rounded border border-gray-700 focus:outline-none"
        value={selectedId}
        onChange={(event) => void handleSelect(event.target.value)}
        onFocus={() => void refreshRuns()}
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
          onClick={() => void refreshRuns()}
          className={`px-2 py-1 text-[10px] uppercase rounded border ${
            refreshingRuns
              ? 'border-blue-500 text-blue-300'
              : 'border-gray-500 text-gray-200 hover:border-blue-500'
          }`}
          title="Refresh run list"
        >
          <RefreshCcw size={12} className={refreshingRuns ? 'animate-spin' : ''} />
        </button>
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
        <div className="flex items-center gap-1">
          {isRecording ? (
            <span
              aria-label="Recording active"
              className="h-2 w-2 rounded-full bg-red-500 animate-pulse"
            />
          ) : null}
          <button
            type="button"
            onClick={handleRecordToggle}
            disabled={!canRecord}
            className={`px-2 py-1 text-[10px] uppercase rounded border ${
              canRecord
                ? isRecording
                  ? 'border-red-500 text-red-300 hover:border-red-400'
                  : 'border-gray-500 text-gray-200 hover:border-blue-500'
                : 'border-gray-800 text-gray-600'
            }`}
            title={
              isRecording
                ? 'Stop recording and save video'
                : 'Record the current viewer playback'
            }
          >
            {isRecording ? 'Save' : 'Rec'}
          </button>
        </div>
        <button
          type="button"
          onClick={() => void handleReplay()}
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
        aria-label="Playback speed"
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
        <option value={20}>20x</option>
        <option value={50}>50x</option>
        <option value={100}>100x</option>
      </select>

      {loading ? <span className="text-[10px] uppercase text-gray-400">Loading</span> : null}
      {recordingError ? (
        <span className="text-[10px] text-amber-400" title={recordingError}>
          {recordingError}
        </span>
      ) : null}
    </div>
  );
}

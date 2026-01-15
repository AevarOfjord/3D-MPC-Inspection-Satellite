import { useEffect, useRef } from 'react';
import { telemetry } from '../services/telemetry';
import { useTelemetryStore } from '../store/telemetryStore';

const POS_OK_THRESHOLD = 0.03; // meters
const ANG_OK_THRESHOLD_DEG = 2.0; // degrees
const SOLVE_WARN_MS = 40;

export function TelemetryBridge() {
  const pushSample = useTelemetryStore(s => s.pushSample);
  const addEvent = useTelemetryStore(s => s.addEvent);

  const lastPathLen = useRef<number>(0);
  const posOk = useRef<boolean>(false);
  const angOk = useRef<boolean>(false);
  const lastSolveWarn = useRef<number>(-Infinity);

  useEffect(() => {
    const unsubData = telemetry.subscribe((data) => {
      pushSample(data);

      const posOkNow = (data.pos_error ?? 0) <= POS_OK_THRESHOLD;
      if (posOkNow && !posOk.current) {
        addEvent('info', 'Position error within target threshold', data.time);
      }
      posOk.current = posOkNow;

      const angErrDeg = (data.ang_error ?? 0) * (180 / Math.PI);
      const angOkNow = angErrDeg <= ANG_OK_THRESHOLD_DEG;
      if (angOkNow && !angOk.current) {
        addEvent('info', 'Attitude error within target threshold', data.time);
      }
      angOk.current = angOkNow;

      const pathLen = data.planned_path?.length ?? 0;
      if (pathLen > 0 && pathLen !== lastPathLen.current) {
        addEvent('info', `Path replanned (${pathLen} waypoints)`, data.time);
      }
      if (pathLen === 0 && lastPathLen.current > 0) {
        addEvent('warn', 'Planner returned no valid path', data.time);
      }
      lastPathLen.current = pathLen;

      const solveMs = (data.solve_time ?? 0) * 1000;
      if (solveMs > SOLVE_WARN_MS && data.time - lastSolveWarn.current > 1.0) {
        addEvent('warn', `MPC solve time high (${solveMs.toFixed(1)} ms)`, data.time);
        lastSolveWarn.current = data.time;
      }
    });

    return () => {
      unsubData();
    };
  }, [addEvent, pushSample]);

  return null;
}

import { useCallback, useRef } from 'react';
import { API_BASE_URL } from '../../config/endpoints';
import { useStudioStore } from './useStudioStore';
import type { ScanPass } from './useStudioStore';

async function fetchScanPath(pass: ScanPass): Promise<[number, number, number][]> {
  const body = {
    axis: pass.axis,
    plane_a: pass.planeAOffset,
    plane_b: pass.planeBOffset,
    level_spacing_m: pass.levelHeight,
    key_levels: pass.keyLevels,
  };
  const res = await fetch(`${API_BASE_URL}/api/models/generate_scan_path`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`generate_scan_path failed: ${res.status}`);
  const data = (await res.json()) as { waypoints: [number, number, number][] };
  return data.waypoints;
}

export function useRegenerateWaypoints() {
  const timerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const regenerate = useCallback((scanId: string, debounceMs = 0) => {
    if (timerRef.current !== null) clearTimeout(timerRef.current);
    timerRef.current = setTimeout(() => {
      const pass = useStudioStore.getState().scanPasses.find((p) => p.id === scanId);
      if (!pass) return;
      void fetchScanPath(pass).then((waypoints) => {
        useStudioStore.getState().setWaypointsFromBackend(scanId, waypoints);
      });
    }, debounceMs);
  }, []);

  return regenerate;
}

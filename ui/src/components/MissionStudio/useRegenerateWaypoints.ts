import { useCallback, useRef } from 'react';
import { API_BASE_URL } from '../../config/endpoints';
import { useStudioStore } from './useStudioStore';
import type { StudioPath } from './useStudioStore';

async function fetchScanPath(path: StudioPath): Promise<[number, number, number][]> {
  const body = {
    axis_seed: path.axisSeed,
    plane_a: path.planeA,
    plane_b: path.planeB,
    ellipse: {
      radius_x: path.ellipse.radiusX,
      radius_y: path.ellipse.radiusY,
    },
    level_spacing_m: path.levelSpacing,
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

  const regenerate = useCallback((pathId: string, debounceMs = 0) => {
    if (timerRef.current !== null) clearTimeout(timerRef.current);
    timerRef.current = setTimeout(() => {
      const path = useStudioStore.getState().paths.find((p) => p.id === pathId);
      if (!path) return;
      void fetchScanPath(path).then((waypoints) => {
        useStudioStore.getState().setWaypointsFromBackend(pathId, waypoints);
      });
    }, debounceMs);
  }, []);

  return regenerate;
}

import { useEffect, type Dispatch, type SetStateAction } from 'react';

import { downsamplePath } from '../utils/pathResample';
import { computePathLength } from '../utils/pathMetrics';

interface UseMissionPathEffectsArgs<SelectionId extends string> {
  previewPath: [number, number, number][];
  editPointLimit: number;
  isManualMode: boolean;
  speedMax: number;
  selectedObjectId: SelectionId | null;
  setPreviewPath: (path: [number, number, number][]) => void;
  setSelectedObjectId: Dispatch<SetStateAction<SelectionId | null>>;
  stats: { duration: number; length: number; points: number } | null;
  setStats: Dispatch<SetStateAction<{ duration: number; length: number; points: number } | null>>;
  removeWaypoint: () => void;
}

export function useMissionPathEffects<SelectionId extends string>({
  previewPath,
  editPointLimit,
  isManualMode,
  speedMax,
  selectedObjectId,
  setPreviewPath,
  setSelectedObjectId,
  stats,
  setStats,
  removeWaypoint,
}: UseMissionPathEffectsArgs<SelectionId>) {
  useEffect(() => {
    if (previewPath.length === 0) return;
    const nextPath = downsamplePath(previewPath, editPointLimit);
    if (nextPath.length === previewPath.length) return;
    setPreviewPath(nextPath);
    if (stats) {
      setStats({ ...stats, points: nextPath.length });
    }
    setSelectedObjectId(null);
  }, [
    previewPath,
    editPointLimit,
    setPreviewPath,
    stats,
    setStats,
    setSelectedObjectId,
  ]);

  useEffect(() => {
    if (!isManualMode) return;
    if (!previewPath || previewPath.length === 0) return;
    const length = computePathLength(previewPath);
    const speed = speedMax > 0 ? speedMax : 0.1;
    setStats({
      duration: speed > 0 ? length / speed : 0,
      length,
      points: previewPath.length,
    });
  }, [isManualMode, previewPath, speedMax, computePathLength, setStats]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== 'Delete' && event.key !== 'Backspace') return;
      const active = document.activeElement;
      if (active && (active.tagName === 'INPUT' || active.tagName === 'TEXTAREA')) return;
      if (selectedObjectId && selectedObjectId.startsWith('waypoint-')) {
        event.preventDefault();
        removeWaypoint();
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [selectedObjectId, previewPath, removeWaypoint]);
}

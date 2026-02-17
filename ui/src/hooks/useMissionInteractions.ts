import type { Dispatch, SetStateAction } from 'react';

export type TransformMode = 'translate' | 'rotate';

interface HistoryAdapter {
  set: (next: [number, number, number][]) => void;
  updatePresent: (next: [number, number, number][]) => void;
}

interface ObstacleState {
  position: [number, number, number];
  radius: number;
}

interface TransformLike {
  position: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number };
}

interface UseMissionInteractionsArgs<SelectionId extends string> {
  obstacles: ObstacleState[];
  setObstacles: Dispatch<SetStateAction<ObstacleState[]>>;
  selectedObjectId: SelectionId | null;
  setSelectedObjectId: Dispatch<SetStateAction<SelectionId | null>>;
  transformMode: TransformMode;
  setStartPosition: Dispatch<SetStateAction<[number, number, number]>>;
  setStartAngle: Dispatch<SetStateAction<[number, number, number]>>;
  setReferencePosition: Dispatch<SetStateAction<[number, number, number]>>;
  setReferenceAngle: Dispatch<SetStateAction<[number, number, number]>>;
  isManualMode: boolean;
  setIsManualMode: Dispatch<SetStateAction<boolean>>;
  previewPath: [number, number, number][];
  pathHistory: HistoryAdapter;
}

export function useMissionInteractions<SelectionId extends string>({
  obstacles,
  setObstacles,
  selectedObjectId,
  setSelectedObjectId,
  transformMode,
  setStartPosition,
  setStartAngle,
  setReferencePosition,
  setReferenceAngle,
  isManualMode,
  setIsManualMode,
  previewPath,
  pathHistory,
}: UseMissionInteractionsArgs<SelectionId>) {
  const addObstacle = (
    origin?: [number, number, number],
    offset: [number, number, number] = [5, 0, 0]
  ) => {
    const base = origin ?? [0, 0, 0];
    const position: [number, number, number] = [
      base[0] + offset[0],
      base[1] + offset[1],
      base[2] + offset[2],
    ];
    setObstacles([...obstacles, { position, radius: 0.5 }]);
  };

  const removeObstacle = (idx: number) => {
    setObstacles(obstacles.filter((_, i) => i !== idx));
    if (selectedObjectId === `obstacle-${idx}`) {
      setSelectedObjectId(null);
    }
  };

  const updateObstacle = (
    idx: number,
    patch: Partial<{ position: [number, number, number]; radius: number }>
  ) => {
    const next = [...obstacles];
    if (!next[idx]) return;
    if (patch.position) next[idx].position = patch.position;
    if (patch.radius !== undefined) next[idx].radius = patch.radius;
    setObstacles(next);
  };

  const handleWaypointMove = (idx: number, newPos: [number, number, number]) => {
    if (!isManualMode) setIsManualMode(true);
    if (idx === 0) return;

    if (!previewPath || previewPath.length === 0) return;
    const current = previewPath[idx];
    if (!current) return;
    const delta: [number, number, number] = [
      newPos[0] - current[0],
      newPos[1] - current[1],
      newPos[2] - current[2],
    ];

    if (
      Math.abs(delta[0]) < 1e-9 &&
      Math.abs(delta[1]) < 1e-9 &&
      Math.abs(delta[2]) < 1e-9
    ) {
      return;
    }

    const n = previewPath.length;
    if (n < 2) {
      const single = [...previewPath];
      single[idx] = newPos;
      pathHistory.updatePresent(single);
      return;
    }

    const arc: number[] = new Array(n).fill(0);
    for (let i = 1; i < n; i++) {
      const a = previewPath[i - 1];
      const b = previewPath[i];
      const dx = b[0] - a[0];
      const dy = b[1] - a[1];
      const dz = b[2] - a[2];
      arc[i] = arc[i - 1] + Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    const totalLength = arc[n - 1];
    const avgSpacing = totalLength / Math.max(1, n - 1);
    const localStart = Math.max(0, idx - 3);
    const localEnd = Math.min(n - 2, idx + 2);
    let localSum = 0;
    let localCount = 0;
    for (let i = localStart; i <= localEnd; i++) {
      const seg = arc[i + 1] - arc[i];
      if (seg > 0) {
        localSum += seg;
        localCount += 1;
      }
    }
    const localSpacing = localCount > 0 ? localSum / localCount : avgSpacing;
    const radius = Math.max(
      (localSpacing || avgSpacing || 1.0) * 6,
      localSpacing || avgSpacing || 1.0
    );
    const s0 = arc[idx];

    const nextPath = previewPath.map((p, i) => {
      if (i === 0) {
        return [p[0], p[1], p[2]] as [number, number, number];
      }
      const d = Math.abs(arc[i] - s0);
      const t = radius > 0 ? Math.max(0, 1 - d / radius) : 0;
      const w = t * t;
      return [
        p[0] + delta[0] * w,
        p[1] + delta[1] * w,
        p[2] + delta[2] * w,
      ] as [number, number, number];
    });

    pathHistory.updatePresent(nextPath);
  };

  const commitWaypointMove = () => {
    pathHistory.set([...previewPath]);
  };

  const addWaypoint = () => {
    if (previewPath.length === 0) {
      pathHistory.set([[0, 0, 0]]);
      setIsManualMode(true);
      return;
    }
    const selectedIdx =
      selectedObjectId && selectedObjectId.startsWith('waypoint-')
        ? Number.parseInt(selectedObjectId.split('-')[1], 10)
        : null;
    let insertIndex = previewPath.length;
    let newPoint = previewPath[previewPath.length - 1];
    if (typeof selectedIdx === 'number' && !Number.isNaN(selectedIdx)) {
      if (selectedIdx >= 0 && selectedIdx < previewPath.length - 1) {
        const p0 = previewPath[selectedIdx];
        const p1 = previewPath[selectedIdx + 1];
        newPoint = [
          (p0[0] + p1[0]) / 2,
          (p0[1] + p1[1]) / 2,
          (p0[2] + p1[2]) / 2,
        ];
        insertIndex = selectedIdx + 1;
      } else if (selectedIdx >= 0 && selectedIdx < previewPath.length) {
        newPoint = previewPath[selectedIdx];
        insertIndex = selectedIdx + 1;
      }
    }

    const nextPath = [...previewPath];
    nextPath.splice(insertIndex, 0, newPoint);
    pathHistory.set(nextPath);
    setIsManualMode(true);
  };

  const removeWaypoint = () => {
    if (previewPath.length <= 2) return;
    const selectedIdx =
      selectedObjectId && selectedObjectId.startsWith('waypoint-')
        ? Number.parseInt(selectedObjectId.split('-')[1], 10)
        : null;
    if (typeof selectedIdx !== 'number' || Number.isNaN(selectedIdx)) return;
    if (selectedIdx === 0) return;
    if (selectedIdx < 0 || selectedIdx >= previewPath.length) return;
    const nextPath = previewPath.filter((_, i) => i !== selectedIdx);
    pathHistory.set(nextPath);
    setIsManualMode(true);
    setSelectedObjectId(null);
  };

  const removeWaypointAtIndex = (idx: number) => {
    if (previewPath.length <= 2) return;
    if (!Number.isFinite(idx) || idx < 0 || idx >= previewPath.length) return;
    if (idx === 0) return;
    const nextPath = previewPath.filter((_, i) => i !== idx);
    pathHistory.set(nextPath);
    setIsManualMode(true);
    if (selectedObjectId === `waypoint-${idx}`) {
      setSelectedObjectId(null);
    }
  };

  const handleObjectTransform = (key: string, object: TransformLike) => {
    const pos: [number, number, number] = [
      object.position.x,
      object.position.y,
      object.position.z,
    ];
    const rot: [number, number, number] = [
      object.rotation.x * (180 / Math.PI),
      object.rotation.y * (180 / Math.PI),
      object.rotation.z * (180 / Math.PI),
    ];

    if (key === 'satellite') {
      if (transformMode === 'translate') setStartPosition(pos);
      else setStartAngle(rot);
      return;
    }

    if (key === 'reference') {
      if (transformMode === 'translate') setReferencePosition(pos);
      else setReferenceAngle(rot);
      return;
    }

    if (key.startsWith('obstacle-')) {
      const idx = Number.parseInt(key.split('-')[1], 10);
      if (!Number.isFinite(idx) || !obstacles[idx]) return;
      const next = [...obstacles];
      next[idx].position = pos;
      setObstacles(next);
      return;
    }

    if (key.startsWith('waypoint-')) {
      const idx = Number.parseInt(key.split('-')[1], 10);
      if (Number.isFinite(idx)) {
        handleWaypointMove(idx, pos);
      }
    }
  };

  return {
    actions: {
      addObstacle,
      removeObstacle,
      updateObstacle,
      handleWaypointMove,
      commitWaypointMove,
      addWaypoint,
      removeWaypoint,
      removeWaypointAtIndex,
      handleObjectTransform,
    },
  };
}

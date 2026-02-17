import { useState } from 'react';

export type TransformMode = 'translate' | 'rotate';
export type SelectionType =
  | 'satellite'
  | 'reference'
  | `obstacle-${number}`
  | `waypoint-${number}`
  | `spline-${number}`
  | null;

export function useMissionSceneState() {
  const [startPosition, setStartPosition] = useState<[number, number, number]>([10, 0, 0]);
  const [startFrame, setStartFrame] = useState<'ECI' | 'LVLH'>('ECI');
  const [startTargetId, setStartTargetId] = useState<string | undefined>(undefined);
  const [startAngle, setStartAngle] = useState<[number, number, number]>([0, 0, 0]);
  const [referencePosition, setReferencePosition] = useState<[number, number, number]>([0, 0, 0]);
  const [referenceAngle, setReferenceAngle] = useState<[number, number, number]>([0, 0, 0]);
  const [obstacles, setObstacles] = useState<{ position: [number, number, number]; radius: number }[]>([]);
  const [selectedObjectId, setSelectedObjectId] = useState<SelectionType>(null);
  const [transformMode, setTransformMode] = useState<TransformMode>('translate');

  return {
    state: {
      startPosition,
      startFrame,
      startTargetId,
      startAngle,
      referencePosition,
      referenceAngle,
      obstacles,
      selectedObjectId,
      transformMode,
    },
    setters: {
      setStartPosition,
      setStartFrame,
      setStartTargetId,
      setStartAngle,
      setReferencePosition,
      setReferenceAngle,
      setObstacles,
      setSelectedObjectId,
      setTransformMode,
    },
  };
}

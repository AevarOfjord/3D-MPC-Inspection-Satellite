import { useEffect, type Dispatch, type SetStateAction } from 'react';
import * as THREE from 'three';

interface UseScanPlaneControlsArgs {
  referencePosition: [number, number, number];
  referenceAngle: [number, number, number];
  scanPlaneAxis: 'X' | 'Y' | 'Z';
  setScanPlaneAxis: Dispatch<SetStateAction<'X' | 'Y' | 'Z'>>;
  setScanPlaneA: Dispatch<SetStateAction<[number, number, number]>>;
  setScanPlaneB: Dispatch<SetStateAction<[number, number, number]>>;
}

export function useScanPlaneControls({
  referencePosition,
  referenceAngle,
  scanPlaneAxis,
  setScanPlaneAxis,
  setScanPlaneA,
  setScanPlaneB,
}: UseScanPlaneControlsArgs) {
  const resolveScanPlaneNormal = (
    axis: 'X' | 'Y' | 'Z' = scanPlaneAxis
  ): [number, number, number] => {
    const basis: [number, number, number] =
      axis === 'X' ? [1, 0, 0] : axis === 'Y' ? [0, 1, 0] : [0, 0, 1];
    const e = new THREE.Euler(
      (referenceAngle[0] * Math.PI) / 180,
      (referenceAngle[1] * Math.PI) / 180,
      (referenceAngle[2] * Math.PI) / 180
    );
    const v = new THREE.Vector3(basis[0], basis[1], basis[2]).applyEuler(e).normalize();
    return [v.x, v.y, v.z];
  };

  const projectPointToAxis = (
    point: [number, number, number],
    axis: [number, number, number]
  ): [number, number, number] => {
    const rel = [
      point[0] - referencePosition[0],
      point[1] - referencePosition[1],
      point[2] - referencePosition[2],
    ] as [number, number, number];
    const t = rel[0] * axis[0] + rel[1] * axis[1] + rel[2] * axis[2];
    return [
      referencePosition[0] + axis[0] * t,
      referencePosition[1] + axis[1] * t,
      referencePosition[2] + axis[2] * t,
    ];
  };

  const moveScanPlaneHandle = (handle: 'a' | 'b', position: [number, number, number]) => {
    const normal = resolveScanPlaneNormal();
    const constrained = projectPointToAxis(position, normal);
    if (handle === 'a') setScanPlaneA(constrained);
    else setScanPlaneB(constrained);
  };

  const setScanPlaneAxisAligned = (axis: 'X' | 'Y' | 'Z') => {
    const normal = resolveScanPlaneNormal(axis);
    setScanPlaneAxis(axis);
    setScanPlaneA((prev) => projectPointToAxis(prev, normal));
    setScanPlaneB((prev) => projectPointToAxis(prev, normal));
  };

  useEffect(() => {
    const normal = resolveScanPlaneNormal();
    setScanPlaneA((prev) => projectPointToAxis(prev, normal));
    setScanPlaneB((prev) => projectPointToAxis(prev, normal));
  }, [
    referenceAngle[0],
    referenceAngle[1],
    referenceAngle[2],
    referencePosition[0],
    referencePosition[1],
    referencePosition[2],
    scanPlaneAxis,
    setScanPlaneA,
    setScanPlaneB,
  ]);

  return {
    actions: {
      moveScanPlaneHandle,
      setScanPlaneAxisAligned,
    },
  };
}

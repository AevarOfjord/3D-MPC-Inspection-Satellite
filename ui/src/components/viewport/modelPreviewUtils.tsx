import * as THREE from 'three';

import { ISSModel } from '../ISSModel';
import { StarlinkModel } from '../StarlinkModel';

export function resolvePreviewModel(modelPath?: string) {
  if (!modelPath) return null;
  const lower = modelPath.toLowerCase();
  // Preserve OBJ+MTL workflows: when the user selects an OBJ, let ObjWithMtl load
  // the chosen file set (materials/textures/colors) instead of force-switching models.
  if (lower.endsWith('.obj')) {
    return null;
  }
  if (lower.includes('starlink')) {
    return (
      <StarlinkModel
        position={[0, 0, 0]}
        orientation={[0, 0, 0]}
        realSpanMeters={11}
        scale={1}
        pivot="origin"
      />
    );
  }
  if (lower.includes('iss')) {
    return (
      <ISSModel
        position={[0, 0, 0]}
        orientation={[0, 0, 0]}
        realSpanMeters={109}
        scale={1}
      />
    );
  }
  return null;
}

export function computeFacingEuler(
  position: [number, number, number],
  baseAxis: [number, number, number] = [0, 0, -1],
  fallback: [number, number, number] = [0, 0, 0]
) {
  const toEarth = new THREE.Vector3(-position[0], -position[1], -position[2]);
  if (toEarth.lengthSq() < 1e-8) return fallback;
  toEarth.normalize();
  const base = new THREE.Vector3(baseAxis[0], baseAxis[1], baseAxis[2]);
  if (base.lengthSq() < 1e-8) return fallback;
  base.normalize();
  const quat = new THREE.Quaternion().setFromUnitVectors(base, toEarth);
  const euler = new THREE.Euler().setFromQuaternion(quat);
  return [euler.x, euler.y, euler.z] as [number, number, number];
}

import { useMemo, Suspense, lazy } from 'react';
import { Line } from '@react-three/drei';
import * as THREE from 'three';

import { StarlinkModel } from '../StarlinkModel';
import { ISSModel } from '../ISSModel';
import { ORBIT_SCALE, EARTH_RADIUS_M, orbitSnapshot } from '../../data/orbitSnapshot';
import { computeFacingEuler } from './ObjModelLoader';

const EarthModelLayer = lazy(() =>
  import('./EarthModelLayer').then((m) => ({ default: m.EarthModelLayer }))
);

export function OrbitObjectsLayer() {
  const orbitObjects = useMemo(
    () =>
      orbitSnapshot.objects.map((obj) => ({
        ...obj,
        position: [
          obj.position_m[0] * ORBIT_SCALE,
          obj.position_m[1] * ORBIT_SCALE,
          obj.position_m[2] * ORBIT_SCALE,
        ] as [number, number, number],
        scaleBoost: obj.visual_scale_boost ?? 1,
        resolvedOrientation: obj.align_to_earth
          ? computeFacingEuler(
              [
                obj.position_m[0] * ORBIT_SCALE,
                obj.position_m[1] * ORBIT_SCALE,
                obj.position_m[2] * ORBIT_SCALE,
              ],
              obj.base_axis ?? [0, 0, -1],
              obj.orientation ?? [0, 0, 0]
            )
          : (obj.orientation ?? [0, 0, 0]),
      })),
    []
  );

  return (
    <Suspense fallback={null}>
      {orbitObjects.map((obj) => {
        return obj.type === 'iss' ? (
          <ISSModel
            key={obj.id}
            position={obj.position}
            orientation={obj.resolvedOrientation}
            realSpanMeters={obj.real_span_m}
            scale={obj.scaleBoost}
          />
        ) : (
          <StarlinkModel
            key={obj.id}
            position={obj.position}
            orientation={obj.resolvedOrientation}
            realSpanMeters={obj.real_span_m}
            scale={obj.scaleBoost}
            pivot={obj.pivot}
          />
        );
      })}
    </Suspense>
  );
}

export function OrbitRingsLayer() {
  const orbitObjects = useMemo(
    () =>
      orbitSnapshot.objects.map((obj) => ({
        id: obj.id,
        type: obj.type,
        position: [
          obj.position_m[0] * ORBIT_SCALE,
          obj.position_m[1] * ORBIT_SCALE,
          obj.position_m[2] * ORBIT_SCALE,
        ] as [number, number, number],
      })),
    []
  );

  const buildOrbitPoints = (radius: number, normal: THREE.Vector3, startPos: THREE.Vector3, segments = 2048) => {
    const safeNormal = normal.clone().normalize();
    const axisA = startPos.clone().normalize();
    const axisB = new THREE.Vector3().crossVectors(safeNormal, axisA).normalize();
    const points: [number, number, number][] = [];
    for (let i = 0; i <= segments; i += 1) {
      const t = (i / segments) * Math.PI * 2;
      const cos = Math.cos(t);
      const sin = Math.sin(t);
      const p = axisA.clone().multiplyScalar(radius * cos).add(axisB.clone().multiplyScalar(radius * sin));
      points.push([p.x, p.y, p.z]);
    }
    return points;
  };

  return (
    <group>
      {orbitObjects.map((obj) => {
        const pos = new THREE.Vector3(obj.position[0], obj.position[1], obj.position[2]);
        const orbitRadius = pos.length();
        let normal = new THREE.Vector3().crossVectors(pos, new THREE.Vector3(0, 1, 0));
        if (normal.lengthSq() < 1e-6) {
          normal = new THREE.Vector3().crossVectors(pos, new THREE.Vector3(1, 0, 0));
        }
        const points = buildOrbitPoints(orbitRadius, normal, pos, 2048);
        return (
          <Line
            key={`${obj.id}-orbit`}
            points={points}
            color={obj.type === 'iss' ? '#38bdf8' : '#a78bfa'}
            lineWidth={1.5}
            transparent
            opacity={0.6}
          />
        );
      })}
    </group>
  );
}

export function EarthLayer() {
  const earthRadius = EARTH_RADIUS_M * ORBIT_SCALE;
  return (
    <Suspense fallback={null}>
      <EarthModelLayer earthRadius={earthRadius} />
      <mesh>
        <sphereGeometry args={[earthRadius * 1.02, 32, 32]} />
        <meshStandardMaterial color="#4cc9f0" transparent opacity={0.08} />
      </mesh>
    </Suspense>
  );
}

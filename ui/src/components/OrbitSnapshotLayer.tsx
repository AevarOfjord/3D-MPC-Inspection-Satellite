import { Suspense, useMemo } from 'react';
import type { ThreeEvent } from '@react-three/fiber';
import { useGLTF } from '@react-three/drei';
import { Line } from '@react-three/drei';
import * as THREE from 'three';
import { orbitSnapshot, ORBIT_SCALE, EARTH_RADIUS_M } from '../data/orbitSnapshot';
import { StarlinkModel } from './StarlinkModel';
import { ISSModel } from './ISSModel';

const buildOrbitPoints = (radius: number, normal: THREE.Vector3, segments = 128) => {
  const safeNormal = normal.clone().normalize();
  const reference = Math.abs(safeNormal.dot(new THREE.Vector3(0, 1, 0))) > 0.95
    ? new THREE.Vector3(1, 0, 0)
    : new THREE.Vector3(0, 1, 0);
  const axisA = new THREE.Vector3().crossVectors(reference, safeNormal).normalize();
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

function scalePosition(pos: [number, number, number]) {
  return [
    pos[0] * ORBIT_SCALE,
    pos[1] * ORBIT_SCALE,
    pos[2] * ORBIT_SCALE,
  ] as [number, number, number];
}

function computeFacingEuler(
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

export function OrbitSnapshotLayer({
  onSelectTarget,
  selectedTargetId,
  orbitVisibility,
}: {
  onSelectTarget?: (
    targetId: string,
    positionMeters: [number, number, number],
    positionScene: [number, number, number],
    focusDistance?: number
  ) => void;
  selectedTargetId?: string | null;
  orbitVisibility?: Record<string, boolean>;
}) {
  const earthRadius = EARTH_RADIUS_M * ORBIT_SCALE;
  const earthGltf = useGLTF('/OBJ_files/Earth/Earth.glb');
  const earthScale = useMemo(() => {
    const box = new THREE.Box3().setFromObject(earthGltf.scene);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    if (!Number.isFinite(maxDim) || maxDim <= 0) return 1;
    return (earthRadius * 2) / maxDim;
  }, [earthGltf, earthRadius]);
  const orbitObjects = useMemo(
    () =>
      orbitSnapshot.objects.map((obj) => ({
        ...obj,
        position: scalePosition(obj.position_m),
        scaleBoost: obj.visual_scale_boost ?? 1,
        orbitRadius: new THREE.Vector3(
          obj.position_m[0] * ORBIT_SCALE,
          obj.position_m[1] * ORBIT_SCALE,
          obj.position_m[2] * ORBIT_SCALE
        ).length(),
        resolvedOrientation: obj.align_to_earth
          ? computeFacingEuler(
              scalePosition(obj.position_m),
              obj.base_axis ?? [0, 0, -1],
              obj.orientation ?? [0, 0, 0]
            )
          : (obj.orientation ?? [0, 0, 0]),
        focusDistance: obj.real_span_m ? Math.max(obj.real_span_m * 5, 10) : undefined,
      })),
    []
  );

  return (
    <group>
      <Suspense fallback={null}>
        <primitive object={earthGltf.scene} scale={[earthScale, earthScale, earthScale]} />
      </Suspense>
      <mesh>
        <sphereGeometry args={[earthRadius * 1.02, 32, 32]} />
        <meshStandardMaterial color="#4cc9f0" transparent opacity={0.1} />
      </mesh>

      <Suspense fallback={null}>
        {orbitObjects.map((obj) => {
          if (!orbitVisibility?.[obj.id]) return null;
          const pos = new THREE.Vector3(obj.position[0], obj.position[1], obj.position[2]);
          let normal = new THREE.Vector3().crossVectors(pos, new THREE.Vector3(0, 1, 0));
          if (normal.lengthSq() < 1e-6) {
            normal = new THREE.Vector3().crossVectors(pos, new THREE.Vector3(1, 0, 0));
          }
          const points = buildOrbitPoints(obj.orbitRadius, normal, 180);
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
        {orbitObjects.map((obj) => {
          const isSelected = selectedTargetId === obj.id;
          const handleSelect = (event: ThreeEvent<PointerEvent>) => {
            event.stopPropagation();
            onSelectTarget?.(obj.id, obj.position_m, obj.position, obj.focusDistance);
          };

          return (
            <group key={obj.id} onPointerDown={handleSelect}>
              {obj.type === 'iss' ? (
                <ISSModel
                  position={obj.position}
                  orientation={obj.resolvedOrientation}
                  realSpanMeters={obj.real_span_m}
                  scale={obj.scaleBoost}
                />
              ) : (
                <StarlinkModel
                  position={obj.position}
                  orientation={obj.resolvedOrientation}
                  realSpanMeters={obj.real_span_m}
                  scale={obj.scaleBoost}
                />
              )}
              {/* Selection handled via highlight in model materials; no overlay markers */}
            </group>
          );
        })}
      </Suspense>
    </group>
  );
}

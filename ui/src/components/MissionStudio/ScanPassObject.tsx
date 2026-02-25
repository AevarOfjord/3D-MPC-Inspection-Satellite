import { useMemo, useEffect } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';
import { useRegenerateWaypoints } from './useRegenerateWaypoints';

interface ScanPassObjectProps {
  scanId: string;
}

function qToEuler(qwxyz: [number, number, number, number]): [number, number, number] {
  const q = new THREE.Quaternion(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]);
  const e = new THREE.Euler().setFromQuaternion(q);
  return [e.x, e.y, e.z];
}

function PlaneMesh({
  position,
  orientation,
  color,
}: {
  position: [number, number, number];
  orientation: [number, number, number, number];
  color: string;
}) {
  return (
    <mesh position={position} rotation={qToEuler(orientation)}>
      <planeGeometry args={[8, 8]} />
      <meshBasicMaterial color={color} opacity={0.16} transparent side={THREE.DoubleSide} />
    </mesh>
  );
}

export function ScanPassObject({ scanId }: ScanPassObjectProps) {
  const path = useStudioStore((s) => s.paths.find((p) => p.id === scanId));
  const selectedPathId = useStudioStore((s) => s.selectedPathId);
  const activeTool = useStudioStore((s) => s.activeTool);
  const regenerate = useRegenerateWaypoints();

  useEffect(() => {
    if (!path) return;
    regenerate(scanId, 0);
  }, [scanId, path?.planeA, path?.planeB, path?.ellipse, path?.levelSpacing, regenerate, path]);

  const lineGeometry = useMemo(() => {
    if (!path || path.waypoints.length < 2) return null;
    const points = path.waypoints.map(([x, y, z]) => new THREE.Vector3(x, y, z));
    return new THREE.BufferGeometry().setFromPoints(points);
  }, [path?.waypoints]);

  const startPos = path?.waypoints[0] ?? null;
  const endPos = path && path.waypoints.length > 0 ? path.waypoints[path.waypoints.length - 1] : null;
  const isSelected = selectedPathId === scanId;
  const color = path?.color ?? '#22d3ee';

  if (!path || !lineGeometry) return null;

  const showPlaneGizmos = isSelected && activeTool === 'create_path';
  const holdStride = Math.max(1, Math.floor(path.waypoints.length / 48));

  return (
    <group>
      <primitive
        object={
          new THREE.Line(
            lineGeometry,
            new THREE.LineBasicMaterial({
              color,
              linewidth: isSelected ? 2 : 1,
              opacity: isSelected ? 1 : 0.7,
              transparent: true,
            })
          )
        }
      />

      {startPos && (
        <mesh position={startPos} onClick={() => useStudioStore.getState().selectPath(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#22d3ee" />
        </mesh>
      )}
      {endPos && (
        <mesh position={endPos} onClick={() => useStudioStore.getState().selectPath(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#a78bfa" />
        </mesh>
      )}

      <PlaneMesh position={path.planeA.position} orientation={path.planeA.orientation} color={color} />
      <PlaneMesh position={path.planeB.position} orientation={path.planeB.orientation} color={color} />

      {showPlaneGizmos && (
        <>
          <TransformControls
            mode="translate"
            position={path.planeA.position}
            rotation={qToEuler(path.planeA.orientation)}
            onObjectChange={(e: any) => {
              const obj = e?.target?.object as THREE.Object3D | undefined;
              if (!obj) return;
              const q = obj.quaternion;
              useStudioStore.getState().updatePathPlane(scanId, 'planeA', {
                position: [obj.position.x, obj.position.y, obj.position.z],
                orientation: [q.w, q.x, q.y, q.z],
              });
              regenerate(scanId, 120);
            }}
          />
          <TransformControls
            mode="rotate"
            position={path.planeA.position}
            rotation={qToEuler(path.planeA.orientation)}
            onObjectChange={(e: any) => {
              const obj = e?.target?.object as THREE.Object3D | undefined;
              if (!obj) return;
              const q = obj.quaternion;
              useStudioStore.getState().updatePathPlane(scanId, 'planeA', {
                orientation: [q.w, q.x, q.y, q.z],
              });
              regenerate(scanId, 120);
            }}
          />
          <TransformControls
            mode="translate"
            position={path.planeB.position}
            rotation={qToEuler(path.planeB.orientation)}
            onObjectChange={(e: any) => {
              const obj = e?.target?.object as THREE.Object3D | undefined;
              if (!obj) return;
              const q = obj.quaternion;
              useStudioStore.getState().updatePathPlane(scanId, 'planeB', {
                position: [obj.position.x, obj.position.y, obj.position.z],
                orientation: [q.w, q.x, q.y, q.z],
              });
              regenerate(scanId, 120);
            }}
          />
          <TransformControls
            mode="rotate"
            position={path.planeB.position}
            rotation={qToEuler(path.planeB.orientation)}
            onObjectChange={(e: any) => {
              const obj = e?.target?.object as THREE.Object3D | undefined;
              if (!obj) return;
              const q = obj.quaternion;
              useStudioStore.getState().updatePathPlane(scanId, 'planeB', {
                orientation: [q.w, q.x, q.y, q.z],
              });
              regenerate(scanId, 120);
            }}
          />
        </>
      )}

      {activeTool === 'hold' &&
        path.waypoints.map((wp, i) => {
          if (i % holdStride !== 0) return null;
          return (
            <mesh
              key={`${scanId}-hold-${i}`}
              position={wp}
              onClick={(e) => {
                e.stopPropagation();
                useStudioStore.getState().addHold({
                  id: `hold-${Date.now()}-${i}`,
                  pathId: scanId,
                  waypointIndex: i,
                  duration: 5,
                });
              }}
            >
              <sphereGeometry args={[0.13, 8, 8]} />
              <meshBasicMaterial color="#fbbf24" opacity={0.85} transparent />
            </mesh>
          );
        })}
    </group>
  );
}

import { useMemo, useState } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';
import { useRegenerateWaypoints } from './useRegenerateWaypoints';

const LOCAL_PLANE_FRAME = { u: new THREE.Vector3(1, 0, 0) };

function basisAtPlaneA(path: ReturnType<typeof useStudioStore.getState>['paths'][number]) {
  // Match the same centerline-perpendicular frame used for spiral generation,
  // but anchor handle positions on Plane A (not midpoint).
  const qA = new THREE.Quaternion(
    path.planeA.orientation[1],
    path.planeA.orientation[2],
    path.planeA.orientation[3],
    path.planeA.orientation[0]
  ).normalize();
  const a = new THREE.Vector3(path.planeA.position[0], path.planeA.position[1], path.planeA.position[2]);
  const b = new THREE.Vector3(path.planeB.position[0], path.planeB.position[1], path.planeB.position[2]);
  const axis = b.clone().sub(a);
  if (axis.lengthSq() < 1e-9) axis.set(0, 0, 1);
  const nAxis = axis.normalize();
  const uSeed = LOCAL_PLANE_FRAME.u.clone().applyQuaternion(qA);
  const uProj = uSeed.clone().sub(nAxis.clone().multiplyScalar(uSeed.dot(nAxis)));
  let u = uProj;
  if (u.lengthSq() < 1e-9) {
    const fallback = Math.abs(nAxis.dot(new THREE.Vector3(0, 0, 1))) < 0.9
      ? new THREE.Vector3(0, 0, 1)
      : new THREE.Vector3(1, 0, 0);
    u = fallback.clone().cross(nAxis);
  }
  u.normalize();
  const v = nAxis.clone().cross(u).normalize();
  const anchor = new THREE.Vector3(path.planeA.position[0], path.planeA.position[1], path.planeA.position[2]);
  return { anchor, u, v };
}

export function EllipseHandles({ scanId }: { scanId: string }) {
  const path = useStudioStore((s) => s.paths.find((p) => p.id === scanId));
  const setSelectedHandle = useStudioStore((s) => s.setSelectedHandle);
  const selectedHandleId = useStudioStore((s) => s.paths.find((p) => p.id === scanId)?.selectedHandleId ?? null);
  const [hoveredId, setHoveredId] = useState<string | null>(null);
  const regenerate = useRegenerateWaypoints();

  const basis = useMemo(() => (path ? basisAtPlaneA(path) : null), [path]);
  if (!path || !basis) return null;

  const { anchor, u, v } = basis;
  const rx = Math.max(0.1, path.ellipse.radiusX);
  const ry = Math.max(0.1, path.ellipse.radiusY);

  const handles = [
    { id: 'rx_pos', pos: anchor.clone().add(u.clone().multiplyScalar(rx)), axis: 'x' as const, sign: 1, color: '#34d399' },
    { id: 'rx_neg', pos: anchor.clone().add(u.clone().multiplyScalar(-rx)), axis: 'x' as const, sign: -1, color: '#34d399' },
    { id: 'ry_pos', pos: anchor.clone().add(v.clone().multiplyScalar(ry)), axis: 'y' as const, sign: 1, color: '#60a5fa' },
    { id: 'ry_neg', pos: anchor.clone().add(v.clone().multiplyScalar(-ry)), axis: 'y' as const, sign: -1, color: '#60a5fa' },
  ];

  return (
    <>
      {handles.map((h) => {
        const isHovered = hoveredId === h.id;
        const isSelected = selectedHandleId === h.id;
        const pos: [number, number, number] = [h.pos.x, h.pos.y, h.pos.z];

        return (
          <group key={h.id}>
            <mesh
              position={pos}
              onPointerOver={(e) => {
                e.stopPropagation();
                setHoveredId(h.id);
              }}
              onPointerOut={(e) => {
                e.stopPropagation();
                setHoveredId((prev) => (prev === h.id ? null : prev));
              }}
              onClick={(e) => {
                e.stopPropagation();
                setSelectedHandle(scanId, isSelected ? null : (h.id as any));
              }}
            >
              <sphereGeometry args={[0.22 * (isHovered ? 1.2 : 1.0), 12, 12]} />
              <meshBasicMaterial color={isSelected ? '#fde047' : isHovered ? '#ffffff' : h.color} />
            </mesh>

            {isSelected && (
              <TransformControls
                mode="translate"
                position={pos}
                showX
                showY
                showZ
                onObjectChange={(e: any) => {
                  const obj = e?.target?.object as THREE.Object3D | undefined;
                  if (!obj) return;
                  const rel = new THREE.Vector3(obj.position.x, obj.position.y, obj.position.z).sub(anchor);
                  if (h.axis === 'x') {
                    const newRadius = Math.max(0.1, Math.abs(rel.dot(u)));
                    useStudioStore.getState().updatePathEllipse(scanId, { radiusX: newRadius });
                  } else {
                    const newRadius = Math.max(0.1, Math.abs(rel.dot(v)));
                    useStudioStore.getState().updatePathEllipse(scanId, { radiusY: newRadius });
                  }
                  regenerate(scanId, 120);
                }}
              />
            )}
          </group>
        );
      })}
    </>
  );
}

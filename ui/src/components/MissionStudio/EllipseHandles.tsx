import { useMemo, useState } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';
import { useRegenerateWaypoints } from './useRegenerateWaypoints';

const AXIS_FRAME: Record<'X' | 'Y' | 'Z', { u: THREE.Vector3; v: THREE.Vector3 }> = {
  X: { u: new THREE.Vector3(0, 1, 0), v: new THREE.Vector3(0, 0, 1) },
  Y: { u: new THREE.Vector3(1, 0, 0), v: new THREE.Vector3(0, 0, 1) },
  Z: { u: new THREE.Vector3(1, 0, 0), v: new THREE.Vector3(0, 1, 0) },
};

function basisAtMid(path: ReturnType<typeof useStudioStore.getState>['paths'][number]) {
  const qA = new THREE.Quaternion(path.planeA.orientation[1], path.planeA.orientation[2], path.planeA.orientation[3], path.planeA.orientation[0]);
  const qB = new THREE.Quaternion(path.planeB.orientation[1], path.planeB.orientation[2], path.planeB.orientation[3], path.planeB.orientation[0]);
  const qMid = new THREE.Quaternion().copy(qA).slerp(qB, 0.5).normalize();
  const base = AXIS_FRAME[path.axisSeed];
  const u = base.u.clone().applyQuaternion(qMid).normalize();
  const v = base.v.clone().applyQuaternion(qMid).normalize();
  const center = new THREE.Vector3(
    0.5 * (path.planeA.position[0] + path.planeB.position[0]),
    0.5 * (path.planeA.position[1] + path.planeB.position[1]),
    0.5 * (path.planeA.position[2] + path.planeB.position[2]),
  );
  return { center, u, v };
}

export function EllipseHandles({ scanId }: { scanId: string }) {
  const path = useStudioStore((s) => s.paths.find((p) => p.id === scanId));
  const setSelectedHandle = useStudioStore((s) => s.setSelectedHandle);
  const selectedHandleId = useStudioStore((s) => s.paths.find((p) => p.id === scanId)?.selectedHandleId ?? null);
  const [hoveredId, setHoveredId] = useState<string | null>(null);
  const regenerate = useRegenerateWaypoints();

  const basis = useMemo(() => (path ? basisAtMid(path) : null), [path]);
  if (!path || !basis) return null;

  const { center, u, v } = basis;
  const rx = Math.max(0.1, path.ellipse.radiusX);
  const ry = Math.max(0.1, path.ellipse.radiusY);

  const handles = [
    { id: 'rx_pos', pos: center.clone().add(u.clone().multiplyScalar(rx)), axis: 'x' as const, sign: 1, color: '#34d399' },
    { id: 'rx_neg', pos: center.clone().add(u.clone().multiplyScalar(-rx)), axis: 'x' as const, sign: -1, color: '#34d399' },
    { id: 'ry_pos', pos: center.clone().add(v.clone().multiplyScalar(ry)), axis: 'y' as const, sign: 1, color: '#60a5fa' },
    { id: 'ry_neg', pos: center.clone().add(v.clone().multiplyScalar(-ry)), axis: 'y' as const, sign: -1, color: '#60a5fa' },
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
                  const rel = new THREE.Vector3(obj.position.x, obj.position.y, obj.position.z).sub(center);
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

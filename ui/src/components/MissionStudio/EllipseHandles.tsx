import { useState } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';
import { useRegenerateWaypoints } from './useRegenerateWaypoints';
import type { ScanPass } from './useStudioStore';

const AXIS_FRAMES: Record<string, { normal: THREE.Vector3; u: THREE.Vector3; v: THREE.Vector3 }> = {
  Z: { normal: new THREE.Vector3(0, 0, 1), u: new THREE.Vector3(1, 0, 0), v: new THREE.Vector3(0, 1, 0) },
  X: { normal: new THREE.Vector3(1, 0, 0), u: new THREE.Vector3(0, 1, 0), v: new THREE.Vector3(0, 0, 1) },
  Y: { normal: new THREE.Vector3(0, 1, 0), u: new THREE.Vector3(1, 0, 0), v: new THREE.Vector3(0, 0, 1) },
};

type HandleId = 'rx_pos' | 'rx_neg' | 'ry_pos' | 'ry_neg';

function computeHandlePositions(pass: ScanPass): Array<{ id: HandleId; pos: THREE.Vector3; color: string }> {
  const kl = pass.keyLevels[0];
  if (!kl) return [];

  const frame = AXIS_FRAMES[pass.axis];
  const rot = (kl.rotation_deg * Math.PI) / 180;

  const major = frame.u.clone().multiplyScalar(Math.cos(rot)).add(frame.v.clone().multiplyScalar(Math.sin(rot))).normalize();
  const minor = frame.u.clone().multiplyScalar(-Math.sin(rot)).add(frame.v.clone().multiplyScalar(Math.cos(rot))).normalize();

  const axisSpan = pass.planeBOffset - pass.planeAOffset;
  const centerAlong = pass.planeAOffset + axisSpan * kl.t;
  const center =
    pass.axis === 'Z' ? new THREE.Vector3(kl.offset_x, kl.offset_y, centerAlong) :
    pass.axis === 'X' ? new THREE.Vector3(centerAlong, kl.offset_x, kl.offset_y) :
                        new THREE.Vector3(kl.offset_x, centerAlong, kl.offset_y);

  const rx = Math.max(0.1, kl.radius_x);
  const ry = Math.max(0.1, kl.radius_y);

  return [
    { id: 'rx_pos', pos: center.clone().add(major.clone().multiplyScalar(rx)), color: '#34d399' },
    { id: 'rx_neg', pos: center.clone().add(major.clone().multiplyScalar(-rx)), color: '#34d399' },
    { id: 'ry_pos', pos: center.clone().add(minor.clone().multiplyScalar(ry)), color: '#60a5fa' },
    { id: 'ry_neg', pos: center.clone().add(minor.clone().multiplyScalar(-ry)), color: '#60a5fa' },
  ];
}

export function EllipseHandles({ scanId }: { scanId: string }) {
  const pass = useStudioStore((s) => s.scanPasses.find((p) => p.id === scanId));
  const updateKeyLevelHandle = useStudioStore((s) => s.updateKeyLevelHandle);
  const setSelectedHandle = useStudioStore((s) => s.setSelectedHandle);
  const selectedHandleId = useStudioStore((s) => s.scanPasses.find((p) => p.id === scanId)?.selectedHandleId ?? null);
  const [hoveredId, setHoveredId] = useState<string | null>(null);
  const regenerate = useRegenerateWaypoints();

  if (!pass || !pass.keyLevels[0]) return null;

  const handles = computeHandlePositions(pass);
  const handleRadius = 0.25;

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
              onPointerOver={(e) => { e.stopPropagation(); setHoveredId(h.id); }}
              onPointerOut={(e) => { e.stopPropagation(); setHoveredId((prev) => prev === h.id ? null : prev); }}
              onClick={(e) => {
                e.stopPropagation();
                setSelectedHandle(scanId, isSelected ? null : h.id);
              }}
            >
              <sphereGeometry args={[handleRadius * (isHovered ? 1.3 : 1.0), 12, 12]} />
              <meshBasicMaterial
                color={isSelected ? '#fde047' : isHovered ? '#ffffff' : h.color}
                transparent
                opacity={isHovered || isSelected ? 1.0 : 0.9}
              />
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
                  updateKeyLevelHandle(scanId, h.id as HandleId, [obj.position.x, obj.position.y, obj.position.z]);
                  regenerate(scanId, 150);
                }}
              />
            )}
          </group>
        );
      })}
    </>
  );
}

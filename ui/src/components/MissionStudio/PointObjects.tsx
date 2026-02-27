import { useEffect, useState } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';

export function PointObjects() {
  const assembly = useStudioStore((s) => s.assembly);
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);
  const points = useStudioStore((s) => s.points);
  const activeTool = useStudioStore((s) => s.activeTool);
  const updatePoint = useStudioStore((s) => s.updatePoint);
  const [selectedPointId, setSelectedPointId] = useState<string | null>(null);
  const selectedAssembly = selectedAssemblyId ? assembly.find((a) => a.id === selectedAssemblyId) ?? null : null;
  const visiblePoints =
    selectedAssembly == null
      ? points
      : selectedAssembly.type === 'point'
        ? points.filter((p) => p.id === selectedAssembly.pointId)
        : [];
  const selectedPoint = visiblePoints.find((p) => p.id === selectedPointId) ?? null;

  useEffect(() => {
    if (!selectedPointId) return;
    if (!points.some((p) => p.id === selectedPointId)) {
      setSelectedPointId(null);
    }
  }, [points, selectedPointId]);

  useEffect(() => {
    if (activeTool !== 'point') {
      setSelectedPointId(null);
    }
  }, [activeTool]);

  return (
    <>
      {visiblePoints.map((point) => (
        <mesh
          key={point.id}
          position={point.position}
          onClick={(e) => {
            e.stopPropagation();
            if (activeTool !== 'point') return;
            setSelectedPointId((prev) => (prev === point.id ? null : point.id));
          }}
        >
          <sphereGeometry args={[0.22, 16, 16]} />
          <meshBasicMaterial color={selectedPointId === point.id ? '#0ea5e9' : '#38bdf8'} />
        </mesh>
      ))}
      {activeTool === 'point' && selectedPoint && (
        <TransformControls
          mode="translate"
          space="world"
          position={selectedPoint.position}
          onObjectChange={(e: any) => {
            if (!e?.target?.dragging) return;
            const obj = e?.target?.object as THREE.Object3D | undefined;
            if (!obj) return;
            updatePoint(selectedPoint.id, { position: [obj.position.x, obj.position.y, obj.position.z] });
          }}
        />
      )}
    </>
  );
}

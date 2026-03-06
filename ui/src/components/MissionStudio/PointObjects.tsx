import { useEffect } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';

export function PointObjects() {
  const assembly = useStudioStore((s) => s.assembly);
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);
  const points = useStudioStore((s) => s.points);
  const activeTool = useStudioStore((s) => s.activeTool);
  const updatePoint = useStudioStore((s) => s.updatePoint);
  const setSelectedAssemblyId = useStudioStore((s) => s.setSelectedAssemblyId);
  const selectedAssembly = selectedAssemblyId ? assembly.find((a) => a.id === selectedAssemblyId) ?? null : null;
  const visiblePoints =
    selectedAssembly == null
      ? points
      : selectedAssembly.type === 'point'
        ? points.filter((p) => p.id === selectedAssembly.pointId)
        : [];
  const selectedPoint = selectedAssembly?.type === 'point'
    ? points.find((p) => p.id === selectedAssembly.pointId) ?? null
    : null;

  useEffect(() => {
    if (selectedAssembly?.type !== 'point') return;
    if (!points.some((p) => p.id === selectedAssembly.pointId)) {
      setSelectedAssemblyId(null);
    }
  }, [points, selectedAssembly, setSelectedAssemblyId]);

  useEffect(() => {
    if (activeTool !== 'point' && selectedAssembly?.type === 'point') {
      setSelectedAssemblyId(null);
    }
  }, [activeTool, selectedAssembly, setSelectedAssemblyId]);

  return (
    <>
      {visiblePoints.map((point) => (
        <mesh
          key={point.id}
          position={point.position}
          onClick={(e) => {
            e.stopPropagation();
            if (activeTool !== 'point') return;
            const assemblyItem = assembly.find((item) => item.type === 'point' && item.pointId === point.id) ?? null;
            setSelectedAssemblyId(selectedAssemblyId === assemblyItem?.id ? null : (assemblyItem?.id ?? null));
          }}
        >
          <sphereGeometry args={[0.22, 16, 16]} />
          <meshBasicMaterial color={selectedPoint?.id === point.id ? '#0ea5e9' : '#38bdf8'} />
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

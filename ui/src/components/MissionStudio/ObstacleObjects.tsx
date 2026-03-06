import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useEffect } from 'react';
import { useStudioStore } from './useStudioStore';

export function ObstacleObjects() {
  const assembly = useStudioStore((s) => s.assembly);
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);
  const obstacles = useStudioStore((s) => s.obstacles);
  const activeTool = useStudioStore((s) => s.activeTool);
  const updateObstacle = useStudioStore((s) => s.updateObstacle);
  const setSelectedAssemblyId = useStudioStore((s) => s.setSelectedAssemblyId);
  const selectedAssembly = selectedAssemblyId ? assembly.find((a) => a.id === selectedAssemblyId) ?? null : null;
  const visibleObstacles =
    selectedAssembly == null
      ? obstacles
      : selectedAssembly.type === 'obstacle'
        ? obstacles.filter((o) => o.id === selectedAssembly.obstacleId)
        : [];
  const selectedObstacle = selectedAssembly?.type === 'obstacle'
    ? obstacles.find((obs) => obs.id === selectedAssembly.obstacleId) ?? null
    : null;

  useEffect(() => {
    if (activeTool !== 'obstacle' && selectedAssembly?.type === 'obstacle') {
      setSelectedAssemblyId(null);
    }
  }, [activeTool, selectedAssembly, setSelectedAssemblyId]);

  return (
    <>
      {visibleObstacles.map((obs) => (
        <mesh
          key={obs.id}
          position={obs.position}
          onClick={(e) => {
            e.stopPropagation();
            if (activeTool !== 'obstacle') return;
            const assemblyItem = assembly.find((item) => item.type === 'obstacle' && item.obstacleId === obs.id) ?? null;
            setSelectedAssemblyId(selectedAssemblyId === assemblyItem?.id ? null : (assemblyItem?.id ?? null));
          }}
        >
          <sphereGeometry args={[obs.radius, 24, 24]} />
          <meshBasicMaterial
            color={selectedObstacle?.id === obs.id ? '#fb7185' : '#ef4444'}
            opacity={selectedObstacle?.id === obs.id ? 0.34 : 0.25}
            transparent
          />
        </mesh>
      ))}
      {activeTool === 'obstacle' && selectedObstacle && (
        <TransformControls
          mode="translate"
          space="world"
          position={selectedObstacle.position}
          onObjectChange={(e: any) => {
            if (!e?.target?.dragging) return;
            const obj = e?.target?.object as THREE.Object3D | undefined;
            if (!obj) return;
            updateObstacle(selectedObstacle.id, {
              position: [obj.position.x, obj.position.y, obj.position.z],
            });
          }}
        />
      )}
    </>
  );
}

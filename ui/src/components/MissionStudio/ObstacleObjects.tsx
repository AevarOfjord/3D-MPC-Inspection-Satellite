import { useStudioStore } from './useStudioStore';

export function ObstacleObjects() {
  const assembly = useStudioStore((s) => s.assembly);
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);
  const obstacles = useStudioStore((s) => s.obstacles);
  const selectedAssembly = selectedAssemblyId ? assembly.find((a) => a.id === selectedAssemblyId) ?? null : null;
  const visibleObstacles =
    selectedAssembly == null
      ? obstacles
      : selectedAssembly.type === 'obstacle'
        ? obstacles.filter((o) => o.id === selectedAssembly.obstacleId)
        : [];

  return (
    <>
      {visibleObstacles.map((obs) => (
        <mesh key={obs.id} position={obs.position}>
          <sphereGeometry args={[obs.radius, 24, 24]} />
          <meshBasicMaterial color="#ef4444" opacity={0.25} transparent />
        </mesh>
      ))}
    </>
  );
}

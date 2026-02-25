import { useStudioStore } from './useStudioStore';

export function ObstacleObjects() {
  const obstacles = useStudioStore((s) => s.obstacles);

  return (
    <>
      {obstacles.map((obs) => (
        <mesh key={obs.id} position={obs.position}>
          <sphereGeometry args={[obs.radius, 24, 24]} />
          <meshBasicMaterial color="#ef4444" opacity={0.25} transparent />
        </mesh>
      ))}
    </>
  );
}

import * as THREE from 'three';
import { useStudioStore } from './useStudioStore';

export function SatelliteStartNode() {
  const satelliteStart = useStudioStore((s) => s.satelliteStart);

  return (
    <group position={satelliteStart}>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.8, 1.0, 32]} />
        <meshBasicMaterial color="white" side={THREE.DoubleSide} />
      </mesh>
      <mesh>
        <sphereGeometry args={[0.2, 8, 8]} />
        <meshBasicMaterial color="white" />
      </mesh>
    </group>
  );
}

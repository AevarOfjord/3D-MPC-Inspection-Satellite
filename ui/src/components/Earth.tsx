import { useMemo } from 'react';
import { Euler, Quaternion } from 'three';

interface ReferenceMarkerProps {
  position?: [number, number, number];
  orientation?: [number, number, number];
  quaternion?: [number, number, number, number];
  color?: string;
}

export function ReferenceMarker({
  position = [0, 0, 0],
  orientation = [0, 0, 0],
  quaternion,
  color = "#ff4444",
}: ReferenceMarkerProps) {
  const referenceQuat = useMemo(() => {
    if (quaternion) {
      return new Quaternion(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
    }
    const euler = new Euler(orientation[0], orientation[1], orientation[2], 'XYZ');
    return new Quaternion().setFromEuler(euler);
  }, [orientation, quaternion]);

  return (
    <group position={position} quaternion={referenceQuat}>
      <mesh>
        <sphereGeometry args={[0.05, 16, 16]} />
        <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.5} />
      </mesh>
      {/* Interaction/Orientation Arrow Hint */}
      <axesHelper args={[0.5]} />
      <mesh position={[0, 0, 0.25]} rotation={[Math.PI/2, 0, 0]}>
         <cylinderGeometry args={[0.01, 0.01, 0.5, 8]} />
         <meshBasicMaterial color="yellow" opacity={0.5} transparent />
      </mesh>
    </group>
  );
}

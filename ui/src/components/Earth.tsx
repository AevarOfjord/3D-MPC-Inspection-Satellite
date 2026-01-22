import { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Euler, Mesh, Quaternion } from 'three';

export function Earth() {
  const earthRef = useRef<Mesh>(null);

  useFrame(() => {
    if (earthRef.current) {
      earthRef.current.rotation.y += 0.0005; // Slow rotation
    }
  });

  return (
    <mesh ref={earthRef} scale={[6.371, 6.371, 6.371]}> {/* Scale down? Earth is huge. Let's assume units are meters? 6371km radius is too big for cam. Let's keep it small for viz or scale units? */}
      {/* If sim units are meters, Earth is 6,371,000 m radius at (0,0,0). Satellite is at (10,0,0).
          Wait, if satellite is at (0,0,0) relative to Earth Center?
          Usually we simulate relative to a reference point in orbit.
          Linearized dynamics assume a reference frame (LVLH).
          (0,0,0) is the reference position (e.g. another satellite or virtual point).
          Earth is far away "down" in Z direction usually, or somewhere else.
          
          For visualization of "Relative Motion", we don't draw Earth at (0,0,0).
          We draw the reference at (0,0,0).
          
          If we want to draw Earth in background to show orientation:
          Earth should be far away.
          
          Let's just draw a "Reference" marker at (0,0,0) and the satellite.
          And maybe a grid.
      */}
      <sphereGeometry args={[0.5, 32, 32]} />
      <meshStandardMaterial color="#4488ff" wireframe />
    </mesh>
  );
}

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

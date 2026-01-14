import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Mesh } from 'three';

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
          Usually we simulate relative to a target point in orbit.
          Linearized dynamics assume a reference frame (LVLH).
          (0,0,0) is the TARGET position (e.g. another satellite or virtual point).
          Earth is far away "down" in Z direction usually, or somewhere else.
          
          For visualization of "Relative Motion", we don't draw Earth at (0,0,0).
          We draw the target at (0,0,0).
          
          If we want to draw Earth in background to show orientation:
          Earth should be far away.
          
          Let's just draw a "Target" marker at (0,0,0) and the satellite.
          And maybe a grid.
      */}
      <sphereGeometry args={[0.5, 32, 32]} />
      <meshStandardMaterial color="#4488ff" wireframe />
    </mesh>
  );
}

export function TargetMarker() {
  return (
    <mesh position={[0, 0, 0]}>
      <sphereGeometry args={[0.2, 16, 16]} />
      <meshStandardMaterial color="#ff4444" emissive="#ff0000" emissiveIntensity={0.5} />
    </mesh>
  );
}

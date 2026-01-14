import { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import { Group, Mesh } from 'three';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';

// Configuration for 6 Thrusters based on physics.py
// Plume points in direction of gas exhaust (opposite to force)
const THRUSTER_CONFIG = [
    { id: 1, pos: [0.15, 0, 0], rot: [0, 0, -Math.PI/2] }, // +X face, pushes -X -> Plume +X
    { id: 2, pos: [-0.15, 0, 0], rot: [0, 0, Math.PI/2] }, // -X face, pushes +X -> Plume -X
    { id: 3, pos: [0, 0.15, 0], rot: [0, 0, 0] },          // +Y face, pushes -Y -> Plume +Y (default cone up)
    { id: 4, pos: [0, -0.15, 0], rot: [Math.PI, 0, 0] },   // -Y face, pushes +Y -> Plume -Y
    { id: 5, pos: [0, 0, 0.15], rot: [Math.PI/2, 0, 0] },  // +Z face, pushes -Z -> Plume +Z
    { id: 6, pos: [0, 0, -0.15], rot: [-Math.PI/2, 0, 0] },// -Z face, pushes +Z -> Plume -Z
] as const;

export function SatelliteModel() {
  const groupRef = useRef<Group>(null);
  const dataRef = useRef<TelemetryData | null>(null);
  const plumeRefs = useRef<(Mesh | null)[]>([]);

  useEffect(() => {
    const unsubscribe = telemetry.subscribe((data) => {
      dataRef.current = data;
    });
    return () => { unsubscribe(); };
  }, []);

  useFrame(() => {
    if (groupRef.current && dataRef.current) {
      const { position, quaternion, thrusters } = dataRef.current;
      
      // Update Position
      groupRef.current.position.set(position[0], position[1], position[2]);

      // Update Rotation
      const [w, x, y, z] = quaternion;
      groupRef.current.quaternion.set(x, y, z, w);

      // Update Thruster Plumes
      if (thrusters && thrusters.length >= 6) {
          THRUSTER_CONFIG.forEach((_, i) => {
              const mesh = plumeRefs.current[i];
              if (mesh) {
                  // indices 1-6 in physics might map to 0-5. 
                  // app.py sends 12 values? let's assume first 6 map nicely.
                  // If thruster value > 0, show plume
                  const val = thrusters[i];
                  if (val > 0.05) {
                      mesh.visible = true;
                      // Scale length with thrust
                      mesh.scale.set(1, 0.5 + val * 2, 1); 
                      // Update opacity
                      (mesh.material as any).opacity = 0.4 + val * 0.4;
                  } else {
                      mesh.visible = false;
                  }
              }
          });
      }
    }
  });

  return (
    <group ref={groupRef}>
      {/* Main Bus - Gold Foil */}
      <mesh castShadow receiveShadow>
        <boxGeometry args={[0.3, 0.3, 0.3]} />
        <meshStandardMaterial color="#D4AF37" metalness={0.7} roughness={0.3} />
      </mesh>

      {/* Solar Panel Left - REMOVED */}
      {/* <mesh position={[-0.35, 0, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.4, 0.25, 0.02]} />
        <meshStandardMaterial color="#1a237e" metalness={0.5} roughness={0.1} />
      </mesh> */}

      {/* Solar Panel Right - REMOVED */}
      {/* <mesh position={[0.35, 0, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.4, 0.25, 0.02]} />
        <meshStandardMaterial color="#1a237e" metalness={0.5} roughness={0.1} />
      </mesh> */}

      {/* Sensor/Antenna Top */}
      <mesh position={[0, 0.2, 0]}>
         <cylinderGeometry args={[0.02, 0.02, 0.2, 8]} />
         <meshStandardMaterial color="#aaaaaa" metalness={0.9} roughness={0.1} />
      </mesh>

      {/* Thruster Plumes */}
      {THRUSTER_CONFIG.map((cfg, i) => (
          <mesh
            key={cfg.id}
            ref={(el) => { plumeRefs.current[i] = el; }}
            position={cfg.pos}
            rotation={cfg.rot as [number, number, number]}
            visible={false}
          >
             {/* Offset cone so base is at 0,0,0 (thruster nozzle) */}
             <coneGeometry args={[0.03, 0.2, 8]} />
             <meshBasicMaterial color="#00ffff" transparent opacity={0.6} depthWrite={false} />
          </mesh>
      ))}
      
      <axesHelper args={[0.5]} />
    </group>
  );
}

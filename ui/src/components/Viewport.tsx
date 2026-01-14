import { useRef, useEffect, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import { Mesh, Vector3 } from 'three';
import { TargetMarker } from './Earth';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';

function SatelliteModel() {
  const meshRef = useRef<Mesh>(null);
  const dataRef = useRef<TelemetryData | null>(null);

  useEffect(() => {
    // Subscribe to telemetry updates
    const unsubscribe = telemetry.subscribe((data) => {
      dataRef.current = data;
    });
    return () => { unsubscribe(); };
  }, []);

  useFrame(() => {
    if (meshRef.current && dataRef.current) {
      const { position, quaternion } = dataRef.current;
      
      // Update Position
      // Assuming Position is [x, y, z] in meters
      meshRef.current.position.set(position[0], position[1], position[2]);

      // Update Rotation
      // Assuming Quaternion is [w, x, y, z] order from Python?
      // Three.js Quat is (x, y, z, w)
      // Python's Eigen/scipy usually [x, y, z, w] or [w, x, y, z]
      // Project uses standard [w, x, y, z] usually? 
      // Let's assume [w, x, y, z] based on typical C++ Eigen convention often used in this project
      // But three.js expects (x, y, z, w) constructor or set(x, y, z, w).
      
      // Let's check Python code: `q = x_current.segment<4>(3);` -> [qw, qx, qy, qz]
      // So Python sends [w, x, y, z].
      // Three.js set(x, y, z, w).
      
      const [w, x, y, z] = quaternion;
      meshRef.current.quaternion.set(x, y, z, w);
    }
  });

  return (
    <mesh ref={meshRef}>
      <boxGeometry args={[0.3, 0.3, 0.3]} /> {/* 30cm Cube */}
      <meshStandardMaterial color="#ffd700" metalness={0.8} roughness={0.2} />
      <axesHelper args={[0.5]} />
    </mesh>
  );
}

function Obstacles() {
  const [params, setParams] = useState<{obstacles: TelemetryData['obstacles']} | null>(null);

  useEffect(() => {
    // Only update obstacles if we haven't yet, or if they change (simple check)
    // For static obstacles, getting them once is enough, but let's allow updates
    const unsub = telemetry.subscribe(d => {
       if (d.obstacles && (!params || params.obstacles.length !== d.obstacles.length)) {
           setParams({ obstacles: d.obstacles });
       }
    });
    return () => { unsub(); };
  }, [params]);

  if (!params) return null;

  return (
    <group>
      {params.obstacles.map((obs, i) => (
        <mesh key={i} position={new Vector3(...obs.position)}>
          <sphereGeometry args={[obs.radius, 32, 32]} />
          <meshStandardMaterial color="#ff4444" transparent opacity={0.5} wireframe />
        </mesh>
      ))}
    </group>
  );
}

export function Viewport() {
  return (
    <div className="w-full h-full bg-slate-900">
      <Canvas camera={{ position: [5, 5, 5], fov: 45 }}>
        <OrbitControls makeDefault />
        
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10]} intensity={1} />
        
        <Grid infiniteGrid sectionSize={1} cellSize={0.5} fadeDistance={30} />
        
        <TargetMarker />
        <Obstacles />
        <SatelliteModel />
        
      </Canvas>
    </div>
  );
}

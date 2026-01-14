import { useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Stars, GizmoHelper, GizmoViewport } from '@react-three/drei';
import { Vector3 } from 'three';
import { TargetMarker } from './Earth';
import { SatelliteModel } from './SatelliteModel';
import { CameraManager } from './CameraManager';
import { EditableObstacles } from './EditableObstacles';
import { useMissionStore } from '../store/missionStore';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { Trajectory } from './Trajectory';
import { PlannedPath } from './PlannedPath';

function Obstacles() {
  const [params, setParams] = useState<{
    obstacles: TelemetryData['obstacles'], 
    targetPos: TelemetryData['target_position'],
    targetOri: TelemetryData['target_orientation']
  } | null>(null);
  const isEditing = useMissionStore(s => s.isEditing);

  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       setParams({ 
         obstacles: d.obstacles, 
         targetPos: d.target_position,
         targetOri: d.target_orientation || [0,0,0]
       });
    });
    return () => { unsub(); };
  }, []);

  if (!params) return null;

  return (
    <group>
      {!isEditing && <TargetMarker position={params.targetPos} orientation={params.targetOri} />}
      {!isEditing && params.obstacles.map((obs, i) => (
        <mesh key={i} position={new Vector3(...obs.position)}>
          <sphereGeometry args={[obs.radius, 32, 32]} />
          <meshStandardMaterial color="#ff4444" transparent opacity={0.3} wireframe />
        </mesh>
      ))}
    </group>
  );
}

interface ViewportProps {
  viewMode: 'free' | 'chase' | 'top';
}

export function Viewport({ viewMode }: ViewportProps) {
  return (
    <div className="w-full h-full bg-slate-900">
      <Canvas shadows camera={{ position: [5, 5, 5], fov: 45 }}>
        <CameraManager mode={viewMode} />
        <OrbitControls makeDefault enabled={viewMode === 'free'} />
        
        {/* Environment */}
        <color attach="background" args={['#050510']} />
        <Stars radius={100} depth={50} count={5000} factor={4} saturation={0} fade speed={1} />
        
        {/* Lighting */}
        <ambientLight intensity={0.2} />
        <directionalLight 
          position={[10, 10, 5]} 
          intensity={1.5} 
          castShadow 
          shadow-mapSize={[1024, 1024]} 
        />
        <hemisphereLight args={['#ffffff', '#000000', 0.5]} />
        
        <hemisphereLight args={['#ffffff', '#000000', 0.5]} />
        
        {/* Grid Removed */}
        
        <Obstacles />
        <EditableObstacles />
        <SatelliteModel />
        <Trajectory />
        <PlannedPath />
        
        <GizmoHelper alignment="top-right" margin={[80, 80]}>
          <GizmoViewport axisColors={['red', 'green', 'blue']} labelColor="black" />
        </GizmoHelper>
      </Canvas>
    </div>
  );
}

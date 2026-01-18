import { useEffect, useRef, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stars, GizmoHelper, GizmoViewcube } from '@react-three/drei';
import { Vector3 } from 'three';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';
import { TargetMarker } from './Earth';
import { SatelliteModel } from './SatelliteModel';
import { CameraManager } from './CameraManager';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { Trajectory } from './Trajectory';
import { PlannedPath } from './PlannedPath';
import { TargetGuides } from './TargetGuides';
import { CanvasRegistrar } from './CanvasRegistrar';
import { useCameraStore } from '../store/cameraStore';

function Obstacles() {
  const [params, setParams] = useState<{
    obstacles: TelemetryData['obstacles'], 
    targetPos: TelemetryData['target_position'],
    targetOri: TelemetryData['target_orientation'],
    targetQuat?: TelemetryData['target_quaternion'],
    scanObject?: TelemetryData['scan_object']
  } | null>(null);

  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       if (!d || !d.target_position) return;
       // Check NaNs
       if (d.target_position.some(v => !Number.isFinite(v))) return;
       
       setParams({ 
         obstacles: d.obstacles || [], 
         targetPos: d.target_position,
         targetOri: d.target_orientation || [0,0,0],
         targetQuat: d.target_quaternion,
         scanObject: d.scan_object
       });
    });
    return () => { unsub(); };
  }, []);

  if (!params) return null;

  return (
    <group>
      <TargetMarker
        position={params.targetPos}
        orientation={params.targetOri}
        quaternion={params.targetQuat}
      />
      {params.scanObject && params.scanObject.type === 'cylinder' && (
        <group
          position={new Vector3(...params.scanObject.position)}
          rotation={params.scanObject.orientation as [number, number, number]}
        >
          <mesh rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry
              args={[
                params.scanObject.radius,
                params.scanObject.radius,
                params.scanObject.height,
                32,
              ]}
            />
            <meshStandardMaterial color="#ff4444" transparent opacity={0.3} wireframe />
          </mesh>
        </group>
      )}
      {params.obstacles.map((obs, i) => (
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
  const controlsRef = useRef<OrbitControlsImpl | null>(null);
  const setControls = useCameraStore(s => s.setControls);

  return (
    <div className="w-full h-full bg-slate-900">
      <Canvas shadows camera={{ position: [5, 5, 5], fov: 45 }}>
        <CanvasRegistrar />
        <CameraManager mode={viewMode} />
        <OrbitControls 
          ref={(node) => {
            controlsRef.current = node;
            if (node) setControls(node);
          }} 
          makeDefault 
        />
        
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
        
        {/* Grid Removed */}
        
        <Obstacles />
        <SatelliteModel />
        <TargetGuides />
        <Trajectory />
        <PlannedPath />
        
        <GizmoHelper alignment="top-right" margin={[80, 80]}>
          <GizmoViewcube />
        </GizmoHelper>
        
      </Canvas>
    </div>
  );
}

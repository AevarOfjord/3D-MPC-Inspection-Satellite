import { memo, useCallback, useEffect, useRef, useState, Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stars, GizmoHelper, GizmoViewcube } from '@react-three/drei';
import { Vector3 } from 'three';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';
import { ReferenceMarker } from './Earth';
import { SatelliteModel } from './SatelliteModel';
import { StarlinkModel } from './StarlinkModel';
import { CustomMeshModel } from './CustomMeshModel';
import { CameraManager } from './CameraManager';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { Trajectory } from './Trajectory';
import { PlannedPath } from './PlannedPath';
import { CanvasRegistrar } from './CanvasRegistrar';
import { useCameraStore } from '../store/cameraStore';
import { useTelemetryStore } from '../store/telemetryStore';

function FinalStateMarker() {
  const finalState = useTelemetryStore(s => s.playbackFinalState);
  if (!finalState) return null;
  return (
    <ReferenceMarker 
      position={finalState.position} 
      orientation={finalState.reference_orientation} 
      quaternion={finalState.quaternion}
      color="#4ade80" 
    />
  );
}

function Obstacles() {
  const [params, setParams] = useState<{
    obstacles: TelemetryData['obstacles'], 
    referencePos: TelemetryData['reference_position'],
    referenceOri: TelemetryData['reference_orientation'],
    referenceQuat?: TelemetryData['reference_quaternion'],
    scanObject?: TelemetryData['scan_object']
  } | null>(null);

  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       if (!d || !d.reference_position) return;
       // Check NaNs
       if (d.reference_position.some(v => !Number.isFinite(v))) return;
       
       setParams({ 
         obstacles: d.obstacles || [], 
         referencePos: d.reference_position,
         referenceOri: d.reference_orientation || [0,0,0],
         referenceQuat: d.reference_quaternion,
         scanObject: d.scan_object
       });
    });
    return () => { unsub(); };
  }, []);

  if (!params) return null;

  return (
    <group>
      <ReferenceMarker
        position={params.referencePos}
        orientation={params.referenceOri}
        quaternion={params.referenceQuat}
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
      {params.scanObject && params.scanObject.type === 'starlink' && (
        <Suspense fallback={null}>
          <StarlinkModel
            position={params.scanObject.position}
            orientation={params.scanObject.orientation}
          />
        </Suspense>
      )}
      {params.scanObject && params.scanObject.type === 'mesh' && params.scanObject.obj_path && (
        <Suspense fallback={null}>
          <CustomMeshModel
            objPath={params.scanObject.obj_path}
            position={params.scanObject.position}
            orientation={params.scanObject.orientation}
          />
        </Suspense>
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

export const Viewport = memo(function Viewport({ viewMode }: ViewportProps) {
  const controlsRef = useRef<OrbitControlsImpl | null>(null);
  const setControls = useCameraStore(s => s.setControls);

  const handleControlsRef = useCallback((node: OrbitControlsImpl | null) => {
    controlsRef.current = node;
    if (node) setControls(node);
  }, [setControls]);

  return (
    <div className="w-full h-full bg-slate-900">
      <Canvas shadows camera={{ position: [5, 5, 5], fov: 45, near: 0.01, far: 1_000_000 }}>
        <CanvasRegistrar />
        <CameraManager mode={viewMode} />
        <OrbitControls 
          ref={handleControlsRef} 
          makeDefault 
        />
        
        {/* Environment */}
        <color attach="background" args={['#0b1020']} />
        <Stars radius={100} depth={50} count={5000} factor={4.5} saturation={0} fade speed={1} />
        
        {/* Lighting */}
        <ambientLight intensity={0.6} />
        <directionalLight 
          position={[10, 10, 5]} 
          intensity={1.8} 
          castShadow 
          shadow-mapSize={[1024, 1024]} 
        />
        <hemisphereLight args={['#c4d2ff', '#1b2333', 0.35]} />
        
        {/* Grid Removed */}
        
        <Obstacles />
        <SatelliteModel />
        <Trajectory />
        <PlannedPath />
        <FinalStateMarker />
        
        <GizmoHelper alignment="top-right" margin={[80, 80]}>
          <GizmoViewcube faces={['Front', 'Back', 'Right', 'Left', 'Top', 'Bottom']} />
        </GizmoHelper>
        
      </Canvas>
    </div>
  );
});

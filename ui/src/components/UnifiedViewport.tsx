import { useRef, useCallback, Suspense, useState, useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import { TrackballControls, Stars, GizmoHelper, GizmoViewcube, TransformControls, Grid } from '@react-three/drei';
import type { TrackballControls as TrackballControlsImpl } from 'three-stdlib';
import * as THREE from 'three';
import { useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

import { CameraManager } from './CameraManager';
import { CanvasRegistrar } from './CanvasRegistrar';
import { useCameraStore } from '../store/cameraStore';

import { SatelliteModel } from './SatelliteModel';
import { TargetMarker } from './Earth';
import { Trajectory } from './Trajectory';
import { PlannedPath } from './PlannedPath';
import { TargetGuides } from './TargetGuides';
// FinalStateMarker and Obstacles are not exported from Viewport.tsx, so we don't import them.
// LiveObstaclesRender handles obstacles internally now.
// Note: Created duplicate export for Obstacles in Viewport.tsx or extracting it? 
// For now, I'll assume Viewport.tsx exports its internal components or I replicate them.
// Actually, Viewport.tsx defines them internally. I should probably move them to separate files or duplicate logic.
// Let's duplicate logic for LiveObstacles here for safety/cleanliness or assume Viewport Refactor.
// To save time, I will reimplement LiveObstacles here using telemetry.

// --- Live Telemetry Components ---
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { StarlinkModel } from './StarlinkModel';
import { CustomMeshModel } from './CustomMeshModel';
import { HudPanel } from './HudComponents';
import { Move3d, Rotate3d, MousePointer2 } from 'lucide-react';
import type { useMissionBuilder } from '../hooks/useMissionBuilder';
import { EditableTrajectory } from './EditableTrajectory';
import { ConstraintVisualizer } from './ConstraintVisualizer';

function LiveObstaclesRender() {
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
          position={new THREE.Vector3(...params.scanObject.position)}
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
        <mesh key={i} position={new THREE.Vector3(...obs.position)}>
          <sphereGeometry args={[obs.radius, 32, 32]} />
          <meshStandardMaterial color="#ff4444" transparent opacity={0.3} wireframe />
        </mesh>
      ))}
    </group>
  );
}

// --- Plan Mode Components ---

function Model({ url }: { url: string }) {
  const obj = useLoader(OBJLoader, url);
  return <primitive object={obj} />;
}

function SatellitePreview({ position, rotation }: { position: [number, number, number]; rotation: [number, number, number] }) {
    const euler = new THREE.Euler(
        (rotation[0] * Math.PI) / 180,
        (rotation[1] * Math.PI) / 180,
        (rotation[2] * Math.PI) / 180
    );
    return (
        <group position={position} rotation={euler}>
            <mesh>
                <boxGeometry args={[0.5, 0.5, 0.5]} />
                <meshStandardMaterial color="#fbbf24" metalness={0.8} roughness={0.2} />
            </mesh>
            <axesHelper args={[1]} />
        </group>
    );
}

// TrajectoryPath removed as it is replaced by EditableTrajectory

// --- Main Unified Viewport ---

interface UnifiedViewportProps {
    mode: 'monitor' | 'plan';
    viewMode: 'free' | 'chase' | 'top';
    builderState?: ReturnType<typeof useMissionBuilder>['state'];
    builderActions?: ReturnType<typeof useMissionBuilder>['actions'];
}

export function UnifiedViewport({ mode, viewMode, builderState, builderActions }: UnifiedViewportProps) {
  const controlsRef = useRef<TrackballControlsImpl | null>(null);
  const setControls = useCameraStore(s => s.setControls);
  const [hoveredPoint, setHoveredPoint] = useState<[number, number, number] | null>(null);

  const handleControlsRef = useCallback((node: TrackballControlsImpl | null) => {
    controlsRef.current = node;
    if (node) setControls(node as any); // Type assertion for generic store
  }, [setControls]);

  return (
    <div className="w-full h-full bg-slate-950 relative">
      <Canvas shadows camera={{ position: [8, 8, 8], fov: 45 }}>
        <CanvasRegistrar />
        {/* Only use CameraManager in Monitor mode or if not in editing mode? 
            Actually, CameraManager handles 'chase' view.
            In Plan mode, we usually want 'free' view.
        */}
        <CameraManager mode={viewMode} />
        
        <TrackballControls 
          ref={handleControlsRef} 
          makeDefault 
          enabled={!builderState?.selectedObjectId} // Disable orbit when dragging gizmo
          rotateSpeed={4.0}
        />
        
        {/* Environment */}
        <color attach="background" args={['#050510']} />
        <Stars radius={100} depth={50} count={5000} factor={4} saturation={0} fade speed={1} />
        <ambientLight intensity={0.5} />
        <directionalLight position={[10, 10, 5]} intensity={1.5} castShadow />
        


        {mode === 'monitor' && (
            <>
                <LiveObstaclesRender />
                <SatelliteModel />
                <TargetGuides />
                <Trajectory />
                <PlannedPath />
                {/* FinalStateMarker removed for brevity or need import */}
            </>
        )}

        {mode === 'plan' && builderState && builderActions && (
            <Suspense fallback={null}>
                <Grid infiniteGrid sectionColor="#444" cellColor="#222" fadeDistance={30} />
                
                {/* Editable Content */}
                <group>
                     {/* Satellite */}
                     <TransformControls 
                        object={undefined} 
                        mode={builderState.transformMode}
                        enabled={builderState.selectedObjectId === 'satellite'}
                        showX={builderState.selectedObjectId === 'satellite'}
                        showY={builderState.selectedObjectId === 'satellite'}
                        showZ={builderState.selectedObjectId === 'satellite'}
                        onObjectChange={(e) => builderActions.handleObjectTransform('satellite', e)}
                     >
                        <group onClick={(e) => { e.stopPropagation(); builderActions.setSelectedObjectId('satellite'); }}>
                            <SatellitePreview position={builderState.startPosition} rotation={builderState.startAngle} />
                        </group>
                    </TransformControls>

                    {/* Target */}
                    <TransformControls
                        mode={builderState.transformMode}
                        enabled={builderState.selectedObjectId === 'target'}
                        showX={builderState.selectedObjectId === 'target'}
                        showY={builderState.selectedObjectId === 'target'}
                        showZ={builderState.selectedObjectId === 'target'}
                        onObjectChange={(e) => builderActions.handleObjectTransform('target', e)}
                    >
                         <group 
                            position={builderState.objectPosition} 
                            rotation={[
                                builderState.objectAngle[0]*Math.PI/180, 
                                builderState.objectAngle[1]*Math.PI/180, 
                                builderState.objectAngle[2]*Math.PI/180
                            ]}
                            onClick={(e) => { e.stopPropagation(); builderActions.setSelectedObjectId('target'); }}
                         >
                            {builderState.modelUrl ? <Model url={builderState.modelUrl} /> : (
                                <mesh>
                                    <boxGeometry args={[1, 1, 1]} />
                                    <meshStandardMaterial color="#64748b" wireframe />
                                </mesh>
                            )}
                            <axesHelper args={[2]} />
                         </group>
                    </TransformControls>

                    {/* Obstacles */}
                    {builderState.obstacles.map((obs, i) => (
                        <TransformControls
                            key={i}
                            mode="translate"
                            enabled={builderState.selectedObjectId === `obstacle-${i}`}
                            showX={builderState.selectedObjectId === `obstacle-${i}`}
                            showY={builderState.selectedObjectId === `obstacle-${i}`}
                            showZ={builderState.selectedObjectId === `obstacle-${i}`}
                            onObjectChange={(e) => builderActions.handleObjectTransform(`obstacle-${i}`, e)}
                        >
                            <mesh 
                                position={obs.position} 
                                onClick={(e) => { e.stopPropagation(); builderActions.setSelectedObjectId(`obstacle-${i}`); }}
                            >
                                <sphereGeometry args={[obs.radius, 16, 16]} />
                                <meshStandardMaterial color="#ef4444" transparent opacity={0.4} wireframe />
                            </mesh>
                        </TransformControls>
                    ))}

                    {/* Advanced Path Builder */}
                    <EditableTrajectory 
                        points={builderState.previewPath} 
                        onHover={setHoveredPoint} 
                        builderActions={builderActions}
                        selectedId={builderState.selectedObjectId}
                    />
                    <ConstraintVisualizer points={builderState.previewPath} />
                </group>
            </Suspense>
        )}
        <GizmoHelper alignment="top-right" margin={[80, 80]} key={`gizmo-${mode}`}>
           <GizmoViewcube />
        </GizmoHelper>
      </Canvas>
      
      {/* Plan Mode Overlay Controls */}
      {mode === 'plan' && builderState && builderActions && (
          <>
            <div className="absolute top-4 left-4 bg-slate-900/80 backdrop-blur rounded p-2 flex gap-2 border border-slate-700 z-10">
                <button 
                    onClick={() => builderActions.setTransformMode('translate')}
                    className={`p-2 rounded ${builderState.transformMode === 'translate' ? 'bg-cyan-500/20 text-cyan-400' : 'text-slate-400 hover:text-white'}`}
                >
                    <Move3d size={20} />
                </button>
                <button 
                    onClick={() => builderActions.setTransformMode('rotate')}
                    className={`p-2 rounded ${builderState.transformMode === 'rotate' ? 'bg-cyan-500/20 text-cyan-400' : 'text-slate-400 hover:text-white'}`}
                >
                    <Rotate3d size={20} />
                </button>
                <div className="w-px bg-slate-700 mx-1" />
                <button 
                    onClick={() => builderActions.setSelectedObjectId(null)}
                    className="p-2 text-slate-400 hover:text-white"
                >
                    <MousePointer2 size={20} />
                </button>
            </div>
            
            {hoveredPoint && (
                <div className="absolute bottom-4 left-4 pointer-events-none z-10">
                    <HudPanel className="text-xs font-mono">
                         <div className="text-cyan-400 font-bold mb-1">WAYPOINT</div>
                         <div>X: {hoveredPoint[0].toFixed(2)}</div>
                         <div>Y: {hoveredPoint[1].toFixed(2)}</div>
                         <div>Z: {hoveredPoint[2].toFixed(2)}</div>
                    </HudPanel>
                </div>
            )}
          </>
      )}
    </div>
  );
}

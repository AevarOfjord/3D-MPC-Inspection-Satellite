import { useRef, useCallback, Suspense, useState, useEffect, useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { TrackballControls, Stars, GizmoHelper, GizmoViewport } from '@react-three/drei';
import type { TrackballControls as TrackballControlsImpl } from 'three-stdlib';
import * as THREE from 'three';
import { useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

import { CameraManager } from './CameraManager';
import { CanvasRegistrar } from './CanvasRegistrar';
import { useCameraStore } from '../store/cameraStore';

import { SatelliteModel } from './SatelliteModel';
import { ReferenceMarker } from './Earth';
import { Trajectory } from './Trajectory';
import { PlannedPath } from './PlannedPath';
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
import type { useMissionBuilder } from '../hooks/useMissionBuilder';
import { EditableTrajectory } from './EditableTrajectory';
import { ConstraintVisualizer } from './ConstraintVisualizer';
import { OrbitSnapshotLayer } from './OrbitSnapshotLayer';
import { SolarSystemLayer } from './SolarSystemLayer';
import { ORBIT_SCALE } from '../data/orbitSnapshot';

const PLAN_SCALE = 0.1; // meters -> scene units for planner-local visuals

function LiveObstaclesRender() {
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

function SatellitePreview({ position, rotation, sceneScale }: { position: [number, number, number]; rotation: [number, number, number]; sceneScale: number }) {
    const euler = new THREE.Euler(
        (rotation[0] * Math.PI) / 180,
        (rotation[1] * Math.PI) / 180,
        (rotation[2] * Math.PI) / 180
    );
    return (
        <group position={position} rotation={euler}>
            <mesh>
                <boxGeometry args={[0.3 * sceneScale, 0.3 * sceneScale, 0.3 * sceneScale]} />
                <meshStandardMaterial color="#fbbf24" metalness={0.8} roughness={0.2} />
            </mesh>
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
    orbitVisibility?: Record<string, boolean>;
}

export function UnifiedViewport({ mode, viewMode, builderState, builderActions, orbitVisibility }: UnifiedViewportProps) {
  const controlsRef = useRef<TrackballControlsImpl | null>(null);
  const setControls = useCameraStore(s => s.setControls);
  const requestFocus = useCameraStore(s => s.requestFocus);
  const [hoveredPoint, setHoveredPoint] = useState<[number, number, number] | null>(null);
  const scaleToScene = useCallback(
    (vec: [number, number, number]) => [vec[0] * ORBIT_SCALE, vec[1] * ORBIT_SCALE, vec[2] * ORBIT_SCALE] as [number, number, number],
    []
  );
  const targetPosition = useMemo<[number, number, number] | null>(() => {
    if (!builderState?.segments) return null;
    const scan = builderState.segments.find((seg) => seg.type === 'scan') as
      | { target_pose?: { position: [number, number, number] } }
      | undefined;
    return scan?.target_pose?.position ?? null;
  }, [builderState?.segments]);
  const hasTarget = Boolean(targetPosition);
  const planAnchor = (targetPosition ?? [0, 0, 0]) as [number, number, number];
  const planGroupPosition = hasTarget ? scaleToScene(planAnchor) : ([0, 0, 0] as [number, number, number]);
  const planScale = hasTarget ? PLAN_SCALE : ORBIT_SCALE;
  const toPlanLocal = useCallback(
    (vec: [number, number, number]) => {
      if (!hasTarget) return scaleToScene(vec);
      return [
        (vec[0] - planAnchor[0]) * PLAN_SCALE,
        (vec[1] - planAnchor[1]) * PLAN_SCALE,
        (vec[2] - planAnchor[2]) * PLAN_SCALE,
      ] as [number, number, number];
    },
    [hasTarget, planAnchor, scaleToScene]
  );

  const handleControlsRef = useCallback((node: TrackballControlsImpl | null) => {
    controlsRef.current = node;
    if (node) setControls(node as any); // Type assertion for generic store
  }, [setControls]);

  return (
    <div className="w-full h-full bg-slate-950 relative">
      <Canvas shadows camera={{ position: [8, 8, 8], fov: 45, near: 0.01, far: 1_000_000 }}>
        <CanvasRegistrar />
        {/* Only use CameraManager in Monitor mode or if not in editing mode? 
            Actually, CameraManager handles 'chase' view.
            In Plan mode, we usually want 'free' view.
        */}
        <CameraManager mode={viewMode} />
        
        <TrackballControls 
          ref={handleControlsRef} 
          makeDefault 
          enabled
          rotateSpeed={4.0}
        />
        
        {/* Environment */}
        <color attach="background" args={['#0b1020']} />
        <Stars radius={100} depth={50} count={5000} factor={4.5} saturation={0} fade speed={1} />
        <ambientLight intensity={0.8} />
        <directionalLight position={[10, 10, 5]} intensity={1.8} castShadow />
        <hemisphereLight args={['#c4d2ff', '#1b2333', 0.35]} />
        


        {mode === 'monitor' && (
            <>
                <SolarSystemLayer />
                <LiveObstaclesRender />
                <SatelliteModel />
                <Trajectory />
                <PlannedPath />
                {/* FinalStateMarker removed for brevity or need import */}
            </>
        )}

        {mode === 'plan' && builderState && builderActions && (
            <Suspense fallback={null}>
                {/* Grid Removed by User Request */}
                
                {/* Editable Content */}
                <group>
                     <SolarSystemLayer />
                     <OrbitSnapshotLayer
                       selectedTargetId={builderState.selectedOrbitTargetId}
                       orbitVisibility={orbitVisibility}
                       onSelectTarget={(targetId, positionMeters, positionScene) => {
                         builderActions.assignScanTarget(targetId, positionMeters);
                         requestFocus(positionScene);
                       }}
                     />

                     {/* Planner-local visuals anchored to the target */}
                     <group position={planGroupPosition}>
                        {/* Satellite */}
                        <SatellitePreview
                          position={toPlanLocal(builderState.startPosition)}
                          rotation={builderState.startAngle}
                          sceneScale={planScale}
                        />

                        {/* Reference */}
                        <group 
                          position={toPlanLocal(builderState.referencePosition)} 
                          rotation={[
                              builderState.referenceAngle[0]*Math.PI/180, 
                              builderState.referenceAngle[1]*Math.PI/180, 
                              builderState.referenceAngle[2]*Math.PI/180
                          ]}
                        >
                          {builderState.modelUrl ? <Model url={builderState.modelUrl} /> : (
                              <mesh>
                                  <boxGeometry args={[0.6 * planScale, 0.6 * planScale, 0.6 * planScale]} />
                                  <meshStandardMaterial color="#64748b" wireframe />
                              </mesh>
                          )}
                          <axesHelper args={[1 * planScale]} />
                        </group>

                        {/* Obstacles */}
                        {builderState.obstacles.map((obs, i) => (
                            <mesh 
                                key={i}
                                position={toPlanLocal(obs.position)} 
                            >
                                <sphereGeometry args={[obs.radius * planScale, 16, 16]} />
                                <meshStandardMaterial color="#ef4444" transparent opacity={0.4} wireframe />
                            </mesh>
                        ))}

                        {/* Advanced Path Builder */}
                        <EditableTrajectory 
                            points={builderState.previewPath.map(toPlanLocal)} 
                            onHover={(point) => {
                              if (!point) {
                                setHoveredPoint(null);
                              } else if (hasTarget) {
                                setHoveredPoint([point[0] / PLAN_SCALE, point[1] / PLAN_SCALE, point[2] / PLAN_SCALE]);
                              } else {
                                setHoveredPoint([point[0] / ORBIT_SCALE, point[1] / ORBIT_SCALE, point[2] / ORBIT_SCALE]);
                              }
                            }} 
                            builderActions={builderActions}
                            selectedId={builderState.selectedObjectId}
                            sceneScale={planScale}
                        />
                        <ConstraintVisualizer points={builderState.previewPath.map(toPlanLocal)} />
                     </group>
                </group>
            </Suspense>
        )}
        <GizmoHelper alignment="top-right" margin={[80, 80]} key={`gizmo-${mode}`}>
           <GizmoViewport axisColors={['red', '#39ff14', '#00f0ff']} labelColor="white" />
        </GizmoHelper>
      </Canvas>
      
      {mode === 'plan' && hoveredPoint && (
        <div className="absolute bottom-4 left-4 pointer-events-none z-10">
            <HudPanel className="text-xs font-mono">
                 <div className="text-cyan-400 font-bold mb-1">WAYPOINT{hasTarget ? ' (TARGET FRAME)' : ''}</div>
                 <div>X: {hoveredPoint[0].toFixed(2)}</div>
                 <div>Y: {hoveredPoint[1].toFixed(2)}</div>
                 <div>Z: {hoveredPoint[2].toFixed(2)}</div>
            </HudPanel>
        </div>
      )}
    </div>
  );
}

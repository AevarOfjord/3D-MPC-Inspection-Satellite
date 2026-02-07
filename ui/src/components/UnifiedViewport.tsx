import { useRef, useCallback, Suspense, useState, useEffect, useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { TrackballControls, Stars, GizmoHelper, GizmoViewport, Line, useGLTF } from '@react-three/drei';
import type { TrackballControls as TrackballControlsImpl } from 'three-stdlib';
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';

import { CameraManager } from './CameraManager';
import { CanvasRegistrar } from './CanvasRegistrar';
import { useCameraStore } from '../store/cameraStore';
import { useTelemetryStore } from '../store/telemetryStore';

import { SatelliteModel } from './SatelliteModel';
import { ReferenceMarker } from './Earth';
import { Trajectory } from './Trajectory';
import { PlannedPath } from './PlannedPath';
// LiveObstaclesRender handles obstacles internally.
// To save time, I will reimplement LiveObstacles here using telemetry.

// --- Live Telemetry Components ---
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { StarlinkModel } from './StarlinkModel';
import { ISSModel } from './ISSModel';
import { CustomMeshModel } from './CustomMeshModel';
import { HudPanel } from './HudComponents';
import type { useMissionBuilder } from '../hooks/useMissionBuilder';
import { EditableTrajectory } from './EditableTrajectory';
import { ConstraintVisualizer } from './ConstraintVisualizer';
import { OrbitSnapshotLayer } from './OrbitSnapshotLayer';
import { SolarSystemLayer } from './SolarSystemLayer';
import { SplineControlGizmos } from './SplineControlGizmos';
import { ORBIT_SCALE, EARTH_RADIUS_M, orbitSnapshot } from '../data/orbitSnapshot';
import { API_BASE_URL } from '../config/endpoints';

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

function ObjWithMtl({ objPath }: { objPath: string }) {
  const [object, setObject] = useState<THREE.Object3D | null>(null);

  useEffect(() => {
    if (!objPath) {
      setObject(null);
      return;
    }

    let cancelled = false;
    const objUrl = `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(objPath)}`;
    const mtlPath = objPath.replace(/\.obj$/i, '.mtl');
    const mtlUrl = `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(mtlPath)}`;

    const objLoader = new OBJLoader();
    const applyFallback = () => {
      objLoader.load(objUrl, (obj) => {
        if (cancelled) return;
        obj.traverse((child) => {
          if ((child as THREE.Mesh).isMesh) {
            const mesh = child as THREE.Mesh;
            mesh.material = new THREE.MeshStandardMaterial({
              color: '#8b8b8b',
              metalness: 0.2,
              roughness: 0.7,
            });
          }
        });
        setObject(obj);
      });
    };

    const mtlLoader = new MTLLoader();
    mtlLoader.load(
      mtlUrl,
      (materials) => {
        if (cancelled) return;
        materials.preload();
        objLoader.setMaterials(materials);
        objLoader.load(objUrl, (obj) => {
          if (cancelled) return;
          setObject(obj);
        });
      },
      undefined,
      () => applyFallback()
    );

    return () => {
      cancelled = true;
    };
  }, [objPath]);

  if (!object) return null;
  return <primitive object={object} />;
}

function resolvePreviewModel(modelPath?: string) {
  if (!modelPath) return null;
  const lower = modelPath.toLowerCase();
  if (lower.includes('starlink')) {
    return (
      <StarlinkModel
        position={[0, 0, 0]}
        orientation={[0, 0, 0]}
        realSpanMeters={11}
        scale={1}
        pivot="origin"
      />
    );
  }
  if (lower.includes('iss')) {
    return (
      <ISSModel
        position={[0, 0, 0]}
        orientation={[0, 0, 0]}
        realSpanMeters={109}
        scale={1}
      />
    );
  }
  return null;
}

function computeFacingEuler(
  position: [number, number, number],
  baseAxis: [number, number, number] = [0, 0, -1],
  fallback: [number, number, number] = [0, 0, 0]
) {
  const toEarth = new THREE.Vector3(-position[0], -position[1], -position[2]);
  if (toEarth.lengthSq() < 1e-8) return fallback;
  toEarth.normalize();
  const base = new THREE.Vector3(baseAxis[0], baseAxis[1], baseAxis[2]);
  if (base.lengthSq() < 1e-8) return fallback;
  base.normalize();
  const quat = new THREE.Quaternion().setFromUnitVectors(base, toEarth);
  const euler = new THREE.Euler().setFromQuaternion(quat);
  return [euler.x, euler.y, euler.z] as [number, number, number];
}

function OrbitObjectsLayer() {
  const orbitObjects = useMemo(
    () =>
      orbitSnapshot.objects.map((obj) => ({
        ...obj,
        position: [
          obj.position_m[0] * ORBIT_SCALE,
          obj.position_m[1] * ORBIT_SCALE,
          obj.position_m[2] * ORBIT_SCALE,
        ] as [number, number, number],
        scaleBoost: obj.visual_scale_boost ?? 1,
        resolvedOrientation: obj.align_to_earth
          ? computeFacingEuler(
              [
                obj.position_m[0] * ORBIT_SCALE,
                obj.position_m[1] * ORBIT_SCALE,
                obj.position_m[2] * ORBIT_SCALE,
              ],
              obj.base_axis ?? [0, 0, -1],
              obj.orientation ?? [0, 0, 0]
            )
          : (obj.orientation ?? [0, 0, 0]),
      })),
    []
  );

  return (
    <Suspense fallback={null}>
      {orbitObjects.map((obj) => {
        return obj.type === 'iss' ? (
          <ISSModel
            key={obj.id}
            position={obj.position}
            orientation={obj.resolvedOrientation}
            realSpanMeters={obj.real_span_m}
            scale={obj.scaleBoost}
          />
        ) : (
          <StarlinkModel
            key={obj.id}
            position={obj.position}
            orientation={obj.resolvedOrientation}
            realSpanMeters={obj.real_span_m}
            scale={obj.scaleBoost}
            pivot={obj.pivot}
          />
        );
      })}
    </Suspense>
  );
}

function OrbitRingsLayer() {
  const orbitObjects = useMemo(
    () =>
      orbitSnapshot.objects.map((obj) => ({
        id: obj.id,
        type: obj.type,
        position: [
          obj.position_m[0] * ORBIT_SCALE,
          obj.position_m[1] * ORBIT_SCALE,
          obj.position_m[2] * ORBIT_SCALE,
        ] as [number, number, number],
      })),
    []
  );

  const buildOrbitPoints = (radius: number, normal: THREE.Vector3, startPos: THREE.Vector3, segments = 2048) => {
    const safeNormal = normal.clone().normalize();
    const axisA = startPos.clone().normalize();
    const axisB = new THREE.Vector3().crossVectors(safeNormal, axisA).normalize();
    const points: [number, number, number][] = [];
    for (let i = 0; i <= segments; i += 1) {
      const t = (i / segments) * Math.PI * 2;
      const cos = Math.cos(t);
      const sin = Math.sin(t);
      const p = axisA.clone().multiplyScalar(radius * cos).add(axisB.clone().multiplyScalar(radius * sin));
      points.push([p.x, p.y, p.z]);
    }
    return points;
  };

  return (
    <group>
      {orbitObjects.map((obj) => {
        const pos = new THREE.Vector3(obj.position[0], obj.position[1], obj.position[2]);
        const orbitRadius = pos.length();
        let normal = new THREE.Vector3().crossVectors(pos, new THREE.Vector3(0, 1, 0));
        if (normal.lengthSq() < 1e-6) {
          normal = new THREE.Vector3().crossVectors(pos, new THREE.Vector3(1, 0, 0));
        }
        const points = buildOrbitPoints(orbitRadius, normal, pos, 2048);
        return (
          <Line
            key={`${obj.id}-orbit`}
            points={points}
            color={obj.type === 'iss' ? '#38bdf8' : '#a78bfa'}
            lineWidth={1.5}
            transparent
            opacity={0.6}
          />
        );
      })}
    </group>
  );
}

function EarthLayer() {
  const earthRadius = EARTH_RADIUS_M * ORBIT_SCALE;
  const earthGltf = useGLTF('/OBJ_files/Earth/Earth.glb');
  const earthScale = useMemo(() => {
    const box = new THREE.Box3().setFromObject(earthGltf.scene);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    if (!Number.isFinite(maxDim) || maxDim <= 0) return 1;
    return (earthRadius * 2) / maxDim;
  }, [earthGltf, earthRadius]);

  return (
    <Suspense fallback={null}>
      <primitive object={earthGltf.scene} scale={[earthScale, earthScale, earthScale]} />
      <mesh>
        <sphereGeometry args={[earthRadius * 1.02, 32, 32]} />
        <meshStandardMaterial color="#4cc9f0" transparent opacity={0.08} />
      </mesh>
    </Suspense>
  );
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
                <boxGeometry args={[0.3, 0.3, 0.3]} />
                <meshStandardMaterial color="#fbbf24" metalness={0.8} roughness={0.2} />
            </mesh>
        </group>
    );
}

// TrajectoryPath removed as it is replaced by EditableTrajectory

// --- Main Unified Viewport ---

interface UnifiedViewportProps {
    mode: 'viewer' | 'mission' | 'scan';
    viewMode: 'free' | 'chase' | 'top';
    builderState?: ReturnType<typeof useMissionBuilder>['state'];
    builderActions?: ReturnType<typeof useMissionBuilder>['actions'];
}

export function UnifiedViewport({ mode, viewMode, builderState, builderActions }: UnifiedViewportProps) {
  const controlsRef = useRef<TrackballControlsImpl | null>(null);
  const setControls = useCameraStore(s => s.setControls);
  const requestFocus = useCameraStore(s => s.requestFocus);
  const latestTelemetry = useTelemetryStore(s => s.latest);
  const [hoveredPoint, setHoveredPoint] = useState<[number, number, number] | null>(null);
  const isPlanning = mode !== 'viewer';
  const showOrbitLayer = mode === 'mission';
  const [viewerOrigin, setViewerOrigin] = useState<[number, number, number]>([0, 0, 0]);

  useEffect(() => {
    if (mode !== 'viewer') return;
    const frameOrigin = latestTelemetry?.frame_origin;
    const scanPos = latestTelemetry?.scan_object?.position;
    const nextOrigin = (frameOrigin && frameOrigin.length === 3)
      ? frameOrigin
      : (scanPos && scanPos.length === 3 ? scanPos : null);
    if (nextOrigin) {
      setViewerOrigin([nextOrigin[0], nextOrigin[1], nextOrigin[2]]);
    }
  }, [
    mode,
    latestTelemetry?.frame_origin?.[0],
    latestTelemetry?.frame_origin?.[1],
    latestTelemetry?.frame_origin?.[2],
    latestTelemetry?.scan_object?.position?.[0],
    latestTelemetry?.scan_object?.position?.[1],
    latestTelemetry?.scan_object?.position?.[2],
  ]);
  // --- Floating Origin Logic ---
  // To prevent Z-fighting/Jitter at 6,000,000m, we shift the world so the active target is at (0,0,0).
  const sceneOrigin = useMemo(() => {
      let origin: [number, number, number] = [0, 0, 0];
      
      // In Plan mode, center on the selected target or start target
      if (isPlanning && builderState) {
          const targetId = builderState.selectedOrbitTargetId || builderState.startTargetId;
          if (targetId) {
             const obj = orbitSnapshot.objects.find(o => o.id === targetId);
             if (obj) origin = obj.position_m;
          } else {
             // If no target, maybe center on start position?
             // origin = builderState.startPosition; 
             // Better to keep Earth center if defining from scratch, unless zoomed in?
             // Let's stick to Target. If no target, Earth Center (0,0,0).
          }
      }
      return origin;
  }, [mode, builderState?.selectedOrbitTargetId, builderState?.startTargetId]);

  const scaleToScene = useCallback(
    (vec: [number, number, number]) => [
        (vec[0] - sceneOrigin[0]) * ORBIT_SCALE, 
        (vec[1] - sceneOrigin[1]) * ORBIT_SCALE, 
        (vec[2] - sceneOrigin[2]) * ORBIT_SCALE
    ] as [number, number, number],
    [sceneOrigin]
  );

  const initialCameraPosition = [
    (EARTH_RADIUS_M * 2.5 - sceneOrigin[0]) * ORBIT_SCALE,
    (EARTH_RADIUS_M * 0.9 - sceneOrigin[1]) * ORBIT_SCALE,
    (EARTH_RADIUS_M * 0.6 - sceneOrigin[2]) * ORBIT_SCALE,
  ] as [number, number, number];

  const handleControlsRef = useCallback((node: TrackballControlsImpl | null) => {
    controlsRef.current = node;
    // We might need to retarget controls if origin changes?
    if (node) {
        setControls(node as any); 
        // Reset target to 0,0,0 (which is now our scene origin)
        // node.target.set(0, 0, 0);
    }
  }, [setControls, sceneOrigin]);

  return (
    <div className="w-full h-full bg-slate-950 relative">
      <Canvas
        shadows
        gl={{ logarithmicDepthBuffer: true }}
        camera={{ position: initialCameraPosition, fov: 45, near: 0.1, far: 2_000_000_000_000 }}
      >
        <CanvasRegistrar />
        {/* Only use CameraManager in Monitor mode or if not in editing mode? 
            Actually, CameraManager handles 'chase' view.
            In Plan mode, we usually want 'free' view.
        */}
        <CameraManager mode={viewMode} origin={mode === 'viewer' ? viewerOrigin : [0, 0, 0]} />
        
        <TrackballControls 
          ref={handleControlsRef} 
          makeDefault 
          enabled
          rotateSpeed={4.0}
        />
        
        {/* Environment */}
        <color attach="background" args={['#1a2233']} />
        <Stars radius={EARTH_RADIUS_M * 50} depth={EARTH_RADIUS_M * 50} count={5000} factor={4.5} saturation={0} fade speed={1} />
        <ambientLight intensity={1.15} />
        <directionalLight position={[10, 10, 5]} intensity={1.8} castShadow />
        <hemisphereLight args={['#c4d2ff', '#1b2333', 0.5]} />
        


        {mode === 'viewer' && (
          <>
            <group position={[
              -viewerOrigin[0] * ORBIT_SCALE,
              -viewerOrigin[1] * ORBIT_SCALE,
              -viewerOrigin[2] * ORBIT_SCALE,
            ]}>
                <EarthLayer />
                <OrbitRingsLayer />
                <OrbitObjectsLayer />
                <LiveObstaclesRender />
                <SatelliteModel />
                {/* FinalStateMarker removed for brevity or need import */}
            </group>
            {/* Trajectory and PlannedPath rendered with floating origin subtraction to prevent jitter */}
            <Trajectory origin={viewerOrigin} />
            <PlannedPath origin={viewerOrigin} />
          </>
        )}

        {isPlanning && builderState && builderActions && (
            <Suspense fallback={null}>
                {/* Grid Removed by User Request */}
                
                {/* Editable Content */}
                <group>
                {showOrbitLayer && (
                    <group position={scaleToScene([0,0,0])}>
                         <SolarSystemLayer />
                         <OrbitSnapshotLayer
                           onSelectTarget={(targetId, positionMeters, positionScene, focusDistance) => {
                             // We need to pass the "Scene" position back, which is relative to the floating origin now.
                             // OrbitSnapshotLayer returns absolute scene position (scaled).
                             // We need to adjust it to be relative to the group shift? 
                             // No, OrbitSnapshotLayer thinks it's at P_abs. 
                             // It is rendered at P_abs + Shift.
                             // The click event returns P_abs.
                             // The Camera Focus needs P_render = P_abs + Shift.
                             // So we should adjust the positionScene passed back.
                             // Or easier: Just calculate it here.
                             
                             const obj = orbitSnapshot.objects.find(o => o.id === targetId);
                             if (obj) {
                                 const scenePos = scaleToScene(obj.position_m);
                                 builderActions.assignScanTarget(targetId, positionMeters);
                                 // Focus on the object in the *floating scene*
                                 requestFocus(scenePos, focusDistance);
                             }
                           }}
                         />
                    </group>
                )}

                     {/* Satellite */}
                     <group>
                        {(() => {
                             let posMeters = [...builderState.startPosition] as [number, number, number];
                             if (builderState.startFrame === 'LVLH' && builderState.startTargetId) {
                                  const target = orbitSnapshot.objects.find(o => o.id === builderState?.startTargetId);
                                  if (target) {
                                      posMeters = [
                                          target.position_m[0] + posMeters[0],
                                          target.position_m[1] + posMeters[1],
                                          target.position_m[2] + posMeters[2]
                                      ];
                                  }
                             }
                            return (
                                <SatellitePreview
                                    position={scaleToScene(posMeters)}
                                    rotation={builderState.startAngle}
                                />
                            );
                        })()}
                    </group>

                    {/* Reference */}
                    <group 
                      position={scaleToScene(builderState.referencePosition)} 
                      rotation={[
                          builderState.referenceAngle[0]*Math.PI/180, 
                          builderState.referenceAngle[1]*Math.PI/180, 
                          builderState.referenceAngle[2]*Math.PI/180
                      ]}
                    >
                      {builderState.modelPath ? (
                        resolvePreviewModel(builderState.modelPath) ?? (
                          <ObjWithMtl objPath={builderState.modelPath} />
                        )
                      ) : (
                        <mesh>
                          <boxGeometry args={[1, 1, 1]} />
                          <meshStandardMaterial color="#64748b" wireframe />
                        </mesh>
                      )}
                      <axesHelper args={[2]} />
                    </group>

                    {/* Obstacles */}
                    {builderState.obstacles.map((obs, i) => (
                        <mesh 
                            key={i}
                            position={scaleToScene(obs.position)} 
                        >
                            <sphereGeometry args={[obs.radius * ORBIT_SCALE, 16, 16]} />
                            <meshStandardMaterial color="#ef4444" transparent opacity={0.4} wireframe />
                        </mesh>
                    ))}

                    {/* Advanced Path Builder */}
                    <EditableTrajectory 
                        points={builderState.previewPath.map(scaleToScene)} 
                        onHover={(point) => {
                          if (!point) {
                            setHoveredPoint(null);
                          } else {
                            setHoveredPoint([point[0] / ORBIT_SCALE, point[1] / ORBIT_SCALE, point[2] / ORBIT_SCALE]);
                          }
                        }} 
                        builderActions={builderActions}
                        selectedId={builderState.selectedObjectId}
                        sceneScale={ORBIT_SCALE}
                        sceneOrigin={sceneOrigin}
                        fixedAnchor={scaleToScene(builderState.referencePosition)}
                    />

                    <ConstraintVisualizer points={builderState.previewPath.map(scaleToScene)} />

                    {/* Spline Controls */}
                    <SplineControlGizmos
                        controls={builderState.splineControls.map(c => ({
                            ...c,
                            position: scaleToScene(c.position)
                        }))}
                        onUpdate={(idx, next) => {
                             // Convert back to meters
                             const metersPos: [number, number, number] = [
                                 next.position[0] / ORBIT_SCALE,
                                 next.position[1] / ORBIT_SCALE,
                                 next.position[2] / ORBIT_SCALE
                             ];
                             builderActions.updateSplineControl(idx, { ...next, position: metersPos });
                        }}
                        onSelect={(idx) => builderActions.setSelectedObjectId(`spline-${idx}`)}
                        selectedId={builderState.selectedObjectId}
                    />
                </group>
            </Suspense>
        )}
        <GizmoHelper alignment="top-right" margin={[80, 80]} key={`gizmo-${mode}`}>
           <GizmoViewport axisColors={['red', '#39ff14', '#00f0ff']} labelColor="white" />
        </GizmoHelper>
      </Canvas>
      
      {isPlanning && hoveredPoint && (
        <div className="absolute bottom-4 left-4 pointer-events-none z-10">
            <HudPanel className="text-xs font-mono">
                 <div className="text-cyan-400 font-bold mb-1">WAYPOINT</div>
                 <div>X: {hoveredPoint[0].toFixed(2)}</div>
                 <div>Y: {hoveredPoint[1].toFixed(2)}</div>
                 <div>Z: {hoveredPoint[2].toFixed(2)}</div>
            </HudPanel>
        </div>
      )}
    </div>
  );
}

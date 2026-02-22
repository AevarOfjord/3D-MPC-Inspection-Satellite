import { lazy, useRef, useCallback, Suspense, useState, useEffect, useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { TrackballControls, Stars, GizmoHelper, GizmoViewport, Line, Text, TransformControls } from '@react-three/drei';
import type { TrackballControls as TrackballControlsImpl } from 'three-stdlib';
import * as THREE from 'three';

import { CameraManager } from './CameraManager';
import { CanvasRegistrar } from './CanvasRegistrar';
import { useCameraStore } from '../store/cameraStore';
import { useTelemetryStore } from '../store/telemetryStore';
import type { useMissionBuilder } from '../hooks/useMissionBuilder';
import { ORBIT_SCALE, EARTH_RADIUS_M, orbitSnapshot } from '../data/orbitSnapshot';
import { HudPanel } from './HudComponents';
import { LiveObstaclesRender } from './viewport/LiveObstacles';
import { ObjWithMtl } from './viewport/ObjModelLoader';
import { resolvePreviewModel } from './viewport/modelPreviewUtils';
import { OrbitObjectsLayer, OrbitRingsLayer, EarthLayer } from './viewport/OrbitLayers';
import { SatellitePreview, ReferenceModelFallback } from './viewport/SatellitePreview';

const SatelliteModel = lazy(() =>
  import('./SatelliteModel').then((m) => ({ default: m.SatelliteModel }))
);
const Trajectory = lazy(() => import('./Trajectory').then((m) => ({ default: m.Trajectory })));
const PlannedPath = lazy(() => import('./PlannedPath').then((m) => ({ default: m.PlannedPath })));
const EditableTrajectory = lazy(() =>
  import('./EditableTrajectory').then((m) => ({ default: m.EditableTrajectory }))
);
const ConstraintVisualizer = lazy(() =>
  import('./ConstraintVisualizer').then((m) => ({ default: m.ConstraintVisualizer }))
);
const OrbitSnapshotLayer = lazy(() =>
  import('./OrbitSnapshotLayer').then((m) => ({ default: m.OrbitSnapshotLayer }))
);
const SolarSystemLayer = lazy(() =>
  import('./SolarSystemLayer').then((m) => ({ default: m.SolarSystemLayer }))
);
const SplineControlGizmos = lazy(() =>
  import('./SplineControlGizmos').then((m) => ({ default: m.SplineControlGizmos }))
);


// --- Main Unified Viewport ---

interface UnifiedViewportProps {
    mode: 'viewer' | 'mission' | 'scan';
    viewMode: 'free' | 'chase' | 'top';
    builderState?: ReturnType<typeof useMissionBuilder>['state'];
    builderActions?: ReturnType<typeof useMissionBuilder>['actions'];
}

export function UnifiedViewport({
  mode,
  viewMode,
  builderState,
  builderActions,
}: UnifiedViewportProps) {
  const controlsRef = useRef<TrackballControlsImpl | null>(null);
  const setControls = useCameraStore(s => s.setControls);
  const requestFocus = useCameraStore(s => s.requestFocus);
  const latestTelemetry = useTelemetryStore(s => s.latest);
  const [hoveredPoint, setHoveredPoint] = useState<[number, number, number] | null>(null);
  const [hoveredPlannerPointId, setHoveredPlannerPointId] = useState<string | null>(null);
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
  const homeFocus = useMemo(() => {
    if (isPlanning && builderState) {
      const targetId = builderState.selectedOrbitTargetId || builderState.startTargetId;
      if (targetId) {
        const targetObj = orbitSnapshot.objects.find((o) => o.id === targetId);
        if (targetObj) {
          const spanMeters = targetObj.real_span_m ?? 4;
          return {
            target: scaleToScene(targetObj.position_m),
            distance: Math.max(spanMeters * 6, 4) * ORBIT_SCALE,
          };
        }
      }
      return {
        target: scaleToScene(builderState.referencePosition),
        distance: Math.max(8 * ORBIT_SCALE, 4),
      };
    }

    const viewerTarget = latestTelemetry?.reference_position ?? latestTelemetry?.scan_object?.position;
    if (viewerTarget && viewerTarget.length === 3) {
      return {
        target: [viewerTarget[0], viewerTarget[1], viewerTarget[2]] as [number, number, number],
        distance: Math.max(8 * ORBIT_SCALE, 4),
      };
    }

    return null;
  }, [
    builderState,
    isPlanning,
    latestTelemetry?.reference_position,
    latestTelemetry?.scan_object?.position,
    scaleToScene,
  ]);

  const handleControlsRef = useCallback((node: TrackballControlsImpl | null) => {
    controlsRef.current = node;
    // We might need to retarget controls if origin changes?
    if (node) {
        setControls(node as any);
        // Reset target to 0,0,0 (which is now our scene origin)
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
                <Suspense fallback={null}>
                  <SatelliteModel />
                </Suspense>
            </group>
            {/* Trajectory and PlannedPath rendered with floating origin subtraction to prevent jitter */}
            <Suspense fallback={null}>
              <Trajectory origin={viewerOrigin} />
              <PlannedPath origin={viewerOrigin} />
            </Suspense>
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
                        <Suspense fallback={<ReferenceModelFallback />}>
                          {resolvePreviewModel(builderState.modelPath) ?? (
                            <ObjWithMtl objPath={builderState.modelPath} />
                          )}
                        </Suspense>
                      ) : (
                        <mesh>
                          <boxGeometry args={[1, 1, 1]} />
                          <meshStandardMaterial color="#64748b" wireframe />
                        </mesh>
                      )}
                      <axesHelper args={[2]} />
                    </group>

                    {/* Obstacles */}
                    {builderState.obstacles.map((obs, i) => {
                      let obstacleMeters: [number, number, number] = [...obs.position];
                      if (builderState.startFrame === 'LVLH' && builderState.startTargetId) {
                        const target = orbitSnapshot.objects.find((o) => o.id === builderState.startTargetId);
                        if (target) {
                          obstacleMeters = [
                            target.position_m[0] + obs.position[0],
                            target.position_m[1] + obs.position[1],
                            target.position_m[2] + obs.position[2],
                          ];
                        }
                      }
                      return (
                        <mesh key={i} position={scaleToScene(obstacleMeters)}>
                          <sphereGeometry args={[Math.max(obs.radius, 0.1) * ORBIT_SCALE, 20, 20]} />
                          <meshStandardMaterial color="#ef4444" transparent opacity={0.5} wireframe />
                        </mesh>
                      );
                    })}

                    {/* Scan Project Authoring Overlays */}
                    {mode === 'scan' && builderState.scanProject?.scans?.length > 0 && (
                      <>
                        {builderState.scanProject.scans.map((scan: any, scanIdx: number) => {
                          const a = scan.plane_a as [number, number, number];
                          const b = scan.plane_b as [number, number, number];
                          const d = new THREE.Vector3(
                            b[0] - a[0],
                            b[1] - a[1],
                            b[2] - a[2]
                          );
                          const len = d.length();
                          const basis =
                            scan.axis === 'X'
                              ? new THREE.Vector3(1, 0, 0)
                              : scan.axis === 'Y'
                                ? new THREE.Vector3(0, 1, 0)
                                : new THREE.Vector3(0, 0, 1);
                          const bodyEuler = new THREE.Euler(
                            (builderState.referenceAngle[0] * Math.PI) / 180,
                            (builderState.referenceAngle[1] * Math.PI) / 180,
                            (builderState.referenceAngle[2] * Math.PI) / 180
                          );
                          const normal = basis.clone().applyEuler(bodyEuler).normalize();
                          const basisU =
                            scan.axis === 'X'
                              ? new THREE.Vector3(0, 1, 0)
                              : scan.axis === 'Y'
                                ? new THREE.Vector3(1, 0, 0)
                                : new THREE.Vector3(1, 0, 0);
                          const basisV =
                            scan.axis === 'X'
                              ? new THREE.Vector3(0, 0, 1)
                              : scan.axis === 'Y'
                                ? new THREE.Vector3(0, 0, 1)
                                : new THREE.Vector3(0, 1, 0);
                          const uAxis = basisU.clone().applyEuler(bodyEuler).normalize();
                          const vAxis = basisV.clone().applyEuler(bodyEuler).normalize();
                          const q = new THREE.Quaternion().setFromUnitVectors(
                            new THREE.Vector3(0, 0, 1),
                            normal
                          );
                          const qa: [number, number, number, number] = [q.x, q.y, q.z, q.w];
                          const planeSizeMeters = Math.max(1.5, len * 2.0 + 1.0);
                          const planeSizeScene = planeSizeMeters * ORBIT_SCALE;
                          const accentA = ['#f59e0b', '#a3e635', '#38bdf8', '#fb7185'][
                            scanIdx % 4
                          ];
                          const accentB = ['#22d3ee', '#86efac', '#fbbf24', '#f472b6'][
                            scanIdx % 4
                          ];
                          const isPlaneASelected =
                            builderState.selectedProjectScanPlaneHandle?.scanId === scan.id &&
                            builderState.selectedProjectScanPlaneHandle?.handle === 'a';
                          const isPlaneBSelected =
                            builderState.selectedProjectScanPlaneHandle?.scanId === scan.id &&
                            builderState.selectedProjectScanPlaneHandle?.handle === 'b';
                          const scanCenterPos: [number, number, number] = [
                            0.5 * (a[0] + b[0]),
                            0.5 * (a[1] + b[1]),
                            0.5 * (a[2] + b[2]),
                          ];
                          const isScanCenterSelected =
                            builderState.selectedScanCenterHandle?.scanId === scan.id;
                          return (
                            <group key={`scan-project-${scan.id}`}>
                              <Line
                                points={[scaleToScene(a), scaleToScene(b)]}
                                color="#67e8f9"
                                lineWidth={1.2}
                                transparent
                                opacity={0.55}
                              />

                              <mesh
                                position={scaleToScene(a)}
                                quaternion={qa}
                                renderOrder={3}
                                onPointerOver={(e) => {
                                  e.stopPropagation();
                                  setHoveredPlannerPointId(`plane-surface:${scan.id}:a`);
                                }}
                                onPointerOut={(e) => {
                                  e.stopPropagation();
                                  setHoveredPlannerPointId((prev) =>
                                    prev === `plane-surface:${scan.id}:a` ? null : prev
                                  );
                                }}
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (
                                    builderState.selectedScanCenterHandle?.scanId === scan.id ||
                                    builderState.centerDragActive
                                  ) {
                                    return;
                                  }
                                  builderActions.setSelectedScanId(scan.id);
                                  const alreadySelected =
                                    builderState.selectedProjectScanPlaneHandle?.scanId === scan.id &&
                                    builderState.selectedProjectScanPlaneHandle?.handle === 'a';
                                  const next = alreadySelected ? null : { scanId: scan.id, handle: 'a' as const };
                                  builderActions.setSelectedProjectScanPlaneHandle(next);
                                  builderActions.setSelectedScanCenterHandle(null);
                                  if (next) {
                                    builderActions.setSelectedKeyLevelHandle(null);
                                    builderActions.setSelectedConnectorControl(null);
                                  }
                                }}
                              >
                                <planeGeometry args={[planeSizeScene, planeSizeScene]} />
                                <meshBasicMaterial
                                  color={accentA}
                                  transparent
                                  opacity={
                                    builderState.selectedProjectScanPlaneHandle?.scanId === scan.id &&
                                    builderState.selectedProjectScanPlaneHandle?.handle === 'a'
                                      ? 0.18
                                      : hoveredPlannerPointId === `plane-surface:${scan.id}:a`
                                        ? 0.15
                                        : 0.09
                                  }
                                  side={THREE.DoubleSide}
                                  depthWrite={false}
                                />
                              </mesh>
                              <mesh
                                position={scaleToScene(b)}
                                quaternion={qa}
                                renderOrder={3}
                                onPointerOver={(e) => {
                                  e.stopPropagation();
                                  setHoveredPlannerPointId(`plane-surface:${scan.id}:b`);
                                }}
                                onPointerOut={(e) => {
                                  e.stopPropagation();
                                  setHoveredPlannerPointId((prev) =>
                                    prev === `plane-surface:${scan.id}:b` ? null : prev
                                  );
                                }}
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (
                                    builderState.selectedScanCenterHandle?.scanId === scan.id ||
                                    builderState.centerDragActive
                                  ) {
                                    return;
                                  }
                                  builderActions.setSelectedScanId(scan.id);
                                  const alreadySelected =
                                    builderState.selectedProjectScanPlaneHandle?.scanId === scan.id &&
                                    builderState.selectedProjectScanPlaneHandle?.handle === 'b';
                                  const next = alreadySelected ? null : { scanId: scan.id, handle: 'b' as const };
                                  builderActions.setSelectedProjectScanPlaneHandle(next);
                                  builderActions.setSelectedScanCenterHandle(null);
                                  if (next) {
                                    builderActions.setSelectedKeyLevelHandle(null);
                                    builderActions.setSelectedConnectorControl(null);
                                  }
                                }}
                              >
                                <planeGeometry args={[planeSizeScene, planeSizeScene]} />
                                <meshBasicMaterial
                                  color={accentB}
                                  transparent
                                  opacity={
                                    builderState.selectedProjectScanPlaneHandle?.scanId === scan.id &&
                                    builderState.selectedProjectScanPlaneHandle?.handle === 'b'
                                      ? 0.18
                                      : hoveredPlannerPointId === `plane-surface:${scan.id}:b`
                                        ? 0.15
                                        : 0.09
                                  }
                                  side={THREE.DoubleSide}
                                  depthWrite={false}
                                />
                              </mesh>

                              {([
                                { id: 'a' as const, pos: a, color: accentA, selected: isPlaneASelected },
                                { id: 'b' as const, pos: b, color: accentB, selected: isPlaneBSelected },
                              ]).map((h) => {
                                const scenePos = scaleToScene(h.pos);
                                const hoverId = `plane:${scan.id}:${h.id}`;
                                const hovered = hoveredPlannerPointId === hoverId;
                                return (
                                  <group key={`scan-plane-${scan.id}-${h.id}`}>
                                    <mesh
                                      position={scenePos}
                                      onPointerOver={(e) => {
                                        e.stopPropagation();
                                        setHoveredPlannerPointId(hoverId);
                                      }}
                                      onPointerOut={(e) => {
                                        e.stopPropagation();
                                        setHoveredPlannerPointId((prev) =>
                                          prev === hoverId ? null : prev
                                        );
                                      }}
                                      onClick={(e) => {
                                        e.stopPropagation();
                                        if (
                                          builderState.selectedScanCenterHandle?.scanId === scan.id ||
                                          builderState.centerDragActive
                                        ) {
                                          return;
                                        }
                                        builderActions.setSelectedScanId(scan.id);
                                        const alreadySelected =
                                          builderState.selectedProjectScanPlaneHandle?.scanId ===
                                            scan.id &&
                                          builderState.selectedProjectScanPlaneHandle?.handle === h.id;
                                        const next =
                                          alreadySelected
                                            ? null
                                            : ({
                                                scanId: scan.id,
                                                handle: h.id,
                                              } as const);
                                        builderActions.setSelectedProjectScanPlaneHandle(next);
                                        builderActions.setSelectedScanCenterHandle(null);
                                        if (next) {
                                          builderActions.setSelectedKeyLevelHandle(null);
                                          builderActions.setSelectedConnectorControl(null);
                                        }
                                      }}
                                    >
                                      <sphereGeometry
                                        args={[
                                          Math.max(0.06 * ORBIT_SCALE, 0.000005) *
                                            (hovered ? 1.25 : 1.0),
                                          12,
                                          12,
                                        ]}
                                      />
                                      <meshBasicMaterial
                                        color={h.selected ? '#fde047' : hovered ? '#ffffff' : h.color}
                                        transparent
                                        opacity={hovered || h.selected ? 1.0 : 0.95}
                                      />
                                    </mesh>
                                    {h.selected && (
                                      <TransformControls
                                        mode="translate"
                                        position={scenePos}
                                        showX
                                        showY
                                        showZ
                                        onObjectChange={(e: any) => {
                                          const obj = e?.target?.object as THREE.Object3D;
                                          if (!obj) return;
                                          const next: [number, number, number] = [
                                            obj.position.x / ORBIT_SCALE + sceneOrigin[0],
                                            obj.position.y / ORBIT_SCALE + sceneOrigin[1],
                                            obj.position.z / ORBIT_SCALE + sceneOrigin[2],
                                          ];
                                          builderActions.moveProjectScanPlaneHandle(scan.id, h.id, next);
                                        }}
                                      />
                                    )}
                                  </group>
                                );
                              })}

                              <Text
                                position={scaleToScene(a)}
                                fontSize={Math.max(0.06 * ORBIT_SCALE, 0.000007)}
                                color={accentA}
                                anchorX="center"
                                anchorY="middle"
                              >
                                {scan.name} A
                              </Text>
                              <Text
                                position={scaleToScene(b)}
                                fontSize={Math.max(0.06 * ORBIT_SCALE, 0.000007)}
                                color={accentB}
                                anchorX="center"
                                anchorY="middle"
                              >
                                {scan.name} B
                              </Text>

                              {builderState.selectedScanId === scan.id && (
                                <group key={`scan-center-${scan.id}`}>
                                  {(() => {
                                    const hoverId = `scan-center:${scan.id}`;
                                    const hovered = hoveredPlannerPointId === hoverId;
                                    const scenePos = scaleToScene(scanCenterPos);
                                    const markerRadius = Math.max(0.06 * ORBIT_SCALE, 0.000005);
                                    return (
                                      <>
                                        <mesh
                                          position={scenePos}
                                          onPointerOver={(e) => {
                                            e.stopPropagation();
                                            setHoveredPlannerPointId(hoverId);
                                          }}
                                          onPointerOut={(e) => {
                                            e.stopPropagation();
                                            setHoveredPlannerPointId((prev) =>
                                              prev === hoverId ? null : prev
                                            );
                                          }}
                                          onClick={(e) => {
                                            e.stopPropagation();
                                            builderActions.setSelectedScanId(scan.id);
                                            const next = { scanId: scan.id } as const;
                                            builderActions.setSelectedScanCenterHandle(next);
                                            builderActions.setSelectedProjectScanPlaneHandle(null);
                                            builderActions.setSelectedKeyLevelHandle(null);
                                            builderActions.setSelectedConnectorControl(null);
                                          }}
                                        >
                                          <sphereGeometry
                                            args={[
                                              markerRadius * (hovered || isScanCenterSelected ? 1.25 : 1.0),
                                              12,
                                              12,
                                            ]}
                                          />
                                          <meshBasicMaterial
                                            color={
                                              isScanCenterSelected
                                                ? '#fde047'
                                                : hovered
                                                  ? '#ffffff'
                                                  : '#e879f9'
                                            }
                                            transparent
                                            opacity={hovered || isScanCenterSelected ? 1.0 : 0.95}
                                          />
                                        </mesh>
                                        {isScanCenterSelected && (
                                          <TransformControls
                                            mode="translate"
                                            position={scenePos}
                                            showX
                                            showY
                                            showZ
                                            onMouseDown={() => {
                                              builderActions.beginScanCenterDrag(scan.id);
                                            }}
                                            onObjectChange={(e: any) => {
                                              const obj = e?.target?.object as THREE.Object3D;
                                              if (!obj) return;
                                              const next: [number, number, number] = [
                                                obj.position.x / ORBIT_SCALE + sceneOrigin[0],
                                                obj.position.y / ORBIT_SCALE + sceneOrigin[1],
                                                obj.position.z / ORBIT_SCALE + sceneOrigin[2],
                                              ];
                                              builderActions.updateScanCenterDrag(scan.id, next);
                                            }}
                                            onMouseUp={() => {
                                              builderActions.endScanCenterDrag();
                                            }}
                                          />
                                        )}
                                      </>
                                    );
                                  })()}
                                </group>
                              )}

                              {builderState.selectedScanId === scan.id &&
                                (() => {
                                  const keyLevels = (scan.key_levels ?? []) as any[];
                                  if (keyLevels.length === 0) return null;
                                  const keyLevel =
                                    keyLevels.find((item) => item.id === builderState.selectedKeyLevelId) ??
                                    keyLevels[0];
                                  if (!keyLevel) return null;
                                  const t = Math.max(0, Math.min(1, Number(keyLevel.t ?? 0)));
                                  const centerBase = new THREE.Vector3(
                                    a[0] + (b[0] - a[0]) * t,
                                    a[1] + (b[1] - a[1]) * t,
                                    a[2] + (b[2] - a[2]) * t
                                  );
                                  const center = centerBase.clone();
                                  const rot = ((Number(keyLevel.rotation_deg) || 0) * Math.PI) / 180;
                                  const major = uAxis
                                    .clone()
                                    .multiplyScalar(Math.cos(rot))
                                    .add(vAxis.clone().multiplyScalar(Math.sin(rot)))
                                    .normalize();
                                  const minor = uAxis
                                    .clone()
                                    .multiplyScalar(-Math.sin(rot))
                                    .add(vAxis.clone().multiplyScalar(Math.cos(rot)))
                                    .normalize();
                                  const rx = Math.max(0.01, Number(keyLevel.radius_x) || 1);
                                  const ry = Math.max(0.01, Number(keyLevel.radius_y) || 1);
                                  const handleRadius = Math.max(0.05 * ORBIT_SCALE, 0.000005);
                                  const handles = [
                                    {
                                      id: 'rx_pos' as const,
                                      pos: center.clone().add(major.clone().multiplyScalar(rx)),
                                      color: '#34d399',
                                    },
                                    {
                                      id: 'rx_neg' as const,
                                      pos: center.clone().add(major.clone().multiplyScalar(-rx)),
                                      color: '#34d399',
                                    },
                                    {
                                      id: 'ry_pos' as const,
                                      pos: center.clone().add(minor.clone().multiplyScalar(ry)),
                                      color: '#60a5fa',
                                    },
                                    {
                                      id: 'ry_neg' as const,
                                      pos: center.clone().add(minor.clone().multiplyScalar(-ry)),
                                      color: '#60a5fa',
                                    },
                                  ];
                                  return handles.map((h) => {
                                    const meterPos: [number, number, number] = [
                                      h.pos.x,
                                      h.pos.y,
                                      h.pos.z,
                                    ];
                                    const scenePos = scaleToScene(meterPos);
                                    const hoverId = `key:${scan.id}:${keyLevel.id}:${h.id}`;
                                    const hovered = hoveredPlannerPointId === hoverId;
                                    const selected =
                                      builderState.selectedKeyLevelHandle?.scanId === scan.id &&
                                      builderState.selectedKeyLevelHandle?.keyLevelId === keyLevel.id &&
                                      builderState.selectedKeyLevelHandle?.handle === h.id;
                                    return (
                                      <group key={`key-level-handle-${scan.id}-${keyLevel.id}-${h.id}`}>
                                        <mesh
                                          position={scenePos}
                                          onPointerOver={(e) => {
                                            e.stopPropagation();
                                            setHoveredPlannerPointId(hoverId);
                                          }}
                                          onPointerOut={(e) => {
                                            e.stopPropagation();
                                            setHoveredPlannerPointId((prev) =>
                                              prev === hoverId ? null : prev
                                            );
                                          }}
                                          onClick={(e) => {
                                            e.stopPropagation();
                                            builderActions.setSelectedScanId(scan.id);
                                            builderActions.setSelectedKeyLevelId(keyLevel.id);
                                            builderActions.setSelectedScanCenterHandle(null);
                                            builderActions.setSelectedKeyLevelHandle({
                                              scanId: scan.id,
                                              keyLevelId: keyLevel.id,
                                              handle: h.id,
                                            });
                                          }}
                                        >
                                          <sphereGeometry
                                            args={[handleRadius * (hovered ? 1.25 : 1.0), 12, 12]}
                                          />
                                          <meshBasicMaterial
                                            color={selected ? '#fde047' : hovered ? '#ffffff' : h.color}
                                            transparent
                                            opacity={hovered || selected ? 1.0 : 0.95}
                                          />
                                        </mesh>
                                        {selected && (
                                          <TransformControls
                                            mode="translate"
                                            position={scenePos}
                                            showX
                                            showY
                                            showZ
                                            onObjectChange={(e: any) => {
                                              const obj = e?.target?.object as THREE.Object3D;
                                              if (!obj) return;
                                              const next: [number, number, number] = [
                                                obj.position.x / ORBIT_SCALE + sceneOrigin[0],
                                                obj.position.y / ORBIT_SCALE + sceneOrigin[1],
                                                obj.position.z / ORBIT_SCALE + sceneOrigin[2],
                                              ];
                                              builderActions.updateKeyLevelHandlePosition(
                                                scan.id,
                                                keyLevel.id,
                                                h.id,
                                                next
                                              );
                                            }}
                                          />
                                        )}
                                      </group>
                                    );
                                  });
                                })()}
                            </group>
                          );
                        })}

                        {builderState.compilePreviewState?.endpoints &&
                          Object.entries(builderState.compilePreviewState.endpoints).flatMap(
                            ([scanId, ep]: any) =>
                              ([
                                { key: 'start', color: '#4ade80', label: 'S', pos: ep.start as [number, number, number] },
                                { key: 'end', color: '#38bdf8', label: 'E', pos: ep.end as [number, number, number] },
                              ] as const).map((item) => (
                                <group key={`endpoint-${scanId}-${item.key}`}>
                                  {(() => {
                                    const hoverId = `endpoint:${scanId}:${item.key}`;
                                    const hovered = hoveredPlannerPointId === hoverId;
                                    const isConnectSource =
                                      builderState.connectSourceEndpoint?.scanId === scanId &&
                                      builderState.connectSourceEndpoint?.endpoint === item.key;
                                    const radius = Math.max(0.065 * ORBIT_SCALE, 0.000006);
                                    const pointColor = isConnectSource
                                      ? '#fde047'
                                      : hovered
                                        ? '#ffffff'
                                        : item.color;
                                    return (
                                  <mesh
                                    position={scaleToScene(item.pos)}
                                    onPointerOver={(e) => {
                                      e.stopPropagation();
                                      setHoveredPlannerPointId(hoverId);
                                    }}
                                    onPointerOut={(e) => {
                                      e.stopPropagation();
                                      setHoveredPlannerPointId((prev) =>
                                        prev === hoverId ? null : prev
                                      );
                                    }}
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      builderActions.setSelectedScanId(scanId);
                                      if (builderState.connectMode) {
                                        builderActions.selectEndpointForConnect(scanId, item.key);
                                      } else if (builderState.authoringStep === 'target') {
                                        builderActions.setTransferTargetRef({
                                          scanId,
                                          endpoint: item.key,
                                        });
                                      }
                                    }}
                                  >
                                    <sphereGeometry
                                      args={[radius * (hovered || isConnectSource ? 1.25 : 1.0), 12, 12]}
                                    />
                                    <meshBasicMaterial
                                      color={pointColor}
                                      transparent
                                      opacity={hovered || isConnectSource ? 1.0 : 0.9}
                                    />
                                  </mesh>
                                    );
                                  })()}
                                  <Text
                                    position={scaleToScene(item.pos)}
                                    fontSize={Math.max(0.05 * ORBIT_SCALE, 0.000006)}
                                    color={
                                      builderState.connectSourceEndpoint?.scanId === scanId &&
                                      builderState.connectSourceEndpoint?.endpoint === item.key
                                        ? '#fde047'
                                        : item.color
                                    }
                                    anchorX="center"
                                    anchorY="middle"
                                  >
                                    {item.label}
                                  </Text>
                                </group>
                              ))
                          )}

                        {builderState.scanProject.connectors.map((connector: any) => (
                          <group key={`connector-controls-${connector.id}`}>
                            {(['control1', 'control2'] as const).map((controlName) => {
                              const pos = connector[controlName] as [number, number, number] | null | undefined;
                              if (!pos) return null;
                              const hoverId = `connector:${connector.id}:${controlName}`;
                              const hovered = hoveredPlannerPointId === hoverId;
                              const selected =
                                builderState.selectedConnectorControl?.connectorId === connector.id &&
                                builderState.selectedConnectorControl?.control === controlName;
                              const scenePos = scaleToScene(pos);
                              return (
                                <group key={`connector-${connector.id}-${controlName}`}>
                                  <mesh
                                    position={scenePos}
                                    onPointerOver={(e) => {
                                      e.stopPropagation();
                                      setHoveredPlannerPointId(hoverId);
                                    }}
                                    onPointerOut={(e) => {
                                      e.stopPropagation();
                                      setHoveredPlannerPointId((prev) =>
                                        prev === hoverId ? null : prev
                                      );
                                    }}
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      builderActions.setSelectedConnectorControl({
                                        connectorId: connector.id,
                                        control: controlName,
                                      });
                                    }}
                                  >
                                    <sphereGeometry
                                      args={[
                                        Math.max(0.05 * ORBIT_SCALE, 0.000004) *
                                          (hovered ? 1.25 : 1.0),
                                        12,
                                        12,
                                      ]}
                                    />
                                    <meshBasicMaterial
                                      color={selected ? '#fde047' : hovered ? '#ffffff' : '#f97316'}
                                      transparent
                                      opacity={hovered || selected ? 1.0 : 0.95}
                                    />
                                  </mesh>
                                  {selected && (
                                    <TransformControls
                                      mode="translate"
                                      position={scenePos}
                                      showX
                                      showY
                                      showZ
                                      onObjectChange={(e: any) => {
                                        const obj = e?.target?.object as THREE.Object3D;
                                        if (!obj) return;
                                        const next: [number, number, number] = [
                                          obj.position.x / ORBIT_SCALE + sceneOrigin[0],
                                          obj.position.y / ORBIT_SCALE + sceneOrigin[1],
                                          obj.position.z / ORBIT_SCALE + sceneOrigin[2],
                                        ];
                                        builderActions.updateConnectorControl(
                                          connector.id,
                                          controlName,
                                          next
                                        );
                                      }}
                                    />
                                  )}
                                </group>
                              );
                            })}
                          </group>
                        ))}

                        {builderState.compilePreviewState?.scan_paths?.map((segment: any) => {
                          const pts = (segment.path ?? []) as [number, number, number][];
                          if (!pts || pts.length < 2) return null;
                          const minClear = segment.min_clearance_m as number | null | undefined;
                          const threshold =
                            builderState.compilePreviewState?.diagnostics?.clearance_threshold_m ?? 0.05;
                          const color =
                            minClear != null && minClear < threshold ? '#ef4444' : '#22d3ee';
                          const clearances = (segment.clearance_per_point ?? []) as number[];
                          return (
                            <group key={`compiled-segment-${segment.id}`}>
                              <Line
                                points={pts.map(scaleToScene)}
                                color={color}
                                lineWidth={1.6}
                                transparent
                                opacity={0.85}
                                depthTest={false}
                              />
                              {clearances.length === pts.length &&
                                pts.map((p, idx) => {
                                  if (clearances[idx] >= threshold) return null;
                                  return (
                                    <mesh key={`risk-${segment.id}-${idx}`} position={scaleToScene(p)}>
                                      <sphereGeometry args={[Math.max(0.022 * ORBIT_SCALE, 0.000003), 8, 8]} />
                                      <meshBasicMaterial color="#ef4444" transparent opacity={0.95} />
                                    </mesh>
                                  );
                                })}
                            </group>
                          );
                        })}

                        {builderState.compilePreviewState?.connector_paths?.map((segment: any) => {
                          const pts = (segment.path ?? []) as [number, number, number][];
                          if (!pts || pts.length < 2) return null;
                          const minClear = segment.min_clearance_m as number | null | undefined;
                          const threshold =
                            builderState.compilePreviewState?.diagnostics?.clearance_threshold_m ?? 0.05;
                          const color =
                            minClear != null && minClear < threshold ? '#ef4444' : '#f59e0b';
                          return (
                            <Line
                              key={`compiled-connector-${segment.id}`}
                              points={pts.map(scaleToScene)}
                              color={color}
                              lineWidth={2.0}
                              transparent
                              opacity={0.9}
                              depthTest={false}
                            />
                          );
                        })}
                      </>
                    )}

                    {mode !== 'scan' &&
                      builderState.authoringStep === 'target' &&
                      builderState.compilePreviewState?.endpoints &&
                      Object.entries(builderState.compilePreviewState.endpoints).flatMap(
                        ([scanId, ep]: any) =>
                          ([
                            { key: 'start', color: '#4ade80', label: 'S', pos: ep.start as [number, number, number] },
                            { key: 'end', color: '#38bdf8', label: 'E', pos: ep.end as [number, number, number] },
                          ] as const).map((item) => (
                            <group key={`transfer-endpoint-${scanId}-${item.key}`}>
                              {(() => {
                                const hoverId = `endpoint:${scanId}:${item.key}`;
                                const hovered = hoveredPlannerPointId === hoverId;
                                const radius = Math.max(0.07 * ORBIT_SCALE, 0.000007);
                                return (
                                  <mesh
                                    position={scaleToScene(item.pos)}
                                    onPointerOver={(e) => {
                                      e.stopPropagation();
                                      setHoveredPlannerPointId(hoverId);
                                    }}
                                    onPointerOut={(e) => {
                                      e.stopPropagation();
                                      setHoveredPlannerPointId((prev) =>
                                        prev === hoverId ? null : prev
                                      );
                                    }}
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      builderActions.setTransferTargetRef({
                                        scanId,
                                        endpoint: item.key,
                                      });
                                    }}
                                  >
                                    <sphereGeometry
                                      args={[radius * (hovered ? 1.25 : 1.0), 12, 12]}
                                    />
                                    <meshBasicMaterial
                                      color={hovered ? '#ffffff' : item.color}
                                      transparent
                                      opacity={hovered ? 1.0 : 0.95}
                                    />
                                  </mesh>
                                );
                              })()}
                              <Text
                                position={scaleToScene(item.pos)}
                                fontSize={Math.max(0.06 * ORBIT_SCALE, 0.000007)}
                                color={item.color}
                                anchorX="center"
                                anchorY="middle"
                              >
                                {item.label}
                              </Text>
                            </group>
                          ))
                      )}

                    {mode !== 'scan' &&
                      builderState.previewPath.length === 0 &&
                      builderState.compilePreviewState?.scan_paths?.map((segment: any) => {
                        const pts = (segment.path ?? []) as [number, number, number][];
                        if (!pts || pts.length < 2) return null;
                        const minClear = segment.min_clearance_m as number | null | undefined;
                        const threshold =
                          builderState.compilePreviewState?.diagnostics?.clearance_threshold_m ?? 0.05;
                        const color =
                          minClear != null && minClear < threshold ? '#ef4444' : '#22d3ee';
                        return (
                          <Line
                            key={`mission-fallback-compiled-segment-${segment.id}`}
                            points={pts.map(scaleToScene)}
                            color={color}
                            lineWidth={1.6}
                            transparent
                            opacity={0.85}
                            depthTest={false}
                          />
                        );
                      })}

                    {mode !== 'scan' &&
                      builderState.previewPath.length === 0 &&
                      builderState.compilePreviewState?.connector_paths?.map((segment: any) => {
                        const pts = (segment.path ?? []) as [number, number, number][];
                        if (!pts || pts.length < 2) return null;
                        const minClear = segment.min_clearance_m as number | null | undefined;
                        const threshold =
                          builderState.compilePreviewState?.diagnostics?.clearance_threshold_m ?? 0.05;
                        const color =
                          minClear != null && minClear < threshold ? '#ef4444' : '#f59e0b';
                        return (
                          <Line
                            key={`mission-fallback-compiled-connector-${segment.id}`}
                            points={pts.map(scaleToScene)}
                            color={color}
                            lineWidth={2.0}
                            transparent
                            opacity={0.9}
                            depthTest={false}
                          />
                        );
                      })}

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

      <button
        type="button"
        className="absolute top-6 right-28 z-20 rounded-md border border-cyan-500/50 bg-slate-900/85 px-3 py-1 text-xs font-semibold uppercase tracking-wide text-cyan-200 transition hover:border-cyan-300 hover:text-white disabled:cursor-not-allowed disabled:opacity-45"
        onClick={() => {
          if (!homeFocus) return;
          requestFocus(homeFocus.target, homeFocus.distance);
        }}
        disabled={!homeFocus}
        title="Home: zoom back to selected object"
      >
        Home
      </button>
    </div>
  );
}

import { useMemo, useEffect, useState } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';
import { useRegenerateWaypoints } from './useRegenerateWaypoints';

interface ScanPassObjectProps {
  scanId: string;
}

function qToEuler(qwxyz: [number, number, number, number]): [number, number, number] {
  const q = new THREE.Quaternion(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]);
  const e = new THREE.Euler().setFromQuaternion(q);
  return [e.x, e.y, e.z];
}

function oppositeFacingQuat(qwxyz: [number, number, number, number]): [number, number, number, number] {
  const q = new THREE.Quaternion(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]).normalize();
  const flipLocalY = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);
  const out = q.clone().multiply(flipLocalY).normalize();
  return [out.w, out.x, out.y, out.z];
}

function normalFromQuat(qwxyz: [number, number, number, number]): THREE.Vector3 {
  const q = new THREE.Quaternion(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]).normalize();
  return new THREE.Vector3(0, 0, 1).applyQuaternion(q).normalize();
}

function gapAlongNormal(
  aPos: [number, number, number],
  bPos: [number, number, number],
  n: THREE.Vector3
): number {
  const a = new THREE.Vector3(...aPos);
  const b = new THREE.Vector3(...bPos);
  const g = b.clone().sub(a).dot(n);
  return Math.abs(g) < 1e-6 ? 10 : g;
}

function PlaneMesh({
  position,
  orientation,
  color,
}: {
  position: [number, number, number];
  orientation: [number, number, number, number];
  color: string;
}) {
  return (
    <mesh position={position} rotation={qToEuler(orientation)}>
      <planeGeometry args={[8, 8]} />
      <meshBasicMaterial color={color} opacity={0.16} transparent side={THREE.DoubleSide} />
    </mesh>
  );
}

export function ScanPassObject({ scanId }: ScanPassObjectProps) {
  const path = useStudioStore((s) => s.paths.find((p) => p.id === scanId));
  const selectedPathId = useStudioStore((s) => s.selectedPathId);
  const activeTool = useStudioStore((s) => s.activeTool);
  const editMode = useStudioStore((s) => s.editMode);
  const pathEditMode = useStudioStore((s) => s.pathEditMode);
  const regenerate = useRegenerateWaypoints();
  const [controlTarget, setControlTarget] = useState<'none' | 'center' | 'A' | 'B'>('none');
  const [selectedWaypointIndex, setSelectedWaypointIndex] = useState<number | null>(null);

  useEffect(() => {
    if (!path) return;
    if (activeTool === 'edit') return;
    if (path.isLocallyEdited) return;
    regenerate(scanId, 0);
  }, [scanId, path?.planeA, path?.planeB, path?.ellipse, path?.levelSpacing, path?.waypointDensity, path?.isLocallyEdited, activeTool, regenerate]);

  const lineGeometry = useMemo(() => {
    if (!path || path.waypoints.length < 2) return null;
    const points = path.waypoints.map(([x, y, z]) => new THREE.Vector3(x, y, z));
    return new THREE.BufferGeometry().setFromPoints(points);
  }, [path?.waypoints]);

  const densitySnippetGeometry = useMemo(() => {
    if (!path || !path.densitySnippetRange) return null;
    const [rawA, rawB] = path.densitySnippetRange;
    const a = Math.max(0, Math.min(rawA, rawB));
    const b = Math.min(path.waypoints.length - 1, Math.max(rawA, rawB));
    if (b - a < 1) return null;
    const points = path.waypoints.slice(a, b + 1).map(([x, y, z]) => new THREE.Vector3(x, y, z));
    if (points.length < 2) return null;
    return new THREE.BufferGeometry().setFromPoints(points);
  }, [path?.waypoints, path?.densitySnippetRange, path]);

  const pointsGeometry = useMemo(() => {
    if (!path || path.waypoints.length < 2) return null;
    const points = path.waypoints.map(([x, y, z]) => new THREE.Vector3(x, y, z));
    return new THREE.BufferGeometry().setFromPoints(points);
  }, [path?.waypoints]);

  const centerLineGeometry = useMemo(() => {
    if (!path) return null;
    const pa = new THREE.Vector3(...path.planeA.position);
    const pb = new THREE.Vector3(...path.planeB.position);
    return new THREE.BufferGeometry().setFromPoints([pa, pb]);
  }, [path?.planeA.position, path?.planeB.position, path]);

  const centerMid = useMemo(() => {
    if (!path) return null;
    return new THREE.Vector3(
      0.5 * (path.planeA.position[0] + path.planeB.position[0]),
      0.5 * (path.planeA.position[1] + path.planeB.position[1]),
      0.5 * (path.planeA.position[2] + path.planeB.position[2]),
    );
  }, [path]);

  const startPos = path?.waypoints[0] ?? null;
  const endPos = path && path.waypoints.length > 0 ? path.waypoints[path.waypoints.length - 1] : null;
  const isSelected = selectedPathId === scanId;
  const color = path?.color ?? '#22d3ee';

  const showPlaneGizmos = isSelected && activeTool === 'create_path';
  const showEditMode = activeTool === 'edit';
  const isConnectMode = activeTool === 'connect';
  const holdStride = Math.max(1, Math.floor((path?.waypoints.length ?? 0) / 48));

  useEffect(() => {
    if (!showPlaneGizmos) setControlTarget('none');
  }, [showPlaneGizmos]);

  useEffect(() => {
    if (!showEditMode || editMode !== 'stretch') setSelectedWaypointIndex(null);
  }, [showEditMode, editMode]);

  if (!path) return null;

  const applyLocalStretch = (idx: number, target: THREE.Vector3) => {
    const p = useStudioStore.getState().paths.find((it) => it.id === scanId);
    if (!p || !p.waypoints[idx]) return;
    const old = p.waypoints[idx];
    const delta: [number, number, number] = [target.x - old[0], target.y - old[1], target.z - old[2]];
    const n = p.waypoints.length;
    if (n < 2) return;
    const arc: number[] = new Array(n).fill(0);
    for (let i = 1; i < n; i += 1) {
      const a = p.waypoints[i - 1];
      const b = p.waypoints[i];
      arc[i] = arc[i - 1] + Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    }
    const totalLength = arc[n - 1];
    const avgSpacing = totalLength / Math.max(1, n - 1);
    const localStart = Math.max(0, idx - 3);
    const localEnd = Math.min(n - 2, idx + 2);
    let localSum = 0;
    let localCount = 0;
    for (let i = localStart; i <= localEnd; i += 1) {
      const seg = arc[i + 1] - arc[i];
      if (seg > 0) {
        localSum += seg;
        localCount += 1;
      }
    }
    const localSpacing = localCount > 0 ? localSum / localCount : avgSpacing;
    const radius = Math.max((localSpacing || avgSpacing || 1.0) * 6, localSpacing || avgSpacing || 1.0);
    const s0 = arc[idx];
    const updated = p.waypoints.map((wp, i) => {
      const d = Math.abs(arc[i] - s0);
      const t = radius > 0 ? Math.max(0, 1 - d / radius) : 0;
      const w = t * t;
      return [wp[0] + delta[0] * w, wp[1] + delta[1] * w, wp[2] + delta[2] * w] as [number, number, number];
    });
    useStudioStore.getState().setPathWaypointsManual(scanId, updated);
  };

  const insertWaypointOnPath = (clickPoint: THREE.Vector3) => {
    const p = useStudioStore.getState().paths.find((it) => it.id === scanId);
    if (!p || p.waypoints.length < 2) return;
    let bestSeg = 0;
    let bestT = 0;
    let bestD2 = Number.POSITIVE_INFINITY;
    for (let i = 0; i < p.waypoints.length - 1; i += 1) {
      const a = new THREE.Vector3(...p.waypoints[i]);
      const b = new THREE.Vector3(...p.waypoints[i + 1]);
      const ab = b.clone().sub(a);
      const len2 = Math.max(1e-9, ab.lengthSq());
      const t = Math.max(0, Math.min(1, clickPoint.clone().sub(a).dot(ab) / len2));
      const proj = a.clone().add(ab.multiplyScalar(t));
      const d2 = proj.distanceToSquared(clickPoint);
      if (d2 < bestD2) {
        bestD2 = d2;
        bestSeg = i;
        bestT = t;
      }
    }
    const a = p.waypoints[bestSeg];
    const b = p.waypoints[bestSeg + 1];
    const inserted: [number, number, number] = [
      a[0] + (b[0] - a[0]) * bestT,
      a[1] + (b[1] - a[1]) * bestT,
      a[2] + (b[2] - a[2]) * bestT,
    ];
    const next = [...p.waypoints.slice(0, bestSeg + 1), inserted, ...p.waypoints.slice(bestSeg + 1)];
    useStudioStore.getState().setPathWaypointsManual(scanId, next);
  };

  return (
    <group>
      {lineGeometry && (
        <line
          geometry={lineGeometry}
          onClick={(e) => {
            if (!showEditMode || editMode !== 'add') return;
            e.stopPropagation();
            useStudioStore.getState().selectPath(scanId);
            insertWaypointOnPath(e.point);
          }}
        >
          <lineBasicMaterial
            color={showEditMode && editMode === 'density' && isSelected ? '#f59e0b' : '#22d3ee'}
            linewidth={isSelected ? 2 : 1}
            opacity={isSelected ? 1 : 0.95}
            transparent
          />
        </line>
      )}
      {showEditMode && editMode === 'density' && isSelected && densitySnippetGeometry && (
        <line geometry={densitySnippetGeometry}>
          <lineBasicMaterial color="#f97316" linewidth={3} opacity={1} transparent />
        </line>
      )}
      {!isConnectMode && pointsGeometry && (
        <points geometry={pointsGeometry}>
          <pointsMaterial color="#67e8f9" size={0.08} sizeAttenuation />
        </points>
      )}

      {!isConnectMode && centerLineGeometry && (
        <line
          geometry={centerLineGeometry}
          onClick={(e) => {
            e.stopPropagation();
            setControlTarget((prev) => (prev === 'center' ? 'none' : 'center'));
          }}
        >
          <lineBasicMaterial color="#facc15" opacity={0.95} transparent />
        </line>
      )}

      {!isConnectMode && startPos && (
        <mesh position={startPos} onClick={() => useStudioStore.getState().selectPath(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#22d3ee" />
        </mesh>
      )}
      {!isConnectMode && endPos && (
        <mesh position={endPos} onClick={() => useStudioStore.getState().selectPath(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#a78bfa" />
        </mesh>
      )}

      {!isConnectMode && (
        <>
          <group
            onClick={(e) => {
              e.stopPropagation();
              setControlTarget((prev) => (prev === 'A' ? 'none' : 'A'));
            }}
          >
            <PlaneMesh
              position={path.planeA.position}
              orientation={path.planeA.orientation}
              color={controlTarget === 'A' ? '#22d3ee' : color}
            />
          </group>
          <group
            onClick={(e) => {
              e.stopPropagation();
              setControlTarget((prev) => (prev === 'B' ? 'none' : 'B'));
            }}
          >
            <PlaneMesh
              position={path.planeB.position}
              orientation={path.planeB.orientation}
              color={controlTarget === 'B' ? '#22d3ee' : color}
            />
          </group>
        </>
      )}

      {showPlaneGizmos && centerMid && (
        <>
          {/* Exactly one shared control at a time to avoid control-frame feedback. */}
          {pathEditMode === 'translate' && controlTarget === 'center' && (
            <TransformControls
              mode="translate"
              position={[centerMid.x, centerMid.y, centerMid.z]}
              space="world"
              onObjectChange={(e: any) => {
                if (!e?.target?.dragging) return;
                const obj = e?.target?.object as THREE.Object3D | undefined;
                if (!obj) return;
                const s = useStudioStore.getState();
                const p = s.paths.find((it) => it.id === scanId);
                if (!p) return;
                const oldMid = new THREE.Vector3(
                  0.5 * (p.planeA.position[0] + p.planeB.position[0]),
                  0.5 * (p.planeA.position[1] + p.planeB.position[1]),
                  0.5 * (p.planeA.position[2] + p.planeB.position[2]),
                );
                const delta = new THREE.Vector3(obj.position.x, obj.position.y, obj.position.z).sub(oldMid);
                const a = new THREE.Vector3(...p.planeA.position).add(delta);
                const b = new THREE.Vector3(...p.planeB.position).add(delta);
                s.updatePathPlane(scanId, 'planeA', { position: [a.x, a.y, a.z] });
                s.updatePathPlane(scanId, 'planeB', { position: [b.x, b.y, b.z] });
                regenerate(scanId, 120);
              }}
            />
          )}
          {pathEditMode === 'rotate' && controlTarget === 'center' && (
            <TransformControls
              mode="rotate"
              position={[centerMid.x, centerMid.y, centerMid.z]}
              rotation={qToEuler(path.planeA.orientation)}
              onObjectChange={(e: any) => {
                if (!e?.target?.dragging) return;
                const obj = e?.target?.object as THREE.Object3D | undefined;
                if (!obj) return;
                const s = useStudioStore.getState();
                const p = s.paths.find((it) => it.id === scanId);
                if (!p) return;
                const qA: [number, number, number, number] = [obj.quaternion.w, obj.quaternion.x, obj.quaternion.y, obj.quaternion.z];
                const n = normalFromQuat(qA);
                const oldMid = new THREE.Vector3(
                  0.5 * (p.planeA.position[0] + p.planeB.position[0]),
                  0.5 * (p.planeA.position[1] + p.planeB.position[1]),
                  0.5 * (p.planeA.position[2] + p.planeB.position[2]),
                );
                const g = gapAlongNormal(p.planeA.position, p.planeB.position, n);
                const a = oldMid.clone().add(n.clone().multiplyScalar(-0.5 * g));
                const b = oldMid.clone().add(n.clone().multiplyScalar(0.5 * g));
                s.updatePathPlane(scanId, 'planeA', { orientation: qA, position: [a.x, a.y, a.z] });
                s.updatePathPlane(scanId, 'planeB', {
                  orientation: oppositeFacingQuat(qA),
                  position: [b.x, b.y, b.z],
                });
                regenerate(scanId, 120);
              }}
            />
          )}

          {/* Individual plane slide controls constrained to centerline */}
          {pathEditMode === 'translate' && controlTarget === 'A' && (
            <TransformControls
              mode="translate"
              position={path.planeA.position}
              space="world"
              onObjectChange={(e: any) => {
                if (!e?.target?.dragging) return;
                const obj = e?.target?.object as THREE.Object3D | undefined;
                if (!obj) return;
                const s = useStudioStore.getState();
                const p = s.paths.find((it) => it.id === scanId);
                if (!p) return;
                const n = normalFromQuat(p.planeA.orientation);
                const b = new THREE.Vector3(...p.planeB.position);
                const proposed = new THREE.Vector3(obj.position.x, obj.position.y, obj.position.z);
                const t = proposed.clone().sub(b).dot(n);
                const a = b.clone().add(n.clone().multiplyScalar(t));
                s.updatePathPlane(scanId, 'planeA', { position: [a.x, a.y, a.z] });
                regenerate(scanId, 120);
              }}
            />
          )}
          {pathEditMode === 'translate' && controlTarget === 'B' && (
            <TransformControls
              mode="translate"
              position={path.planeB.position}
              space="world"
              onObjectChange={(e: any) => {
                if (!e?.target?.dragging) return;
                const obj = e?.target?.object as THREE.Object3D | undefined;
                if (!obj) return;
                const s = useStudioStore.getState();
                const p = s.paths.find((it) => it.id === scanId);
                if (!p) return;
                const n = normalFromQuat(p.planeA.orientation);
                const a = new THREE.Vector3(...p.planeA.position);
                const proposed = new THREE.Vector3(obj.position.x, obj.position.y, obj.position.z);
                const t = proposed.clone().sub(a).dot(n);
                const b = a.clone().add(n.clone().multiplyScalar(t));
                s.updatePathPlane(scanId, 'planeB', { position: [b.x, b.y, b.z] });
                regenerate(scanId, 120);
              }}
            />
          )}

        </>
      )}

      {showEditMode && (
        <>
          {path.waypoints.map((wp, idx) => (
            <mesh
              key={`${scanId}-edit-${idx}`}
              position={wp}
              onClick={(e) => {
                e.stopPropagation();
                useStudioStore.getState().selectPath(scanId);
                if (editMode === 'delete') {
                  const p = useStudioStore.getState().paths.find((it) => it.id === scanId);
                  if (!p || p.waypoints.length <= 2) return;
                  const next = p.waypoints.filter((_, i) => i !== idx);
                  useStudioStore.getState().setPathWaypointsManual(scanId, next);
                  setSelectedWaypointIndex(null);
                  return;
                }
                if (editMode === 'stretch') {
                  setSelectedWaypointIndex((prev) => (prev === idx ? null : idx));
                }
                if (editMode === 'density') {
                  const p = useStudioStore.getState().paths.find((it) => it.id === scanId);
                  if (!p || (p.densityScope ?? 'total') !== 'snippet') return;
                  const range = p.densitySnippetRange;
                  if (!range) {
                    useStudioStore.getState().updatePath(scanId, { densitySnippetRange: [idx, idx] });
                    return;
                  }
                  if (range[0] === range[1]) {
                    useStudioStore.getState().updatePath(scanId, {
                      densitySnippetRange: [Math.min(range[0], idx), Math.max(range[0], idx)],
                    });
                    return;
                  }
                  useStudioStore.getState().updatePath(scanId, { densitySnippetRange: [idx, idx] });
                }
              }}
            >
              <sphereGeometry args={[selectedWaypointIndex === idx ? 0.14 : 0.1, 8, 8]} />
              <meshBasicMaterial
                color={
                  editMode === 'delete'
                    ? '#f87171'
                    : editMode === 'density' && path.densityScope === 'snippet' && path.densitySnippetRange
                        && idx >= Math.min(path.densitySnippetRange[0], path.densitySnippetRange[1])
                        && idx <= Math.max(path.densitySnippetRange[0], path.densitySnippetRange[1])
                      ? '#f97316'
                      : selectedWaypointIndex === idx
                        ? '#fde047'
                        : '#7dd3fc'
                }
                opacity={selectedWaypointIndex === idx ? 1 : 0.85}
                transparent
              />
            </mesh>
          ))}

          {editMode === 'stretch' && selectedWaypointIndex !== null && path.waypoints[selectedWaypointIndex] && (
            <TransformControls
              mode="translate"
              space="world"
              position={path.waypoints[selectedWaypointIndex]}
              onObjectChange={(e: any) => {
                if (!e?.target?.dragging) return;
                const obj = e?.target?.object as THREE.Object3D | undefined;
                if (!obj) return;
                const s = useStudioStore.getState();
                const p = s.paths.find((it) => it.id === scanId);
                if (!p || p.waypoints.length < 2) return;
                const i = selectedWaypointIndex;
                if (i === null || !p.waypoints[i]) return;
                applyLocalStretch(i, new THREE.Vector3(obj.position.x, obj.position.y, obj.position.z));
              }}
            />
          )}
        </>
      )}

      {activeTool === 'hold' &&
        path.waypoints.map((wp, i) => {
          if (i % holdStride !== 0) return null;
          return (
            <mesh
              key={`${scanId}-hold-${i}`}
              position={wp}
              onClick={(e) => {
                e.stopPropagation();
                useStudioStore.getState().addHold({
                  id: `hold-${Date.now()}-${i}`,
                  pathId: scanId,
                  waypointIndex: i,
                  duration: 5,
                });
              }}
            >
              <sphereGeometry args={[0.13, 8, 8]} />
              <meshBasicMaterial color="#fbbf24" opacity={0.85} transparent />
            </mesh>
          );
        })}
    </group>
  );
}

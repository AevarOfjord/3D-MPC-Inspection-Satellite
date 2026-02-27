import { useRef, useCallback, useEffect, useMemo, useState } from 'react';
import * as THREE from 'three';
import { useThree, useFrame } from '@react-three/fiber';
import { TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';
import { fairCorners, sampleCatmullRomBySpacing } from './splineUtils';

type NodePositionState = Pick<ReturnType<typeof useStudioStore.getState>, 'satelliteStart' | 'paths' | 'points'>;

function resolveNodePosition(nodeId: string, state: NodePositionState): [number, number, number] | null {
  if (nodeId === 'satellite:start') return state.satelliteStart;
  if (nodeId.startsWith('point:')) {
    const pointId = nodeId.slice('point:'.length);
    const point = state.points.find((p) => p.id === pointId);
    return point?.position ?? null;
  }
  const parts = nodeId.split(':');
  if (parts.length < 3 || parts[0] !== 'path') return null;
  const pathId = parts[1];
  const endpoint = parts[2];
  const path = state.paths.find((p) => p.id === pathId);
  if (!path || path.waypoints.length === 0) return null;
  return endpoint === 'start' ? path.waypoints[0] : path.waypoints[path.waypoints.length - 1];
}

function parsePathNode(nodeId: string): { pathId: string; endpoint: 'start' | 'end' } | null {
  const parts = nodeId.split(':');
  if (parts.length !== 3 || parts[0] !== 'path') return null;
  if (parts[2] !== 'start' && parts[2] !== 'end') return null;
  return { pathId: parts[1], endpoint: parts[2] };
}

function wireDensityScale(wire: { fromNodeId: string; toNodeId: string }, state: NodePositionState): number {
  const from = parsePathNode(wire.fromNodeId);
  const to = parsePathNode(wire.toNodeId);
  const fromDensity = from ? state.paths.find((p) => p.id === from.pathId)?.waypointDensity ?? 1 : 1;
  const toDensity = to ? state.paths.find((p) => p.id === to.pathId)?.waypointDensity ?? 1 : 1;
  const avg = 0.5 * (fromDensity + toDensity);
  return Math.max(0.25, Math.min(25, avg));
}

function normalizeVec(v: [number, number, number]): [number, number, number] {
  const n = Math.hypot(v[0], v[1], v[2]);
  if (n <= 1e-9) return [0, 0, 0];
  return [v[0] / n, v[1] / n, v[2] / n];
}

function lengthVec(v: [number, number, number]): number {
  return Math.hypot(v[0], v[1], v[2]);
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

function resolveNodeTangent(
  nodeId: string,
  state: NodePositionState,
  role: 'from' | 'to',
  other: [number, number, number]
): [number, number, number] {
  const pos = resolveNodePosition(nodeId, state);
  if (!pos) return [0, 0, 0];
  const parsed = parsePathNode(nodeId);
  if (!parsed) {
    // satellite endpoint fallback
    return normalizeVec(
      role === 'from'
        ? [other[0] - pos[0], other[1] - pos[1], other[2] - pos[2]]
        : [pos[0] - other[0], pos[1] - other[1], pos[2] - other[2]]
    );
  }
  const path = state.paths.find((p) => p.id === parsed.pathId);
  if (!path || path.waypoints.length < 2) {
    return normalizeVec(
      role === 'from'
        ? [other[0] - pos[0], other[1] - pos[1], other[2] - pos[2]]
        : [pos[0] - other[0], pos[1] - other[1], pos[2] - other[2]]
    );
  }

  // Keep connector seam tangent aligned to the same smoothed controls used for spiral rendering.
  const controls = fairCorners(path.waypoints, 150, 2);
  const n = controls.length;
  if (parsed.endpoint === 'start') {
    const intoPath: [number, number, number] = [
      controls[1][0] - controls[0][0],
      controls[1][1] - controls[0][1],
      controls[1][2] - controls[0][2],
    ];
    return normalizeVec(role === 'to' ? intoPath : ([-intoPath[0], -intoPath[1], -intoPath[2]] as [number, number, number]));
  }
  const intoPathFromEnd: [number, number, number] = [
    controls[n - 2][0] - controls[n - 1][0],
    controls[n - 2][1] - controls[n - 1][1],
    controls[n - 2][2] - controls[n - 1][2],
  ];
  return normalizeVec(role === 'to' ? intoPathFromEnd : ([-intoPathFromEnd[0], -intoPathFromEnd[1], -intoPathFromEnd[2]] as [number, number, number]));
}

function constrainWireControls(
  controls: [number, number, number][],
  fromNodeId: string,
  toNodeId: string,
  state: NodePositionState
): [number, number, number][] {
  const src = resolveNodePosition(fromNodeId, state);
  const dst = resolveNodePosition(toNodeId, state);
  if (!src || !dst) return controls;
  if (!controls || controls.length < 2) return [src, dst];
  if (controls.length < 4) {
    return autoWireControls(fromNodeId, toNodeId, state) ?? [src, dst];
  }
  const next = controls.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
  next[0] = src;
  next[next.length - 1] = dst;

  const dist = Math.hypot(dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]);
  if (dist <= 1e-9) return [src];

  const tSrc = resolveNodeTangent(fromNodeId, state, 'from', dst);
  const tDst = resolveNodeTangent(toNodeId, state, 'to', src);
  const srcHandleMin = Math.max(0.2, dist * 0.03);
  const srcHandleMax = Math.max(srcHandleMin, dist * 0.95);
  const last = next.length - 1;
  const srcRaw: [number, number, number] = [
    next[1][0] - src[0],
    next[1][1] - src[1],
    next[1][2] - src[2],
  ];
  const dstRaw: [number, number, number] = [
    next[last - 1][0] - dst[0],
    next[last - 1][1] - dst[1],
    next[last - 1][2] - dst[2],
  ];
  const srcHandle = clamp(lengthVec(srcRaw), srcHandleMin, srcHandleMax);
  const dstHandle = clamp(lengthVec(dstRaw), srcHandleMin, srcHandleMax);
  next[1] = [src[0] + tSrc[0] * srcHandle, src[1] + tSrc[1] * srcHandle, src[2] + tSrc[2] * srcHandle];
  next[last - 1] = [dst[0] - tDst[0] * dstHandle, dst[1] - tDst[1] * dstHandle, dst[2] - tDst[2] * dstHandle];
  return next;
}

function autoWireControls(
  fromNodeId: string,
  toNodeId: string,
  state: NodePositionState
): [number, number, number][] | null {
  const src = resolveNodePosition(fromNodeId, state);
  const dst = resolveNodePosition(toNodeId, state);
  if (!src || !dst) return null;
  const dx = dst[0] - src[0];
  const dy = dst[1] - src[1];
  const dz = dst[2] - src[2];
  const dist = Math.hypot(dx, dy, dz);
  if (dist <= 1e-9) return [src];
  const tSrc = resolveNodeTangent(fromNodeId, state, 'from', dst);
  const tDst = resolveNodeTangent(toNodeId, state, 'to', src);
  const handle = Math.max(0.5, Math.min(dist * 0.45, 0.28 * dist + 0.6));
  const p1: [number, number, number] = [src[0] + tSrc[0] * handle, src[1] + tSrc[1] * handle, src[2] + tSrc[2] * handle];
  const p2: [number, number, number] = [dst[0] - tDst[0] * handle, dst[1] - tDst[1] * handle, dst[2] - tDst[2] * handle];
  return [src, p1, p2, dst];
}

function sampleConnectorPoints(
  src: [number, number, number],
  dst: [number, number, number],
  densityScale: number
): [number, number, number][] {
  const dx = dst[0] - src[0];
  const dy = dst[1] - src[1];
  const dz = dst[2] - src[2];
  const dist = Math.hypot(dx, dy, dz);
  if (dist <= 1e-9) return [src];
  const spacing = 1 / Math.max(0.25, Math.min(25, densityScale || 1)); // 1x => 1m
  const steps = Math.max(1, Math.ceil(dist / spacing));
  const out: [number, number, number][] = [];
  for (let i = 0; i <= steps; i += 1) {
    const t = i / steps;
    out.push([src[0] + dx * t, src[1] + dy * t, src[2] + dz * t]);
  }
  return out;
}

function applyLocalWireDeform(
  controls: [number, number, number][],
  index: number,
  target: [number, number, number]
): [number, number, number][] {
  if (controls.length < 3 || index <= 0 || index >= controls.length - 1) return controls;
  const old = controls[index];
  const delta: [number, number, number] = [
    target[0] - old[0],
    target[1] - old[1],
    target[2] - old[2],
  ];
  if (Math.hypot(delta[0], delta[1], delta[2]) <= 1e-9) return controls;

  const n = controls.length;
  const arc: number[] = new Array(n).fill(0);
  for (let i = 1; i < n; i += 1) {
    const a = controls[i - 1];
    const b = controls[i];
    arc[i] = arc[i - 1] + Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  }
  const localStart = Math.max(0, index - 3);
  const localEnd = Math.min(n - 2, index + 2);
  let localSum = 0;
  let localCount = 0;
  for (let i = localStart; i <= localEnd; i += 1) {
    const seg = arc[i + 1] - arc[i];
    if (seg > 0) {
      localSum += seg;
      localCount += 1;
    }
  }
  const avgSpacing = arc[n - 1] / Math.max(1, n - 1);
  const localSpacing = localCount > 0 ? localSum / localCount : avgSpacing;
  const radius = Math.max((localSpacing || avgSpacing || 1) * 6, localSpacing || avgSpacing || 1);
  const s0 = arc[index];

  return controls.map((p, i) => {
    if (i === 0 || i === controls.length - 1) return [p[0], p[1], p[2]] as [number, number, number];
    const d = Math.abs(arc[i] - s0);
    const t = radius > 0 ? Math.max(0, 1 - d / radius) : 0;
    const w = t * t;
    return [
      p[0] + delta[0] * w,
      p[1] + delta[1] * w,
      p[2] + delta[2] * w,
    ] as [number, number, number];
  });
}

interface EndpointNodesProps {
  visibleWireIds?: string[] | null;
  connectNodeFilter?: string[] | null;
}

export function EndpointNodes({ visibleWireIds = null, connectNodeFilter = null }: EndpointNodesProps) {
  const paths = useStudioStore((s) => s.paths);
  const satelliteStart = useStudioStore((s) => s.satelliteStart);
  const points = useStudioStore((s) => s.points);
  const wires = useStudioStore((s) => s.wires);
  const wireDrag = useStudioStore((s) => s.wireDrag);
  const activeTool = useStudioStore((s) => s.activeTool);
  const setWireDrag = useStudioStore((s) => s.setWireDrag);
  const addWire = useStudioStore((s) => s.addWire);
  const setWireWaypoints = useStudioStore((s) => s.setWireWaypoints);
  const { camera, gl } = useThree();
  const raycaster = useRef(new THREE.Raycaster());
  const dragLineRef = useRef<THREE.Line>(null);
  const nodeState = { paths, satelliteStart, points };
  const [selectedWirePoint, setSelectedWirePoint] = useState<{ wireId: string; index: number } | null>(null);
  const visibleWireSet = useMemo(
    () => (visibleWireIds && visibleWireIds.length > 0 ? new Set(visibleWireIds) : null),
    [visibleWireIds]
  );
  const connectNodeSet = useMemo(
    () => (connectNodeFilter && connectNodeFilter.length > 0 ? new Set(connectNodeFilter) : null),
    [connectNodeFilter]
  );
  const visibleWires = useMemo(
    () => (visibleWireSet ? wires.filter((wire) => visibleWireSet.has(wire.id)) : wires),
    [wires, visibleWireSet]
  );

  useFrame(() => {
    if (wireDrag.phase !== 'dragging' || !dragLineRef.current) return;
    const state = useStudioStore.getState();
    const srcPos = resolveNodePosition(wireDrag.sourceNodeId, state);
    if (!srcPos) return;
    const points = [new THREE.Vector3(...srcPos), new THREE.Vector3(...wireDrag.cursorWorld)];
    (dragLineRef.current.geometry as THREE.BufferGeometry).setFromPoints(points);
  });

  const handlePointerMove = useCallback((e: PointerEvent) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging') return;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(plane, intersection);
    setWireDrag({ phase: 'dragging', sourceNodeId: drag.sourceNodeId, cursorWorld: [intersection.x, intersection.y, intersection.z] });
  }, [camera, gl, setWireDrag]);

  const startDrag = useCallback((nodeId: string) => {
    if (activeTool !== 'connect') return;
    setWireDrag({ phase: 'dragging', sourceNodeId: nodeId, cursorWorld: [0, 0, 0] });
    gl.domElement.addEventListener('pointermove', handlePointerMove);
  }, [activeTool, gl, setWireDrag, handlePointerMove]);

  const completeDrag = useCallback((targetNodeId: string) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging' || drag.sourceNodeId === targetNodeId) {
      setWireDrag({ phase: 'idle' });
      gl.domElement.removeEventListener('pointermove', handlePointerMove);
      return;
    }
    const st = useStudioStore.getState();
    const controls = autoWireControls(drag.sourceNodeId, targetNodeId, {
      paths: st.paths,
      satelliteStart: st.satelliteStart,
      points: st.points,
    });
    addWire({
      id: `wire-${Date.now()}`,
      fromNodeId: drag.sourceNodeId,
      toNodeId: targetNodeId,
      waypoints: controls ?? undefined,
    });
    setWireDrag({ phase: 'idle' });
    gl.domElement.removeEventListener('pointermove', handlePointerMove);
  }, [addWire, setWireDrag, gl, handlePointerMove]);

  const handleNodeClick = useCallback((nodeId: string) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging') {
      startDrag(nodeId);
      return;
    }
    completeDrag(nodeId);
  }, [startDrag, completeDrag]);

  useEffect(() => {
    if (activeTool === 'connect') return;
    gl.domElement.removeEventListener('pointermove', handlePointerMove);
    if (useStudioStore.getState().wireDrag.phase === 'dragging') {
      setWireDrag({ phase: 'idle' });
    }
    setSelectedWirePoint(null);
  }, [activeTool, gl, handlePointerMove, setWireDrag]);

  useEffect(() => {
    if (!selectedWirePoint) return;
    const stillVisible = visibleWires.some((wire) => wire.id === selectedWirePoint.wireId);
    if (!stillVisible) {
      setSelectedWirePoint(null);
    }
  }, [selectedWirePoint, visibleWires]);

  return (
    <group>
      {visibleWires.map((wire) => {
        const src = resolveNodePosition(wire.fromNodeId, nodeState);
        const dst = resolveNodePosition(wire.toNodeId, nodeState);
        if (!src || !dst) return null;
        const density = wireDensityScale(wire, nodeState);
        const unconstrained = wire.waypoints && wire.waypoints.length >= 2
          ? [...wire.waypoints]
          : autoWireControls(wire.fromNodeId, wire.toNodeId, nodeState) ?? sampleConnectorPoints(src, dst, density);
        const controls = constrainWireControls(unconstrained, wire.fromNodeId, wire.toNodeId, nodeState);
        const denseSpacing = Math.min(0.05, 1 / Math.max(0.25, Math.min(25, density)));
        const dense = sampleCatmullRomBySpacing(controls, denseSpacing);
        dense[0] = src;
        dense[dense.length - 1] = dst;
        const dotSpacing = 1 / Math.max(0.25, Math.min(25, density));
        const sampled = sampleCatmullRomBySpacing(controls, dotSpacing);
        sampled[0] = src;
        sampled[sampled.length - 1] = dst;
        const lineVec = dense.map((p) => new THREE.Vector3(...p));
        const dotVec = sampled.map((p) => new THREE.Vector3(...p));
        const geom = new THREE.BufferGeometry().setFromPoints(lineVec);
        const pointGeom = new THREE.BufferGeometry().setFromPoints(dotVec);
        return (
          <group key={wire.id}>
            <line geometry={geom}>
              <lineBasicMaterial color="#f59e0b" opacity={0.98} transparent />
            </line>
            <points geometry={pointGeom}>
              <pointsMaterial color="#fdba74" size={0.06} sizeAttenuation opacity={0.95} transparent />
            </points>
            {activeTool === 'edit' && controls.map((p, i) => {
              const isEndpoint = i === 0 || i === controls.length - 1;
              const selected = selectedWirePoint?.wireId === wire.id && selectedWirePoint?.index === i;
              return (
                <mesh
                  key={`${wire.id}-wp-${i}`}
                  position={p}
                  onClick={(e) => {
                    e.stopPropagation();
                    if (isEndpoint) return;
                    setSelectedWirePoint((prev) =>
                      prev && prev.wireId === wire.id && prev.index === i ? null : { wireId: wire.id, index: i }
                    );
                  }}
                >
                  <sphereGeometry args={[selected ? 0.11 : 0.08, 10, 10]} />
                  <meshBasicMaterial color={selected ? '#fde047' : '#fbbf24'} opacity={0.95} transparent />
                </mesh>
              );
            })}
            {activeTool === 'edit' &&
              selectedWirePoint?.wireId === wire.id &&
              selectedWirePoint.index > 0 &&
              selectedWirePoint.index < controls.length - 1 && (
                <TransformControls
                  mode="translate"
                  space="world"
                  position={controls[selectedWirePoint.index]}
                  onObjectChange={(e: any) => {
                    if (!e?.target?.dragging) return;
                    const obj = e?.target?.object as THREE.Object3D | undefined;
                    if (!obj) return;
                    const next = applyLocalWireDeform(
                      [...controls] as [number, number, number][],
                      selectedWirePoint.index,
                      [obj.position.x, obj.position.y, obj.position.z]
                    );
                    next[0] = src;
                    next[next.length - 1] = dst;
                    const constrained = constrainWireControls(next, wire.fromNodeId, wire.toNodeId, nodeState);
                    setWireWaypoints(wire.id, constrained);
                  }}
                />
              )}
          </group>
        );
      })}

      {activeTool === 'connect' && (
        <>
      {wireDrag.phase === 'dragging' && (
        // @ts-expect-error r3f ref typing
        <line ref={dragLineRef}>
          <bufferGeometry />
          <lineDashedMaterial color="#22d3ee" dashSize={0.3} gapSize={0.2} linewidth={1} />
        </line>
      )}

      {(connectNodeSet == null || connectNodeSet.has('satellite:start')) && (
        <EndpointSphere
          position={satelliteStart}
          color="#ffffff"
          pulse
          onClick={() => handleNodeClick('satellite:start')}
        />
      )}

      {paths.map((path) => {
        if (path.waypoints.length === 0) return null;
        const startId = `path:${path.id}:start`;
        const endId = `path:${path.id}:end`;
        const showStart = connectNodeSet == null || connectNodeSet.has(startId);
        const showEnd = connectNodeSet == null || connectNodeSet.has(endId);
        if (!showStart && !showEnd) return null;
        return (
          <group key={path.id}>
            {showStart && (
              <EndpointSphere
                position={path.waypoints[0]}
                color="#22d3ee"
                pulse
                onClick={() => handleNodeClick(startId)}
              />
            )}
            {showEnd && (
              <EndpointSphere
                position={path.waypoints[path.waypoints.length - 1]}
                color="#a78bfa"
                pulse
                onClick={() => handleNodeClick(endId)}
              />
            )}
          </group>
        );
      })}
      {points.map((point) => {
        const nodeId = `point:${point.id}`;
        const show = connectNodeSet == null || connectNodeSet.has(nodeId);
        if (!show) return null;
        return (
          <EndpointSphere
            key={point.id}
            position={point.position}
            color="#38bdf8"
            pulse
            onClick={() => handleNodeClick(nodeId)}
          />
        );
      })}
        </>
      )}
    </group>
  );
}

function EndpointSphere({
  position,
  color,
  pulse,
  onClick,
}: {
  position: [number, number, number];
  color: string;
  pulse: boolean;
  onClick: () => void;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  useFrame(({ clock }) => {
    if (!meshRef.current || !pulse) return;
    const s = 1 + 0.2 * Math.sin(clock.getElapsedTime() * 4);
    meshRef.current.scale.setScalar(s);
  });
  return (
    <mesh
      ref={meshRef}
      position={position}
      onClick={(e) => {
        e.stopPropagation();
        onClick();
      }}
    >
      <sphereGeometry args={[0.45, 16, 16]} />
      <meshBasicMaterial color={color} />
    </mesh>
  );
}

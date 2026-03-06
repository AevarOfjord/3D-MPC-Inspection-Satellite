import { useRef, useCallback, useEffect, useMemo, useState } from 'react';
import * as THREE from 'three';
import { useThree, useFrame } from '@react-three/fiber';
import { Html, Line, TransformControls } from '@react-three/drei';
import { useStudioStore } from './useStudioStore';
import { fairCorners, sampleCatmullRomBySpacing } from './splineUtils';
import { canConnectStudioNodes } from './studioRouteDiagnostics';

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

function anchorWireEndpoints(
  controls: [number, number, number][],
  src: [number, number, number],
  dst: [number, number, number]
): [number, number, number][] {
  if (!controls || controls.length < 2) return [src, dst];
  const next = controls.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
  next[0] = src;
  next[next.length - 1] = dst;
  return next;
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
  const holds = useStudioStore((s) => s.holds);
  const wireDrag = useStudioStore((s) => s.wireDrag);
  const activeTool = useStudioStore((s) => s.activeTool);
  const setWireDrag = useStudioStore((s) => s.setWireDrag);
  const addWire = useStudioStore((s) => s.addWire);
  const setWireWaypoints = useStudioStore((s) => s.setWireWaypoints);
  const assembly = useStudioStore((s) => s.assembly);
  const setSelectedAssemblyId = useStudioStore((s) => s.setSelectedAssemblyId);
  const { camera, gl } = useThree();
  const raycaster = useRef(new THREE.Raycaster());
  const dragLineRef = useRef<THREE.Line>(null);
  const nodeState = { paths, satelliteStart, points };
  const [selectedWirePoint, setSelectedWirePoint] = useState<{ wireId: string; index: number } | null>(null);
  const [hoveredNodeId, setHoveredNodeId] = useState<string | null>(null);
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
  const connectState = useMemo(
    () => ({
      referenceObjectPath: null,
      paths,
      wires,
      holds,
      points,
      assembly,
    }),
    [paths, wires, holds, points, assembly]
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
    if (drag.phase !== 'dragging') {
      return;
    }
    if (drag.sourceNodeId === targetNodeId) {
      return;
    }
    const st = useStudioStore.getState();
    const connectCheck = canConnectStudioNodes(
      {
        referenceObjectPath: null,
        paths: st.paths,
        wires: st.wires,
        holds: st.holds,
        points: st.points,
        assembly: st.assembly,
      },
      drag.sourceNodeId,
      targetNodeId
    );
    if (!connectCheck.ok) {
      return;
    }
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
      constraintMode: 'constrained',
    });
    setWireDrag({ phase: 'idle' });
    setHoveredNodeId(null);
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
    setHoveredNodeId(null);
    setSelectedWirePoint(null);
  }, [activeTool, gl, handlePointerMove, setWireDrag]);

  useEffect(() => {
    if (!selectedWirePoint) return;
    const stillVisible = visibleWires.some((wire) => wire.id === selectedWirePoint.wireId);
    if (!stillVisible) {
      setSelectedWirePoint(null);
    }
  }, [selectedWirePoint, visibleWires]);

  const getNodeVisual = useCallback(
    (nodeId: string, baseColor: string) => {
      if (wireDrag.phase !== 'dragging') {
        return {
          color: baseColor,
          haloColor: baseColor,
          pulse: true,
          emphasis: 'idle' as const,
          tooltip: null,
        };
      }
      if (wireDrag.sourceNodeId === nodeId) {
        return {
          color: '#f8fafc',
          haloColor: '#38bdf8',
          pulse: true,
          emphasis: 'source' as const,
          tooltip: 'Connection source',
        };
      }
      const check = canConnectStudioNodes(connectState, wireDrag.sourceNodeId, nodeId);
      if (check.ok) {
        return {
          color: '#4ade80',
          haloColor: hoveredNodeId === nodeId ? '#86efac' : '#22c55e',
          pulse: true,
          emphasis: hoveredNodeId === nodeId ? ('hover-valid' as const) : ('valid' as const),
          tooltip: hoveredNodeId === nodeId ? 'Click to connect' : null,
        };
      }
      return {
        color: '#f87171',
        haloColor: hoveredNodeId === nodeId ? '#fca5a5' : '#ef4444',
        pulse: false,
        emphasis: hoveredNodeId === nodeId ? ('hover-invalid' as const) : ('invalid' as const),
        tooltip: hoveredNodeId === nodeId ? check.reason ?? 'Invalid target' : null,
      };
    },
    [connectState, hoveredNodeId, wireDrag]
  );

  return (
    <group>
      {visibleWires.map((wire) => {
        const src = resolveNodePosition(wire.fromNodeId, nodeState);
        const dst = resolveNodePosition(wire.toNodeId, nodeState);
        if (!src || !dst) return null;
        const density = wireDensityScale(wire, nodeState);
        const constraintMode = wire.constraintMode ?? 'constrained';
        const unconstrained = wire.waypoints && wire.waypoints.length >= 2
          ? [...wire.waypoints]
          : autoWireControls(wire.fromNodeId, wire.toNodeId, nodeState) ?? sampleConnectorPoints(src, dst, density);
        const controls = constraintMode === 'free'
          ? anchorWireEndpoints(unconstrained, src, dst)
          : constrainWireControls(unconstrained, wire.fromNodeId, wire.toNodeId, nodeState);
        const denseSpacing = Math.min(0.05, 1 / Math.max(0.25, Math.min(25, density)));
        const dense = sampleCatmullRomBySpacing(controls, denseSpacing);
        dense[0] = src;
        dense[dense.length - 1] = dst;
        const dotSpacing = 1 / Math.max(0.25, Math.min(25, density));
        const sampled = sampleCatmullRomBySpacing(controls, dotSpacing);
        sampled[0] = src;
        sampled[sampled.length - 1] = dst;
        const dotVec = sampled.map((p) => new THREE.Vector3(...p));
        const pointGeom = new THREE.BufferGeometry().setFromPoints(dotVec);
        return (
          <group key={wire.id}>
            <Line points={dense} color="#f59e0b" transparent opacity={0.98} lineWidth={1.5} />
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
                    const connectAssembly = assembly.find((item) => item.type === 'connect' && item.wireId === wire.id);
                    setSelectedAssemblyId(connectAssembly?.id ?? null);
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
                    const endpointAnchored = anchorWireEndpoints(next, src, dst);
                    const updated = constraintMode === 'free'
                      ? endpointAnchored
                      : constrainWireControls(endpointAnchored, wire.fromNodeId, wire.toNodeId, nodeState);
                    setWireWaypoints(wire.id, updated);
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
          {...getNodeVisual('satellite:start', '#ffffff')}
          onClick={() => handleNodeClick('satellite:start')}
          onHoverChange={(hovered) => setHoveredNodeId(hovered ? 'satellite:start' : null)}
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
                {...getNodeVisual(startId, '#22d3ee')}
                onClick={() => handleNodeClick(startId)}
                onHoverChange={(hovered) => setHoveredNodeId(hovered ? startId : null)}
              />
            )}
            {showEnd && (
              <EndpointSphere
                position={path.waypoints[path.waypoints.length - 1]}
                {...getNodeVisual(endId, '#a78bfa')}
                onClick={() => handleNodeClick(endId)}
                onHoverChange={(hovered) => setHoveredNodeId(hovered ? endId : null)}
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
            {...getNodeVisual(nodeId, '#38bdf8')}
            onClick={() => handleNodeClick(nodeId)}
            onHoverChange={(hovered) => setHoveredNodeId(hovered ? nodeId : null)}
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
  haloColor,
  pulse,
  emphasis,
  tooltip,
  onClick,
  onHoverChange,
}: {
  position: [number, number, number];
  color: string;
  haloColor: string;
  pulse: boolean;
  emphasis: 'idle' | 'source' | 'valid' | 'invalid' | 'hover-valid' | 'hover-invalid';
  tooltip: string | null;
  onClick: () => void;
  onHoverChange: (hovered: boolean) => void;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  useFrame(({ clock }) => {
    if (!meshRef.current || !pulse) return;
    const s = 1 + 0.2 * Math.sin(clock.getElapsedTime() * 4);
    meshRef.current.scale.setScalar(s);
  });
  const haloScale =
    emphasis === 'source'
      ? 1.95
      : emphasis === 'hover-valid' || emphasis === 'hover-invalid'
        ? 1.8
        : emphasis === 'valid'
          ? 1.65
          : emphasis === 'invalid'
            ? 1.55
            : 1.5;
  const haloOpacity =
    emphasis === 'source'
      ? 0.5
      : emphasis === 'hover-valid' || emphasis === 'hover-invalid'
        ? 0.42
        : emphasis === 'invalid'
          ? 0.28
          : 0.22;
  const coreRadius =
    emphasis === 'hover-valid' || emphasis === 'hover-invalid'
      ? 0.52
      : emphasis === 'source'
        ? 0.5
        : 0.45;
  return (
    <group
      position={position}
      onPointerOver={(e) => {
        e.stopPropagation();
        onHoverChange(true);
      }}
      onPointerOut={(e) => {
        e.stopPropagation();
        onHoverChange(false);
      }}
      onClick={(e) => {
        e.stopPropagation();
        onClick();
      }}
    >
      <mesh scale={haloScale}>
        <sphereGeometry args={[0.45, 16, 16]} />
        <meshBasicMaterial color={haloColor} transparent opacity={haloOpacity} depthWrite={false} />
      </mesh>
      <mesh ref={meshRef}>
        <sphereGeometry args={[coreRadius, 16, 16]} />
        <meshBasicMaterial color={color} />
      </mesh>
      {tooltip ? (
        <Html position={[0, 0.95, 0]} center distanceFactor={14}>
          <div
            className={`rounded-md border px-2 py-1 text-[10px] font-medium shadow-lg ${
              emphasis === 'hover-invalid'
                ? 'border-red-500/60 bg-red-950/90 text-red-100'
                : 'border-cyan-500/60 bg-cyan-950/90 text-cyan-50'
            }`}
          >
            {tooltip}
          </div>
        </Html>
      ) : null}
    </group>
  );
}

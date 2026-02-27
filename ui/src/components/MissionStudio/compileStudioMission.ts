import type { StudioState } from './useStudioStore';
import type {
  UnifiedMission,
  MissionSegment,
  TransferSegment,
  ScanSegment,
  HoldSegment,
} from '../../api/unifiedMission';
import { studioTargetIdFromModelPath } from './studioReference';
import { fairCorners, sampleCatmullRomBySpacing } from './splineUtils';

interface RouteBuildResult {
  manualPath: [number, number, number][];
  segments: MissionSegment[];
  holdSchedule: { path_index: number; duration_s: number }[];
}

function samePoint(a: [number, number, number], b: [number, number, number], eps = 1e-6): boolean {
  return Math.abs(a[0] - b[0]) <= eps && Math.abs(a[1] - b[1]) <= eps && Math.abs(a[2] - b[2]) <= eps;
}

function parsePathNode(nodeId: string): { pathId: string; endpoint: 'start' | 'end' } | null {
  const parts = nodeId.split(':');
  if (parts.length !== 3 || parts[0] !== 'path') return null;
  if (parts[2] !== 'start' && parts[2] !== 'end') return null;
  return { pathId: parts[1], endpoint: parts[2] };
}

function parsePointNode(nodeId: string): { pointId: string } | null {
  if (!nodeId.startsWith('point:')) return null;
  const pointId = nodeId.slice('point:'.length);
  return pointId.length > 0 ? { pointId } : null;
}

function resolveNodePosition(nodeId: string, state: StudioState): [number, number, number] {
  if (nodeId === 'satellite:start') return state.satelliteStart;
  const point = parsePointNode(nodeId);
  if (point) {
    const p = state.points.find((item) => item.id === point.pointId);
    if (!p) throw new Error(`Unknown point node: ${nodeId}`);
    return p.position;
  }
  const parsed = parsePathNode(nodeId);
  if (!parsed) throw new Error(`Unknown node: ${nodeId}`);
  const path = state.paths.find((p) => p.id === parsed.pathId);
  if (!path || path.waypoints.length < 2) {
    throw new Error(`Path '${parsed.pathId}' is missing or has <2 waypoints`);
  }
  return parsed.endpoint === 'start' ? path.waypoints[0] : path.waypoints[path.waypoints.length - 1];
}

function normalizeVec(v: [number, number, number]): [number, number, number] {
  const n = Math.hypot(v[0], v[1], v[2]);
  if (n <= 1e-9) return [0, 0, 0];
  return [v[0] / n, v[1] / n, v[2] / n];
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

function resolveNodeTangent(
  nodeId: string,
  state: StudioState,
  role: 'from' | 'to',
  other: [number, number, number]
): [number, number, number] {
  const pos = resolveNodePosition(nodeId, state);
  const parsed = parsePathNode(nodeId);
  if (!parsed) {
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

function autoWireControls(
  fromNodeId: string,
  toNodeId: string,
  state: StudioState
): [number, number, number][] {
  const src = resolveNodePosition(fromNodeId, state);
  const dst = resolveNodePosition(toNodeId, state);
  const dist = Math.hypot(dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]);
  if (dist <= 1e-9) return [src];
  const tSrc = resolveNodeTangent(fromNodeId, state, 'from', dst);
  const tDst = resolveNodeTangent(toNodeId, state, 'to', src);
  const handle = Math.max(0.5, Math.min(dist * 0.45, 0.28 * dist + 0.6));
  return [
    src,
    [src[0] + tSrc[0] * handle, src[1] + tSrc[1] * handle, src[2] + tSrc[2] * handle],
    [dst[0] - tDst[0] * handle, dst[1] - tDst[1] * handle, dst[2] - tDst[2] * handle],
    dst,
  ];
}

function constrainWireControls(
  controls: [number, number, number][],
  fromNodeId: string,
  toNodeId: string,
  state: StudioState
): [number, number, number][] {
  if (!controls || controls.length < 2) return autoWireControls(fromNodeId, toNodeId, state);
  const src = resolveNodePosition(fromNodeId, state);
  const dst = resolveNodePosition(toNodeId, state);
  if (controls.length < 4) return autoWireControls(fromNodeId, toNodeId, state);

  const next = controls.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
  next[0] = src;
  next[next.length - 1] = dst;
  const dist = Math.hypot(dst[0] - src[0], dst[1] - src[1], dst[2] - src[2]);
  if (dist <= 1e-9) return [src];

  const tSrc = resolveNodeTangent(fromNodeId, state, 'from', dst);
  const tDst = resolveNodeTangent(toNodeId, state, 'to', src);
  const handleMin = Math.max(0.2, dist * 0.03);
  const handleMax = Math.max(handleMin, dist * 0.95);
  const last = next.length - 1;
  const srcHandle = clamp(
    Math.hypot(next[1][0] - src[0], next[1][1] - src[1], next[1][2] - src[2]),
    handleMin,
    handleMax
  );
  const dstHandle = clamp(
    Math.hypot(next[last - 1][0] - dst[0], next[last - 1][1] - dst[1], next[last - 1][2] - dst[2]),
    handleMin,
    handleMax
  );
  next[1] = [src[0] + tSrc[0] * srcHandle, src[1] + tSrc[1] * srcHandle, src[2] + tSrc[2] * srcHandle];
  next[last - 1] = [dst[0] - tDst[0] * dstHandle, dst[1] - tDst[1] * dstHandle, dst[2] - tDst[2] * dstHandle];
  return next;
}

function appendSampledConnector(
  out: [number, number, number][],
  from: [number, number, number],
  to: [number, number, number],
  waypointDensity: number
) {
  const dx = to[0] - from[0];
  const dy = to[1] - from[1];
  const dz = to[2] - from[2];
  const dist = Math.hypot(dx, dy, dz);
  if (dist <= 1e-9) return;
  const density = Math.max(0.25, Math.min(25, waypointDensity || 1));
  const spacing = 1.0 / density; // 1x => 1m spacing
  const steps = Math.max(1, Math.ceil(dist / spacing));
  for (let i = 1; i <= steps; i += 1) {
    const t = i / steps;
    out.push([from[0] + dx * t, from[1] + dy * t, from[2] + dz * t]);
  }
}

function appendConnectorFromWire(
  out: [number, number, number][],
  wire: { fromNodeId: string; toNodeId: string; waypoints?: [number, number, number][] } | null,
  from: [number, number, number],
  to: [number, number, number],
  waypointDensity: number,
  state: StudioState
) {
  const custom = wire?.waypoints;
  if (custom && custom.length >= 2) {
    const fwd = samePoint(custom[0], from, 1e-3) && samePoint(custom[custom.length - 1], to, 1e-3);
    const rev = samePoint(custom[0], to, 1e-3) && samePoint(custom[custom.length - 1], from, 1e-3);
    const orientedRaw = fwd ? custom : rev ? [...custom].reverse() : custom;
    const oriented = constrainWireControls(orientedRaw, wire.fromNodeId, wire.toNodeId, state);
    const density = Math.max(0.25, Math.min(25, waypointDensity || 1));
    const sampled = sampleCatmullRomBySpacing(oriented, Math.min(0.1, 1 / density));
    if (out.length === 0 || !samePoint(out[out.length - 1], sampled[0])) out.push(sampled[0]);
    for (let i = 1; i < sampled.length; i += 1) out.push(sampled[i]);
    return;
  }
  if (wire) {
    const auto = constrainWireControls(autoWireControls(wire.fromNodeId, wire.toNodeId, state), wire.fromNodeId, wire.toNodeId, state);
    const density = Math.max(0.25, Math.min(25, waypointDensity || 1));
    const sampled = sampleCatmullRomBySpacing(auto, Math.min(0.1, 1 / density));
    if (out.length === 0 || !samePoint(out[out.length - 1], sampled[0])) out.push(sampled[0]);
    for (let i = 1; i < sampled.length; i += 1) out.push(sampled[i]);
    return;
  }
  appendSampledConnector(out, from, to, waypointDensity);
}

function buildRouteGraph(state: StudioState): RouteBuildResult {
  const outgoing = new Map<string, { id: string; to: string }>();
  const incoming = new Map<string, { id: string; from: string }>();

  for (const w of state.wires) {
    if (!resolveNodePositionSafe(w.fromNodeId, state) || !resolveNodePositionSafe(w.toNodeId, state)) {
      throw new Error(`Wire '${w.id}' references unknown nodes`);
    }
    if (outgoing.has(w.fromNodeId)) throw new Error(`Branching is not allowed at node '${w.fromNodeId}'`);
    if (incoming.has(w.toNodeId)) throw new Error(`Multiple incoming edges are not allowed at node '${w.toNodeId}'`);
    outgoing.set(w.fromNodeId, { id: w.id, to: w.toNodeId });
    incoming.set(w.toNodeId, { id: w.id, from: w.fromNodeId });
  }

  if (incoming.has('satellite:start')) {
    throw new Error('satellite:start cannot have incoming connections');
  }

  const manualPath: [number, number, number][] = [state.satelliteStart];
  const holdSchedule: { path_index: number; duration_s: number }[] = [];
  const segments: MissionSegment[] = [];

  let currentNode = 'satellite:start';
  let currentPos: [number, number, number] = state.satelliteStart;
  const visitedWireIds = new Set<string>();
  const visitedPathIds = new Set<string>();

  const maxSteps = state.wires.length + state.paths.length + 10;
  let step = 0;

  while (step < maxSteps) {
    step += 1;
    const edge = outgoing.get(currentNode);
    if (!edge) break;
    if (visitedWireIds.has(edge.id)) throw new Error('Cycle detected in connection graph');
    visitedWireIds.add(edge.id);

    const parsedPath = parsePathNode(edge.to);
    if (!parsedPath) {
      const parsedPoint = parsePointNode(edge.to);
      if (!parsedPoint) {
        throw new Error(`Wire target '${edge.to}' is invalid. Connect only to path endpoints or points.`);
      }
      const pointPos = resolveNodePosition(edge.to, state);
      if (!samePoint(currentPos, pointPos)) {
        const wire = state.wires.find((w) => w.id === edge.id) ?? null;
        appendConnectorFromWire(manualPath, wire, currentPos, pointPos, 1, state);
        const transferSeg: TransferSegment = {
          segment_id: `transfer-${edge.id}`,
          type: 'transfer',
          end_pose: {
            frame: 'LVLH',
            position: pointPos,
            orientation: [1, 0, 0, 0],
          },
        };
        segments.push(transferSeg);
      }
      currentPos = pointPos;
      currentNode = edge.to;
      continue;
    }

    const path = state.paths.find((p) => p.id === parsedPath.pathId);
    if (!path || path.waypoints.length < 2) throw new Error(`Path '${parsedPath.pathId}' is missing or invalid`);
    if (visitedPathIds.has(path.id)) throw new Error(`Path '${path.id}' is connected more than once`);

    const controlOriented = parsedPath.endpoint === 'start'
      ? fairCorners(path.waypoints, 150, 2)
      : fairCorners([...path.waypoints].reverse(), 150, 2);
    const density = Math.max(0.25, Math.min(25, path.waypointDensity ?? 1));
    const oriented = sampleCatmullRomBySpacing(controlOriented, Math.min(0.1, 1 / density));
    const entry = oriented[0];

    if (!samePoint(currentPos, entry)) {
      const wire = state.wires.find((w) => w.id === edge.id) ?? null;
      appendConnectorFromWire(manualPath, wire, currentPos, entry, path.waypointDensity ?? 1, state);
      const transferSeg: TransferSegment = {
        segment_id: `transfer-${edge.id}`,
        type: 'transfer',
        end_pose: {
          frame: 'LVLH',
          position: entry,
          orientation: [1, 0, 0, 0],
        },
      };
      segments.push(transferSeg);
    }

    const pathBaseIndex = manualPath.length - 1;
    for (let i = 1; i < oriented.length; i += 1) manualPath.push(oriented[i]);

    const scanSeg: ScanSegment = {
      segment_id: `scan-${path.id}`,
      type: 'scan',
      target_id: studioTargetIdFromModelPath(state.referenceObjectPath),
      target_pose: {
        frame: 'LVLH',
        position: [0, 0, 0],
        orientation: [1, 0, 0, 0],
      },
      scan: {
        frame: 'LVLH',
        axis: `+${path.axisSeed}` as '+X' | '+Y' | '+Z',
        standoff: 10,
        overlap: 0.1,
        fov_deg: 60,
        revolutions: Math.max(1, Math.round(path.waypoints.length / 24)),
        direction: 'CW',
        sensor_axis: '+Y',
        pattern: 'spiral',
        level_spacing_m: path.levelSpacing,
      },
    };
    segments.push(scanSeg);

    const holdsForPath = state.holds
      .filter((h) => h.pathId === path.id)
      .sort((a, b) => a.waypointIndex - b.waypointIndex);

    for (const hold of holdsForPath) {
      const localControlIdx = parsedPath.endpoint === 'start'
        ? hold.waypointIndex
        : Math.max(0, path.waypoints.length - 1 - hold.waypointIndex);
      const ratio = path.waypoints.length > 1
        ? localControlIdx / Math.max(1, path.waypoints.length - 1)
        : 0;
      const localSampleIdx = Math.round(ratio * Math.max(0, oriented.length - 1));
      const globalIdx = Math.max(0, Math.min(pathBaseIndex + localSampleIdx, manualPath.length - 1));
      holdSchedule.push({ path_index: globalIdx, duration_s: Math.max(0, hold.duration) });
      const holdSeg: HoldSegment = {
        segment_id: `hold-${hold.id}`,
        type: 'hold',
        duration: Math.max(0, hold.duration),
      };
      segments.push(holdSeg);
    }

    visitedPathIds.add(path.id);
    currentPos = oriented[oriented.length - 1];
    currentNode = `path:${path.id}:${parsedPath.endpoint === 'start' ? 'end' : 'start'}`;
  }

  if (state.wires.length > 0 && !outgoing.has('satellite:start')) {
    throw new Error('Executable route must begin at satellite:start');
  }
  if (visitedWireIds.size !== state.wires.length) {
    throw new Error('All connections must belong to one continuous unbranched route from satellite:start');
  }

  const connectedPathCount = visitedPathIds.size;
  const authoredPathCount = state.paths.filter((p) => p.waypoints.length >= 2).length;
  if (connectedPathCount !== authoredPathCount) {
    throw new Error('Every valid path must be connected into the executable route exactly once');
  }

  for (const hold of state.holds) {
    if (!visitedPathIds.has(hold.pathId)) {
      throw new Error(`Hold '${hold.id}' is on path '${hold.pathId}' which is not in executable route`);
    }
  }

  return { manualPath, segments, holdSchedule };
}

function resolveNodePositionSafe(nodeId: string, state: StudioState): [number, number, number] | null {
  try {
    return resolveNodePosition(nodeId, state);
  } catch {
    return null;
  }
}

export function compileStudioMission(state: StudioState): UnifiedMission {
  const referenceTargetId = studioTargetIdFromModelPath(state.referenceObjectPath);
  for (const obstacle of state.obstacles) {
    if (!(obstacle.radius > 0)) {
      throw new Error(`Obstacle '${obstacle.id}' must have radius > 0`);
    }
  }
  const route = buildRouteGraph(state);

  const mission: UnifiedMission = {
    schema_version: 2,
    mission_id: `studio-${Date.now()}`,
    name: state.missionName || 'Untitled Studio Mission',
    epoch: new Date().toISOString(),
    start_pose: {
      frame: 'LVLH',
      position: state.satelliteStart,
      orientation: [1, 0, 0, 0],
    },
    start_target_id: referenceTargetId,
    segments: route.segments,
    obstacles: state.obstacles.map((o) => ({
      position: o.position,
      radius: o.radius,
    })),
    overrides: {
      manual_path: route.manualPath,
      hold_schedule: route.holdSchedule,
    },
    metadata: {
      version: 1,
      created_at: new Date().toISOString(),
    },
  };

  return mission;
}

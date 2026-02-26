import type { StudioState } from './useStudioStore';
import type {
  UnifiedMission,
  MissionSegment,
  TransferSegment,
  ScanSegment,
  HoldSegment,
} from '../../api/unifiedMission';
import { studioTargetIdFromModelPath } from './studioReference';

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

function resolveNodePosition(nodeId: string, state: StudioState): [number, number, number] {
  if (nodeId === 'satellite:start') return state.satelliteStart;
  const parsed = parsePathNode(nodeId);
  if (!parsed) throw new Error(`Unknown node: ${nodeId}`);
  const path = state.paths.find((p) => p.id === parsed.pathId);
  if (!path || path.waypoints.length < 2) {
    throw new Error(`Path '${parsed.pathId}' is missing or has <2 waypoints`);
  }
  return parsed.endpoint === 'start' ? path.waypoints[0] : path.waypoints[path.waypoints.length - 1];
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
  waypointDensity: number
) {
  const custom = wire?.waypoints;
  if (custom && custom.length >= 2) {
    const fwd = samePoint(custom[0], from, 1e-3) && samePoint(custom[custom.length - 1], to, 1e-3);
    const rev = samePoint(custom[0], to, 1e-3) && samePoint(custom[custom.length - 1], from, 1e-3);
    const oriented = fwd ? custom : rev ? [...custom].reverse() : custom;
    if (out.length === 0 || !samePoint(out[out.length - 1], oriented[0])) out.push(oriented[0]);
    for (let i = 1; i < oriented.length; i += 1) out.push(oriented[i]);
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

    const parsed = parsePathNode(edge.to);
    if (!parsed) {
      throw new Error(`Wire target '${edge.to}' is invalid. Connect only to path endpoints.`);
    }

    const path = state.paths.find((p) => p.id === parsed.pathId);
    if (!path || path.waypoints.length < 2) throw new Error(`Path '${parsed.pathId}' is missing or invalid`);
    if (visitedPathIds.has(path.id)) throw new Error(`Path '${path.id}' is connected more than once`);

    const oriented = parsed.endpoint === 'start' ? path.waypoints : [...path.waypoints].reverse();
    const entry = oriented[0];

    if (!samePoint(currentPos, entry)) {
      const wire = state.wires.find((w) => w.id === edge.id) ?? null;
      appendConnectorFromWire(manualPath, wire, currentPos, entry, path.waypointDensity ?? 1);
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
      const localIdx = parsed.endpoint === 'start'
        ? hold.waypointIndex
        : Math.max(0, path.waypoints.length - 1 - hold.waypointIndex);
      const globalIdx = Math.max(0, Math.min(pathBaseIndex + localIdx, manualPath.length - 1));
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
    currentNode = `path:${path.id}:${parsed.endpoint === 'start' ? 'end' : 'start'}`;
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

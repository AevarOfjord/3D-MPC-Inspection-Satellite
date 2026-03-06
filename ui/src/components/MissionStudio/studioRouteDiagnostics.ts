import type { StudioState } from './useStudioStore';

type StudioGraphState = Pick<
  StudioState,
  'referenceObjectPath' | 'paths' | 'wires' | 'holds' | 'points' | 'assembly'
>;

export type StudioRouteStatus =
  | 'empty'
  | 'draft'
  | 'incomplete'
  | 'invalid'
  | 'executable';

export interface StudioRouteDiagnostics {
  status: StudioRouteStatus;
  executable: boolean;
  targetMode: 'object' | 'local';
  hasSatellitePlacement: boolean;
  totalPathCount: number;
  validPathCount: number;
  totalWireCount: number;
  pointCount: number;
  holdCount: number;
  invalidPathIds: string[];
  invalidWireIds: string[];
  branchingSources: string[];
  multiIncomingTargets: string[];
  disconnectedPathIds: string[];
  disconnectedWireIds: string[];
  unconnectedHoldIds: string[];
  cycleDetected: boolean;
  routeStartsAtSatellite: boolean;
  visitedPathIds: string[];
  nextAction: string;
  detailLines: string[];
}

export interface StudioConnectionCheck {
  ok: boolean;
  reason?: string;
}

function parsePathNode(
  nodeId: string
): { pathId: string; endpoint: 'start' | 'end' } | null {
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

function advanceTraversalNode(nodeId: string): string {
  const parsed = parsePathNode(nodeId);
  if (!parsed) return nodeId;
  return `path:${parsed.pathId}:${parsed.endpoint === 'start' ? 'end' : 'start'}`;
}

function buildNodeSet(state: StudioGraphState): Set<string> {
  const nodes = new Set<string>(['satellite:start']);
  for (const point of state.points) {
    nodes.add(`point:${point.id}`);
  }
  for (const path of state.paths) {
    if (path.waypoints.length >= 2) {
      nodes.add(`path:${path.id}:start`);
      nodes.add(`path:${path.id}:end`);
    }
  }
  return nodes;
}

export function getStudioRouteDiagnostics(
  state: StudioGraphState
): StudioRouteDiagnostics {
  const validPathIds = state.paths
    .filter((path) => path.waypoints.length >= 2)
    .map((path) => path.id);
  const invalidPathIds = state.paths
    .filter((path) => path.waypoints.length < 2)
    .map((path) => path.id);
  const validPathIdSet = new Set(validPathIds);
  const nodes = buildNodeSet(state);

  const outgoing = new Map<string, { id: string; to: string }>();
  const incoming = new Map<string, { id: string; from: string }>();
  const outgoingCounts = new Map<string, number>();
  const incomingCounts = new Map<string, number>();
  const invalidWireIds: string[] = [];
  const branchingSources = new Set<string>();
  const multiIncomingTargets = new Set<string>();

  for (const wire of state.wires) {
    const srcValid = nodes.has(wire.fromNodeId);
    const dstValid = nodes.has(wire.toNodeId);
    const selfLoop = wire.fromNodeId === wire.toNodeId;
    if (!srcValid || !dstValid || selfLoop) {
      invalidWireIds.push(wire.id);
      continue;
    }

    const nextOutgoing = (outgoingCounts.get(wire.fromNodeId) ?? 0) + 1;
    outgoingCounts.set(wire.fromNodeId, nextOutgoing);
    if (nextOutgoing > 1) branchingSources.add(wire.fromNodeId);

    const nextIncoming = (incomingCounts.get(wire.toNodeId) ?? 0) + 1;
    incomingCounts.set(wire.toNodeId, nextIncoming);
    if (nextIncoming > 1 || wire.toNodeId === 'satellite:start') {
      multiIncomingTargets.add(wire.toNodeId);
    }

    if (!outgoing.has(wire.fromNodeId)) {
      outgoing.set(wire.fromNodeId, { id: wire.id, to: wire.toNodeId });
    }
    if (!incoming.has(wire.toNodeId)) {
      incoming.set(wire.toNodeId, { id: wire.id, from: wire.fromNodeId });
    }
  }

  const visitedWireIds = new Set<string>();
  const visitedPathIds = new Set<string>();
  const maxSteps = state.wires.length + state.paths.length + state.points.length + 8;
  let cycleDetected = false;
  let currentNode = 'satellite:start';

  for (let step = 0; step < maxSteps; step += 1) {
    const edge = outgoing.get(currentNode);
    if (!edge) break;
    if (visitedWireIds.has(edge.id)) {
      cycleDetected = true;
      break;
    }
    visitedWireIds.add(edge.id);

    const nextPoint = parsePointNode(edge.to);
    if (nextPoint) {
      currentNode = edge.to;
      continue;
    }

    const nextPath = parsePathNode(edge.to);
    if (nextPath) {
      visitedPathIds.add(nextPath.pathId);
      currentNode = advanceTraversalNode(edge.to);
      continue;
    }

    break;
  }

  const disconnectedPathIds = validPathIds.filter((pathId) => !visitedPathIds.has(pathId));
  const disconnectedWireIds = state.wires
    .filter((wire) => !invalidWireIds.includes(wire.id) && !visitedWireIds.has(wire.id))
    .map((wire) => wire.id);
  const unconnectedHoldIds = state.holds
    .filter((hold) => !visitedPathIds.has(hold.pathId))
    .map((hold) => hold.id);

  const routeStartsAtSatellite = outgoing.has('satellite:start');
  const hasTopologyError =
    invalidWireIds.length > 0 ||
    branchingSources.size > 0 ||
    multiIncomingTargets.size > 0 ||
    cycleDetected;

  const executable =
    validPathIds.length > 0 &&
    invalidPathIds.length === 0 &&
    !hasTopologyError &&
    routeStartsAtSatellite &&
    disconnectedPathIds.length === 0 &&
    disconnectedWireIds.length === 0 &&
    unconnectedHoldIds.length === 0;

  let status: StudioRouteStatus = 'draft';
  if (validPathIds.length === 0 && state.wires.length === 0 && state.points.length === 0) {
    status = 'empty';
  } else if (executable) {
    status = 'executable';
  } else if (hasTopologyError) {
    status = 'invalid';
  } else {
    status = 'incomplete';
  }

  const detailLines: string[] = [];
  if (invalidPathIds.length > 0) {
    detailLines.push(
      invalidPathIds.length === 1
        ? `Generate waypoints for ${invalidPathIds[0]} before routing it.`
        : `Generate waypoints for ${invalidPathIds.length} paths before routing them.`
    );
  }
  if (!routeStartsAtSatellite && validPathIds.length > 0) {
    detailLines.push('Connect satellite:start to the first path or point.');
  }
  if (branchingSources.size > 0) {
    detailLines.push('Remove branching so each node has at most one outgoing connection.');
  }
  if (multiIncomingTargets.size > 0) {
    detailLines.push('Remove duplicate incoming edges so each node has at most one predecessor.');
  }
  if (cycleDetected) {
    detailLines.push('Break the route cycle so traversal can terminate cleanly.');
  }
  if (disconnectedPathIds.length > 0) {
    detailLines.push(
      disconnectedPathIds.length === 1
        ? `Connect ${disconnectedPathIds[0]} into the continuous route.`
        : `Connect all ${disconnectedPathIds.length} remaining paths into one continuous route.`
    );
  }
  if (disconnectedWireIds.length > 0) {
    detailLines.push(
      disconnectedWireIds.length === 1
        ? `Wire ${disconnectedWireIds[0]} is outside the executable route.`
        : `${disconnectedWireIds.length} wires are outside the executable route.`
    );
  }
  if (unconnectedHoldIds.length > 0) {
    detailLines.push('Move or remove holds that sit on paths outside the executable route.');
  }
  if (detailLines.length === 0 && executable) {
    detailLines.push('The authored route is continuous, executable, and ready for validation.');
  }
  if (detailLines.length === 0) {
    detailLines.push('Keep authoring the route until every path belongs to one continuous chain.');
  }

  return {
    status,
    executable,
    targetMode: state.referenceObjectPath ? 'object' : 'local',
    hasSatellitePlacement: state.assembly.some((item) => item.type === 'place_satellite'),
    totalPathCount: state.paths.length,
    validPathCount: validPathIds.length,
    totalWireCount: state.wires.length,
    pointCount: state.points.length,
    holdCount: state.holds.length,
    invalidPathIds,
    invalidWireIds,
    branchingSources: Array.from(branchingSources),
    multiIncomingTargets: Array.from(multiIncomingTargets),
    disconnectedPathIds,
    disconnectedWireIds,
    unconnectedHoldIds,
    cycleDetected,
    routeStartsAtSatellite,
    visitedPathIds: Array.from(visitedPathIds),
    nextAction: detailLines[0],
    detailLines,
  };
}

export function canConnectStudioNodes(
  state: StudioGraphState,
  sourceNodeId: string,
  targetNodeId: string
): StudioConnectionCheck {
  const nodes = buildNodeSet(state);
  if (!nodes.has(sourceNodeId) || !nodes.has(targetNodeId)) {
    return { ok: false, reason: 'Unknown endpoint.' };
  }
  if (sourceNodeId === targetNodeId) {
    return { ok: false, reason: 'Choose a different endpoint.' };
  }
  if (targetNodeId === 'satellite:start') {
    return { ok: false, reason: 'satellite:start cannot receive incoming connections.' };
  }

  const outgoing = new Map<string, { to: string }>();
  const incoming = new Map<string, { from: string }>();
  for (const wire of state.wires) {
    outgoing.set(wire.fromNodeId, { to: wire.toNodeId });
    incoming.set(wire.toNodeId, { from: wire.fromNodeId });
  }

  if (outgoing.has(sourceNodeId)) {
    return { ok: false, reason: 'This endpoint already has an outgoing connection.' };
  }
  if (incoming.has(targetNodeId)) {
    return { ok: false, reason: 'This endpoint already has an incoming connection.' };
  }

  let currentNode = advanceTraversalNode(targetNodeId);
  const visited = new Set<string>();
  while (!visited.has(currentNode)) {
    if (currentNode === sourceNodeId) {
      return { ok: false, reason: 'This connection would create a cycle.' };
    }
    visited.add(currentNode);
    const edge = outgoing.get(currentNode);
    if (!edge) break;
    currentNode = advanceTraversalNode(edge.to);
  }

  return { ok: true };
}

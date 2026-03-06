import { describe, expect, it } from 'vitest';

import {
  canConnectStudioNodes,
  getStudioRouteDiagnostics,
} from '../../src/components/MissionStudio/studioRouteDiagnostics';
import type { StudioState } from '../../src/components/MissionStudio/useStudioStore';

type DiagnosticsState = Pick<
  StudioState,
  'referenceObjectPath' | 'paths' | 'wires' | 'holds' | 'points' | 'assembly'
>;

function makeState(overrides: Partial<DiagnosticsState> = {}): DiagnosticsState {
  return {
    referenceObjectPath: 'objects/target.obj',
    paths: [],
    wires: [],
    holds: [],
    points: [],
    assembly: [],
    ...overrides,
  };
}

describe('studio route diagnostics', () => {
  it('marks a continuous authored route as executable', () => {
    const state = makeState({
      paths: [
        {
          id: 'path-a',
          axisSeed: 'Z',
          planeA: { position: [0, 0, -5], orientation: [1, 0, 0, 0] },
          planeB: { position: [0, 0, 5], orientation: [1, 0, 0, 0] },
          ellipse: { radiusX: 5, radiusY: 5 },
          levelSpacing: 0.5,
          waypointDensity: 1,
          densityScope: 'total',
          densitySnippetRange: null,
          isLocallyEdited: false,
          waypoints: [
            [0, 0, -5],
            [0, 0, 5],
          ],
          color: '#22d3ee',
          selectedHandleId: null,
        },
        {
          id: 'path-b',
          axisSeed: 'Z',
          planeA: { position: [5, 0, -5], orientation: [1, 0, 0, 0] },
          planeB: { position: [5, 0, 5], orientation: [1, 0, 0, 0] },
          ellipse: { radiusX: 5, radiusY: 5 },
          levelSpacing: 0.5,
          waypointDensity: 1,
          densityScope: 'total',
          densitySnippetRange: null,
          isLocallyEdited: false,
          waypoints: [
            [5, 0, -5],
            [5, 0, 5],
          ],
          color: '#a78bfa',
          selectedHandleId: null,
        },
      ],
      wires: [
        { id: 'wire-1', fromNodeId: 'satellite:start', toNodeId: 'path:path-a:start' },
        { id: 'wire-2', fromNodeId: 'path:path-a:end', toNodeId: 'path:path-b:start' },
      ],
      assembly: [
        { id: 'asm-1', type: 'place_satellite' },
        { id: 'asm-2', type: 'create_path', pathId: 'path-a' },
        { id: 'asm-3', type: 'connect', wireId: 'wire-1' },
        { id: 'asm-4', type: 'create_path', pathId: 'path-b' },
        { id: 'asm-5', type: 'connect', wireId: 'wire-2' },
      ],
    });

    const diagnostics = getStudioRouteDiagnostics(state);

    expect(diagnostics.status).toBe('executable');
    expect(diagnostics.executable).toBe(true);
    expect(diagnostics.disconnectedPathIds).toEqual([]);
    expect(diagnostics.invalidWireIds).toEqual([]);
  });

  it('rejects connect targets that would branch or cycle the route', () => {
    const state = makeState({
      paths: [
        {
          id: 'path-a',
          axisSeed: 'Z',
          planeA: { position: [0, 0, -5], orientation: [1, 0, 0, 0] },
          planeB: { position: [0, 0, 5], orientation: [1, 0, 0, 0] },
          ellipse: { radiusX: 5, radiusY: 5 },
          levelSpacing: 0.5,
          waypointDensity: 1,
          densityScope: 'total',
          densitySnippetRange: null,
          isLocallyEdited: false,
          waypoints: [
            [0, 0, -5],
            [0, 0, 5],
          ],
          color: '#22d3ee',
          selectedHandleId: null,
        },
        {
          id: 'path-b',
          axisSeed: 'Z',
          planeA: { position: [5, 0, -5], orientation: [1, 0, 0, 0] },
          planeB: { position: [5, 0, 5], orientation: [1, 0, 0, 0] },
          ellipse: { radiusX: 5, radiusY: 5 },
          levelSpacing: 0.5,
          waypointDensity: 1,
          densityScope: 'total',
          densitySnippetRange: null,
          isLocallyEdited: false,
          waypoints: [
            [5, 0, -5],
            [5, 0, 5],
          ],
          color: '#a78bfa',
          selectedHandleId: null,
        },
      ],
      points: [{ id: 'pt-1', position: [10, 0, 0] }],
      wires: [
        { id: 'wire-1', fromNodeId: 'satellite:start', toNodeId: 'path:path-a:start' },
        { id: 'wire-2', fromNodeId: 'path:path-a:end', toNodeId: 'path:path-b:start' },
        { id: 'wire-3', fromNodeId: 'point:pt-1', toNodeId: 'path:path-a:start' },
      ],
      assembly: [
        { id: 'asm-1', type: 'place_satellite' },
        { id: 'asm-2', type: 'create_path', pathId: 'path-a' },
        { id: 'asm-3', type: 'create_path', pathId: 'path-b' },
        { id: 'asm-4', type: 'connect', wireId: 'wire-1' },
        { id: 'asm-5', type: 'connect', wireId: 'wire-2' },
        { id: 'asm-6', type: 'point', pointId: 'pt-1' },
        { id: 'asm-7', type: 'connect', wireId: 'wire-3' },
      ],
    });

    expect(
      canConnectStudioNodes(state, 'satellite:start', 'path:path-b:start')
    ).toEqual({
      ok: false,
      reason: 'This endpoint already has an outgoing connection.',
    });

    expect(
      canConnectStudioNodes(state, 'path:path-b:end', 'path:path-a:start')
    ).toEqual({
      ok: false,
      reason: 'This endpoint already has an incoming connection.',
    });

    expect(
      canConnectStudioNodes(state, 'path:path-b:end', 'point:pt-1')
    ).toEqual({
      ok: false,
      reason: 'This connection would create a cycle.',
    });
  });
});

import { useStudioStore } from '../useStudioStore';

function seedSimpleRoute() {
  useStudioStore.setState({
    referenceObjectPath: null,
    missionName: 'Test',
    satelliteStart: [0, 0, 0],
    paths: [
      {
        id: 'p1',
        axisSeed: 'Z',
        planeA: { position: [0, 0, -5], orientation: [1, 0, 0, 0] },
        planeB: { position: [0, 0, 5], orientation: [1, 0, 0, 0] },
        ellipse: { radiusX: 2, radiusY: 1 },
        levelSpacing: 0.5,
        waypointDensity: 1,
        waypoints: [
          [1, 0, -1],
          [1, 0, 0],
          [1, 0, 1],
        ],
        color: '#fff',
        selectedHandleId: null,
      },
    ],
    wires: [{ id: 'w1', fromNodeId: 'satellite:start', toNodeId: 'path:p1:start' }],
    holds: [{ id: 'h1', pathId: 'p1', waypointIndex: 1, duration: 5 }],
    obstacles: [],
    points: [],
    assembly: [],
  } as any);
}

describe('compileStudioMission', () => {
  beforeEach(() => {
    (globalThis as any).window = {
      location: { hostname: 'localhost', port: '5173' },
    };
  });

  it('builds manual path and hold schedule from connected route', async () => {
    seedSimpleRoute();
    const { compileStudioMission } = await import('../compileStudioMission');
    const mission = compileStudioMission(useStudioStore.getState());
    expect(mission.overrides?.manual_path?.length).toBeGreaterThanOrEqual(3);
    expect(mission.overrides?.hold_schedule?.length).toBe(1);
    expect(mission.overrides?.hold_schedule?.[0].duration_s).toBe(5);
  });

  it('samples connector points at ~1m for 1x density', async () => {
    seedSimpleRoute();
    const { compileStudioMission } = await import('../compileStudioMission');
    const mission = compileStudioMission(useStudioStore.getState());
    const manual = mission.overrides?.manual_path ?? [];
    // Start->entry is sqrt(2)m, so sampled connector contributes >1 point before scan waypoints.
    expect(manual.length).toBeGreaterThanOrEqual(5);
  });

  it('rejects branching graph', async () => {
    seedSimpleRoute();
    useStudioStore.setState({
      wires: [
        { id: 'w1', fromNodeId: 'satellite:start', toNodeId: 'path:p1:start' },
        { id: 'w2', fromNodeId: 'satellite:start', toNodeId: 'path:p1:end' },
      ],
    } as any);
    const { compileStudioMission } = await import('../compileStudioMission');
    expect(() => compileStudioMission(useStudioStore.getState())).toThrow(/Branching/i);
  });

  it('supports routing through intermediate point nodes', async () => {
    seedSimpleRoute();
    useStudioStore.setState({
      points: [{ id: 'pt1', position: [0, 0, -1] }],
      wires: [
        { id: 'w1', fromNodeId: 'satellite:start', toNodeId: 'point:pt1' },
        { id: 'w2', fromNodeId: 'point:pt1', toNodeId: 'path:p1:start' },
      ],
    } as any);
    const { compileStudioMission } = await import('../compileStudioMission');
    const mission = compileStudioMission(useStudioStore.getState());
    expect((mission.segments ?? []).some((seg: any) => seg.type === 'transfer')).toBe(true);
    expect(mission.overrides?.manual_path?.length).toBeGreaterThanOrEqual(4);
  });

  it('preserves lateral connector deformation in free mode', async () => {
    useStudioStore.setState({
      referenceObjectPath: null,
      missionName: 'WireFreeMode',
      satelliteStart: [0, 0, 20],
      paths: [],
      holds: [],
      obstacles: [],
      points: [{ id: 'pt1', position: [0, 0, 0] }],
      assembly: [],
      wires: [
        {
          id: 'w1',
          fromNodeId: 'satellite:start',
          toNodeId: 'point:pt1',
          constraintMode: 'free',
          waypoints: [
            [0, 0, 20],
            [4, 0, 14],
            [4, 0, 6],
            [0, 0, 0],
          ],
        },
      ],
    } as any);
    const { compileStudioMission } = await import('../compileStudioMission');
    const mission = compileStudioMission(useStudioStore.getState());
    const manual = mission.overrides?.manual_path ?? [];
    const maxAbsX = manual.reduce((acc, p) => Math.max(acc, Math.abs(p[0])), 0);
    expect(maxAbsX).toBeGreaterThan(1);
  });

  it('reconstrains connector handles in constrained mode', async () => {
    useStudioStore.setState({
      referenceObjectPath: null,
      missionName: 'WireConstrainedMode',
      satelliteStart: [0, 0, 20],
      paths: [],
      holds: [],
      obstacles: [],
      points: [{ id: 'pt1', position: [0, 0, 0] }],
      assembly: [],
      wires: [
        {
          id: 'w1',
          fromNodeId: 'satellite:start',
          toNodeId: 'point:pt1',
          constraintMode: 'constrained',
          waypoints: [
            [0, 0, 20],
            [4, 0, 14],
            [4, 0, 6],
            [0, 0, 0],
          ],
        },
      ],
    } as any);
    const { compileStudioMission } = await import('../compileStudioMission');
    const mission = compileStudioMission(useStudioStore.getState());
    const manual = mission.overrides?.manual_path ?? [];
    const maxAbsX = manual.reduce((acc, p) => Math.max(acc, Math.abs(p[0])), 0);
    expect(maxAbsX).toBeLessThan(1e-3);
  });
});

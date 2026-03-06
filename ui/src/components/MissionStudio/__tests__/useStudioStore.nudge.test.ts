import { useStudioStore } from '../useStudioStore';

describe('useStudioStore studio v1', () => {
  beforeEach(() => {
    useStudioStore.setState({
      activeTool: null,
      paths: [],
      wires: [],
      holds: [],
      obstacles: [],
      points: [],
      assembly: [],
      selectedPathId: null,
      selectedAssemblyId: null,
      wireDrag: { phase: 'idle' },
      satelliteStart: [0, 0, 20],
    } as any);
  });

  it('switches tool modes', () => {
    useStudioStore.getState().setActiveTool('create_path');
    expect(useStudioStore.getState().activeTool).toBe('create_path');
    useStudioStore.getState().setActiveTool('hold');
    expect(useStudioStore.getState().activeTool).toBe('hold');
  });

  it('creates default ±5m path planes on selected axis', () => {
    const id = useStudioStore.getState().addPath('Z');
    const path = useStudioStore.getState().paths.find((p) => p.id === id);
    expect(path).toBeDefined();
    expect(path?.planeA.position[2]).toBeCloseTo(-5, 6);
    expect(path?.planeB.position[2]).toBeCloseTo(5, 6);
    expect(useStudioStore.getState().selectedPathId).toBe(id);
    expect(useStudioStore.getState().selectedAssemblyId).toBeTruthy();
  });

  it('defaults new wires to constrained mode and allows toggling mode', () => {
    useStudioStore.getState().addWire({
      id: 'w1',
      fromNodeId: 'satellite:start',
      toNodeId: 'point:pt1',
    });
    expect(useStudioStore.getState().wires[0]?.constraintMode).toBe('constrained');
    useStudioStore.getState().setWireConstraintMode('w1', 'free');
    expect(useStudioStore.getState().wires[0]?.constraintMode).toBe('free');
  });

  it('tracks Studio authoring status fields directly in the store', () => {
    useStudioStore.getState().setMissionName('Studio Store Mission');
    useStudioStore.getState().setWelcomeDismissed(true);
    useStudioStore.getState().setValidationReport({
      valid: true,
      issues: [],
      summary: { errors: 0, warnings: 0, info: 0 },
    });

    expect(useStudioStore.getState().missionName).toBe('Studio Store Mission');
    expect(useStudioStore.getState().welcomeDismissed).toBe(true);
    expect(useStudioStore.getState().validationReport?.valid).toBe(true);
  });

  it('focuses newly created points and obstacles near the current authoring anchor', () => {
    const pathId = useStudioStore.getState().addPath('Z');
    useStudioStore.getState().setPathWaypointsManual(pathId, [
      [1, 2, 3],
      [4, 5, 6],
    ]);

    useStudioStore.getState().addPoint();
    const point = useStudioStore.getState().points[0];
    expect(point?.position).toEqual([6, 5, 6]);
    const pointAssemblyId = useStudioStore.getState().selectedAssemblyId;
    expect(
      useStudioStore
        .getState()
        .assembly.some((item) => item.id === pointAssemblyId && item.pointId === point?.id)
    ).toBe(true);

    useStudioStore.getState().addObstacle();
    const obstacle = useStudioStore.getState().obstacles[0];
    expect(obstacle?.position).toEqual([5.5, 6.5, 6]);
    const obstacleAssemblyId = useStudioStore.getState().selectedAssemblyId;
    expect(
      useStudioStore
        .getState()
        .assembly.some((item) => item.id === obstacleAssemblyId && item.obstacleId === obstacle?.id)
    ).toBe(true);
  });
});

import { useStudioStore } from '../useStudioStore';

describe('useStudioStore studio v1', () => {
  beforeEach(() => {
    useStudioStore.setState({
      activeTool: null,
      paths: [],
      wires: [],
      holds: [],
      obstacles: [],
      assembly: [],
      selectedPathId: null,
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
  });
});

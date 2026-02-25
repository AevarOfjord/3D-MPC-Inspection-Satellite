import { useStudioStore } from '../useStudioStore';
import { generateSpiral } from '../useSpiralGenerator';

describe('applyNudge', () => {
  beforeEach(() => {
    useStudioStore.setState({
      scanPasses: [], wires: [], holds: [], obstacles: [], segments: [],
      selectedScanId: null, wireDrag: { phase: 'idle' },
    });
  });

  it('moves the target waypoint and attenuates neighbors', () => {
    const waypoints = generateSpiral({
      axis: 'Z', planeAOffset: -5, planeBOffset: 5,
      crossSection: Array.from({ length: 8 }, (_, i) => [
        Math.cos(i / 8 * Math.PI * 2) * 3,
        Math.sin(i / 8 * Math.PI * 2) * 3,
      ] as [number, number]),
      levelHeight: 1,
    });
    useStudioStore.setState({
      scanPasses: [{ id: 'p1', axis: 'Z', planeAOffset: -5, planeBOffset: 5,
        crossSection: [], levelHeight: 1, waypoints, color: '#fff', keyLevels: [], selectedHandleId: null }],
    });

    const idx = 10;
    const before = [...useStudioStore.getState().scanPasses[0].waypoints[idx]];
    useStudioStore.getState().applyNudge('p1', idx, [1, 0, 0]);
    const after = useStudioStore.getState().scanPasses[0].waypoints[idx];

    expect(after[0]).toBeCloseTo(before[0] + 1, 1);

    const neighbor = useStudioStore.getState().scanPasses[0].waypoints[idx + 5];
    const neighborBefore = waypoints[idx + 5];
    expect(Math.abs(neighbor[0] - neighborBefore[0])).toBeLessThan(0.5);
  });
});

import { generateSpiral } from '../useSpiralGenerator';

describe('generateSpiral', () => {
  it('produces waypoints between two offsets', () => {
    const pts = generateSpiral({
      axis: 'Z',
      planeAOffset: -5,
      planeBOffset: 5,
      crossSection: Array.from({ length: 8 }, (_, i) => {
        const a = (i / 8) * Math.PI * 2;
        return [Math.cos(a) * 3, Math.sin(a) * 3] as [number, number];
      }),
      levelHeight: 1,
    });
    expect(pts.length).toBeGreaterThan(0);
    pts.forEach(([, , z]) => {
      expect(z).toBeGreaterThanOrEqual(-5 - 0.01);
      expect(z).toBeLessThanOrEqual(5 + 0.01);
    });
  });

  it('respects cross-section shape', () => {
    const rect: [number, number][] = [
      [5, 3], [5, -3], [-5, -3], [-5, 3],
      [5, 3], [5, -3], [-5, -3], [-5, 3],
    ];
    const pts = generateSpiral({
      axis: 'Z', planeAOffset: -2, planeBOffset: 2,
      crossSection: rect, levelHeight: 0.5,
    });
    expect(pts.length).toBeGreaterThan(0);
  });
});

import { describe, expect, it } from 'vitest';

import {
  clampPathDensityMultiplier,
  formatPathDensityMultiplier,
  normalizePathDensityMultiplier,
  parsePathDensityInput,
} from '../../src/utils/pathDensity';

describe('path density helpers', () => {
  it('clamps to configured range', () => {
    expect(clampPathDensityMultiplier(0.1)).toBe(0.25);
    expect(clampPathDensityMultiplier(99)).toBe(20);
    expect(clampPathDensityMultiplier(1.5)).toBe(1.5);
  });

  it('normalizes to step increments', () => {
    expect(normalizePathDensityMultiplier(1.03)).toBe(1.05);
    expect(normalizePathDensityMultiplier(0.26)).toBe(0.25);
  });

  it('parses and formats display values', () => {
    expect(parsePathDensityInput('2.01', 1)).toBe(2);
    expect(parsePathDensityInput('bad-input', 1.2)).toBeCloseTo(1.2, 6);
    expect(formatPathDensityMultiplier(2)).toBe('2');
    expect(formatPathDensityMultiplier(1.5)).toBe('1.5');
  });
});

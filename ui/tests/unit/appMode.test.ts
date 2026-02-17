import { describe, expect, it } from 'vitest';

import { parseStoredAppMode } from '../../src/utils/appMode';

describe('app mode storage migration', () => {
  it('maps legacy mission/scan modes to planner', () => {
    expect(parseStoredAppMode('mission')).toBe('planner');
    expect(parseStoredAppMode('scan')).toBe('planner');
  });

  it('accepts canonical modes and rejects unknown values', () => {
    expect(parseStoredAppMode('planner')).toBe('planner');
    expect(parseStoredAppMode('viewer')).toBe('viewer');
    expect(parseStoredAppMode('runner')).toBe('runner');
    expect(parseStoredAppMode('data')).toBe('data');
    expect(parseStoredAppMode('settings')).toBe('settings');
    expect(parseStoredAppMode('other')).toBeNull();
    expect(parseStoredAppMode(null)).toBeNull();
  });
});

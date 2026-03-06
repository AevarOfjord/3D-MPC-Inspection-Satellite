import { describe, expect, it } from 'vitest';

import { parseStoredAppMode } from '../../src/utils/appMode';

describe('parseStoredAppMode', () => {
  it('maps legacy planner-like values to studio', () => {
    expect(parseStoredAppMode('planner')).toBe('studio');
    expect(parseStoredAppMode('mission')).toBe('studio');
    expect(parseStoredAppMode('scan')).toBe('studio');
  });

  it('keeps supported active modes', () => {
    expect(parseStoredAppMode('viewer')).toBe('viewer');
    expect(parseStoredAppMode('studio')).toBe('studio');
    expect(parseStoredAppMode('runner')).toBe('runner');
    expect(parseStoredAppMode('data')).toBe('data');
    expect(parseStoredAppMode('settings')).toBe('settings');
  });

  it('rejects unsupported values', () => {
    expect(parseStoredAppMode('')).toBeNull();
    expect(parseStoredAppMode('planner-v4')).toBeNull();
    expect(parseStoredAppMode(null)).toBeNull();
  });
});

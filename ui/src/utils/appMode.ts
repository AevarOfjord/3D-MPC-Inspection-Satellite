export type AppMode = 'viewer' | 'planner' | 'studio' | 'runner' | 'data' | 'settings';

export function parseStoredAppMode(value: unknown): AppMode | null {
  if (value === 'mission' || value === 'scan') {
    return 'planner';
  }
  if (
    value === 'viewer' ||
    value === 'planner' ||
    value === 'studio' ||
    value === 'runner' ||
    value === 'data' ||
    value === 'settings'
  ) {
    return value;
  }
  return null;
}

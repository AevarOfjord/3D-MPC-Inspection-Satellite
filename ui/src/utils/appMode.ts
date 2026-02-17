export type AppMode = 'viewer' | 'planner' | 'runner' | 'data' | 'settings';

export function parseStoredAppMode(value: unknown): AppMode | null {
  if (value === 'mission' || value === 'scan') {
    return 'planner';
  }
  if (
    value === 'viewer' ||
    value === 'planner' ||
    value === 'runner' ||
    value === 'data' ||
    value === 'settings'
  ) {
    return value;
  }
  return null;
}

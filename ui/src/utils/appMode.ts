export type AppMode = 'viewer' | 'studio' | 'runner' | 'data' | 'settings';

export function parseStoredAppMode(value: unknown): AppMode | null {
  if (value === 'mission' || value === 'scan') {
    return 'studio';
  }
  if (
    value === 'viewer' ||
    value === 'planner' ||
    value === 'studio' ||
    value === 'runner' ||
    value === 'data' ||
    value === 'settings'
  ) {
    return value === 'planner' ? 'studio' : value;
  }
  return null;
}

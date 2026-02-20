export const PATH_DENSITY_MIN = 0.25;
export const PATH_DENSITY_MAX = 20.0;
export const PATH_DENSITY_STEP = 0.05;

export function clampPathDensityMultiplier(value: number): number {
  if (!Number.isFinite(value)) return 1.0;
  return Math.min(PATH_DENSITY_MAX, Math.max(PATH_DENSITY_MIN, value));
}

export function normalizePathDensityMultiplier(value: number): number {
  const clamped = clampPathDensityMultiplier(value);
  return Math.round(clamped / PATH_DENSITY_STEP) * PATH_DENSITY_STEP;
}

export function parsePathDensityInput(raw: string, fallback = 1.0): number {
  const parsed = Number.parseFloat(raw.trim());
  if (!Number.isFinite(parsed)) return normalizePathDensityMultiplier(fallback);
  return normalizePathDensityMultiplier(parsed);
}

export function formatPathDensityMultiplier(value: number): string {
  const normalized = normalizePathDensityMultiplier(value);
  if (Math.abs(normalized - Math.round(normalized)) < 1e-9) {
    return String(Math.round(normalized));
  }
  return normalized.toFixed(2).replace(/\.?0+$/, '');
}

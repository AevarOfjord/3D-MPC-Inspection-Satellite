import { API_BASE_URL } from '../../config/endpoints';

export const STUDIO_LOCAL_TARGET_ID = 'STUDIO_LOCAL_ORIGIN';
export const STUDIO_OBJ_TARGET_PREFIX = 'STUDIO_OBJ::';

export function studioModelPathToUrl(modelPath: string): string {
  return `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(modelPath)}`;
}

export function studioTargetIdFromModelPath(modelPath?: string | null): string {
  const normalized = (modelPath || '').trim();
  if (!normalized) {
    return STUDIO_LOCAL_TARGET_ID;
  }
  return `${STUDIO_OBJ_TARGET_PREFIX}${normalized}`;
}

export function studioReferenceLabel(modelPath?: string | null): string {
  const normalized = (modelPath || '').trim();
  if (!normalized) return 'None (Local Origin)';
  const parts = normalized.replace(/\\/g, '/').split('/');
  return parts[parts.length - 1] || normalized;
}

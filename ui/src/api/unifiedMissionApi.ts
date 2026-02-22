import { API_BASE_URL } from '../config/endpoints';
import type { UnifiedMission } from './unifiedMission';

interface SavedMissionsResponse {
  missions: string[];
}

interface MissionSummaryV2 {
  name: string;
  mission_id: string;
  updated_at?: string | null;
  segments_count: number;
  filename: string;
  schema_version: number;
}

export interface ValidationIssueV2 {
  code: string;
  severity: 'error' | 'warning' | 'info';
  path: string;
  message: string;
  suggestion?: string | null;
}

export interface ValidationReportV2 {
  valid: boolean;
  issues: ValidationIssueV2[];
  summary: {
    errors: number;
    warnings: number;
    info: number;
  };
}

export interface PreviewMissionResponse {
  path: [number, number, number][];
  path_length: number;
  path_speed: number;
  eta_s: number;
  risk_flags: string[];
  constraint_summary: {
    speed_max?: number | null;
    accel_max?: number | null;
    angular_rate_max?: number | null;
  };
}

export interface SaveMissionV2Response {
  mission_id: string;
  version: number;
  saved_at: string;
  filename: string;
}

export interface MissionDraftResponse {
  draft_id: string;
  revision: number;
  saved_at: string;
  mission: UnifiedMission;
}

interface MissionDraftListResponse {
  draft_ids: string[];
}

export const unifiedMissionApi = {
  setMission: async (config: UnifiedMission) => {
    const response = await fetch(`${API_BASE_URL}/mission_v2`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    });

    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Update failed' }));
      throw new Error(err.detail || 'Update failed');
    }
    return response.json();
  },

  saveMission: async (
    name: string,
    config: UnifiedMission
  ): Promise<SaveMissionV2Response> => {
    const response = await fetch(`${API_BASE_URL}/api/v2/missions`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, mission: config }),
    });

    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Save failed' }));
      throw new Error(err.detail || 'Save failed');
    }
    return response.json();
  },

  listSavedMissions: async (): Promise<SavedMissionsResponse> => {
    const response = await fetch(`${API_BASE_URL}/api/v2/missions`);
    if (!response.ok) {
      throw new Error('Failed to list unified missions');
    }
    const summaries = (await response.json()) as MissionSummaryV2[];
    return { missions: summaries.map((item) => item.name) };
  },

  loadMission: async (missionName: string): Promise<UnifiedMission> => {
    const response = await fetch(
      `${API_BASE_URL}/api/v2/missions/${encodeURIComponent(missionName)}`
    );
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Load failed' }));
      throw new Error(err.detail || 'Load failed');
    }
    return response.json();
  },

  previewMission: async (config: UnifiedMission): Promise<PreviewMissionResponse> => {
    const response = await fetch(`${API_BASE_URL}/api/v2/missions/preview`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    });
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Preview failed' }));
      throw new Error(err.detail || 'Preview failed');
    }
    return response.json();
  },

  validateMission: async (config: UnifiedMission): Promise<ValidationReportV2> => {
    const response = await fetch(`${API_BASE_URL}/api/v2/missions/validate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    });
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Validation failed' }));
      throw new Error(err.detail || 'Validation failed');
    }
    return response.json();
  },

  saveDraft: async (
    config: UnifiedMission,
    options?: { draft_id?: string; base_revision?: number }
  ): Promise<MissionDraftResponse> => {
    const response = await fetch(`${API_BASE_URL}/api/v2/missions/drafts`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        draft_id: options?.draft_id,
        base_revision: options?.base_revision,
        mission: config,
      }),
    });
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Draft save failed' }));
      throw new Error(err.detail || 'Draft save failed');
    }
    return response.json();
  },

  loadDraft: async (draftId: string): Promise<MissionDraftResponse> => {
    const response = await fetch(
      `${API_BASE_URL}/api/v2/missions/drafts/${encodeURIComponent(draftId)}`
    );
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Draft load failed' }));
      throw new Error(err.detail || 'Draft load failed');
    }
    return response.json();
  },

  listDrafts: async (): Promise<MissionDraftListResponse> => {
    const response = await fetch(`${API_BASE_URL}/api/v2/missions/drafts/list`);
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Draft list failed' }));
      throw new Error(err.detail || 'Draft list failed');
    }
    return response.json();
  },

  controlSimulation: async (action: 'pause' | 'resume' | 'step', steps = 1) => {
    const response = await fetch(`${API_BASE_URL}/control`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action, steps }),
    });
    if (!response.ok) {
        throw new Error('Control failed');
    }
    return response.json();
  },
};

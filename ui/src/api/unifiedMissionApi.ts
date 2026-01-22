import { API_BASE_URL } from '../config/endpoints';
import type { UnifiedMission } from './unifiedMission';

export interface SavedMissionsResponse {
  missions: string[];
}

export interface PreviewMissionResponse {
  path: [number, number, number][];
  path_length: number;
  path_speed: number;
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

  getMission: async (): Promise<UnifiedMission> => {
    const response = await fetch(`${API_BASE_URL}/mission_v2`);
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Load failed' }));
      throw new Error(err.detail || 'Load failed');
    }
    return response.json();
  },

  saveMission: async (name: string, config: UnifiedMission) => {
    const response = await fetch(`${API_BASE_URL}/save_mission_v2`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, config }),
    });

    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Save failed' }));
      throw new Error(err.detail || 'Save failed');
    }
    return response.json();
  },

  listSavedMissions: async (): Promise<SavedMissionsResponse> => {
    const response = await fetch(`${API_BASE_URL}/saved_missions_v2`);
    if (!response.ok) {
      throw new Error('Failed to list unified missions');
    }
    return response.json();
  },

  loadMission: async (missionName: string): Promise<UnifiedMission> => {
    const response = await fetch(`${API_BASE_URL}/mission_v2/${missionName}`);
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Load failed' }));
      throw new Error(err.detail || 'Load failed');
    }
    return response.json();
  },

  previewMission: async (config: UnifiedMission): Promise<PreviewMissionResponse> => {
    const response = await fetch(`${API_BASE_URL}/mission_v2/preview`, {
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
};

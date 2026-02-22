import { API_BASE_URL } from '../config/endpoints';

export interface PathAssetSummary {
  id: string;
  name: string;
  obj_path: string;
  points: number;
  path_length: number;
  open?: boolean;
  relative_to_obj?: boolean;
  created_at?: string;
  updated_at?: string;
}

interface PathAsset extends PathAssetSummary {
  path: [number, number, number][];
  notes?: string | null;
}

interface PathAssetSaveRequest {
  name: string;
  obj_path: string;
  path: [number, number, number][];
  open?: boolean;
  relative_to_obj?: boolean;
  notes?: string | null;
}

export const pathAssetsApi = {
  list: async (): Promise<PathAssetSummary[]> => {
    const response = await fetch(`${API_BASE_URL}/path_assets`);
    if (!response.ok) {
      throw new Error('Failed to list path assets');
    }
    const data = await response.json();
    return data.assets || [];
  },

  get: async (assetId: string): Promise<PathAsset> => {
    const response = await fetch(`${API_BASE_URL}/path_assets/${encodeURIComponent(assetId)}`);
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Load failed' }));
      throw new Error(err.detail || 'Load failed');
    }
    return response.json();
  },

  save: async (payload: PathAssetSaveRequest): Promise<PathAsset> => {
    const response = await fetch(`${API_BASE_URL}/path_assets`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Save failed' }));
      throw new Error(err.detail || 'Save failed');
    }
    return response.json();
  },
};

import { API_BASE_URL } from '../config/endpoints';

export interface MeshScanConfig {
  obj_path: string;
  standoff: number;
  levels: number;
  level_spacing?: number; // Alternative to levels - distance between scan rings
  points_per_circle: number;
  speed_max: number;
  speed_min: number;
  lateral_accel: number;
  z_margin: number;
  scan_axis: 'X' | 'Y' | 'Z';
  pattern?: 'spiral' | 'rings';
}

export interface PreviewResponse {
  status: string;
  path: [number, number, number][]; // List of points
  points: number;
  estimated_duration: number;
  path_length: number;
}

export interface UploadResponse {
    status: string;
    path: string;
    filename: string;
}

export interface ModelInfo {
  name: string;
  filename: string;
  path: string;
}

export interface ModelBounds {
  center: [number, number, number];
  min_bounds: [number, number, number];
  max_bounds: [number, number, number];
  extents: [number, number, number];
}

export const trajectoryApi = {
  uploadObject: async (file: File): Promise<UploadResponse> => {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('filename', file.name);

    const response = await fetch(`${API_BASE_URL}/upload_object`, {
      method: 'POST',
      body: formData,
    });

    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Upload failed' }));
      throw new Error(err.detail || 'Upload failed');
    }
    return response.json();
  },

  listModels: async (): Promise<ModelInfo[]> => {
    const response = await fetch(`${API_BASE_URL}/api/models/list`);
    if (!response.ok) {
      throw new Error('Failed to list models');
    }
    const data = await response.json();
    return data.models || [];
  },

  getModelBounds: async (path: string): Promise<ModelBounds> => {
    const response = await fetch(
      `${API_BASE_URL}/api/models/bounds?path=${encodeURIComponent(path)}`
    );
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Bounds failed' }));
      throw new Error(err.detail || 'Bounds failed');
    }
    return response.json();
  },

  previewTrajectory: async (config: MeshScanConfig): Promise<PreviewResponse> => {
    const response = await fetch(`${API_BASE_URL}/preview_trajectory`, {
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

  runMission: async (missionName: string) => {
      const response = await fetch(`${API_BASE_URL}/run_mission`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ mission_name: missionName }),
      });

      if (!response.ok) {
          const err = await response.json().catch(() => ({ detail: 'Run failed' }));
          throw new Error(err.detail || 'Run failed');
      }
      return response.json();
  }
};

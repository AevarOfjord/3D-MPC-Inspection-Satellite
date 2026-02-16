import { API_BASE_URL } from '../config/endpoints';
import type {
  ScanCompileRequest,
  ScanCompileResponse,
  ScanProject,
  ScanProjectSummary,
} from '../types/scanProject';

export const scanProjectsApi = {
  listScanProjects: async (): Promise<ScanProjectSummary[]> => {
    const response = await fetch(`${API_BASE_URL}/scan_projects`);
    if (!response.ok) {
      throw new Error('Failed to list scan projects');
    }
    const data = await response.json();
    return data.projects || [];
  },

  loadScanProject: async (projectId: string): Promise<ScanProject> => {
    const response = await fetch(
      `${API_BASE_URL}/scan_projects/${encodeURIComponent(projectId)}`
    );
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Load failed' }));
      throw new Error(err.detail || 'Load failed');
    }
    return response.json();
  },

  saveScanProject: async (project: ScanProject): Promise<ScanProject> => {
    const response = await fetch(`${API_BASE_URL}/scan_projects`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(project),
    });
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Save failed' }));
      throw new Error(err.detail || 'Save failed');
    }
    return response.json();
  },

  compileScanProject: async (
    request: ScanCompileRequest
  ): Promise<ScanCompileResponse> => {
    const response = await fetch(`${API_BASE_URL}/scan_projects/compile`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request),
    });
    if (!response.ok) {
      const err = await response.json().catch(() => ({ detail: 'Compile failed' }));
      throw new Error(err.detail || 'Compile failed');
    }
    return response.json();
  },
};

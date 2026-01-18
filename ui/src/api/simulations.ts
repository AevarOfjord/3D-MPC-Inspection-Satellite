import { API_BASE_URL } from '../config/endpoints';
import type { TelemetryData } from '../services/telemetry';

export interface SimulationRun {
  id: string;
  modified: number;
  has_physics: boolean;
  has_metrics: boolean;
  steps?: number;
  duration?: number;
}

export interface SimulationListResponse {
  runs: SimulationRun[];
}

export interface SimulationTelemetryResponse {
  run_id: string;
  telemetry: TelemetryData[];
}

export const simulationsApi = {
  list: async (): Promise<SimulationListResponse> => {
    const response = await fetch(`${API_BASE_URL}/simulations`);
    if (!response.ok) {
      throw new Error('Failed to fetch simulations');
    }
    return response.json();
  },
  loadTelemetry: async (runId: string, stride: number): Promise<SimulationTelemetryResponse> => {
    const response = await fetch(
      `${API_BASE_URL}/simulations/${encodeURIComponent(runId)}/telemetry?stride=${stride}`
    );
    if (!response.ok) {
      throw new Error('Failed to load telemetry');
    }
    return response.json();
  },
  downloadVideo: async (runId: string) => {
    // Trigger download by opening in new window/tab
    window.open(`${API_BASE_URL}/simulations/${encodeURIComponent(runId)}/video`, '_blank');
  },
};

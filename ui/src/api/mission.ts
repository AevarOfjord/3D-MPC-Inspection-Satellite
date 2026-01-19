import { API_BASE_URL } from '../config/endpoints';

export interface Obstacle {
  position: [number, number, number];
  radius: number;
}

export interface MissionConfig {
  start_position: [number, number, number];
  target_position: [number, number, number];
  target_orientation: [number, number, number];
  obstacles: Obstacle[];
}

export const missionApi = {
  updateMission: async (config: MissionConfig) => {
    const response = await fetch(`${API_BASE_URL}/mission`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(config),
    });

    if (!response.ok) {
      throw new Error('Failed to update mission');
    }

    return response.json();
  }
};

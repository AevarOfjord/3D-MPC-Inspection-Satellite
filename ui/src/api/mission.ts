export interface Obstacle {
  position: [number, number, number];
  radius: number;
}

export interface MissionConfig {
  start_position: [number, number, number];
  obstacles: Obstacle[];
}

export const missionApi = {
  updateMission: async (config: MissionConfig) => {
    const response = await fetch('http://localhost:8000/mission', {
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

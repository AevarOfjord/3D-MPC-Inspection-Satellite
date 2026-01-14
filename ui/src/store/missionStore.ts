import { create } from 'zustand';
import type { MissionConfig, Obstacle } from '../api/mission';

interface MissionState {
  config: MissionConfig;
  isEditing: boolean;
  
  setConfig: (config: MissionConfig) => void;
  setEditing: (isEditing: boolean) => void;
  
  updateStartPos: (index: number, value: number) => void;
  updateTargetPos: (index: number, value: number) => void;
  updateTargetOri: (index: number, value: number) => void;
  
  addObstacle: () => void;
  removeObstacle: (index: number) => void;
  updateObstacle: (index: number, field: keyof Obstacle, value: any, subIndex?: number) => void;
}

export const useMissionStore = create<MissionState>((set) => ({
  config: {
    start_position: [10.0, 0.0, 0.0],
    target_position: [0.0, 0.0, 0.0],
    target_orientation: [0.0, 0.0, 0.0],
    obstacles: []
  },
  isEditing: false,

  setConfig: (config) => set({ config }),
  setEditing: (isEditing) => set({ isEditing }),

  updateStartPos: (index, value) => set((state) => {
    const newPos = [...state.config.start_position] as [number, number, number];
    newPos[index] = value;
    return { config: { ...state.config, start_position: newPos } };
  }),

  updateTargetPos: (index, value) => set((state) => {
    const newPos = [...state.config.target_position] as [number, number, number];
    newPos[index] = value;
    return { config: { ...state.config, target_position: newPos } };
  }),

  updateTargetOri: (index, value) => set((state) => {
    const newOri = [...state.config.target_orientation] as [number, number, number];
    newOri[index] = value;
    return { config: { ...state.config, target_orientation: newOri } };
  }),

  addObstacle: () => set((state) => ({
    config: {
      ...state.config,
      obstacles: [...state.config.obstacles, { position: [5.0, 0.0, 0.0], radius: 1.0 }]
    }
  })),

  removeObstacle: (index) => set((state) => ({
    config: {
      ...state.config,
      obstacles: state.config.obstacles.filter((_, i) => i !== index)
    }
  })),

  updateObstacle: (index, field, value, subIndex) => set((state) => {
    const newObstacles = [...state.config.obstacles];
    const obs = { ...newObstacles[index] };
    
    if (field === 'position' && typeof subIndex === 'number') {
        const newPos = [...obs.position] as [number, number, number];
        newPos[subIndex] = value;
        obs.position = newPos;
    } else if (field === 'radius') {
        obs.radius = value;
    }
    
    newObstacles[index] = obs;
    return { config: { ...state.config, obstacles: newObstacles } };
  })
}));

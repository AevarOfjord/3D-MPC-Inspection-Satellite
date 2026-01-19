import { create } from 'zustand';
import type { MissionConfig, Obstacle } from '../api/mission';

interface MissionState {
  config: MissionConfig;
  isEditing: boolean;
  lastRunConfig: MissionConfig | null;
  selectedObstacleIndex: number | null;
  transformAxis: 'free' | 'x' | 'y' | 'z';
  transformSnap: number | null;
  
  setConfig: (config: MissionConfig) => void;
  setEditing: (isEditing: boolean) => void;
  setLastRunConfig: (config: MissionConfig | null) => void;
  setSelectedObstacleIndex: (index: number | null) => void;
  setTransformAxis: (axis: 'free' | 'x' | 'y' | 'z') => void;
  setTransformSnap: (snap: number | null) => void;
  
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
  lastRunConfig: null,
  selectedObstacleIndex: null,
  transformAxis: 'free',
  transformSnap: 0.5,

  setConfig: (config) => set({ config, selectedObstacleIndex: null }),
  setEditing: (isEditing) => set({ isEditing }),
  setLastRunConfig: (config) => set({ lastRunConfig: config }),
  setSelectedObstacleIndex: (index) => set({ selectedObstacleIndex: index }),
  setTransformAxis: (axis) => set({ transformAxis: axis }),
  setTransformSnap: (snap) => set({ transformSnap: snap }),

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

  addObstacle: () => set((state) => {
    const nextObstacles = [...state.config.obstacles, { position: [5.0, 0.0, 0.0] as [number, number, number], radius: 1.0 }];
    return {
      config: {
        ...state.config,
        obstacles: nextObstacles
      },
      selectedObstacleIndex: nextObstacles.length - 1
    };
  }),

  removeObstacle: (index) => set((state) => {
    const nextObstacles = state.config.obstacles.filter((_, i) => i !== index);
    let nextSelected = state.selectedObstacleIndex;
    if (nextSelected === index) {
      nextSelected = null;
    } else if (typeof nextSelected === 'number' && nextSelected > index) {
      nextSelected -= 1;
    }
    return {
      config: {
        ...state.config,
        obstacles: nextObstacles
      },
      selectedObstacleIndex: nextSelected
    };
  }),

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

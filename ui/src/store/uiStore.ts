import { create } from 'zustand';

interface UiState {
  missionError: string | null;
  setMissionError: (message: string | null) => void;
}

export const useUiStore = create<UiState>((set) => ({
  missionError: null,
  setMissionError: (message) => set({ missionError: message }),
}));

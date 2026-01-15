import { create } from 'zustand';

interface ViewportState {
  canvas: HTMLCanvasElement | null;
  recording: boolean;
  setCanvas: (canvas: HTMLCanvasElement | null) => void;
  setRecording: (recording: boolean) => void;
}

export const useViewportStore = create<ViewportState>((set) => ({
  canvas: null,
  recording: false,
  setCanvas: (canvas) => set({ canvas }),
  setRecording: (recording) => set({ recording }),
}));

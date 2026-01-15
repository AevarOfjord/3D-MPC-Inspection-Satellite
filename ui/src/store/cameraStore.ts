import { create } from 'zustand';

interface CameraState {
  focusTarget: [number, number, number] | null;
  focusNonce: number;
  viewPreset: 'iso' | 'top' | 'front' | 'right' | 'left' | 'back' | null;
  viewNonce: number;
  requestFocus: (target: [number, number, number]) => void;
  requestViewPreset: (preset: CameraState['viewPreset']) => void;
}

export const useCameraStore = create<CameraState>((set) => ({
  focusTarget: null,
  focusNonce: 0,
  viewPreset: null,
  viewNonce: 0,
  requestFocus: (target) =>
    set((state) => ({
      focusTarget: target,
      focusNonce: state.focusNonce + 1,
    })),
  requestViewPreset: (preset) =>
    set((state) => ({
      viewPreset: preset,
      viewNonce: state.viewNonce + 1,
    })),
}));

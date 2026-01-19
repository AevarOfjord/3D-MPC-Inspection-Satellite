import { create } from 'zustand';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';

interface CameraState {
  focusTarget: [number, number, number] | null;
  focusNonce: number;
  viewPreset: 'iso' | 'top' | 'front' | 'right' | 'left' | 'back' | null;
  viewNonce: number;
  controls: OrbitControlsImpl | null;
  requestFocus: (target: [number, number, number]) => void;
  requestViewPreset: (preset: CameraState['viewPreset']) => void;
  setControls: (controls: OrbitControlsImpl | null) => void;
  zoomIn: () => void;
  zoomOut: () => void;
}

export const useCameraStore = create<CameraState>((set) => ({
  focusTarget: null,
  focusNonce: 0,
  viewPreset: null,
  viewNonce: 0,
  controls: null,
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
  setControls: (controls) => set({ controls }),
  zoomIn: () => {
    set((state) => {
      if (state.controls) {
        state.controls.dollyOut(1.2);
        state.controls.update();
      }
      return {};
    });
  },
  zoomOut: () => {
    set((state) => {
      if (state.controls) {
        state.controls.dollyIn(1.2);
        state.controls.update();
      }
      return {};
    });
  },
}));

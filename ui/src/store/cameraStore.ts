import { create } from 'zustand';
import { Vector3 } from 'three';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';

interface CameraState {
  focusTarget: [number, number, number] | null;
  focusDistance: number | null;
  focusNonce: number;
  viewPreset: 'iso' | 'top' | 'front' | 'right' | 'left' | 'back' | null;
  viewNonce: number;
  controls: OrbitControlsImpl | null;
  requestFocus: (target: [number, number, number], distance?: number) => void;
  requestViewPreset: (preset: CameraState['viewPreset']) => void;
  setControls: (controls: OrbitControlsImpl | null) => void;
  zoomIn: () => void;
  zoomOut: () => void;
}

export const useCameraStore = create<CameraState>((set) => ({
  focusTarget: null,
  focusDistance: null,
  focusNonce: 0,
  viewPreset: null,
  viewNonce: 0,
  controls: null,
  requestFocus: (target, distance) =>
    set((state) => {
      if (state.controls) {
        const controls: any = state.controls;
        if (controls.target) {
          controls.target.set(target[0], target[1], target[2]);
        }
        if (typeof controls.update === 'function') {
          controls.update();
        }
      }
      return {
        focusTarget: target,
        focusDistance: Number.isFinite(distance ?? NaN) ? distance! : state.focusDistance,
        focusNonce: state.focusNonce + 1,
      };
    }),
  requestViewPreset: (preset) =>
    set((state) => ({
      viewPreset: preset,
      viewNonce: state.viewNonce + 1,
    })),
  setControls: (controls) => set({ controls }),
  zoomIn: () => {
    set((state) => {
      const controls: any = state.controls;
      if (!controls) return {};
      const camera = controls.object ?? controls.camera;
      const target = controls.target
        ? new Vector3().copy(controls.target)
        : state.focusTarget
          ? new Vector3(state.focusTarget[0], state.focusTarget[1], state.focusTarget[2])
          : new Vector3();
      const zoomFactor = 0.85;

      if (camera?.isPerspectiveCamera) {
        const direction = new Vector3().subVectors(camera.position, target);
        if (direction.lengthSq() > 0) {
          camera.position.copy(target.clone().add(direction.multiplyScalar(zoomFactor)));
        }
      } else if (camera?.isOrthographicCamera) {
        camera.zoom = Math.min(camera.zoom * 1.15, 200);
        camera.updateProjectionMatrix();
      }

      if (state.focusTarget && controls.target) {
        controls.target.set(state.focusTarget[0], state.focusTarget[1], state.focusTarget[2]);
      }
      if (typeof controls.update === 'function') controls.update();
      return {};
    });
  },
  zoomOut: () => {
    set((state) => {
      const controls: any = state.controls;
      if (!controls) return {};
      const camera = controls.object ?? controls.camera;
      const target = controls.target
        ? new Vector3().copy(controls.target)
        : state.focusTarget
          ? new Vector3(state.focusTarget[0], state.focusTarget[1], state.focusTarget[2])
          : new Vector3();
      const zoomFactor = 1.15;

      if (camera?.isPerspectiveCamera) {
        const direction = new Vector3().subVectors(camera.position, target);
        if (direction.lengthSq() > 0) {
          camera.position.copy(target.clone().add(direction.multiplyScalar(zoomFactor)));
        }
      } else if (camera?.isOrthographicCamera) {
        camera.zoom = Math.max(camera.zoom * 0.85, 0.05);
        camera.updateProjectionMatrix();
      }

      if (state.focusTarget && controls.target) {
        controls.target.set(state.focusTarget[0], state.focusTarget[1], state.focusTarget[2]);
      }
      if (typeof controls.update === 'function') controls.update();
      return {};
    });
  },
}));

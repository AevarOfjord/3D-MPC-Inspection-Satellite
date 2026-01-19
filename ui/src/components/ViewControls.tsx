import { Vector3 } from 'three';
import { useCameraStore } from '../store/cameraStore';

const ROTATE_STEP_RAD = Math.PI / 24;
const PAN_STEP_FRACTION = 0.05;

interface ViewControlsProps {
  onRequestFree: () => void;
}

export function ViewControls({ onRequestFree }: ViewControlsProps) {
  const requestViewPreset = useCameraStore(s => s.requestViewPreset);
  const controls = useCameraStore(s => s.controls);

  const handleClick = (preset: Parameters<typeof requestViewPreset>[0]) => {
    onRequestFree();
    requestViewPreset(preset);
  };

  const nudgeRotate = (azimuthDelta: number, polarDelta: number) => {
    if (!controls) return;
    onRequestFree();
    controls.rotateLeft(azimuthDelta);
    controls.rotateUp(polarDelta);
    controls.update();
  };

  const nudgePan = (right: number, up: number) => {
    if (!controls) return;
    onRequestFree();
    const camera = controls.object as unknown as { position: Vector3; matrixWorld: { elements: number[] }; updateMatrixWorld: () => void };
    const target = (controls as unknown as { target: Vector3 }).target;
    camera.updateMatrixWorld();
    const distance = camera.position.distanceTo(target);
    const step = distance * PAN_STEP_FRACTION;
    const rightVec = new Vector3().setFromMatrixColumn(camera.matrixWorld, 0).multiplyScalar(right * step);
    const upVec = new Vector3().setFromMatrixColumn(camera.matrixWorld, 1).multiplyScalar(up * step);
    const offset = rightVec.add(upVec);
    camera.position.add(offset);
    target.add(offset);
    controls.update();
  };

  return (
    <div className="bg-gray-900/80 border border-gray-700 rounded-lg px-2 py-2 flex items-center gap-3 text-[10px] uppercase tracking-wider text-gray-300">
      <div className="flex items-center gap-2">
        <span className="text-gray-500">View</span>
        <button
          onClick={() => handleClick('iso')}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
        >
          Iso
        </button>
        <button
          onClick={() => handleClick('top')}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
        >
          Top
        </button>
        <button
          onClick={() => handleClick('front')}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
        >
          Front
        </button>
        <button
          onClick={() => handleClick('right')}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
        >
          Right
        </button>
      </div>
      <div className="h-5 w-px bg-gray-700" />
      <div className="flex items-center gap-2">
        <span className="text-gray-500">Rotate</span>
        <button
          onClick={() => nudgeRotate(ROTATE_STEP_RAD, 0)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Rotate Left"
          aria-label="Rotate Left"
        >
          L
        </button>
        <button
          onClick={() => nudgeRotate(-ROTATE_STEP_RAD, 0)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Rotate Right"
          aria-label="Rotate Right"
        >
          R
        </button>
        <button
          onClick={() => nudgeRotate(0, ROTATE_STEP_RAD)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Rotate Up"
          aria-label="Rotate Up"
        >
          U
        </button>
        <button
          onClick={() => nudgeRotate(0, -ROTATE_STEP_RAD)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Rotate Down"
          aria-label="Rotate Down"
        >
          D
        </button>
      </div>
      <div className="h-5 w-px bg-gray-700" />
      <div className="flex items-center gap-2">
        <span className="text-gray-500">Pan</span>
        <button
          onClick={() => nudgePan(-1, 0)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Pan Left"
          aria-label="Pan Left"
        >
          L
        </button>
        <button
          onClick={() => nudgePan(1, 0)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Pan Right"
          aria-label="Pan Right"
        >
          R
        </button>
        <button
          onClick={() => nudgePan(0, 1)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Pan Up"
          aria-label="Pan Up"
        >
          U
        </button>
        <button
          onClick={() => nudgePan(0, -1)}
          className="px-2 py-1 rounded border border-gray-700 hover:border-blue-500"
          title="Pan Down"
          aria-label="Pan Down"
        >
          D
        </button>
      </div>
    </div>
  );
}

import { useTelemetryStore } from '../store/telemetryStore';
import { useCameraStore } from '../store/cameraStore';

export function FocusButton() {
  const targetPos = useTelemetryStore(s => s.latest?.target_position);
  const requestFocus = useCameraStore(s => s.requestFocus);

  const handleFocus = () => {
    if (targetPos) {
      requestFocus(targetPos);
    }
  };

  return (
    <button
      onClick={handleFocus}
      className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
      disabled={!targetPos}
      title="Focus Camera on Target"
    >
      Focus Target
    </button>
  );
}

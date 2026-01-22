import { useTelemetryStore } from '../store/telemetryStore';
import { useCameraStore } from '../store/cameraStore';

export function FocusButton() {
  const referencePos = useTelemetryStore(s => s.latest?.reference_position);
  const requestFocus = useCameraStore(s => s.requestFocus);

  const handleFocus = () => {
    if (referencePos) {
      requestFocus(referencePos);
    }
  };

  return (
    <button
      onClick={handleFocus}
      className="px-2 py-1 text-[10px] uppercase rounded border border-gray-700 text-gray-300 hover:border-blue-500"
      disabled={!referencePos}
      title="Focus Camera on Reference"
    >
      Focus Reference
    </button>
  );
}

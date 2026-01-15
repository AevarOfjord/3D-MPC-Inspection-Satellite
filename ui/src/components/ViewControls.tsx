import { useCameraStore } from '../store/cameraStore';

interface ViewControlsProps {
  onRequestFree: () => void;
}

export function ViewControls({ onRequestFree }: ViewControlsProps) {
  const requestViewPreset = useCameraStore(s => s.requestViewPreset);

  const handleClick = (preset: Parameters<typeof requestViewPreset>[0]) => {
    onRequestFree();
    requestViewPreset(preset);
  };

  return (
    <div className="bg-gray-900/80 border border-gray-700 rounded-lg px-2 py-2 flex items-center gap-2 text-[10px] uppercase tracking-wider text-gray-300">
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
  );
}

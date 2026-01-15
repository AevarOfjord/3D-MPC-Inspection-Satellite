import { useUiStore } from '../store/uiStore';

export function ErrorBanner() {
  const missionError = useUiStore(s => s.missionError);
  const setMissionError = useUiStore(s => s.setMissionError);

  if (!missionError) return null;

  return (
    <div className="bg-red-500/20 border-b border-red-500/40 text-red-200 text-sm px-4 py-2 flex items-center justify-between">
      <span>{missionError}</span>
      <button
        onClick={() => setMissionError(null)}
        className="text-xs uppercase text-red-200 hover:text-white"
      >
        Dismiss
      </button>
    </div>
  );
}

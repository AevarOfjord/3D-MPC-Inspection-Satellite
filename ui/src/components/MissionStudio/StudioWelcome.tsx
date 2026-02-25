import { useRef } from 'react';
import { Upload } from 'lucide-react';
import { API_BASE_URL } from '../../config/endpoints';
import { useStudioStore } from './useStudioStore';

// Known built-in models — paths are relative to the repo root as served by the backend
const BUILTIN_MODELS = [
  {
    key: 'ISS',
    label: 'ISS',
    description: 'International Space Station',
    icon: '🛸',
    path: 'data/assets/model_files/ISS/ISS.obj',
  },
  {
    key: 'Starlink',
    label: 'Starlink',
    description: 'Starlink satellite',
    icon: '🛰️',
    path: 'data/assets/model_files/Starlink/starlink.obj',
  },
];

function ModelCard({
  icon,
  label,
  description,
  onClick,
}: {
  icon: React.ReactNode;
  label: string;
  description: string;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      className="flex flex-col items-center gap-3 px-6 py-7 rounded-2xl border border-slate-700 bg-slate-900/70 hover:border-cyan-600 hover:bg-slate-800/80 transition-all duration-150 hover:scale-[1.03] active:scale-[0.99] w-44 text-center"
    >
      <span className="text-4xl leading-none">{icon}</span>
      <div>
        <div className="text-sm font-semibold text-slate-100">{label}</div>
        <div className="text-[11px] text-slate-500 mt-0.5">{description}</div>
      </div>
    </button>
  );
}

export function StudioWelcome() {
  const setModelUrl = useStudioStore((s) => s.setModelUrl);
  const setWelcomeDismissed = useStudioStore((s) => s.setWelcomeDismissed);
  const fileRef = useRef<HTMLInputElement>(null);

  const handlePickBuiltin = (path: string) => {
    const url = `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(path)}`;
    setModelUrl(url);
    setWelcomeDismissed(true);
  };

  const handleEmpty = () => {
    setWelcomeDismissed(true);
  };

  const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    setModelUrl(URL.createObjectURL(file));
    setWelcomeDismissed(true);
  };

  return (
    <div
      className="absolute inset-0 z-20 flex items-center justify-center"
      style={{ background: 'rgba(7,11,20,0.88)', backdropFilter: 'blur(6px)' }}
    >
      <div className="flex flex-col items-center gap-8 max-w-xl w-full px-8">
        {/* Header */}
        <div className="text-center">
          <div className="text-[11px] uppercase tracking-[0.2em] text-cyan-600 font-bold mb-2">Mission Studio</div>
          <h2 className="text-xl font-semibold text-slate-100">Choose a target object</h2>
          <p className="text-sm text-slate-500 mt-1">Select a model to scan around, or start with an empty scene</p>
        </div>

        {/* Cards */}
        <div className="flex items-stretch gap-4 flex-wrap justify-center">
          {BUILTIN_MODELS.map(({ key, label, description, icon, path }) => (
            <ModelCard
              key={key}
              icon={icon}
              label={label}
              description={description}
              onClick={() => handlePickBuiltin(path)}
            />
          ))}

          <ModelCard
            icon="✦"
            label="Empty Scene"
            description="No target object"
            onClick={handleEmpty}
          />
        </div>

        {/* Upload own OBJ */}
        <div className="flex flex-col items-center gap-2">
          <div className="text-[11px] text-slate-600">or</div>
          <button
            type="button"
            onClick={() => fileRef.current?.click()}
            className="flex items-center gap-2 px-4 py-2 rounded-lg border border-slate-700 text-xs font-semibold text-slate-400 hover:border-slate-500 hover:text-slate-200 transition-all"
          >
            <Upload size={12} />
            Load your own OBJ file
          </button>
          <input ref={fileRef} type="file" accept=".obj" className="hidden" onChange={handleFileChange} />
        </div>
      </div>
    </div>
  );
}

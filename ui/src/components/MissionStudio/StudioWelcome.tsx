import { useRef, useState } from 'react';
import { Upload } from 'lucide-react';
import { trajectoryApi } from '../../api/trajectory';
import { useStudioStore } from './useStudioStore';
import { studioModelPathToUrl } from './studioReference';

// Known built-in models — paths are relative to the repo root as served by the backend
const BUILTIN_MODELS = [
  {
    key: 'ISS',
    label: 'ISS',
    description: 'International Space Station',
    path: 'data/assets/model_files/ISS/ISS.obj',
  },
  {
    key: 'Starlink',
    label: 'Starlink',
    description: 'Starlink satellite',
    path: 'data/assets/model_files/Starlink/starlink.obj',
  },
];

function ModelCard({
  label,
  description,
  onClick,
}: {
  label: string;
  description: string;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      className="flex flex-col items-center justify-center gap-2 px-6 py-7 rounded-2xl border border-slate-700 bg-slate-900/70 hover:border-cyan-600 hover:bg-slate-800/80 transition-all duration-150 hover:scale-[1.03] active:scale-[0.99] w-44 min-h-[190px] text-center"
    >
      <div>
        <div className="text-sm font-semibold text-slate-100">{label}</div>
        <div className="text-[11px] text-slate-500 mt-0.5">{description}</div>
      </div>
    </button>
  );
}

export function StudioWelcome() {
  const setModelUrl = useStudioStore((s) => s.setModelUrl);
  const setReferenceObjectPath = useStudioStore((s) => s.setReferenceObjectPath);
  const setWelcomeDismissed = useStudioStore((s) => s.setWelcomeDismissed);
  const fileRef = useRef<HTMLInputElement>(null);
  const [uploading, setUploading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handlePickBuiltin = (path: string) => {
    setReferenceObjectPath(path);
    setModelUrl(studioModelPathToUrl(path));
    setWelcomeDismissed(true);
  };

  const handleEmpty = () => {
    setReferenceObjectPath(null);
    setModelUrl(null);
    setWelcomeDismissed(true);
  };

  const handleFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    setUploading(true);
    setError(null);
    try {
      const uploaded = await trajectoryApi.uploadObject(file);
      setReferenceObjectPath(uploaded.path);
      setModelUrl(studioModelPathToUrl(uploaded.path));
      setWelcomeDismissed(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setUploading(false);
      if (fileRef.current) {
        fileRef.current.value = '';
      }
    }
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
          <p className="text-sm text-slate-500 mt-1">
            Selected object is fixed at [0,0,0]. All authored values are LVLH-local to that origin.
          </p>
        </div>

        {/* Cards */}
        <div className="flex items-stretch gap-4 flex-wrap justify-center">
          {BUILTIN_MODELS.map(({ key, label, description, path }) => (
            <ModelCard
              key={key}
              label={label}
              description={description}
              onClick={() => handlePickBuiltin(path)}
            />
          ))}

          <ModelCard
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
            disabled={uploading}
            className="flex items-center gap-2 px-4 py-2 rounded-lg border border-slate-700 text-xs font-semibold text-slate-400 hover:border-slate-500 hover:text-slate-200 transition-all"
          >
            <Upload size={12} />
            {uploading ? 'Uploading...' : 'Load your own OBJ file'}
          </button>
          <input ref={fileRef} type="file" accept=".obj" className="hidden" onChange={handleFileChange} />
          {error && <div className="text-[11px] text-red-400 max-w-sm text-center">{error}</div>}
        </div>
      </div>
    </div>
  );
}

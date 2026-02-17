import { useState } from 'react';

export function useMissionRuntimeState() {
  const [modelUrl, setModelUrl] = useState<string | null>(null);
  const [modelPath, setModelPath] = useState<string>('');
  const [loading, setLoading] = useState(false);
  const [isManualMode, setIsManualMode] = useState(false);
  const [stats, setStats] = useState<{ duration: number; length: number; points: number } | null>(
    null
  );

  return {
    state: {
      modelUrl,
      modelPath,
      loading,
      isManualMode,
      stats,
    },
    setters: {
      setModelUrl,
      setModelPath,
      setLoading,
      setIsManualMode,
      setStats,
    },
  };
}

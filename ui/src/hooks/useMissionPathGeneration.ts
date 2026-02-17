import type { Dispatch, SetStateAction } from 'react';

import { trajectoryApi, type MeshScanConfig } from '../api/trajectory';
import { downsamplePath } from '../utils/pathResample';
import { computePathLength } from '../utils/pathMetrics';
import { useToast } from '../feedback/feedbackContext';

interface HistoryAdapter {
  set: (next: [number, number, number][]) => void;
}

interface UseMissionPathGenerationArgs {
  config: MeshScanConfig;
  levelSpacing: number;
  editPointLimit: number;
  pathHistory: HistoryAdapter;
  setIsManualMode: Dispatch<SetStateAction<boolean>>;
  setLoading: Dispatch<SetStateAction<boolean>>;
  setStats: Dispatch<
    SetStateAction<{ duration: number; length: number; points: number } | null>
  >;
}

export function useMissionPathGeneration({
  config,
  levelSpacing,
  editPointLimit,
  pathHistory,
  setIsManualMode,
  setLoading,
  setStats,
}: UseMissionPathGenerationArgs) {
  const { showToast } = useToast();

  const handlePreview = async (overrideConfig?: Partial<MeshScanConfig>) => {
    if (!config.obj_path) {
      showToast({ tone: 'error', title: 'Missing Model', message: 'Please upload a model first.' });
      return;
    }
    setLoading(true);
    try {
      const previewConfig: MeshScanConfig = { ...config, ...(overrideConfig ?? {}) };
      if (levelSpacing > 0 && (!previewConfig.passes || previewConfig.passes.length === 0)) {
        previewConfig.level_spacing = levelSpacing;
      }
      const res = await trajectoryApi.previewTrajectory(previewConfig);
      const editablePath = downsamplePath(res.path, editPointLimit);
      pathHistory.set(editablePath);
      setIsManualMode(false);
      setStats({
        duration: res.estimated_duration,
        length: res.path_length,
        points: editablePath.length,
      });
    } catch (err) {
      console.error(err);
      showToast({
        tone: 'error',
        title: 'Preview Failed',
        message: 'Preview generation failed.',
      });
    } finally {
      setLoading(false);
    }
  };

  const setManualPath = (path: [number, number, number][]) => {
    const editablePath = downsamplePath(path, editPointLimit);
    pathHistory.set(editablePath);
    setIsManualMode(true);
    const length = computePathLength(editablePath);
    const speed = config.speed_max > 0 ? config.speed_max : 0.1;
    setStats({
      duration: speed > 0 ? length / speed : 0,
      length,
      points: editablePath.length,
    });
  };

  return {
    actions: {
      handlePreview,
      setManualPath,
    },
  };
}

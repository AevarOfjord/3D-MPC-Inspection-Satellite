import { trajectoryApi, type MeshScanConfig, type ModelInfo } from '../api/trajectory';
import { pathAssetsApi, type PathAssetSummary } from '../api/pathAssets';
import { scanProjectsApi } from '../api/scanProjects';
import type { ScanProject, ScanProjectSummary } from '../types/scanProject';
import { API_BASE_URL } from '../config/endpoints';
import { useCameraStore } from '../store/cameraStore';
import { ORBIT_SCALE } from '../data/orbitSnapshot';
import { resamplePath, downsamplePath } from '../utils/pathResample';
import type { Dispatch, SetStateAction } from 'react';
import { useCallback } from 'react';
import { useToast } from '../feedback/feedbackContext';

interface HistoryAdapter {
  set: (next: [number, number, number][]) => void;
}

interface UseMissionAssetsArgs {
  config: MeshScanConfig;
  setConfig: Dispatch<SetStateAction<MeshScanConfig>>;
  setLoading: Dispatch<SetStateAction<boolean>>;
  setModelUrl: Dispatch<SetStateAction<string | null>>;
  setModelPath: Dispatch<SetStateAction<string>>;
  setScanProject: Dispatch<SetStateAction<ScanProject>>;
  setScanProjectAutoPreviewEnabled: Dispatch<SetStateAction<boolean>>;
  setAvailableModels: Dispatch<SetStateAction<ModelInfo[]>>;
  setPathAssets: Dispatch<SetStateAction<PathAssetSummary[]>>;
  setScanProjects: Dispatch<SetStateAction<ScanProjectSummary[]>>;
  setStats: Dispatch<
    SetStateAction<{ duration: number; length: number; points: number } | null>
  >;
  setIsManualMode: Dispatch<SetStateAction<boolean>>;
  pathHistory: HistoryAdapter;
  previewPath: [number, number, number][];
  editPointLimit: number;
  savePointMultiplier: number;
}

export function useMissionAssets({
  config,
  setConfig,
  setLoading,
  setModelUrl,
  setModelPath,
  setScanProject,
  setScanProjectAutoPreviewEnabled,
  setAvailableModels,
  setPathAssets,
  setScanProjects,
  setStats,
  setIsManualMode,
  pathHistory,
  previewPath,
  editPointLimit,
  savePointMultiplier,
}: UseMissionAssetsArgs) {
  const { showToast } = useToast();

  const refreshModelList = useCallback(async () => {
    const models = await trajectoryApi.listModels();
    setAvailableModels(models);
    return models;
  }, [setAvailableModels]);

  const refreshPathAssets = useCallback(async () => {
    const assets = await pathAssetsApi.list();
    setPathAssets(assets);
    return assets;
  }, [setPathAssets]);

  const refreshScanProjects = useCallback(async () => {
    const projects = await scanProjectsApi.listScanProjects();
    setScanProjects(projects);
    return projects;
  }, [setScanProjects]);

  const selectModelPath = useCallback((path: string) => {
    if (!path) return;
    setModelPath(path);
    setConfig((prev) => ({ ...prev, obj_path: path }));
    setScanProject((prev) => ({ ...prev, obj_path: path }));
    setScanProjectAutoPreviewEnabled(false);
    setModelUrl(`${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(path)}`);
    trajectoryApi
      .getModelBounds(path)
      .then((bounds) => {
        const extent = Math.max(bounds.extents[0], bounds.extents[1], bounds.extents[2]);
        const distance = Math.max(extent * 2.5, 5);
        // Planner uses floating-origin scene space; center focus on local origin.
        useCameraStore.getState().requestFocus([0, 0, 0], distance * ORBIT_SCALE);
      })
      .catch(() => null);
  }, [
    setModelPath,
    setConfig,
    setScanProject,
    setScanProjectAutoPreviewEnabled,
    setModelUrl,
  ]);

  const handleFileUpload = useCallback(async (file: File) => {
    setLoading(true);
    try {
      const url = URL.createObjectURL(file);
      setModelUrl(url);
      const res = await trajectoryApi.uploadObject(file);
      setModelPath(res.path);
      setConfig((prev) => ({ ...prev, obj_path: res.path }));
      setScanProject((prev) => ({ ...prev, obj_path: res.path }));
      setScanProjectAutoPreviewEnabled(false);
      trajectoryApi.listModels().then(setAvailableModels).catch(() => null);
    } catch (err) {
      console.error(err);
      showToast({ tone: 'error', title: 'Upload Failed', message: 'Upload failed.' });
    } finally {
      setLoading(false);
    }
  }, [
    setLoading,
    setModelUrl,
    setModelPath,
    setConfig,
    setScanProject,
    setScanProjectAutoPreviewEnabled,
    setAvailableModels,
    showToast,
  ]);

  const savePathAsset = useCallback(async (name: string) => {
    const trimmed = name.trim();
    if (!trimmed) {
      showToast({
        tone: 'error',
        title: 'Missing Name',
        message: 'Please enter a path asset name.',
      });
      return;
    }
    if (!config.obj_path) {
      showToast({
        tone: 'error',
        title: 'Missing Model',
        message: 'Select an OBJ model first.',
      });
      return;
    }
    if (previewPath.length === 0) {
      showToast({
        tone: 'error',
        title: 'Missing Path',
        message: 'Generate or load a path before saving.',
      });
      return;
    }
    const densePath = resamplePath(previewPath, savePointMultiplier);
    const payload = {
      name: trimmed,
      obj_path: config.obj_path,
      path: densePath,
      open: true,
      relative_to_obj: true,
    };
    const saved = await pathAssetsApi.save(payload);
    await refreshPathAssets();
    return saved;
  }, [
    config.obj_path,
    previewPath,
    savePointMultiplier,
    showToast,
    refreshPathAssets,
  ]);

  const loadPathAsset = useCallback(async (assetId: string) => {
    const asset = await pathAssetsApi.get(assetId);
    if (asset.path && asset.path.length > 0) {
      const editablePath = downsamplePath(asset.path, editPointLimit);
      pathHistory.set(editablePath);
      setIsManualMode(true);
      const speed = config.speed_max > 0 ? config.speed_max : 0.1;
      setStats({
        duration: asset.path_length > 0 ? asset.path_length / speed : 0,
        length: asset.path_length,
        points: editablePath.length,
      });
    }
    if (asset.obj_path) {
      setModelPath(asset.obj_path);
      setConfig((prev) => ({ ...prev, obj_path: asset.obj_path }));
      setModelUrl(`${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(asset.obj_path)}`);
    }
    return asset;
  }, [
    editPointLimit,
    pathHistory,
    setIsManualMode,
    config.speed_max,
    setStats,
    setModelPath,
    setConfig,
    setModelUrl,
  ]);

  return {
    actions: {
      refreshModelList,
      refreshPathAssets,
      refreshScanProjects,
      selectModelPath,
      handleFileUpload,
      savePathAsset,
      loadPathAsset,
    },
  };
}

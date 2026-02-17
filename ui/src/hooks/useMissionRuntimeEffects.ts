import { useEffect, useRef } from 'react';

import { telemetry } from '../services/telemetry';
import type { MeshScanConfig } from '../api/trajectory';
import type { ScanSegment } from '../api/unifiedMission';

interface HistoryAdapter {
  set: (next: [number, number, number][]) => void;
}

interface UseMissionRuntimeEffectsArgs {
  segmentsLength: number;
  setSegments: (next: ScanSegment[]) => void;
  defaultScanSegment: () => ScanSegment;
  refreshModelList: () => Promise<unknown>;
  refreshPathAssets: () => Promise<unknown>;
  refreshScanProjects: () => Promise<unknown>;
  isManualMode: boolean;
  pathHistory: HistoryAdapter;
  previewPath: [number, number, number][];
  config: MeshScanConfig;
  setStats: (next: { duration: number; length: number; points: number }) => void;
  loading: boolean;
  handlePreview: () => Promise<void>;
}

export function useMissionRuntimeEffects({
  segmentsLength,
  setSegments,
  defaultScanSegment,
  refreshModelList,
  refreshPathAssets,
  refreshScanProjects,
  isManualMode,
  pathHistory,
  previewPath,
  config,
  setStats,
  loading,
  handlePreview,
}: UseMissionRuntimeEffectsArgs) {
  useEffect(() => {
    if (segmentsLength === 0) {
      setSegments([defaultScanSegment()]);
    }
  }, [segmentsLength, setSegments, defaultScanSegment]);

  useEffect(() => {
    refreshModelList().catch((err) => {
      console.warn('Failed to load model list', err);
    });
    refreshPathAssets().catch((err) => {
      console.warn('Failed to load path assets', err);
    });
    refreshScanProjects().catch((err) => {
      console.warn('Failed to load scan projects', err);
    });
  }, [refreshModelList, refreshPathAssets, refreshScanProjects]);

  useEffect(() => {
    const unsub = telemetry.subscribe((data) => {
      if (isManualMode) return;
      const planned = data.planned_path;
      if (planned && planned.length > 0) {
        pathHistory.set(planned as [number, number, number][]);
      }
    });
    return () => {
      unsub();
    };
  }, [isManualMode, pathHistory]);

  useEffect(() => {
    if (previewPath.length < 2) return;
    const length = previewPath.reduce((acc, point, idx) => {
      if (idx === 0) return acc;
      const prev = previewPath[idx - 1];
      const dx = point[0] - prev[0];
      const dy = point[1] - prev[1];
      const dz = point[2] - prev[2];
      return acc + Math.sqrt(dx * dx + dy * dy + dz * dz);
    }, 0);
    const speed = config.speed_max > 0 ? config.speed_max : 0.1;
    setStats({
      duration: length / speed,
      length,
      points: previewPath.length,
    });
  }, [previewPath, config.speed_max, setStats]);

  const prevAxisRef = useRef(config.scan_axis);
  useEffect(() => {
    if (prevAxisRef.current === config.scan_axis) return;
    prevAxisRef.current = config.scan_axis;
    if (!config.obj_path || previewPath.length === 0) return;
    if (loading) return;
    handlePreview().catch((err) => {
      console.warn('Auto preview failed', err);
    });
  }, [config.scan_axis]);
}

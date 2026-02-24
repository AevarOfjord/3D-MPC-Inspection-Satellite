import { useEffect, useRef } from 'react';
import type { Dispatch, SetStateAction } from 'react';

import { telemetry } from '../services/telemetry';
import type { MeshScanConfig } from '../api/trajectory';
import type { MissionSegment, ScanSegment, TransferSegment } from '../api/unifiedMission';
import type { ScanProject, BodyAxis } from '../types/scanProject';

interface HistoryAdapter {
  set: (next: [number, number, number][]) => void;
}

interface UseMissionRuntimeEffectsArgs {
  segments: MissionSegment[];
  setSegments: (next: MissionSegment[] | ((prev: MissionSegment[]) => MissionSegment[])) => void;
  defaultScanSegment: () => ScanSegment;
  defaultTransferToPathSegment: () => TransferSegment;
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
  setScanProject?: Dispatch<SetStateAction<ScanProject>>;
}

export function useMissionRuntimeEffects({
  segments,
  setSegments,
  defaultScanSegment,
  defaultTransferToPathSegment,
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
  setScanProject,
}: UseMissionRuntimeEffectsArgs) {
  useEffect(() => {
    const hasScan = segments.some((segment) => segment.type === 'scan');
    const transferIndices = segments
      .map((segment, index) => (segment.type === 'transfer' ? index : -1))
      .filter((index) => index >= 0);
    const hasCoreTransfer = segments.some(
      (segment) => segment.type === 'transfer' && segment.title === 'Transfer To Path'
    );
    if (hasScan && transferIndices.length > 0) {
      if (!hasCoreTransfer && transferIndices.length > 0) {
        setSegments((prev) => {
          let promoted = false;
          return prev.map((segment, index) => {
            if (
              !promoted &&
              index === transferIndices[0] &&
              segment.type === 'transfer'
            ) {
              promoted = true;
              return { ...segment, title: 'Transfer To Path' };
            }
            return segment;
          });
        });
      }
      return;
    }
    setSegments((prev) => {
      const next = [...prev];
      const nextHasScan = next.some((segment) => segment.type === 'scan');
      if (!nextHasScan) {
        next.push(defaultScanSegment());
      }
      const nextHasTransfer = next.some((segment) => segment.type === 'transfer');
      if (!nextHasTransfer) {
        next.unshift(defaultTransferToPathSegment());
      }
      return next;
    });
  }, [segments, setSegments, defaultScanSegment, defaultTransferToPathSegment]);

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

    // Sync the path-maker scan axis to the default scan project so
    // buildUnifiedMissionPayload inserts the correct axis into the mission JSON.
    if (setScanProject) {
      const newAxis = config.scan_axis as BodyAxis;
      setScanProject((prev) => ({
        ...prev,
        scans: prev.scans.map((scan) => ({ ...scan, axis: newAxis })),
      }));
    }

    if (!config.obj_path || previewPath.length === 0) return;
    if (loading) return;
    handlePreview().catch((err) => {
      console.warn('Auto preview failed', err);
    });
  }, [config.scan_axis]);
}

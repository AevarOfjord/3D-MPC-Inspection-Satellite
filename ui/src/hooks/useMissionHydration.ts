import type { Dispatch, SetStateAction } from 'react';

import type { ScanSegment, SplineControl, UnifiedMission } from '../api/unifiedMission';
import type { ValidationReportV2 } from '../api/unifiedMissionApi';
import { computePathLength } from '../utils/pathMetrics';

interface UseMissionHydrationArgs {
  nextMissionId: () => string;
  nextSegmentId: (prefix: string) => string;
  resolveOrbitTargetPose: (targetId: string) =>
    | { frame: 'ECI'; position: [number, number, number]; orientation?: [number, number, number, number] }
    | undefined;
  setMissionId: Dispatch<SetStateAction<string>>;
  setMissionName: Dispatch<SetStateAction<string>>;
  setEpoch: Dispatch<SetStateAction<string>>;
  setSegments: Dispatch<SetStateAction<UnifiedMission['segments']>>;
  setSplineControls: Dispatch<SetStateAction<SplineControl[]>>;
  setPreviewPath: (path: [number, number, number][]) => void;
  setIsManualMode: Dispatch<SetStateAction<boolean>>;
  speedMax: number;
  setStats: Dispatch<SetStateAction<{ duration: number; length: number; points: number } | null>>;
  setSelectedSegmentIndex: Dispatch<SetStateAction<number | null>>;
  setStartPosition: Dispatch<SetStateAction<[number, number, number]>>;
  setObstacles: Dispatch<SetStateAction<{ position: [number, number, number]; radius: number }[]>>;
  setSelectedOrbitTargetId: Dispatch<SetStateAction<string | null>>;
  setValidationReport: Dispatch<SetStateAction<ValidationReportV2 | null>>;
}

export function useMissionHydration({
  nextMissionId,
  nextSegmentId,
  resolveOrbitTargetPose,
  setMissionId,
  setMissionName,
  setEpoch,
  setSegments,
  setSplineControls,
  setPreviewPath,
  setIsManualMode,
  speedMax,
  setStats,
  setSelectedSegmentIndex,
  setStartPosition,
  setObstacles,
  setSelectedOrbitTargetId,
  setValidationReport,
}: UseMissionHydrationArgs) {
  const applyLoadedMission = (mission: UnifiedMission, fallbackName?: string) => {
    setMissionId(mission.mission_id || nextMissionId());
    setMissionName(mission.name || fallbackName || 'Mission_V2');
    setEpoch(mission.epoch);

    const hydratedSegments = mission.segments.map((seg) => {
      const segWithId = {
        ...seg,
        segment_id: seg.segment_id || nextSegmentId(seg.type || 'segment'),
        title: seg.title ?? null,
        notes: seg.notes ?? null,
      };
      if (seg.type === 'scan' && seg.target_id && !seg.target_pose) {
        const resolvedPose = resolveOrbitTargetPose(seg.target_id);
        if (resolvedPose) {
          return {
            ...segWithId,
            target_pose: resolvedPose,
          } as ScanSegment;
        }
      }
      return segWithId;
    });

    setSegments(hydratedSegments);
    setSplineControls(mission.overrides?.spline_controls || []);

    const manualPath = mission.overrides?.manual_path || [];
    if (manualPath.length > 0) {
      const normalizedPath = manualPath.map(
        (p) => [p[0], p[1], p[2]] as [number, number, number]
      );
      setPreviewPath(normalizedPath);
      setIsManualMode(true);
      const length = computePathLength(normalizedPath);
      const speed = speedMax > 0 ? speedMax : 0.1;
      setStats({
        duration: speed > 0 ? length / speed : 0,
        length,
        points: normalizedPath.length,
      });
    } else {
      setPreviewPath([]);
      setIsManualMode(false);
    }

    setSelectedSegmentIndex(null);
    setStartPosition([...mission.start_pose.position] as [number, number, number]);
    if (mission.obstacles) {
      setObstacles(
        mission.obstacles.map((o) => ({
          position: [...o.position] as [number, number, number],
          radius: o.radius,
        }))
      );
    }

    const firstScan = hydratedSegments.find((seg) => seg.type === 'scan') as
      | ScanSegment
      | undefined;
    setSelectedOrbitTargetId(firstScan?.target_id ?? null);
    setValidationReport(null);
  };

  return {
    actions: {
      applyLoadedMission,
    },
  };
}

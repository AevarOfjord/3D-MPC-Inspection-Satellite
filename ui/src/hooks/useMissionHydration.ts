import { useRef, type Dispatch, type SetStateAction } from 'react';

import type {
  ScanSegment,
  SplineControl,
  TransferSegment,
  UnifiedMission,
} from '../api/unifiedMission';
import type { ValidationReportV2 } from '../api/unifiedMissionApi';
import type { TransferTargetRef } from '../types/plannerUx';
import type { ScanProject } from '../types/scanProject';
import { computePathLength } from '../utils/pathMetrics';
import { normalizePathDensityMultiplier } from '../utils/pathDensity';
import { useToast } from '../feedback/feedbackContext';

const SCAN_AXIS_MIGRATION_NOTICE_TAG = 'migration:scan_axis_asset_mismatch';

type OrbitPoseResolver = (
  targetId: string
) =>
  | { frame: 'ECI'; position: [number, number, number]; orientation?: [number, number, number, number] }
  | undefined;

type OriginResolution = {
  targetId?: string;
  origin?: [number, number, number];
};

function resolveMissionOrigin(
  mission: UnifiedMission,
  resolveOrbitTargetPose: OrbitPoseResolver
): OriginResolution {
  const candidateTargetIds: string[] = [];
  if (mission.start_target_id) {
    candidateTargetIds.push(mission.start_target_id);
  }
  for (const segment of mission.segments) {
    if (segment.type === 'scan' && segment.target_id && !candidateTargetIds.includes(segment.target_id)) {
      candidateTargetIds.push(segment.target_id);
    }
  }
  for (const targetId of candidateTargetIds) {
    const resolved = resolveOrbitTargetPose(targetId);
    if (resolved) {
      return { targetId, origin: resolved.position };
    }
  }
  for (const segment of mission.segments) {
    if (segment.type === 'scan' && segment.target_pose?.position && segment.target_pose.position.length === 3) {
      return {
        targetId: segment.target_id || undefined,
        origin: [
          segment.target_pose.position[0],
          segment.target_pose.position[1],
          segment.target_pose.position[2],
        ],
      };
    }
  }
  return {};
}

function normalizeManualPathForEditor(
  manualPath: [number, number, number][],
  origin?: [number, number, number]
): { path: [number, number, number][]; legacyAbsoluteLike: boolean } {
  if (!origin || manualPath.length === 0) {
    return { path: manualPath, legacyAbsoluteLike: false };
  }
  const originNorm = Math.hypot(origin[0], origin[1], origin[2]);
  if (originNorm < 1e5) {
    return { path: manualPath, legacyAbsoluteLike: false };
  }

  let absoluteLike = 0;
  let localLike = 0;
  for (const point of manualPath) {
    const pointNorm = Math.hypot(point[0], point[1], point[2]);
    const distToOrigin = Math.hypot(
      point[0] - origin[0],
      point[1] - origin[1],
      point[2] - origin[2]
    );
    if (pointNorm > 1e5 && distToOrigin < 1e5) {
      absoluteLike += 1;
    }
    if (pointNorm < 1e5) {
      localLike += 1;
    }
  }
  const threshold = Math.max(1, Math.floor(manualPath.length / 2));
  if (absoluteLike >= threshold && absoluteLike > localLike) {
    return { path: manualPath, legacyAbsoluteLike: true };
  }
  if (localLike >= threshold && localLike >= absoluteLike) {
    return {
      path: manualPath.map((point) => [
        point[0] + origin[0],
        point[1] + origin[1],
        point[2] + origin[2],
      ]),
      legacyAbsoluteLike: false,
    };
  }
  return { path: manualPath, legacyAbsoluteLike: false };
}

interface UseMissionHydrationArgs {
  nextMissionId: () => string;
  nextSegmentId: (prefix: string) => string;
  resolveOrbitTargetPose: OrbitPoseResolver;
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
  setStartFrame: Dispatch<SetStateAction<'ECI' | 'LVLH'>>;
  setStartTargetId: Dispatch<SetStateAction<string | undefined>>;
  setStartPosition: Dispatch<SetStateAction<[number, number, number]>>;
  setObstacles: Dispatch<SetStateAction<{ position: [number, number, number]; radius: number }[]>>;
  setSelectedOrbitTargetId: Dispatch<SetStateAction<string | null>>;
  setTransferTargetRef: Dispatch<SetStateAction<TransferTargetRef>>;
  setValidationReport: Dispatch<SetStateAction<ValidationReportV2 | null>>;
  setScanProject: Dispatch<SetStateAction<ScanProject>>;
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
  setStartFrame,
  setStartTargetId,
  setStartPosition,
  setObstacles,
  setSelectedOrbitTargetId,
  setTransferTargetRef,
  setValidationReport,
  setScanProject,
}: UseMissionHydrationArgs) {
  const { showToast } = useToast();
  const migrationToastByMissionRef = useRef<Set<string>>(new Set());
  const axisMigrationToastByMissionRef = useRef<Set<string>>(new Set());

  const applyLoadedMission = (mission: UnifiedMission, fallbackName?: string) => {
    const resolvedMissionId = mission.mission_id || nextMissionId();
    setMissionId(resolvedMissionId);
    setMissionName(mission.name || fallbackName || 'Mission_V2');
    setEpoch(mission.epoch);
    const { targetId: inferredTargetId, origin } = resolveMissionOrigin(
      mission,
      resolveOrbitTargetPose
    );
    const startTargetId = mission.start_target_id || inferredTargetId;
    let migrated = false;
    let hydratedStartPosition = [...mission.start_pose.position] as [number, number, number];
    if (mission.start_pose.frame === 'ECI' && origin) {
      hydratedStartPosition = [
        mission.start_pose.position[0] - origin[0],
        mission.start_pose.position[1] - origin[1],
        mission.start_pose.position[2] - origin[2],
      ];
      migrated = true;
    }
    if (mission.start_pose.frame !== 'LVLH') {
      migrated = true;
    }

    const hydratedSegments: UnifiedMission['segments'] = mission.segments.map((seg) => {
      const segWithId = {
        ...seg,
        segment_id: seg.segment_id || nextSegmentId(seg.type || 'segment'),
        title: seg.title ?? null,
        notes: seg.notes ?? null,
      };
      if (seg.type === 'transfer') {
        let transferPosition = [...seg.end_pose.position] as [number, number, number];
        if (seg.end_pose.frame === 'ECI' && origin) {
          transferPosition = [
            seg.end_pose.position[0] - origin[0],
            seg.end_pose.position[1] - origin[1],
            seg.end_pose.position[2] - origin[2],
          ];
          migrated = true;
        }
        if (seg.end_pose.frame !== 'LVLH') {
          migrated = true;
        }
        return {
          ...segWithId,
          target_id: seg.target_id || startTargetId,
          end_pose: {
            ...seg.end_pose,
            frame: 'LVLH' as const,
            position: transferPosition,
          },
        } as TransferSegment;
      }
      if (seg.type === 'scan' && seg.target_id && !seg.target_pose) {
        const resolvedPose = resolveOrbitTargetPose(seg.target_id);
        if (resolvedPose) {
          return {
            ...segWithId,
            target_pose: resolvedPose,
            scan: {
              ...seg.scan,
              frame: 'LVLH' as const,
            },
          } as ScanSegment;
        }
      }
      if (seg.type === 'scan' && seg.scan.frame !== 'LVLH') {
        migrated = true;
        return {
          ...segWithId,
          scan: {
            ...seg.scan,
            frame: 'LVLH' as const,
          },
        } as ScanSegment;
      }
      return segWithId as UnifiedMission['segments'][number];
    });

    setSegments(hydratedSegments);
    setSplineControls(mission.overrides?.spline_controls || []);

    const manualPath = mission.overrides?.manual_path || [];
    if (manualPath.length > 0) {
      const normalizedPath = manualPath.map(
        (p) => [p[0], p[1], p[2]] as [number, number, number]
      );
      const { path: pathForEditor, legacyAbsoluteLike } = normalizeManualPathForEditor(
        normalizedPath,
        origin
      );
      if (legacyAbsoluteLike) {
        migrated = true;
      }
      setPreviewPath(pathForEditor);
      setIsManualMode(true);
      const length = computePathLength(pathForEditor);
      const speed = speedMax > 0 ? speedMax : 0.1;
      setStats({
        duration: speed > 0 ? length / speed : 0,
        length,
        points: pathForEditor.length,
      });
    } else {
      setPreviewPath([]);
      setIsManualMode(false);
    }

    setSelectedSegmentIndex(null);
    setStartFrame('LVLH');
    setStartTargetId(startTargetId);
    setStartPosition(hydratedStartPosition);
    if (mission.obstacles) {
      setObstacles(
        mission.obstacles.map((o) => ({
          position: [...o.position] as [number, number, number],
          radius: o.radius,
        }))
      );
    } else {
      setObstacles([]);
    }

    const firstScan = hydratedSegments.find((seg) => seg.type === 'scan') as
      | ScanSegment
      | undefined;
    setSelectedOrbitTargetId(firstScan?.target_id ?? startTargetId ?? null);
    setTransferTargetRef(null);
    setValidationReport(null);
    const missionDensity = normalizePathDensityMultiplier(
      mission.overrides?.path_density_multiplier ?? 1.0
    );
    setScanProject((prev) => ({
      ...prev,
      path_density_multiplier: missionDensity,
    }));

    if (migrated && !migrationToastByMissionRef.current.has(resolvedMissionId)) {
      migrationToastByMissionRef.current.add(resolvedMissionId);
      showToast({
        tone: 'info',
        title: 'Mission migrated to LVLH',
        message: 'Legacy ECI fields were mapped to LVLH for planner editing.',
      });
    }

    const tags = mission.metadata?.tags || [];
    const hasAxisMigrationNotice = tags.includes(SCAN_AXIS_MIGRATION_NOTICE_TAG);
    if (
      hasAxisMigrationNotice &&
      !axisMigrationToastByMissionRef.current.has(resolvedMissionId)
    ) {
      axisMigrationToastByMissionRef.current.add(resolvedMissionId);
      showToast({
        tone: 'warning',
        title: 'Scan axis auto-migrated',
        message:
          'Legacy scan.axis metadata did not match the attached path asset and was adjusted from planner geometry.',
      });
    }
  };

  return {
    actions: {
      applyLoadedMission,
    },
  };
}

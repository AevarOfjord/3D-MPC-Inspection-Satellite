import type {
  UnifiedMission,
  ScanSegment,
  SplineControl,
  TransferSegment,
} from '../api/unifiedMission';
import { orbitSnapshot } from '../data/orbitSnapshot';

interface BuildUnifiedMissionArgs {
  includeManualPath?: boolean;
  missionId: string;
  missionName: string;
  epoch: string;
  startFrame: 'ECI' | 'LVLH';
  startTargetId?: string;
  startPosition: [number, number, number];
  segments: UnifiedMission['segments'];
  splineControls: SplineControl[];
  isManualMode: boolean;
  previewPath: [number, number, number][];
  obstacles: { position: [number, number, number]; radius: number }[];
  draftRevision: number | null;
  nextSegmentId: (prefix: string) => string;
  resolveOrbitTargetPose: (targetId: string) =>
    | { frame: 'ECI'; position: [number, number, number]; orientation?: [number, number, number, number] }
    | undefined;
}

export function buildUnifiedMissionPayload({
  includeManualPath = true,
  missionId,
  missionName,
  epoch,
  startFrame,
  startTargetId,
  startPosition,
  segments,
  splineControls,
  isManualMode,
  previewPath,
  obstacles,
  draftRevision,
  nextSegmentId,
  resolveOrbitTargetPose,
}: BuildUnifiedMissionArgs): UnifiedMission {
  let resolvedStartPose = {
    frame: startFrame,
    position: [...startPosition] as [number, number, number],
  };
  let resolvedStartTargetId = startTargetId;

  if (startFrame === 'LVLH' && startTargetId) {
    const targetObj = orbitSnapshot.objects.find((o) => o.id === startTargetId);
    if (targetObj) {
      const absPos: [number, number, number] = [
        targetObj.position_m[0] + startPosition[0],
        targetObj.position_m[1] + startPosition[1],
        targetObj.position_m[2] + startPosition[2],
      ];
      resolvedStartPose = { frame: 'ECI', position: absPos };
      resolvedStartTargetId = undefined;
    }
  }

  const hasManualPath = includeManualPath && isManualMode && previewPath.length > 0;
  const overrides: UnifiedMission['overrides'] = {};
  if (splineControls.length > 0) {
    overrides.spline_controls = splineControls;
  }
  if (hasManualPath) {
    overrides.manual_path = previewPath.map(
      (p) => [p[0], p[1], p[2]] as [number, number, number]
    );
  }

  return {
    schema_version: 2,
    mission_id: missionId,
    name: missionName,
    epoch,
    start_pose: resolvedStartPose,
    start_target_id: resolvedStartTargetId,
    segments: segments.map((seg) => {
      const segmentId = seg.segment_id || nextSegmentId(seg.type || 'segment');
      if (seg.type === 'transfer' && seg.end_pose.frame === 'LVLH' && seg.target_id) {
        const targetObj = orbitSnapshot.objects.find((o) => o.id === seg.target_id);
        if (targetObj) {
          const absPos: [number, number, number] = [
            targetObj.position_m[0] + seg.end_pose.position[0],
            targetObj.position_m[1] + seg.end_pose.position[1],
            targetObj.position_m[2] + seg.end_pose.position[2],
          ];

          return {
            ...seg,
            segment_id: segmentId,
            end_pose: {
              frame: 'ECI',
              position: absPos,
              orientation: seg.end_pose.orientation,
            },
            target_id: undefined,
          } as TransferSegment;
        }
      }
      if (seg.type === 'scan' && seg.target_id) {
        const resolvedPose = resolveOrbitTargetPose(seg.target_id);
        if (resolvedPose) {
          return {
            ...seg,
            segment_id: segmentId,
            target_pose: resolvedPose,
          } as ScanSegment;
        }
      }
      return {
        ...seg,
        segment_id: segmentId,
        title: seg.title ?? null,
        notes: seg.notes ?? null,
      };
    }),
    obstacles: obstacles.map((o) => ({
      position: [...o.position] as [number, number, number],
      radius: o.radius,
    })),
    overrides: Object.keys(overrides).length > 0 ? overrides : undefined,
    metadata: {
      version: Math.max(1, draftRevision ?? 1),
      updated_at: new Date().toISOString(),
    },
  };
}

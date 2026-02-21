import type {
  UnifiedMission,
  ScanSegment,
  SplineControl,
  TransferSegment,
} from '../api/unifiedMission';
import type { ScanDefinition } from '../types/scanProject';
import { normalizePathDensityMultiplier } from '../utils/pathDensity';

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
  pathDensityMultiplier: number;
  scanProjectScans?: ScanDefinition[];
  selectedScanId?: string | null;
  nextSegmentId: (prefix: string) => string;
  resolveOrbitTargetPose: (targetId: string) =>
    | { frame: 'ECI'; position: [number, number, number]; orientation?: [number, number, number, number] }
    | undefined;
}

type PositiveSpiralAxis = '+X' | '+Y' | '+Z';

function toPositiveSpiralAxis(axis: string | null | undefined): PositiveSpiralAxis | null {
  const token = String(axis || '').trim().toUpperCase();
  if (token === 'X') return '+X';
  if (token === 'Y') return '+Y';
  if (token === 'Z') return '+Z';
  return null;
}

function resolveScanAxisOverrides(
  scanSegmentCount: number,
  scanProjectScans: ScanDefinition[] | undefined,
  selectedScanId: string | null | undefined
): Map<number, PositiveSpiralAxis> {
  const overrides = new Map<number, PositiveSpiralAxis>();
  const projectScans = Array.isArray(scanProjectScans) ? scanProjectScans : [];
  if (scanSegmentCount <= 0 || projectScans.length <= 0) {
    return overrides;
  }

  if (scanSegmentCount === projectScans.length) {
    for (let idx = 0; idx < scanSegmentCount; idx += 1) {
      const token = toPositiveSpiralAxis(projectScans[idx]?.axis);
      if (token) {
        overrides.set(idx, token);
      }
    }
    return overrides;
  }

  if (scanSegmentCount === 1) {
    const selectedScan =
      projectScans.find((scan) => scan.id === selectedScanId) ?? projectScans[0];
    const token = toPositiveSpiralAxis(selectedScan?.axis);
    if (token) {
      overrides.set(0, token);
    }
    return overrides;
  }

  const limit = Math.min(scanSegmentCount, projectScans.length);
  for (let idx = 0; idx < limit; idx += 1) {
    const token = toPositiveSpiralAxis(projectScans[idx]?.axis);
    if (token) {
      overrides.set(idx, token);
    }
  }
  return overrides;
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
  pathDensityMultiplier,
  scanProjectScans,
  selectedScanId,
  nextSegmentId,
  resolveOrbitTargetPose,
}: BuildUnifiedMissionArgs): UnifiedMission {
  const firstScanTargetId = segments.find(
    (seg): seg is ScanSegment => seg.type === 'scan' && Boolean(seg.target_id)
  )?.target_id;
  const resolvedStartTargetId = startTargetId || firstScanTargetId;
  let resolvedStartPosition = [...startPosition] as [number, number, number];
  if (startFrame === 'ECI' && resolvedStartTargetId) {
    const pose = resolveOrbitTargetPose(resolvedStartTargetId);
    if (pose) {
      resolvedStartPosition = [
        startPosition[0] - pose.position[0],
        startPosition[1] - pose.position[1],
        startPosition[2] - pose.position[2],
      ];
    }
  }
  const resolvedStartPose = {
    frame: 'LVLH' as const,
    position: resolvedStartPosition,
  };

  const hasManualPath = includeManualPath && isManualMode && previewPath.length > 0;
  const overrides: UnifiedMission['overrides'] = {};
  if (splineControls.length > 0) {
    overrides.spline_controls = splineControls;
  }
  if (hasManualPath) {
    const origin = resolvedStartTargetId
      ? resolveOrbitTargetPose(resolvedStartTargetId)?.position
      : undefined;
    overrides.manual_path = previewPath.map((p) => {
      if (!origin) return [p[0], p[1], p[2]] as [number, number, number];
      return [
        p[0] - origin[0],
        p[1] - origin[1],
        p[2] - origin[2],
      ] as [number, number, number];
    });
  }
  const density = normalizePathDensityMultiplier(pathDensityMultiplier);
  overrides.path_density_multiplier = density;
  const scanSegmentCount = segments.reduce(
    (count, seg) => (seg.type === 'scan' ? count + 1 : count),
    0
  );
  const scanAxisOverrides = resolveScanAxisOverrides(
    scanSegmentCount,
    scanProjectScans,
    selectedScanId
  );
  let scanSegmentIndex = 0;

  return {
    schema_version: 2,
    mission_id: missionId,
    name: missionName,
    epoch,
    start_pose: resolvedStartPose,
    start_target_id: resolvedStartTargetId,
    segments: segments.map((seg) => {
      const segmentId = seg.segment_id || nextSegmentId(seg.type || 'segment');
      if (seg.type === 'transfer') {
        const transferTargetId = seg.target_id || resolvedStartTargetId;
        let transferPosition = [...seg.end_pose.position] as [number, number, number];
        if (seg.end_pose.frame === 'ECI' && transferTargetId) {
          const targetPose = resolveOrbitTargetPose(transferTargetId);
          if (targetPose) {
            transferPosition = [
              seg.end_pose.position[0] - targetPose.position[0],
              seg.end_pose.position[1] - targetPose.position[1],
              seg.end_pose.position[2] - targetPose.position[2],
            ];
          }
        }
        return {
          ...seg,
          segment_id: segmentId,
          title: seg.title ?? null,
          notes: seg.notes ?? null,
          target_id: transferTargetId,
          end_pose: {
            ...seg.end_pose,
            frame: 'LVLH',
            position: transferPosition,
          },
        } as TransferSegment;
      }
      if (seg.type === 'scan') {
        const currentScanIndex = scanSegmentIndex;
        scanSegmentIndex += 1;
        const scanAxisOverride = scanAxisOverrides.get(currentScanIndex);

        if (seg.target_id) {
          const resolvedPose = resolveOrbitTargetPose(seg.target_id);
          if (resolvedPose) {
            return {
              ...seg,
              segment_id: segmentId,
              title: seg.title ?? null,
              notes: seg.notes ?? null,
              scan: {
                ...seg.scan,
                frame: 'LVLH',
                ...(scanAxisOverride ? { axis: scanAxisOverride } : {}),
              },
              target_pose: resolvedPose,
            } as ScanSegment;
          }
        }

        return {
          ...seg,
          segment_id: segmentId,
          title: seg.title ?? null,
          notes: seg.notes ?? null,
          scan: {
            ...seg.scan,
            frame: 'LVLH' as const,
            ...(scanAxisOverride ? { axis: scanAxisOverride } : {}),
          },
        } as ScanSegment;
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

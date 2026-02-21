import { describe, expect, it } from 'vitest';

import { buildUnifiedMissionPayload } from '../../src/hooks/useMissionSerializer';
import type { ScanSegment } from '../../src/api/unifiedMission';
import type { ScanDefinition } from '../../src/types/scanProject';

function makeScanSegment(overrides?: Partial<ScanSegment>): ScanSegment {
  return {
    segment_id: 'seg_scan_001',
    type: 'scan',
    target_id: 'STARLINK-1008',
    scan: {
      frame: 'LVLH',
      axis: '+Z',
      standoff: 10,
      overlap: 0.25,
      fov_deg: 60,
      pitch: null,
      revolutions: 4,
      direction: 'CW',
      sensor_axis: '+Y',
      pattern: 'spiral',
    },
    ...overrides,
  };
}

function makeProjectScan(id: string, axis: 'X' | 'Y' | 'Z'): ScanDefinition {
  return {
    id,
    name: id,
    axis,
    plane_a: [0, 0, 0],
    plane_b: [0, 0, 1],
    coarse_points_per_turn: 4,
    densify_multiplier: 8,
    speed_max: 0.2,
    key_levels: [
      {
        id: `${id}_k0`,
        t: 0,
        center_offset: [0, 0],
        radius_x: 1,
        radius_y: 1,
        rotation_deg: 0,
      },
      {
        id: `${id}_k1`,
        t: 1,
        center_offset: [0, 0],
        radius_x: 1,
        radius_y: 1,
        rotation_deg: 0,
      },
    ],
  };
}

describe('mission serializer', () => {
  it('stores path density multiplier in mission overrides', () => {
    const mission = buildUnifiedMissionPayload({
      missionId: 'mission_1',
      missionName: 'Mission 1',
      epoch: '2026-02-19T00:00:00Z',
      startFrame: 'LVLH',
      startTargetId: 'STARLINK-1008',
      startPosition: [10, 0, 0],
      segments: [],
      splineControls: [],
      isManualMode: false,
      previewPath: [],
      obstacles: [],
      draftRevision: 1,
      pathDensityMultiplier: 2.0,
      nextSegmentId: () => 'seg_1',
      resolveOrbitTargetPose: () => undefined,
    });

    expect(mission.overrides?.path_density_multiplier).toBe(2.0);
  });

  it('overrides scan axis from scan-project pair axis by order', () => {
    const mission = buildUnifiedMissionPayload({
      missionId: 'mission_axis_order',
      missionName: 'Mission Axis Order',
      epoch: '2026-02-19T00:00:00Z',
      startFrame: 'LVLH',
      startTargetId: 'STARLINK-1008',
      startPosition: [10, 0, 0],
      segments: [makeScanSegment()],
      splineControls: [],
      isManualMode: false,
      previewPath: [],
      obstacles: [],
      draftRevision: 1,
      pathDensityMultiplier: 1.0,
      scanProjectScans: [makeProjectScan('scan_a', 'Y')],
      selectedScanId: 'scan_a',
      nextSegmentId: () => 'seg_1',
      resolveOrbitTargetPose: () => undefined,
    });

    const scan = mission.segments[0] as ScanSegment;
    expect(scan.scan.axis).toBe('+Y');
  });

  it('uses selected scan axis when mission has a single scan segment', () => {
    const mission = buildUnifiedMissionPayload({
      missionId: 'mission_axis_selected',
      missionName: 'Mission Axis Selected',
      epoch: '2026-02-19T00:00:00Z',
      startFrame: 'LVLH',
      startTargetId: 'STARLINK-1008',
      startPosition: [10, 0, 0],
      segments: [makeScanSegment()],
      splineControls: [],
      isManualMode: false,
      previewPath: [],
      obstacles: [],
      draftRevision: 1,
      pathDensityMultiplier: 1.0,
      scanProjectScans: [
        makeProjectScan('scan_a', 'Z'),
        makeProjectScan('scan_b', 'X'),
      ],
      selectedScanId: 'scan_b',
      nextSegmentId: () => 'seg_1',
      resolveOrbitTargetPose: () => undefined,
    });

    const scan = mission.segments[0] as ScanSegment;
    expect(scan.scan.axis).toBe('+X');
  });
});

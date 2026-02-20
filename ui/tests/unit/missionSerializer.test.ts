import { describe, expect, it } from 'vitest';

import { buildUnifiedMissionPayload } from '../../src/hooks/useMissionSerializer';

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
});

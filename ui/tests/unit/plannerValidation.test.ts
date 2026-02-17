import { describe, expect, it } from 'vitest';

import {
  isSaveLaunchReady,
  mapIssuePathToPlannerStep,
} from '../../src/utils/plannerValidation';

describe('planner validation helpers', () => {
  it('maps issue paths to planner steps', () => {
    expect(mapIssuePathToPlannerStep('start_pose.position[0]')).toBe('target');
    expect(mapIssuePathToPlannerStep('segments[0].path_asset')).toBe('scan_definition');
    expect(mapIssuePathToPlannerStep('segments[2].scan.standoff')).toBe('scan_definition');
    expect(mapIssuePathToPlannerStep('segments[1].constraints.speed_max')).toBe('constraints');
    expect(mapIssuePathToPlannerStep('segments[1].target_id')).toBe('segments');
    expect(mapIssuePathToPlannerStep('metadata.anything')).toBe('target');
  });

  it('gates save/launch on validation pass', () => {
    expect(isSaveLaunchReady(null)).toBe(false);
    expect(
      isSaveLaunchReady({
        valid: false,
        issues: [],
        summary: { errors: 1, warnings: 0, info: 0 },
      })
    ).toBe(false);
    expect(
      isSaveLaunchReady({
        valid: true,
        issues: [],
        summary: { errors: 0, warnings: 0, info: 0 },
      })
    ).toBe(true);
  });
});

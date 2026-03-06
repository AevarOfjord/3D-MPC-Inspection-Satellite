import { describe, expect, it } from 'vitest';

import {
  isSaveLaunchReady,
  mapIssuePathToAuthoringPhase,
} from '../../src/utils/authoringValidation';

describe('authoring validation helpers', () => {
  it('maps issue paths to Studio authoring phases', () => {
    expect(mapIssuePathToAuthoringPhase('start_pose.position[0]')).toBe('target');
    expect(mapIssuePathToAuthoringPhase('segments[0].path_asset')).toBe(
      'scan_definition'
    );
    expect(mapIssuePathToAuthoringPhase('segments[2].scan.standoff')).toBe(
      'scan_definition'
    );
    expect(mapIssuePathToAuthoringPhase('segments[1].constraints.speed_max')).toBe(
      'constraints'
    );
    expect(mapIssuePathToAuthoringPhase('segments[1].target_id')).toBe('segments');
    expect(mapIssuePathToAuthoringPhase('metadata.anything')).toBe('target');
  });

  it('gates save readiness on validation pass', () => {
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

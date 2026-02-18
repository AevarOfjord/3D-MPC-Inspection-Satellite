import { describe, expect, it } from 'vitest';

import { buildPlannerStepStatusMap, canAccessPlannerStep } from '../../src/utils/plannerCompletion';

describe('planner completion status', () => {
  it('locks downstream steps in guided flow when prerequisites are missing', () => {
    const statuses = buildPlannerStepStatusMap({
      startFrame: 'LVLH',
      startTargetId: undefined,
      segments: [],
      validationReport: null,
    });

    expect(statuses.target).toBe('ready');
    expect(statuses.segments).toBe('locked');
    expect(statuses.scan_definition).toBe('locked');
    expect(statuses.save_launch).toBe('locked');
    expect(canAccessPlannerStep('scan_definition', statuses, 'guided')).toBe(false);
    expect(canAccessPlannerStep('scan_definition', statuses, 'advanced')).toBe(true);
  });

  it('marks save_launch ready when validation report is valid', () => {
    const statuses = buildPlannerStepStatusMap({
      startFrame: 'ECI',
      startTargetId: undefined,
      segments: [
        {
          segment_id: 'scan-1',
          type: 'scan',
          target_id: 'ISS',
          path_asset: 'asset-1',
          scan: {
            frame: 'ECI',
            axis: '+X',
            standoff: 10,
            overlap: 0.2,
            fov_deg: 60,
            pitch: null,
            revolutions: 2,
            direction: 'CW',
            sensor_axis: '+Y',
            pattern: 'spiral',
          },
          constraints: {
            speed_max: 0.7,
            accel_max: 0.2,
            angular_rate_max: 0.15,
          },
        },
      ],
      validationReport: {
        valid: true,
        issues: [],
        summary: { errors: 0, warnings: 0, info: 0 },
      },
    });

    expect(statuses.target).toBe('complete');
    expect(statuses.segments).toBe('complete');
    expect(statuses.scan_definition).toBe('complete');
    expect(statuses.constraints).toBe('complete');
    expect(statuses.validate).toBe('complete');
    expect(statuses.save_launch).toBe('ready');
  });
});

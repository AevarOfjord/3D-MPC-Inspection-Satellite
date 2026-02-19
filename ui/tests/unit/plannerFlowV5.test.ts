import { describe, expect, it } from 'vitest';

import {
  buildPlannerFlowStepStatusMap,
  canAccessFlowStep,
  mapFlowStepToInternalStep,
  mapInternalStepToFlowStep,
} from '../../src/utils/plannerFlowV5';

describe('planner flow v5', () => {
  it('keeps transfer available while blocking downstream guided steps', () => {
    const statuses = buildPlannerFlowStepStatusMap({
      startFrame: 'ECI',
      startTargetId: undefined,
      segments: [],
      validationReport: null,
      scanPairCount: 1,
      scanEndpointCount: 2,
      transferTargetSelected: false,
      obstaclesCount: 0,
      previewPathPoints: 0,
      isManualMode: false,
    });

    expect(statuses.path_maker).toBe('complete');
    expect(statuses.transfer).toBe('ready');
    expect(statuses.obstacles).toBe('locked');
    expect(canAccessFlowStep('transfer', statuses)).toBe(true);
  });

  it('maps v5 flow steps to internal planner steps and back', () => {
    expect(mapFlowStepToInternalStep('path_maker')).toBe('scan_definition');
    expect(mapFlowStepToInternalStep('transfer')).toBe('target');
    expect(mapFlowStepToInternalStep('obstacles')).toBe('constraints');
    expect(mapFlowStepToInternalStep('path_edit')).toBe('segments');
    expect(mapFlowStepToInternalStep('mission_saver')).toBe('save_launch');

    expect(mapInternalStepToFlowStep('scan_definition')).toBe('path_maker');
    expect(mapInternalStepToFlowStep('target')).toBe('transfer');
    expect(mapInternalStepToFlowStep('constraints')).toBe('obstacles');
    expect(mapInternalStepToFlowStep('segments')).toBe('path_edit');
    expect(mapInternalStepToFlowStep('validate')).toBe('mission_saver');
    expect(mapInternalStepToFlowStep('save_launch')).toBe('mission_saver');
  });
});

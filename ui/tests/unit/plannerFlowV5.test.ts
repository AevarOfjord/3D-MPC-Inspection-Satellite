import { describe, expect, it } from 'vitest';

import {
  buildPlannerFlowStepStatusMap,
  canAccessFlowStep,
  mapFlowStepToInternalStep,
  mapInternalStepToFlowStep,
} from '../../src/utils/plannerFlowV5';

describe('planner flow v5', () => {
  it('keeps start transfer available while blocking downstream guided steps', () => {
    const statuses = buildPlannerFlowStepStatusMap({
      startFrame: 'ECI',
      startTargetId: undefined,
      segments: [],
      validationReport: null,
      obstaclesCount: 0,
      previewPathPoints: 0,
      isManualMode: false,
    });

    expect(statuses.path_library).toBe('ready');
    expect(statuses.start_transfer).toBe('ready');
    expect(statuses.obstacles).toBe('locked');
    expect(canAccessFlowStep('start_transfer', statuses, 'guided')).toBe(true);
    expect(canAccessFlowStep('start_transfer', statuses, 'advanced')).toBe(true);
  });

  it('maps v5 flow steps to internal planner steps and back', () => {
    expect(mapFlowStepToInternalStep('path_library')).toBe('scan_definition');
    expect(mapFlowStepToInternalStep('start_transfer')).toBe('target');
    expect(mapFlowStepToInternalStep('obstacles')).toBe('constraints');
    expect(mapFlowStepToInternalStep('path_edit')).toBe('segments');
    expect(mapFlowStepToInternalStep('save')).toBe('save_launch');

    expect(mapInternalStepToFlowStep('scan_definition')).toBe('path_library');
    expect(mapInternalStepToFlowStep('target')).toBe('start_transfer');
    expect(mapInternalStepToFlowStep('constraints')).toBe('obstacles');
    expect(mapInternalStepToFlowStep('segments')).toBe('path_edit');
    expect(mapInternalStepToFlowStep('validate')).toBe('save');
    expect(mapInternalStepToFlowStep('save_launch')).toBe('save');
  });
});

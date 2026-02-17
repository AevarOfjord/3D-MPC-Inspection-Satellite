import { describe, expect, it } from 'vitest';

import {
  createDefaultScanProject,
  validateScanProject,
} from '../../src/utils/scanProjectValidation';

describe('scan project validation', () => {
  it('creates schema v2 defaults', () => {
    const project = createDefaultScanProject('/tmp/model.obj');
    expect(project.schema_version).toBe(2);
    expect(project.scans.length).toBeGreaterThan(0);
    expect(project.scans[0].key_levels.length).toBeGreaterThanOrEqual(2);
  });

  it('reports missing object path', () => {
    const project = createDefaultScanProject('');
    const error = validateScanProject(project);
    expect(error).toBe('Select an OBJ model first.');
  });
});

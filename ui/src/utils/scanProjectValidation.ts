import type {
  BodyAxis,
  ScanDefinition,
  ScanKeyLevel,
  ScanProject,
} from '../types/scanProject';
import { normalizePathDensityMultiplier } from './pathDensity';

let idCounter = 0;

export function makeId(prefix: string): string {
  idCounter += 1;
  return `${prefix}_${idCounter.toString().padStart(6, '0')}`;
}

function createDefaultKeyLevels(): ScanKeyLevel[] {
  return [
    {
      id: makeId('kl'),
      t: 0,
      center_offset: [0, 0],
      radius_x: 1.0,
      radius_y: 1.0,
      rotation_deg: 0,
    },
    {
      id: makeId('kl'),
      t: 1,
      center_offset: [0, 0],
      radius_x: 1.0,
      radius_y: 1.0,
      rotation_deg: 0,
    },
  ];
}

export function createDefaultScan(
  index = 1,
  axis: BodyAxis = 'Z',
  center: [number, number, number] = [0, 0, 0]
): ScanDefinition {
  const planeA: [number, number, number] = [center[0], center[1], center[2] - 0.5];
  const planeB: [number, number, number] = [center[0], center[1], center[2] + 0.5];

  if (axis === 'X') {
    planeA[0] = center[0] - 0.5;
    planeA[2] = center[2];
    planeB[0] = center[0] + 0.5;
    planeB[2] = center[2];
  }
  if (axis === 'Y') {
    planeA[1] = center[1] - 0.5;
    planeA[2] = center[2];
    planeB[1] = center[1] + 0.5;
    planeB[2] = center[2];
  }

  return {
    id: makeId('scan'),
    name: `Scan ${index}`,
    axis,
    plane_a: planeA,
    plane_b: planeB,
    level_spacing_m: 0.1,
    coarse_points_per_turn: 4,
    densify_multiplier: 8,
    speed_max: 0.2,
    key_levels: createDefaultKeyLevels(),
  };
}

export function createDefaultScanProject(objPath = ''): ScanProject {
  return {
    schema_version: 2,
    id: null,
    name: 'Scan Project 1',
    obj_path: objPath,
    path_density_multiplier: 1.0,
    scans: [createDefaultScan(1, 'Z')],
    connectors: [],
  };
}

export function validateScanProject(project: ScanProject): string | null {
  if (!project.name.trim()) return 'Project name is required.';
  if (!project.obj_path.trim()) return 'Select an OBJ model first.';
  if (!project.scans.length) return 'At least one scan is required.';
  if (!Number.isFinite(project.path_density_multiplier) || project.path_density_multiplier <= 0) {
    return 'Path density multiplier must be > 0.';
  }
  const normalizedDensity = normalizePathDensityMultiplier(project.path_density_multiplier);
  if (!Number.isFinite(normalizedDensity) || normalizedDensity <= 0) {
    return 'Path density multiplier must be > 0.';
  }

  const scanIds = new Set<string>();
  for (const scan of project.scans) {
    if (!scan.id) return 'Each scan needs an id.';
    if (scanIds.has(scan.id)) return `Duplicate scan id: ${scan.id}`;
    scanIds.add(scan.id);
    if (scan.key_levels.length < 2) {
      return `${scan.name} must have at least 2 key levels.`;
    }
    if ((scan.level_spacing_m ?? 0.1) <= 0) {
      return `${scan.name} must have level spacing > 0.`;
    }
    for (const level of scan.key_levels) {
      if (level.t < 0 || level.t > 1) {
        return `${scan.name} has a key level with t outside [0, 1].`;
      }
    }
  }

  for (const connector of project.connectors) {
    if (!scanIds.has(connector.from_scan_id) || !scanIds.has(connector.to_scan_id)) {
      return `Connector ${connector.id} references an unknown scan.`;
    }
    if (connector.from_scan_id === connector.to_scan_id) {
      return `Connector ${connector.id} must connect different scans.`;
    }
  }

  return null;
}

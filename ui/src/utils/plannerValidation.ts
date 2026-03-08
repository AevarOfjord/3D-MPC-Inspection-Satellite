import type { ValidationReportV2 } from '../api/unifiedMissionApi';

export type PlannerStep =
  | 'target'
  | 'segments'
  | 'scan_definition'
  | 'constraints'
  | 'validate'
  | 'save_launch';

export function mapIssuePathToPlannerStep(path: string): PlannerStep {
  if (
    path.includes('start_pose') ||
    path.includes('start_target_id') ||
    path.includes('epoch')
  ) {
    return 'target';
  }
  if (
    path.includes('.path_asset') ||
    path.includes('.scan') ||
    path.includes('scan_project') ||
    path.includes('manual_path')
  ) {
    return 'scan_definition';
  }
  if (
    path.includes('.constraints') ||
    path.includes('speed_max') ||
    path.includes('accel_max') ||
    path.includes('angular_rate_max')
  ) {
    return 'constraints';
  }
  if (path.includes('segments[')) {
    return 'segments';
  }
  return 'target';
}

export function isSaveLaunchReady(report: ValidationReportV2 | null): boolean {
  return Boolean(report?.valid);
}

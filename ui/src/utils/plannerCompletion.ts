import type { ValidationReportV2 } from '../api/unifiedMissionApi';
import type { MissionSegment, ScanSegment } from '../api/unifiedMission';
import { mapIssuePathToPlannerStep, type PlannerStep } from './plannerValidation';
import type { PlannerStepStatusMap } from '../types/plannerUx';

interface BuildPlannerStatusArgs {
  startFrame: 'ECI' | 'LVLH';
  startTargetId?: string;
  segments: MissionSegment[];
  validationReport: ValidationReportV2 | null;
}

function hasTargetReady(startFrame: 'ECI' | 'LVLH', startTargetId?: string): boolean {
  void startFrame;
  return Boolean(startTargetId);
}

function hasScanAssets(segments: MissionSegment[]): boolean {
  const scanSegments = segments.filter((segment) => segment.type === 'scan') as ScanSegment[];
  if (scanSegments.length === 0) return false;
  return scanSegments.every((segment) => Boolean(segment.target_id));
}

function hasConstraintsConfigured(segments: MissionSegment[]): boolean {
  if (segments.length === 0) return false;
  return segments.every((segment) => {
    const constraints = segment.constraints;
    return Boolean(
      constraints &&
        typeof constraints.speed_max === 'number' &&
        typeof constraints.accel_max === 'number' &&
        typeof constraints.angular_rate_max === 'number'
    );
  });
}

export function buildPlannerStepStatusMap({
  startFrame,
  startTargetId,
  segments,
  validationReport,
}: BuildPlannerStatusArgs): PlannerStepStatusMap {
  const targetComplete = hasTargetReady(startFrame, startTargetId);
  const segmentsComplete = segments.length > 0;
  const scanComplete = hasScanAssets(segments);
  const constraintsComplete = hasConstraintsConfigured(segments);
  const validationComplete = Boolean(validationReport?.valid);

  const statuses: PlannerStepStatusMap = {
    target: targetComplete ? 'complete' : 'ready',
    segments: targetComplete ? (segmentsComplete ? 'complete' : 'ready') : 'locked',
    scan_definition: segmentsComplete ? (scanComplete ? 'complete' : 'ready') : 'locked',
    constraints:
      segmentsComplete && scanComplete
        ? constraintsComplete
          ? 'complete'
          : 'ready'
        : 'locked',
    validate: segmentsComplete ? (validationComplete ? 'complete' : 'ready') : 'locked',
    save_launch: validationComplete ? 'ready' : 'locked',
  };

  for (const issue of validationReport?.issues ?? []) {
    if (issue.severity !== 'error') continue;
    const step = mapIssuePathToPlannerStep(issue.path);
    if (statuses[step] === 'locked') continue;
    statuses[step] = 'error';
  }

  return statuses;
}

export function canAccessPlannerStep(
  step: PlannerStep,
  statuses: PlannerStepStatusMap,
  mode: 'guided'
): boolean {
  void mode;
  return statuses[step] !== 'locked';
}

export function nextPlannerStep(current: PlannerStep): PlannerStep {
  if (current === 'target') return 'segments';
  if (current === 'segments') return 'scan_definition';
  if (current === 'scan_definition') return 'constraints';
  if (current === 'constraints') return 'validate';
  if (current === 'validate') return 'save_launch';
  return 'save_launch';
}

export function previousPlannerStep(current: PlannerStep): PlannerStep {
  if (current === 'save_launch') return 'validate';
  if (current === 'validate') return 'constraints';
  if (current === 'constraints') return 'scan_definition';
  if (current === 'scan_definition') return 'segments';
  if (current === 'segments') return 'target';
  return 'target';
}

export function getStepIssueCounts(report: ValidationReportV2 | null): Record<PlannerStep, number> {
  const counts: Record<PlannerStep, number> = {
    target: 0,
    segments: 0,
    scan_definition: 0,
    constraints: 0,
    validate: 0,
    save_launch: 0,
  };
  for (const issue of report?.issues ?? []) {
    const step = mapIssuePathToPlannerStep(issue.path);
    counts[step] += 1;
  }
  return counts;
}

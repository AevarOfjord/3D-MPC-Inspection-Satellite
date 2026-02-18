import type { MissionSegment, ScanSegment } from '../api/unifiedMission';
import type { ValidationReportV2 } from '../api/unifiedMissionApi';
import type {
  PlannerFlowStepStatusMap,
  PlannerFlowStepV5,
  PlannerUxMode,
} from '../types/plannerUx';
import { mapIssuePathToPlannerStep, type PlannerStep } from './plannerValidation';

interface BuildPlannerFlowStatusArgs {
  startFrame: 'ECI' | 'LVLH';
  startTargetId?: string;
  segments: MissionSegment[];
  validationReport: ValidationReportV2 | null;
  obstaclesCount: number;
  previewPathPoints: number;
  isManualMode: boolean;
}

function hasTargetReady(startFrame: 'ECI' | 'LVLH', startTargetId?: string): boolean {
  if (startFrame === 'ECI') return true;
  return Boolean(startTargetId);
}

function hasPathLibraryReady(segments: MissionSegment[]): boolean {
  const scans = segments.filter((segment) => segment.type === 'scan') as ScanSegment[];
  if (scans.length === 0) return false;
  return scans.some((scan) => Boolean(scan.path_asset));
}

function hasTransferSegment(segments: MissionSegment[]): boolean {
  return segments.some((segment) => segment.type === 'transfer');
}

export function mapFlowStepToInternalStep(step: PlannerFlowStepV5): PlannerStep {
  if (step === 'path_library') return 'scan_definition';
  if (step === 'start_transfer') return 'target';
  if (step === 'obstacles') return 'constraints';
  if (step === 'path_edit') return 'segments';
  return 'save_launch';
}

export function mapInternalStepToFlowStep(step: PlannerStep): PlannerFlowStepV5 {
  if (step === 'scan_definition') return 'path_library';
  if (step === 'target') return 'start_transfer';
  if (step === 'constraints') return 'obstacles';
  if (step === 'segments') return 'path_edit';
  return 'save';
}

export function buildPlannerFlowStepStatusMap({
  startFrame,
  startTargetId,
  segments,
  validationReport,
  obstaclesCount,
  previewPathPoints,
  isManualMode,
}: BuildPlannerFlowStatusArgs): PlannerFlowStepStatusMap {
  const pathLibraryComplete = hasPathLibraryReady(segments);
  const startTransferComplete =
    hasTargetReady(startFrame, startTargetId) &&
    hasTransferSegment(segments) &&
    previewPathPoints > 1;
  const obstaclesComplete = obstaclesCount > 0;
  const pathEditComplete = isManualMode && previewPathPoints > 2;
  const validationReady = Boolean(validationReport?.valid);

  const statuses: PlannerFlowStepStatusMap = {
    path_library: pathLibraryComplete ? 'complete' : 'ready',
    start_transfer: startTransferComplete ? 'complete' : 'ready',
    obstacles: startTransferComplete
      ? obstaclesComplete
        ? 'complete'
        : 'ready'
      : 'locked',
    path_edit: startTransferComplete ? (pathEditComplete ? 'complete' : 'ready') : 'locked',
    save: startTransferComplete ? (validationReady ? 'ready' : 'ready') : 'locked',
  };

  for (const issue of validationReport?.issues ?? []) {
    if (issue.severity !== 'error') continue;
    const step = mapInternalStepToFlowStep(mapIssuePathToPlannerStep(issue.path));
    if (statuses[step] !== 'locked') {
      statuses[step] = 'error';
    }
  }

  return statuses;
}

export function canAccessFlowStep(
  step: PlannerFlowStepV5,
  statuses: PlannerFlowStepStatusMap,
  mode: PlannerUxMode
): boolean {
  if (mode === 'advanced') return true;
  return statuses[step] !== 'locked';
}

export function nextFlowStep(step: PlannerFlowStepV5): PlannerFlowStepV5 {
  if (step === 'path_library') return 'start_transfer';
  if (step === 'start_transfer') return 'obstacles';
  if (step === 'obstacles') return 'path_edit';
  if (step === 'path_edit') return 'save';
  return 'save';
}

export function previousFlowStep(step: PlannerFlowStepV5): PlannerFlowStepV5 {
  if (step === 'save') return 'path_edit';
  if (step === 'path_edit') return 'obstacles';
  if (step === 'obstacles') return 'start_transfer';
  if (step === 'start_transfer') return 'path_library';
  return 'path_library';
}

export function getFlowStepIssueCounts(
  report: ValidationReportV2 | null
): Record<PlannerFlowStepV5, number> {
  const counts: Record<PlannerFlowStepV5, number> = {
    path_library: 0,
    start_transfer: 0,
    obstacles: 0,
    path_edit: 0,
    save: 0,
  };
  for (const issue of report?.issues ?? []) {
    const flowStep = mapInternalStepToFlowStep(mapIssuePathToPlannerStep(issue.path));
    counts[flowStep] += 1;
  }
  return counts;
}

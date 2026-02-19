import type { MissionSegment } from '../api/unifiedMission';
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
  scanPairCount: number;
  scanEndpointCount: number;
  transferTargetSelected: boolean;
  obstaclesCount: number;
  previewPathPoints: number;
  isManualMode: boolean;
}

function hasTargetReady(startFrame: 'ECI' | 'LVLH', startTargetId?: string): boolean {
  if (startFrame === 'ECI') return true;
  return Boolean(startTargetId);
}

function hasPathMakerReady(scanPairCount: number, scanEndpointCount: number): boolean {
  return scanPairCount > 0 && scanEndpointCount > 0;
}

function hasTransferSegment(segments: MissionSegment[]): boolean {
  return segments.some((segment) => segment.type === 'transfer');
}

export function mapFlowStepToInternalStep(step: PlannerFlowStepV5): PlannerStep {
  if (step === 'path_maker') return 'scan_definition';
  if (step === 'transfer') return 'target';
  if (step === 'obstacles') return 'constraints';
  if (step === 'path_edit') return 'segments';
  return 'save_launch';
}

export function mapInternalStepToFlowStep(step: PlannerStep): PlannerFlowStepV5 {
  if (step === 'scan_definition') return 'path_maker';
  if (step === 'target') return 'transfer';
  if (step === 'constraints') return 'obstacles';
  if (step === 'segments') return 'path_edit';
  return 'mission_saver';
}

export function buildPlannerFlowStepStatusMap({
  startFrame,
  startTargetId,
  segments,
  validationReport,
  scanPairCount,
  scanEndpointCount,
  transferTargetSelected,
  obstaclesCount,
  previewPathPoints,
  isManualMode,
}: BuildPlannerFlowStatusArgs): PlannerFlowStepStatusMap {
  const pathMakerComplete = hasPathMakerReady(scanPairCount, scanEndpointCount);
  const startTransferComplete =
    hasTargetReady(startFrame, startTargetId) &&
    hasTransferSegment(segments) &&
    transferTargetSelected &&
    previewPathPoints > 1;
  const obstaclesComplete = obstaclesCount > 0;
  const pathEditComplete = isManualMode && previewPathPoints > 2;

  const statuses: PlannerFlowStepStatusMap = {
    path_maker: pathMakerComplete ? 'complete' : 'ready',
    transfer: pathMakerComplete
      ? startTransferComplete
        ? 'complete'
        : 'ready'
      : 'locked',
    obstacles: startTransferComplete
      ? obstaclesComplete
        ? 'complete'
        : 'ready'
      : 'locked',
    path_edit: startTransferComplete ? (pathEditComplete ? 'complete' : 'ready') : 'locked',
    mission_saver: startTransferComplete ? 'ready' : 'locked',
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
  if (step === 'path_maker') return 'transfer';
  if (step === 'transfer') return 'obstacles';
  if (step === 'obstacles') return 'path_edit';
  if (step === 'path_edit') return 'mission_saver';
  return 'mission_saver';
}

export function previousFlowStep(step: PlannerFlowStepV5): PlannerFlowStepV5 {
  if (step === 'mission_saver') return 'path_edit';
  if (step === 'path_edit') return 'obstacles';
  if (step === 'obstacles') return 'transfer';
  if (step === 'transfer') return 'path_maker';
  return 'path_maker';
}

export function getFlowStepIssueCounts(
  report: ValidationReportV2 | null
): Record<PlannerFlowStepV5, number> {
  const counts: Record<PlannerFlowStepV5, number> = {
    path_maker: 0,
    transfer: 0,
    obstacles: 0,
    path_edit: 0,
    mission_saver: 0,
  };
  for (const issue of report?.issues ?? []) {
    const flowStep = mapInternalStepToFlowStep(mapIssuePathToPlannerStep(issue.path));
    counts[flowStep] += 1;
  }
  return counts;
}

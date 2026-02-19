import type { PlannerStep } from '../utils/plannerValidation';

export type PlannerStepStatus = 'locked' | 'ready' | 'complete' | 'error';
export type PlannerFlowStepStatus = PlannerStepStatus;

export type PlannerFlowStepV42 =
  | 'path_maker'
  | 'transfer'
  | 'obstacles'
  | 'path_edit'
  | 'mission_saver';
export type PlannerFlowStepV5 = PlannerFlowStepV42;

export interface SpiralEndpointRef {
  scanId: string;
  endpoint: 'start' | 'end';
}

export type TransferTargetRef = SpiralEndpointRef | null;
export type PathPairId = string;

export type CoachmarkId =
  | 'step_rail'
  | 'context_panel'
  | 'path_edit'
  | 'save'
  | 'validation'
  | 'save_launch';

export interface PlannerStepStatusMap {
  target: PlannerStepStatus;
  segments: PlannerStepStatus;
  scan_definition: PlannerStepStatus;
  constraints: PlannerStepStatus;
  validate: PlannerStepStatus;
  save_launch: PlannerStepStatus;
}

export interface PlannerFlowStepStatusMap {
  path_maker: PlannerFlowStepStatus;
  transfer: PlannerFlowStepStatus;
  obstacles: PlannerFlowStepStatus;
  path_edit: PlannerFlowStepStatus;
  mission_saver: PlannerFlowStepStatus;
}

export const PLANNER_FLOW_STATE_STORAGE_KEY = 'mission_control_planner_flow_v5_state_v1';
export const PLANNER_COACHMARKS_STORAGE_KEY = 'mission_control_coachmarks_v1';

export const PLANNER_STEP_ORDER: PlannerStep[] = [
  'target',
  'segments',
  'scan_definition',
  'constraints',
  'validate',
  'save_launch',
];

export const PLANNER_FLOW_STEP_ORDER_V5: PlannerFlowStepV5[] = [
  'path_maker',
  'transfer',
  'obstacles',
  'path_edit',
  'mission_saver',
];

export interface PlannerCoachmarkState {
  introSeen: boolean;
  neverShowAgain: boolean;
  dismissedIds: CoachmarkId[];
}

export const DEFAULT_COACHMARK_STATE: PlannerCoachmarkState = {
  introSeen: false,
  neverShowAgain: false,
  dismissedIds: [],
};

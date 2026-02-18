import type { PlannerStep } from '../utils/plannerValidation';

export type PlannerUxMode = 'guided' | 'advanced';

export type PlannerStepStatus = 'locked' | 'ready' | 'complete' | 'error';
export type PlannerFlowStepStatus = PlannerStepStatus;

export type PlannerFlowStepV5 =
  | 'path_library'
  | 'start_transfer'
  | 'obstacles'
  | 'path_edit'
  | 'save';

export type CoachmarkId =
  | 'step_rail'
  | 'templates'
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
  path_library: PlannerFlowStepStatus;
  start_transfer: PlannerFlowStepStatus;
  obstacles: PlannerFlowStepStatus;
  path_edit: PlannerFlowStepStatus;
  save: PlannerFlowStepStatus;
}

export const PLANNER_UX_MODE_STORAGE_KEY = 'mission_control_planner_ux_mode_v1';
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
  'path_library',
  'start_transfer',
  'obstacles',
  'path_edit',
  'save',
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

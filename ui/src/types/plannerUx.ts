import type { PlannerStep } from '../utils/plannerValidation';

export type PlannerUxMode = 'guided' | 'advanced';

export type PlannerStepStatus = 'locked' | 'ready' | 'complete' | 'error';

export type CoachmarkId =
  | 'step_rail'
  | 'templates'
  | 'context_panel'
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

export const PLANNER_UX_MODE_STORAGE_KEY = 'mission_control_planner_ux_mode_v1';
export const PLANNER_COACHMARKS_STORAGE_KEY = 'mission_control_coachmarks_v1';

export const PLANNER_STEP_ORDER: PlannerStep[] = [
  'target',
  'segments',
  'scan_definition',
  'constraints',
  'validate',
  'save_launch',
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

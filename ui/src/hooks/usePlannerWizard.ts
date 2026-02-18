import { useEffect, useMemo, useState } from 'react';

import type { ValidationReportV2 } from '../api/unifiedMissionApi';
import type { MissionSegment } from '../api/unifiedMission';
import {
  canAccessPlannerStep,
  buildPlannerStepStatusMap,
  nextPlannerStep,
  previousPlannerStep,
  getStepIssueCounts,
} from '../utils/plannerCompletion';
import type { PlannerStep } from '../utils/plannerValidation';
import {
  PLANNER_UX_MODE_STORAGE_KEY,
  type PlannerStepStatusMap,
  type PlannerUxMode,
} from '../types/plannerUx';

interface UsePlannerWizardArgs {
  authoringStep: PlannerStep;
  setAuthoringStep: (step: PlannerStep) => void;
  startFrame: 'ECI' | 'LVLH';
  startTargetId?: string;
  segments: MissionSegment[];
  validationReport: ValidationReportV2 | null;
}

function parseStoredUxMode(raw: string | null): PlannerUxMode {
  if (raw === 'advanced') return 'advanced';
  return 'guided';
}

export function usePlannerWizard({
  authoringStep,
  setAuthoringStep,
  startFrame,
  startTargetId,
  segments,
  validationReport,
}: UsePlannerWizardArgs) {
  const [uxMode, setUxModeState] = useState<PlannerUxMode>(() => {
    try {
      return parseStoredUxMode(window.localStorage.getItem(PLANNER_UX_MODE_STORAGE_KEY));
    } catch {
      return 'guided';
    }
  });

  useEffect(() => {
    try {
      window.localStorage.setItem(PLANNER_UX_MODE_STORAGE_KEY, uxMode);
    } catch {
      // ignore storage write errors
    }
  }, [uxMode]);

  const stepStatuses = useMemo<PlannerStepStatusMap>(
    () =>
      buildPlannerStepStatusMap({
        startFrame,
        startTargetId,
        segments,
        validationReport,
      }),
    [startFrame, startTargetId, segments, validationReport]
  );

  const stepIssueCounts = useMemo(
    () => getStepIssueCounts(validationReport),
    [validationReport]
  );

  useEffect(() => {
    if (canAccessPlannerStep(authoringStep, stepStatuses, uxMode)) return;
    setAuthoringStep('target');
  }, [authoringStep, stepStatuses, uxMode, setAuthoringStep]);

  const goToStep = (step: PlannerStep) => {
    if (!canAccessPlannerStep(step, stepStatuses, uxMode)) return;
    setAuthoringStep(step);
  };

  const goNext = () => {
    const step = nextPlannerStep(authoringStep);
    goToStep(step);
  };

  const goPrevious = () => {
    const step = previousPlannerStep(authoringStep);
    goToStep(step);
  };

  const completedCount = useMemo(
    () =>
      Object.values(stepStatuses).filter(
        (status) => status === 'complete' || status === 'ready'
      ).length,
    [stepStatuses]
  );

  const setUxMode = (mode: PlannerUxMode) => {
    setUxModeState(mode);
  };

  return {
    state: {
      uxMode,
      stepStatuses,
      stepIssueCounts,
      completedCount,
    },
    actions: {
      setUxMode,
      goToStep,
      goNext,
      goPrevious,
    },
  };
}

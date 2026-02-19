import { useEffect, useMemo, useRef, useState } from 'react';

import type { ValidationReportV2 } from '../api/unifiedMissionApi';
import type { MissionSegment } from '../api/unifiedMission';
import {
  buildPlannerFlowStepStatusMap,
  canAccessFlowStep,
  getFlowStepIssueCounts,
  mapFlowStepToInternalStep,
  mapInternalStepToFlowStep,
  nextFlowStep,
  previousFlowStep,
} from '../utils/plannerFlowV5';
import type { PlannerStep } from '../utils/plannerValidation';
import {
  PLANNER_FLOW_STATE_STORAGE_KEY,
  PLANNER_UX_MODE_STORAGE_KEY,
  type PlannerFlowStepStatusMap,
  type PlannerFlowStepV5,
  type PlannerUxMode,
} from '../types/plannerUx';

interface UsePlannerWizardArgs {
  authoringStep: PlannerStep;
  setAuthoringStep: (step: PlannerStep) => void;
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

function parseStoredUxMode(raw: string | null): PlannerUxMode {
  if (raw === 'advanced') return 'advanced';
  return 'guided';
}

function parseStoredFlowStep(raw: string | null): PlannerFlowStepV5 | null {
  if (raw === 'path_maker' || raw === 'transfer' || raw === 'obstacles' || raw === 'path_edit' || raw === 'mission_saver') {
    return raw;
  }
  if (raw === 'path_library') return 'path_maker';
  if (raw === 'start_transfer') return 'transfer';
  if (raw === 'save') return 'mission_saver';
  return null;
}

export function usePlannerWizard({
  authoringStep,
  setAuthoringStep,
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
}: UsePlannerWizardArgs) {
  const syncingFromFlowRef = useRef(false);
  const syncingFromAuthoringRef = useRef(false);
  const [uxMode, setUxModeState] = useState<PlannerUxMode>(() => {
    try {
      return parseStoredUxMode(window.localStorage.getItem(PLANNER_UX_MODE_STORAGE_KEY));
    } catch {
      return 'guided';
    }
  });
  const [flowStep, setFlowStepState] = useState<PlannerFlowStepV5>(() => {
    try {
      const stored = parseStoredFlowStep(
        window.localStorage.getItem(PLANNER_FLOW_STATE_STORAGE_KEY)
      );
      if (stored) return stored;
    } catch {
      // ignore storage read errors
    }
    return mapInternalStepToFlowStep(authoringStep);
  });

  useEffect(() => {
    try {
      window.localStorage.setItem(PLANNER_UX_MODE_STORAGE_KEY, uxMode);
    } catch {
      // ignore storage write errors
    }
  }, [uxMode]);
  useEffect(() => {
    try {
      window.localStorage.setItem(PLANNER_FLOW_STATE_STORAGE_KEY, flowStep);
    } catch {
      // ignore storage write errors
    }
  }, [flowStep]);

  const stepStatuses = useMemo<PlannerFlowStepStatusMap>(
    () =>
      buildPlannerFlowStepStatusMap({
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
      }),
    [
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
    ]
  );

  const stepIssueCounts = useMemo(
    () => getFlowStepIssueCounts(validationReport),
    [validationReport]
  );

  useEffect(() => {
    if (syncingFromFlowRef.current) {
      syncingFromFlowRef.current = false;
      return;
    }
    const mappedFlowStep = mapInternalStepToFlowStep(authoringStep);
    setFlowStepState((prev) => {
      if (prev === mappedFlowStep) return prev;
      syncingFromAuthoringRef.current = true;
      return mappedFlowStep;
    });
  }, [authoringStep]);

  useEffect(() => {
    if (canAccessFlowStep(flowStep, stepStatuses, uxMode)) return;
    setFlowStepState('path_maker');
  }, [flowStep, stepStatuses, uxMode]);

  useEffect(() => {
    if (syncingFromAuthoringRef.current) {
      syncingFromAuthoringRef.current = false;
      return;
    }
    const internalStep = mapFlowStepToInternalStep(flowStep);
    // Avoid ping-pong updates for equivalent mappings (e.g. validate/save_launch -> mission_saver).
    if (mapInternalStepToFlowStep(authoringStep) === flowStep) return;
    if (authoringStep === internalStep) return;
    syncingFromFlowRef.current = true;
    setAuthoringStep(internalStep);
  }, [flowStep, authoringStep, setAuthoringStep]);

  const goToStep = (step: PlannerFlowStepV5) => {
    if (!canAccessFlowStep(step, stepStatuses, uxMode)) return;
    setFlowStepState(step);
  };

  const goNext = () => {
    const step = nextFlowStep(flowStep);
    goToStep(step);
  };

  const goPrevious = () => {
    const step = previousFlowStep(flowStep);
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
      flowStep,
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

import { useState } from 'react';

import { unifiedMissionApi, type ValidationReportV2 } from '../api/unifiedMissionApi';
import type { UnifiedMission } from '../api/unifiedMission';

import type { MissionAuthoringStep } from './useMissionState';

interface UseMissionValidationArgs {
  buildMission: () => UnifiedMission;
  onFocusSegment: (index: number) => void;
  setAuthoringStep: (step: MissionAuthoringStep) => void;
}

export function useMissionValidation({
  buildMission,
  onFocusSegment,
  setAuthoringStep,
}: UseMissionValidationArgs) {
  const [validationReport, setValidationReport] = useState<ValidationReportV2 | null>(
    null
  );
  const [validationBusy, setValidationBusy] = useState<boolean>(false);

  const validateUnifiedMission = async () => {
    setValidationBusy(true);
    try {
      const mission = buildMission();
      const report = await unifiedMissionApi.validateMission(mission);
      setValidationReport(report);
      if (!report.valid && report.issues.length > 0) {
        const firstIssue = report.issues[0];
        const segmentMatch = /segments\[(\d+)\]/.exec(firstIssue.path);
        if (segmentMatch) {
          const segIndex = Number.parseInt(segmentMatch[1], 10);
          if (!Number.isNaN(segIndex)) {
            onFocusSegment(segIndex);
            setAuthoringStep('constraints');
          }
        } else {
          setAuthoringStep('target');
        }
      }
      return report;
    } finally {
      setValidationBusy(false);
    }
  };

  return {
    state: {
      validationReport,
      validationBusy,
    },
    setters: {
      setValidationReport,
    },
    actions: {
      validateUnifiedMission,
    },
  };
}

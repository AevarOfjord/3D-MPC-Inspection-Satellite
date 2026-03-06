import { useState } from 'react';

import { unifiedMissionApi, type ValidationReportV2 } from '../api/unifiedMissionApi';
import type { UnifiedMission } from '../api/unifiedMission';
import { mapIssuePathToAuthoringPhase } from '../utils/authoringValidation';
import type { MissionAuthoringPhase } from './useMissionState';

interface UseMissionValidationArgs {
  buildMission: () => UnifiedMission;
  jumpToFirstIssue?: boolean;
  onFocusSegment?: (index: number) => void;
  setAuthoringPhase?: (phase: MissionAuthoringPhase) => void;
}

export function useMissionValidation({
  buildMission,
  jumpToFirstIssue = true,
  onFocusSegment,
  setAuthoringPhase,
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
      if (jumpToFirstIssue && !report.valid && report.issues.length > 0 && setAuthoringPhase) {
        const firstIssue = report.issues[0];
        const targetPhase = mapIssuePathToAuthoringPhase(firstIssue.path);
        const segmentMatch = /segments\[(\d+)\]/.exec(firstIssue.path);
        if (segmentMatch && onFocusSegment) {
          const segIndex = Number.parseInt(segmentMatch[1], 10);
          if (!Number.isNaN(segIndex)) {
            onFocusSegment(segIndex);
            setAuthoringPhase(targetPhase);
          }
        } else if (!segmentMatch) {
          setAuthoringPhase(targetPhase);
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

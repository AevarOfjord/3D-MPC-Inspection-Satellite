import { useState } from 'react';

import type {
  MissionSegment,
  Pose,
  ScanSegment,
  SplineControl,
  TransferSegment,
} from '../api/unifiedMission';
import type { AuthoringPhase } from '../utils/authoringValidation';

export type MissionAuthoringPhase = AuthoringPhase;

interface UseMissionStateArgs {
  defaultMissionId: () => string;
  defaultTransferSegment: () => TransferSegment;
  defaultScanSegment: () => ScanSegment;
  defaultHoldSegment: () => MissionSegment;
  resolveOrbitTargetPose: (targetId: string) => Pose | undefined;
}

export function useMissionState({
  defaultMissionId,
  defaultTransferSegment,
  defaultScanSegment,
  defaultHoldSegment,
  resolveOrbitTargetPose,
}: UseMissionStateArgs) {
  const [missionId, setMissionId] = useState<string>(() => defaultMissionId());
  const [missionName, setMissionName] = useState<string>('Mission_V2');
  const [epoch, setEpoch] = useState<string>(new Date().toISOString());
  const [segments, setSegments] = useState<MissionSegment[]>([]);
  const [selectedSegmentIndex, setSelectedSegmentIndex] = useState<number | null>(
    null
  );
  const [splineControls, setSplineControls] = useState<SplineControl[]>([]);
  const [savedUnifiedMissions, setSavedUnifiedMissions] = useState<string[]>([]);
  const [authoringPhase, setAuthoringPhase] =
    useState<MissionAuthoringPhase>('target');
  const [selectedOrbitTargetId, setSelectedOrbitTargetId] = useState<
    string | null
  >(null);

  const addTransferSegment = () => {
    setSegments((prev) => {
      const next = [...prev, defaultTransferSegment()];
      setSelectedSegmentIndex(next.length - 1);
      return next;
    });
  };

  const addScanSegment = () => {
    setSegments((prev) => {
      const next = [...prev, defaultScanSegment()];
      setSelectedSegmentIndex(next.length - 1);
      setSelectedOrbitTargetId(null);
      return next;
    });
  };

  const addHoldSegment = () => {
    setSegments((prev) => {
      const next = [...prev, defaultHoldSegment()];
      setSelectedSegmentIndex(next.length - 1);
      return next;
    });
  };

  const removeSegment = (index: number) => {
    setSegments((prev) => prev.filter((_, i) => i !== index));
    setSelectedSegmentIndex((prev) => {
      if (prev === null) return null;
      if (prev === index) return null;
      if (prev > index) return prev - 1;
      return prev;
    });
  };

  const updateSegment = (index: number, next: MissionSegment) => {
    setSegments((prev) => prev.map((seg, i) => (i === index ? next : seg)));
  };

  const applyPathAssetToSegment = (assetId: string, index?: number) => {
    setSegments((prev) => {
      let targetIndex = index ?? selectedSegmentIndex ?? -1;
      if (
        targetIndex < 0 ||
        !prev[targetIndex] ||
        prev[targetIndex].type !== 'scan'
      ) {
        targetIndex = prev.findIndex((seg) => seg.type === 'scan');
      }
      if (targetIndex < 0) return prev;
      const seg = prev[targetIndex] as ScanSegment;
      const next = prev.map((s, i) =>
        i === targetIndex
          ? {
              ...seg,
              path_asset: assetId,
            }
          : s
      );
      setSelectedSegmentIndex(targetIndex);
      return next;
    });
  };

  const reorderSegments = (fromIndex: number, toIndex: number) => {
    setSegments((prev) => {
      const next = [...prev];
      const [moved] = next.splice(fromIndex, 1);
      next.splice(toIndex, 0, moved);
      return next;
    });
    setSelectedSegmentIndex((prev) => {
      if (prev === null) return null;
      if (prev === fromIndex) return toIndex;
      if (fromIndex < prev && toIndex >= prev) return prev - 1;
      if (fromIndex > prev && toIndex <= prev) return prev + 1;
      return prev;
    });
  };

  const addSplineControl = (position?: [number, number, number]) => {
    const nextControl: SplineControl = {
      position: position
        ? ([...position] as [number, number, number])
        : [0, 0, 0],
      weight: 1.0,
    };
    setSplineControls((prev) => [...prev, nextControl]);
  };

  const updateSplineControl = (index: number, next: SplineControl) => {
    setSplineControls((prev) => prev.map((c, i) => (i === index ? next : c)));
  };

  const removeSplineControl = (index: number) => {
    setSplineControls((prev) => prev.filter((_, i) => i !== index));
  };

  const assignScanTarget = (
    targetId: string,
    targetPosition?: [number, number, number]
  ) => {
    setSelectedOrbitTargetId(targetId);
    const resolvedPose = targetId ? resolveOrbitTargetPose(targetId) : undefined;
    setSegments((prev) => {
      const applyPrefill = (seg: ScanSegment) => {
        const standoff = seg.scan.standoff > 0 ? seg.scan.standoff : 10;
        const overlap = Number.isFinite(seg.scan.overlap) ? seg.scan.overlap : 0.25;
        const fovDeg = Number.isFinite(seg.scan.fov_deg) ? seg.scan.fov_deg : 60;
        return {
          ...seg,
          target_id: targetId,
          target_pose:
            resolvedPose ??
            (targetPosition
              ? ({
                  frame: 'ECI' as const,
                  position: [...targetPosition] as [number, number, number],
                } as Pose)
              : seg.target_pose),
          scan: {
            ...seg.scan,
            standoff,
            overlap,
            fov_deg: fovDeg,
            pitch: seg.scan.pitch ?? null,
          },
        };
      };

      let targetIndex: number | null = null;
      if (selectedSegmentIndex !== null && prev[selectedSegmentIndex]?.type === 'scan') {
        targetIndex = selectedSegmentIndex;
      } else {
        const scanIndices = prev
          .map((seg, idx) => (seg.type === 'scan' ? idx : -1))
          .filter((idx) => idx >= 0);
        if (scanIndices.length === 1) {
          targetIndex = scanIndices[0];
        }
      }

      if (targetIndex !== null && targetIndex >= 0) {
        const seg = prev[targetIndex] as ScanSegment;
        const next = prev.map((s, i) => (i === targetIndex ? applyPrefill(seg) : s));
        setSelectedSegmentIndex(targetIndex);
        return next;
      }

      const next = [...prev, applyPrefill({ ...defaultScanSegment(), target_id: targetId })];
      setSelectedSegmentIndex(next.length - 1);
      return next;
    });
  };

  const validateScanSegments = (): string | null => {
    const scanSegments = segments
      .map((seg, idx) => ({ seg, idx }))
      .filter(({ seg }) => seg.type === 'scan') as {
      seg: ScanSegment;
      idx: number;
    }[];
    for (const { seg, idx } of scanSegments) {
      if (!seg.target_id) {
        setSelectedSegmentIndex(idx);
        return 'Scan segment requires a target object. Select one in the Inspector.';
      }
    }
    return null;
  };

  return {
    state: {
      missionId,
      missionName,
      epoch,
      segments,
      selectedSegmentIndex,
      splineControls,
      savedUnifiedMissions,
      authoringPhase,
      selectedOrbitTargetId,
    },
    setters: {
      setMissionId,
      setMissionName,
      setEpoch,
      setSegments,
      setSelectedSegmentIndex,
      setSplineControls,
      setSavedUnifiedMissions,
      setAuthoringPhase,
      setSelectedOrbitTargetId,
    },
    actions: {
      addTransferSegment,
      addScanSegment,
      addHoldSegment,
      removeSegment,
      updateSegment,
      applyPathAssetToSegment,
      reorderSegments,
      addSplineControl,
      updateSplineControl,
      removeSplineControl,
      assignScanTarget,
      validateScanSegments,
    },
  };
}

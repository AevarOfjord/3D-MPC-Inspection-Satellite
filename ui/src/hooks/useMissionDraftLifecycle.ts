import { useEffect, useRef } from 'react';

import type { MissionDraftResponse } from '../api/unifiedMissionApi';
import type { UnifiedMission } from '../api/unifiedMission';

interface UseMissionDraftLifecycleArgs {
  storageKey: string;
  loadDraftById: (draftId: string) => Promise<MissionDraftResponse>;
  onRestoreMission: (mission: UnifiedMission, fallbackName?: string) => void;
  onDraftMetadata: (draftId: string, revision: number, savedAt: string) => void;
  saveMissionDraft: () => Promise<MissionDraftResponse>;
  missionId: string;
  missionName: string;
  epoch: string;
  startFrame: 'ECI' | 'LVLH';
  startTargetId?: string;
  startPosition: [number, number, number];
  segmentsCount: number;
  splineControlsCount: number;
  obstaclesCount: number;
  previewPathPoints: number;
  isManualMode: boolean;
}

export function useMissionDraftLifecycle({
  storageKey,
  loadDraftById,
  onRestoreMission,
  onDraftMetadata,
  saveMissionDraft,
  missionId,
  missionName,
  epoch,
  startFrame,
  startTargetId,
  startPosition,
  segmentsCount,
  splineControlsCount,
  obstaclesCount,
  previewPathPoints,
  isManualMode,
}: UseMissionDraftLifecycleArgs) {
  const restoreAttemptedRef = useRef(false);

  useEffect(() => {
    if (restoreAttemptedRef.current) return;
    restoreAttemptedRef.current = true;

    const storedDraftId = window.localStorage.getItem(storageKey);
    if (!storedDraftId) return;

    loadDraftById(storedDraftId)
      .then((draft) => {
        onDraftMetadata(draft.draft_id, draft.revision, draft.saved_at);
        const shouldRestore = window.confirm(
          `Restore mission draft from ${new Date(draft.saved_at).toLocaleString()}?`
        );
        if (shouldRestore) {
          onRestoreMission(draft.mission, draft.mission.name);
        }
      })
      .catch(() => {
        window.localStorage.removeItem(storageKey);
      });
  }, [storageKey, loadDraftById, onDraftMetadata, onRestoreMission]);

  useEffect(() => {
    const timer = window.setTimeout(() => {
      void saveMissionDraft().catch((err) => {
        console.warn('Draft autosave failed', err);
      });
    }, 5000);
    return () => window.clearTimeout(timer);
  }, [
    saveMissionDraft,
    missionId,
    missionName,
    epoch,
    startFrame,
    startTargetId,
    startPosition,
    segmentsCount,
    splineControlsCount,
    obstaclesCount,
    previewPathPoints,
    isManualMode,
  ]);
}

import { useState } from 'react';

import type { MissionDraftResponse } from '../api/unifiedMissionApi';
import type { UnifiedMission } from '../api/unifiedMission';

interface UseMissionDraftStateArgs {
  storageKey: string;
  saveDraft: (
    config: UnifiedMission,
    options?: { draft_id?: string; base_revision?: number }
  ) => Promise<MissionDraftResponse>;
}

export function useMissionDraftState({
  storageKey,
  saveDraft,
}: UseMissionDraftStateArgs) {
  const [draftId, setDraftId] = useState<string | null>(null);
  const [draftRevision, setDraftRevision] = useState<number | null>(null);
  const [draftSavedAt, setDraftSavedAt] = useState<string | null>(null);

  const setDraftMetadata = (nextDraftId: string, revision: number, savedAt: string) => {
    setDraftId(nextDraftId);
    setDraftRevision(revision);
    setDraftSavedAt(savedAt);
  };

  const saveMissionDraft = async (mission: UnifiedMission) => {
    try {
      const draft = await saveDraft(mission, {
        draft_id: draftId ?? undefined,
        base_revision: draftRevision ?? undefined,
      });
      setDraftMetadata(draft.draft_id, draft.revision, draft.saved_at);
      window.localStorage.setItem(storageKey, draft.draft_id);
      return draft;
    } catch (err: unknown) {
      // On a 409 revision conflict the server has a newer revision than we know about.
      // Re-try once with no base_revision so we force-overwrite with the latest client state.
      const message = err instanceof Error ? err.message : String(err);
      if (message.includes('409') || message.toLowerCase().includes('conflict')) {
        const draft = await saveDraft(mission, {
          draft_id: draftId ?? undefined,
          // omit base_revision → server skips the conflict check
          base_revision: undefined,
        });
        setDraftMetadata(draft.draft_id, draft.revision, draft.saved_at);
        window.localStorage.setItem(storageKey, draft.draft_id);
        return draft;
      }
      throw err;
    }
  };

  return {
    state: {
      draftId,
      draftRevision,
      draftSavedAt,
    },
    actions: {
      setDraftMetadata,
      saveMissionDraft,
    },
  };
}

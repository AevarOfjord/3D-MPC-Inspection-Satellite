import { unifiedMissionApi } from '../api/unifiedMissionApi';
import type { UnifiedMission } from '../api/unifiedMission';

interface UseMissionPersistenceArgs {
  buildMission: (options?: { includeManualPath?: boolean }) => UnifiedMission;
  onLoadMission: (mission: UnifiedMission, fallbackName?: string) => void;
  setSavedUnifiedMissions: (missions: string[]) => void;
}

export function useMissionPersistence({
  buildMission,
  onLoadMission,
  setSavedUnifiedMissions,
}: UseMissionPersistenceArgs) {
  const refreshUnifiedMissions = async () => {
    const res = await unifiedMissionApi.listSavedMissions();
    setSavedUnifiedMissions(res.missions);
    return res.missions;
  };

  const saveUnifiedMission = async (name: string) => {
    const mission = buildMission({ includeManualPath: true });
    return unifiedMissionApi.saveMission(name, mission);
  };

  const loadUnifiedMission = async (name: string) => {
    const mission = await unifiedMissionApi.loadMission(name);
    onLoadMission(mission, name);
    return mission;
  };

  const pushUnifiedMission = async () => {
    const mission = buildMission({ includeManualPath: true });
    return unifiedMissionApi.setMission(mission);
  };

  return {
    actions: {
      refreshUnifiedMissions,
      saveUnifiedMission,
      loadUnifiedMission,
      pushUnifiedMission,
    },
  };
}

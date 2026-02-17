import { downsamplePath } from '../utils/pathResample';
import type { UnifiedMission } from '../api/unifiedMission';
import type {
  PreviewMissionResponse,
  SaveMissionV2Response,
  ValidationReportV2,
} from '../api/unifiedMissionApi';
import type { MissionAuthoringStep } from './useMissionState';

type BuildMissionFn = (options?: { includeManualPath?: boolean }) => UnifiedMission;

type UseMissionExecutionArgs = {
  buildMission: BuildMissionFn;
  validateScanSegments: () => string | null;
  validateUnifiedMission: () => Promise<ValidationReportV2>;
  saveUnifiedMission: (name: string) => Promise<SaveMissionV2Response>;
  refreshUnifiedMissions: () => Promise<unknown>;
  pushUnifiedMission: () => Promise<unknown>;
  previewMission: (mission: UnifiedMission) => Promise<PreviewMissionResponse>;
  controlSimulation: (action: 'pause' | 'resume' | 'step', steps?: number) => Promise<unknown>;
  setMissionId: (missionId: string) => void;
  setMissionName: (name: string) => void;
  setAuthoringStep: (step: MissionAuthoringStep) => void;
  setLoading: (loading: boolean) => void;
  setIsManualMode: (manual: boolean) => void;
  setPreviewPath: (path: [number, number, number][]) => void;
  setStats: (stats: { duration: number; length: number; points: number } | null) => void;
  editPointLimit: number;
};

export function useMissionExecution({
  buildMission,
  validateScanSegments,
  validateUnifiedMission,
  saveUnifiedMission,
  refreshUnifiedMissions,
  pushUnifiedMission,
  previewMission,
  controlSimulation,
  setMissionId,
  setMissionName,
  setAuthoringStep,
  setLoading,
  setIsManualMode,
  setPreviewPath,
  setStats,
  editPointLimit,
}: UseMissionExecutionArgs) {
  const handleSaveUnifiedMission = async () => {
    const validationError = validateScanSegments();
    if (validationError) {
      alert(validationError);
      return;
    }
    const remoteValidation = await validateUnifiedMission();
    if (!remoteValidation.valid) {
      alert('Mission has validation errors. Open the Validate step to resolve them.');
      setAuthoringStep('validate');
      return;
    }

    const name = prompt('Enter mission name (e.g. Starlink_Scan_M01):');
    if (!name) return;

    try {
      const saved = await saveUnifiedMission(name);
      setMissionId(saved.mission_id);
      setMissionName(name);
      await refreshUnifiedMissions();
      alert(`Mission saved: ${name}`);
      setAuthoringStep('save_launch');
    } catch (err: any) {
      console.error(err);
      alert(`Failed to save mission: ${err.message || err}`);
    }
  };

  const generateUnifiedPath = async () => {
    setLoading(true);
    try {
      const validationError = validateScanSegments();
      if (validationError) {
        alert(validationError);
        return;
      }

      const remoteValidation = await validateUnifiedMission();
      if (!remoteValidation.valid) {
        alert('Mission has validation errors. Resolve them before preview.');
        setAuthoringStep('validate');
        return;
      }

      setIsManualMode(false);
      const mission = buildMission({ includeManualPath: false });
      const preview = await previewMission(mission);
      if (preview.path && preview.path.length > 0) {
        const editablePath = downsamplePath(preview.path, editPointLimit);
        setPreviewPath(editablePath);
        setStats({
          duration: preview.path_speed > 0 ? preview.path_length / preview.path_speed : 0,
          length: preview.path_length,
          points: editablePath.length,
        });
      } else {
        setPreviewPath([]);
        setStats(null);
      }
      return preview;
    } catch (err: any) {
      console.error('Preview Error:', err);
      alert(`Preview Failed: ${err.message || err}`);
      throw err;
    } finally {
      setLoading(false);
    }
  };

  const handleRun = async (onSuccess?: () => void) => {
    setLoading(true);
    try {
      const validationError = validateScanSegments();
      if (validationError) {
        alert(validationError);
        return;
      }

      await pushUnifiedMission();
      await controlSimulation('resume');
      alert('Mission Launched! Switching to Live View.');
      if (onSuccess) onSuccess();
    } catch (err: any) {
      console.error(err);
      alert(`Failed to launch: ${err.message || err}`);
    } finally {
      setLoading(false);
    }
  };

  return {
    actions: {
      handleSaveUnifiedMission,
      generateUnifiedPath,
      handleRun,
    },
  };
}

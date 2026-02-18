import { downsamplePath } from '../utils/pathResample';
import type { UnifiedMission } from '../api/unifiedMission';
import type {
  PreviewMissionResponse,
  SaveMissionV2Response,
  ValidationReportV2,
} from '../api/unifiedMissionApi';
import type { MissionAuthoringStep } from './useMissionState';
import { useDialog, useToast } from '../feedback/feedbackContext';
import type { DialogIntent } from '../feedback/feedbackContext';

type DialogFormFn = (
  options: Omit<Extract<DialogIntent, { type: 'form' }>, 'type'>
) => Promise<Record<string, string> | null>;

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
  const { showToast } = useToast();
  const dialog = useDialog();

  const handleSaveUnifiedMission = async () => {
    const validationError = validateScanSegments();
    if (validationError) {
      showToast({ tone: 'error', title: 'Validation', message: validationError });
      return;
    }
    const remoteValidation = await validateUnifiedMission();
    if (!remoteValidation.valid) {
      showToast({
        tone: 'error',
        title: 'Validation',
        message: 'Mission has validation errors. Open the Validate step to resolve them.',
        actionLabel: 'Open Validate',
        onAction: () => setAuthoringStep('validate'),
      });
      setAuthoringStep('validate');
      return;
    }

    const defaultName = buildMission({ includeManualPath: true }).name;
    let name: string | null = null;

    const maybeForm = (dialog as unknown as { form?: DialogFormFn }).form;
    if (typeof maybeForm === 'function') {
      const values = await maybeForm({
        title: 'Save Mission',
        message: 'Provide a mission name before saving.',
        confirmLabel: 'Save',
        fields: [
          {
            id: 'name',
            label: 'Mission Name',
            placeholder: 'Starlink_Scan_M01',
            defaultValue: defaultName,
            required: true,
          },
        ],
      });
      name = values?.name?.trim() || null;
    }
    if (!name) {
      const promptName = await dialog.prompt('Enter mission name (e.g. Starlink_Scan_M01):', {
        title: 'Save Mission',
        confirmLabel: 'Save',
        placeholder: 'Starlink_Scan_M01',
        defaultValue: defaultName,
        requireNonEmpty: true,
      });
      name = promptName?.trim() || null;
    }
    if (!name) return;

    try {
      const saved = await saveUnifiedMission(name);
      setMissionId(saved.mission_id);
      setMissionName(name);
      await refreshUnifiedMissions();
      showToast({ tone: 'success', title: 'Mission Saved', message: `Mission saved: ${name}` });
      setAuthoringStep('save_launch');
    } catch (err: any) {
      console.error(err);
      showToast({
        tone: 'error',
        title: 'Save Failed',
        message: `Failed to save mission: ${err.message || err}`,
      });
    }
  };

  const generateUnifiedPath = async () => {
    setLoading(true);
    try {
      const validationError = validateScanSegments();
      if (validationError) {
        showToast({ tone: 'error', title: 'Validation', message: validationError });
        return;
      }

      const remoteValidation = await validateUnifiedMission();
      if (!remoteValidation.valid) {
        showToast({
          tone: 'error',
          title: 'Validation',
          message: 'Mission has validation errors. Resolve them before preview.',
          actionLabel: 'Open Validate',
          onAction: () => setAuthoringStep('validate'),
        });
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
      showToast({
        tone: 'error',
        title: 'Preview Failed',
        message: `Preview Failed: ${err.message || err}`,
      });
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
        showToast({ tone: 'error', title: 'Validation', message: validationError });
        return;
      }

      await pushUnifiedMission();
      await controlSimulation('resume');
      showToast({
        tone: 'success',
        title: 'Mission Launched',
        message: 'Mission launched. Switching to live view.',
      });
      if (onSuccess) onSuccess();
    } catch (err: any) {
      console.error(err);
      showToast({
        tone: 'error',
        title: 'Launch Failed',
        message: `Failed to launch: ${err.message || err}`,
      });
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

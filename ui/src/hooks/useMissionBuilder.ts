import {
  unifiedMissionApi,
} from '../api/unifiedMissionApi';
import type {
  UnifiedMission,
} from '../api/unifiedMission';
import { useHistory } from './useHistory';
import { useScanProjectEditor } from './useScanProjectEditor';
import {
  useMissionState,
  type MissionAuthoringStep,
} from './useMissionState';
import { useMissionValidation } from './useMissionValidation';
import { useMissionPersistence } from './useMissionPersistence';
import { useMissionInteractions } from './useMissionInteractions';
import { useScanPlaneControls } from './useScanPlaneControls';
import { useMissionAssets } from './useMissionAssets';
import { useMissionPathGeneration } from './useMissionPathGeneration';
import { useScanAuthoringState } from './useScanAuthoringState';
import { useMissionSceneState } from './useMissionSceneState';
import { buildUnifiedMissionPayload } from './useMissionSerializer';
import { useMissionDraftLifecycle } from './useMissionDraftLifecycle';
import { useMissionExecution } from './useMissionExecution';
import { useMissionHydration } from './useMissionHydration';
import { useMissionPathEffects } from './useMissionPathEffects';
import { useMissionRuntimeEffects } from './useMissionRuntimeEffects';
import { useMissionRuntimeState } from './useMissionRuntimeState';
import { useMissionDraftState } from './useMissionDraftState';
import { buildMissionBuilderResult } from './useMissionBuilderResult';
import {
  defaultHoldSegment,
  defaultScanSegment,
  defaultTransferSegment,
  defaultTransferToPathSegment,
  nextMissionId,
  nextSegmentId,
  resolveOrbitTargetPose,
} from './missionDefaults';
import { orbitSnapshot } from '../data/orbitSnapshot';

const DRAFT_ID_STORAGE_KEY = 'mission_control_draft_id_v2';

export function useMissionBuilder() {
  const runtimeState = useMissionRuntimeState();
  const {
    state: { modelUrl, modelPath, loading, isManualMode, stats },
    setters: { setModelUrl, setModelPath, setLoading, setIsManualMode, setStats },
  } = runtimeState;

  const pathHistory = useHistory<[number, number, number][]>([]);
  const previewPath = pathHistory.state;

  const sceneState = useMissionSceneState();
  const {
    state: {
      startPosition,
      startFrame,
      startTargetId,
      startAngle,
      referencePosition,
      referenceAngle,
      obstacles,
      selectedObjectId,
      transformMode,
    },
    setters: {
      setStartPosition,
      setStartFrame,
      setStartTargetId,
      setStartAngle,
      setReferencePosition,
      setReferenceAngle,
      setObstacles,
      setSelectedObjectId,
      setTransformMode,
    },
  } = sceneState;

  const scanAuthoringState = useScanAuthoringState();
  const {
    state: {
      config,
      levelSpacing,
      availableModels,
      pathAssets,
      scanProjects,
      editPointLimit,
      savePointMultiplier,
      scanPlaneEnabled,
      scanPlaneA,
      scanPlaneB,
      selectedScanPlaneHandle,
      scanPlaneAxis,
      scanProject,
      selectedScanId,
      selectedKeyLevelId,
      selectedConnectorId,
      selectedProjectScanPlaneHandle,
      selectedScanCenterHandle,
      selectedKeyLevelHandle,
      selectedConnectorControl,
      connectMode,
      connectSourceEndpoint,
      compilePreviewState,
      transferTargetRef,
      compilePending,
      scanProjectAutoPreviewEnabled,
      centerDragActive,
    },
    setters: {
      setConfig,
      setLevelSpacing,
      setAvailableModels,
      setPathAssets,
      setScanProjects,
      setEditPointLimit,
      setSavePointMultiplier,
      setScanPlaneEnabled,
      setScanPlaneA,
      setScanPlaneB,
      setSelectedScanPlaneHandle,
      setScanPlaneAxis,
      setScanProject,
      setSelectedScanId,
      setSelectedKeyLevelId,
      setSelectedConnectorId,
      setSelectedProjectScanPlaneHandle,
      setSelectedScanCenterHandle,
      setSelectedKeyLevelHandle,
      setSelectedConnectorControl,
      setConnectMode,
      setConnectSourceEndpoint,
      setCompilePreviewState,
      setTransferTargetRef,
      setCompilePending,
      setScanProjectAutoPreviewEnabled,
      setCenterDragActive,
    },
  } = scanAuthoringState;

  // Unified Mission (V2)
  const missionState = useMissionState({
    defaultMissionId: nextMissionId,
    defaultTransferSegment,
    defaultScanSegment,
    defaultHoldSegment,
    resolveOrbitTargetPose,
  });
  const {
    state: {
      missionId,
      missionName,
      epoch,
      segments,
      selectedSegmentIndex,
      splineControls,
      savedUnifiedMissions,
      authoringStep,
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
      setAuthoringStep,
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
  } = missionState;

  const missionAssets = useMissionAssets({
    config,
    setConfig,
    setLoading,
    setModelUrl,
    setModelPath,
    setScanProject,
    setScanProjectAutoPreviewEnabled,
    setAvailableModels,
    setPathAssets,
    setScanProjects,
    setStats,
    setIsManualMode,
    pathHistory,
    previewPath,
    editPointLimit,
    savePointMultiplier,
  });
  const {
    actions: {
      refreshModelList,
      refreshPathAssets,
      refreshScanProjects,
      selectModelPath,
      handleFileUpload,
      savePathAsset,
      loadPathAsset,
    },
  } = missionAssets;

  const missionPathGeneration = useMissionPathGeneration({
    config,
    levelSpacing,
    editPointLimit,
    pathHistory,
    setIsManualMode,
    setLoading,
    setStats,
  });
  const {
    actions: { handlePreview, setManualPath },
  } = missionPathGeneration;

  useMissionRuntimeEffects({
    segments,
    setSegments,
    defaultScanSegment,
    defaultTransferToPathSegment,
    refreshModelList,
    refreshPathAssets,
    refreshScanProjects,
    isManualMode,
    pathHistory,
    previewPath,
    config,
    setStats,
    loading,
    handlePreview,
  });

  const scanProjectEditor = useScanProjectEditor({
    authoringStep,
    scanProject,
    setScanProject,
    configObjPath: config.obj_path,
    referencePosition,
    referenceAngle,
    selectedScanId,
    setSelectedScanId,
    selectedKeyLevelId,
    setSelectedKeyLevelId,
    selectedConnectorId,
    setSelectedConnectorId,
    setSelectedProjectScanPlaneHandle,
    selectedScanCenterHandle,
    setSelectedScanCenterHandle,
    selectedKeyLevelHandle,
    setSelectedKeyLevelHandle,
    selectedConnectorControl,
    setSelectedConnectorControl,
    connectMode,
    setConnectMode,
    connectSourceEndpoint,
    setConnectSourceEndpoint,
    compilePreviewState,
    setCompilePreviewState,
    setCompilePending,
    scanProjectAutoPreviewEnabled,
    setScanProjectAutoPreviewEnabled,
    setCenterDragActive,
    setManualPath,
    setLoading,
    setStats,
    refreshPathAssets,
    refreshScanProjects,
    selectModelPath,
  });
  const {
    actions: {
      createDefaultScanProjectState,
      updateScanProject,
      addScan,
      removeScan,
      updateScan,
      addKeyLevel,
      updateKeyLevel,
      removeKeyLevel,
      setScanAxisAligned,
      moveProjectScanPlaneHandle,
      beginScanCenterDrag,
      updateScanCenterDrag,
      endScanCenterDrag,
      updateScanCenterPosition,
      updateKeyLevelHandlePosition,
      updateConnector,
      removeConnector,
      startConnectMode,
      cancelConnectMode,
      selectEndpointForConnect,
      updateConnectorControl,
      compileScanProjectNow,
      compileScanProjectDebounced,
      previewScanProject,
      saveScanProject,
      loadScanProjectById,
      saveBakedPathFromCompiled,
      setPathDensityMultiplier,
    },
  } = scanProjectEditor;

  const missionInteractions = useMissionInteractions({
    obstacles,
    setObstacles,
    selectedObjectId,
    setSelectedObjectId,
    transformMode,
    setStartPosition,
    setStartAngle,
    setReferencePosition,
    setReferenceAngle,
    isManualMode,
    setIsManualMode,
    previewPath,
    pathHistory,
  });
  const {
    actions: {
      addObstacle,
      removeObstacle,
      updateObstacle,
      handleWaypointMove,
      commitWaypointMove,
      addWaypoint,
      removeWaypoint,
      removeWaypointAtIndex,
      handleObjectTransform,
    },
  } = missionInteractions;

  const missionDraftState = useMissionDraftState({
    storageKey: DRAFT_ID_STORAGE_KEY,
    saveDraft: unifiedMissionApi.saveDraft,
  });
  const {
    state: { draftId, draftRevision, draftSavedAt },
    actions: { setDraftMetadata, saveMissionDraft: persistMissionDraft },
  } = missionDraftState;

  // --- Unified Mission Helpers ---

  const buildUnifiedMission = (options?: { includeManualPath?: boolean }): UnifiedMission => {
    return buildUnifiedMissionPayload({
      includeManualPath: options?.includeManualPath ?? true,
      missionId,
      missionName,
      epoch,
      startFrame,
      startTargetId,
      startPosition,
      segments,
      splineControls,
      isManualMode,
      previewPath,
      obstacles,
      draftRevision,
      pathDensityMultiplier: scanProject.path_density_multiplier,
      scanProjectScans: scanProject.scans,
      selectedScanId,
      nextSegmentId,
      resolveOrbitTargetPose,
    });
  };

  const missionValidation = useMissionValidation({
    buildMission: () => buildUnifiedMission({ includeManualPath: true }),
    jumpToFirstIssue: false,
    onFocusSegment: setSelectedSegmentIndex,
    setAuthoringStep,
  });
  const {
    state: { validationReport, validationBusy },
    setters: { setValidationReport },
    actions: { validateUnifiedMission },
  } = missionValidation;

  const missionHydration = useMissionHydration({
    nextMissionId,
    nextSegmentId,
    resolveOrbitTargetPose,
    setMissionId,
    setMissionName,
    setEpoch,
    setSegments,
    setSplineControls,
    setPreviewPath: pathHistory.set,
    setIsManualMode,
    speedMax: config.speed_max,
    setStats,
    setSelectedSegmentIndex,
    setStartFrame,
    setStartTargetId,
    setStartPosition,
    setObstacles,
    setSelectedOrbitTargetId,
    setTransferTargetRef,
    setValidationReport,
    setScanProject,
  });
  const {
    actions: { applyLoadedMission },
  } = missionHydration;

  const missionPersistence = useMissionPersistence({
    buildMission: buildUnifiedMission,
    onLoadMission: applyLoadedMission,
    setSavedUnifiedMissions,
  });
  const {
    actions: {
      refreshUnifiedMissions,
      saveUnifiedMission,
      loadUnifiedMission,
      pushUnifiedMission,
    },
  } = missionPersistence;

  const missionExecution = useMissionExecution({
    buildMission: buildUnifiedMission,
    validateScanSegments,
    validateUnifiedMission,
    saveUnifiedMission,
    refreshUnifiedMissions,
    pushUnifiedMission,
    previewMission: unifiedMissionApi.previewMission,
    controlSimulation: unifiedMissionApi.controlSimulation,
    setMissionId,
    setMissionName,
    setAuthoringStep,
    setLoading,
    setIsManualMode,
    setPreviewPath: pathHistory.set,
    setStats,
    editPointLimit,
  });
  const {
    actions: { handleSaveUnifiedMission, generateUnifiedPath, handleRun },
  } = missionExecution;

  const saveMissionDraft = () =>
    persistMissionDraft(buildUnifiedMission({ includeManualPath: true }));

  const missionDraftLifecycle = useMissionDraftLifecycle({
    storageKey: DRAFT_ID_STORAGE_KEY,
    listDraftIds: unifiedMissionApi.listDrafts,
    loadDraftById: unifiedMissionApi.loadDraft,
    onRestoreMission: applyLoadedMission,
    onDraftMetadata: setDraftMetadata,
    saveMissionDraft,
    missionId,
    missionName,
    epoch,
    startFrame,
    startTargetId,
    startPosition,
    segmentsCount: segments.length,
    splineControlsCount: splineControls.length,
    obstaclesCount: obstacles.length,
    previewPathPoints: previewPath.length,
    isManualMode,
  });
  const {
    state: { pendingDraftRestore },
    actions: { restorePendingDraft, discardPendingDraft },
  } = missionDraftLifecycle;

  const scanPlaneControls = useScanPlaneControls({
    referencePosition,
    referenceAngle,
    scanPlaneAxis,
    setScanPlaneAxis,
    setScanPlaneA,
    setScanPlaneB,
  });
  const {
    actions: { moveScanPlaneHandle, setScanPlaneAxisAligned },
  } = scanPlaneControls;

  useMissionPathEffects({
    previewPath,
    editPointLimit,
    isManualMode,
    speedMax: config.speed_max,
    selectedObjectId,
    setPreviewPath: pathHistory.set,
    setSelectedObjectId,
    stats,
    setStats,
    removeWaypoint,
  });

  return buildMissionBuilderResult({
    baseState: {
      modelUrl,
      modelPath,
      loading,
      previewPath,
      isManualMode,
      stats,
      startPosition,
      startFrame,
      startTargetId,
      startAngle,
      referencePosition,
      referenceAngle,
      obstacles,
      selectedObjectId,
      transformMode,
    },
    scanState: {
      config,
      levelSpacing,
      availableModels,
      pathAssets,
      scanProjects,
      editPointLimit,
      savePointMultiplier,
      scanPlaneEnabled,
      scanPlaneA,
      scanPlaneB,
      scanPlaneAxis,
      selectedScanPlaneHandle,
      scanProject,
      selectedScanId,
      selectedKeyLevelId,
      selectedConnectorId,
      selectedProjectScanPlaneHandle,
      selectedScanCenterHandle,
      selectedKeyLevelHandle,
      selectedConnectorControl,
      connectMode,
      connectSourceEndpoint,
      centerDragActive,
      compilePreviewState,
      transferTargetRef,
      compilePending,
      scanProjectAutoPreviewEnabled,
    },
    missionState: {
      missionId,
      missionName,
      epoch,
      segments,
      selectedSegmentIndex,
      splineControls,
      savedUnifiedMissions,
      selectedOrbitTargetId,
      authoringStep,
    },
    validationState: {
      validationReport,
      validationBusy,
    },
    draftState: {
      draftId,
      draftRevision,
      draftSavedAt,
      pendingDraftRestore,
    },
    historyState: {
      canUndo: pathHistory.canUndo,
      canRedo: pathHistory.canRedo,
    },
    baseSetters: {
      setStartPosition,
      setStartFrame,
      setStartTargetId,
      setStartAngle,
      setReferencePosition,
      setReferenceAngle,
      setSelectedObjectId,
      setTransformMode,
    },
    scanSetters: {
      setConfig,
      setLevelSpacing,
      setEditPointLimit,
      setSavePointMultiplier,
      setScanPlaneEnabled,
      setScanPlaneA,
      setScanPlaneB,
      setScanPlaneAxis,
      setSelectedScanPlaneHandle,
      setScanProject,
      setSelectedScanId,
      setSelectedKeyLevelId,
      setSelectedConnectorId,
      setSelectedProjectScanPlaneHandle,
      setSelectedScanCenterHandle,
      setSelectedKeyLevelHandle,
      setSelectedConnectorControl,
      setConnectMode,
      setConnectSourceEndpoint,
      setCompilePreviewState,
      setTransferTargetRef,
    },
    missionSetters: {
      setMissionId,
      setMissionName,
      setEpoch,
      setSelectedSegmentIndex,
      setSegments,
      setAuthoringStep,
    },
    validationSetters: {
      setValidationReport,
    },
    ioActions: {
      handleFileUpload,
      handlePreview,
      setManualPath,
      selectModelPath,
      refreshModelList,
      refreshPathAssets,
      refreshScanProjects,
      savePathAsset,
      loadPathAsset,
    },
    scanProjectActions: {
      createDefaultScanProjectState,
      updateScanProject,
      addScan,
      removeScan,
      updateScan,
      addKeyLevel,
      updateKeyLevel,
      removeKeyLevel,
      setScanAxisAligned,
      moveProjectScanPlaneHandle,
      beginScanCenterDrag,
      updateScanCenterDrag,
      endScanCenterDrag,
      updateScanCenterPosition,
      updateKeyLevelHandlePosition,
      setSelectedProjectScanPlaneHandle,
      setSelectedScanCenterHandle,
      setSelectedKeyLevelHandle,
      updateConnector,
      removeConnector,
      startConnectMode,
      cancelConnectMode,
      selectEndpointForConnect,
      updateConnectorControl,
      setSelectedConnectorControl,
      compileScanProjectNow,
      compileScanProjectDebounced,
      previewScanProject,
      saveScanProject,
      loadScanProjectById,
      saveBakedPathFromCompiled,
      setPathDensityMultiplier,
      setSelectedScanId,
      setSelectedKeyLevelId,
      setSelectedConnectorId,
      setTransferTargetRef,
    },
    interactionActions: {
      addObstacle,
      removeObstacle,
      updateObstacle,
      handleObjectTransform,
      setSelectedObjectId,
      setTransformMode,
      handleWaypointMove,
      commitWaypointMove,
      addWaypoint,
      removeWaypoint,
      removeWaypointAtIndex,
    },
    missionActions: {
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
      setSelectedOrbitTargetId,
      setAuthoringStep,
    },
    validationActions: {
      validateUnifiedMission,
    },
    persistenceActions: {
      saveMissionDraft,
      refreshUnifiedMissions,
      saveUnifiedMission,
      loadUnifiedMission,
      pushUnifiedMission,
      restorePendingDraft,
      discardPendingDraft,
    },
    executionActions: {
      handleRun,
      handleSaveUnifiedMission,
      generateUnifiedPath,
    },
    scanPlaneActions: {
      setSelectedScanPlaneHandle,
      moveScanPlaneHandle,
      setScanPlaneAxisAligned,
    },
    miscActions: {
      undo: pathHistory.undo,
      redo: pathHistory.redo,
      selectSegment: setSelectedSegmentIndex,
    },
  });
}

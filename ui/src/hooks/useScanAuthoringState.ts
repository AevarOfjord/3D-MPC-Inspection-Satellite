import { useState } from 'react';

import type { PathAssetSummary } from '../api/pathAssets';
import type { MeshScanConfig, ModelInfo } from '../api/trajectory';
import type { ScanCompileResponse, ScanProject, ScanProjectSummary } from '../types/scanProject';
import type {
  ConnectEndpoint,
  SelectedConnectorControl,
  SelectedKeyLevelHandle,
  SelectedProjectPlaneHandle,
  SelectedScanCenterHandle,
} from './useScanProjectEditor';
import { createDefaultScanProject } from '../utils/scanProjectValidation';
import type { SelectedTransferEndpoint } from '../types/authoring';

export function useScanAuthoringState() {
  const [config, setConfig] = useState<MeshScanConfig>({
    obj_path: '',
    standoff: 0.5,
    levels: 8,
    points_per_circle: 72,
    speed_max: 0.2,
    speed_min: 0.05,
    lateral_accel: 0.05,
    z_margin: 0.0,
    scan_axis: 'Z',
    pattern: 'spiral',
  });
  const [levelSpacing, setLevelSpacing] = useState<number>(0.1);
  const [availableModels, setAvailableModels] = useState<ModelInfo[]>([]);
  const [pathAssets, setPathAssets] = useState<PathAssetSummary[]>([]);
  const [scanProjects, setScanProjects] = useState<ScanProjectSummary[]>([]);
  const [editPointLimit, setEditPointLimit] = useState<number>(Number.MAX_SAFE_INTEGER);
  const [savePointMultiplier, setSavePointMultiplier] = useState<number>(10);
  const [scanPlaneEnabled, setScanPlaneEnabled] = useState<boolean>(false);
  const [scanPlaneA, setScanPlaneA] = useState<[number, number, number]>([0, 0, -0.5]);
  const [scanPlaneB, setScanPlaneB] = useState<[number, number, number]>([0, 0, 0.5]);
  const [selectedScanPlaneHandle, setSelectedScanPlaneHandle] = useState<'a' | 'b' | null>(null);
  const [scanPlaneAxis, setScanPlaneAxis] = useState<'X' | 'Y' | 'Z'>('Z');
  const [scanProject, setScanProject] = useState<ScanProject>(() =>
    createDefaultScanProject('')
  );
  const [selectedScanId, setSelectedScanId] = useState<string | null>(null);
  const [selectedKeyLevelId, setSelectedKeyLevelId] = useState<string | null>(null);
  const [selectedConnectorId, setSelectedConnectorId] = useState<string | null>(null);
  const [selectedProjectScanPlaneHandle, setSelectedProjectScanPlaneHandle] =
    useState<SelectedProjectPlaneHandle>(null);
  const [selectedScanCenterHandle, setSelectedScanCenterHandle] =
    useState<SelectedScanCenterHandle>(null);
  const [selectedKeyLevelHandle, setSelectedKeyLevelHandle] =
    useState<SelectedKeyLevelHandle>(null);
  const [selectedConnectorControl, setSelectedConnectorControl] =
    useState<SelectedConnectorControl>(null);
  const [connectMode, setConnectMode] = useState<boolean>(false);
  const [connectSourceEndpoint, setConnectSourceEndpoint] = useState<ConnectEndpoint>(null);
  const [compilePreviewState, setCompilePreviewState] =
    useState<ScanCompileResponse | null>(null);
  const [selectedTransferEndpoint, setSelectedTransferEndpoint] =
    useState<SelectedTransferEndpoint>(null);
  const [compilePending, setCompilePending] = useState<boolean>(false);
  const [scanProjectAutoPreviewEnabled, setScanProjectAutoPreviewEnabled] = useState<boolean>(true);
  const [centerDragActive, setCenterDragActive] = useState<boolean>(false);

  return {
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
      selectedTransferEndpoint,
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
      setSelectedTransferEndpoint,
      setCompilePending,
      setScanProjectAutoPreviewEnabled,
      setCenterDragActive,
    },
  };
}

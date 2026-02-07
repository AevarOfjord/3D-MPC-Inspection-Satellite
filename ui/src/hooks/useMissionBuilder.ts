import { useEffect, useState, useRef } from 'react';
import * as THREE from 'three';
import { trajectoryApi, type MeshScanConfig, type ModelInfo } from '../api/trajectory';
import { unifiedMissionApi } from '../api/unifiedMissionApi';
import { pathAssetsApi, type PathAssetSummary } from '../api/pathAssets';
import type {
  MissionSegment,
  TransferSegment,
  ScanSegment,
  HoldSegment,
  ScanConfig,
  SplineControl,
  UnifiedMission,
} from '../api/unifiedMission';
import { useHistory } from './useHistory';
import { resamplePath, downsamplePath } from '../utils/pathResample';
import { telemetry } from '../services/telemetry';
import { orbitSnapshot } from '../data/orbitSnapshot';
import { API_BASE_URL } from '../config/endpoints';
import { useCameraStore } from '../store/cameraStore';
import { ORBIT_SCALE } from '../data/orbitSnapshot';

export type TransformMode = 'translate' | 'rotate';
export type SelectionType = 'satellite' | 'reference' | `obstacle-${number}` | `waypoint-${number}` | `spline-${number}` | null;

const defaultTransferSegment = (): TransferSegment => ({
  type: 'transfer',
  end_pose: { frame: 'ECI', position: [0, 0, 0] },
  constraints: { speed_max: 0.25, accel_max: 0.05, angular_rate_max: 0.1 },
});

const defaultScanConfig = (): ScanConfig => ({
  frame: 'LVLH',
  axis: '+Z',
  standoff: 10,
  overlap: 0.25,
  fov_deg: 60,
  pitch: null,
  revolutions: 4,
  direction: 'CW',
  sensor_axis: '+Y',
  pattern: 'spiral',
});

const defaultScanSegment = (): ScanSegment => ({
  type: 'scan',
  target_id: '',
  scan: defaultScanConfig(),
  constraints: { speed_max: 0.2, accel_max: 0.03, angular_rate_max: 0.08 },
});

const defaultHoldSegment = (): HoldSegment => ({
  type: 'hold',
  duration: 0,
  constraints: { speed_max: 0.1 },
});

const computeFacingEuler = (
  position: [number, number, number],
  baseAxis: [number, number, number] = [0, 0, -1],
  fallback: [number, number, number] = [0, 0, 0]
) => {
  const toEarth = new THREE.Vector3(-position[0], -position[1], -position[2]);
  if (toEarth.lengthSq() < 1e-8) return fallback;
  toEarth.normalize();
  const base = new THREE.Vector3(baseAxis[0], baseAxis[1], baseAxis[2]);
  if (base.lengthSq() < 1e-8) return fallback;
  base.normalize();
  const quat = new THREE.Quaternion().setFromUnitVectors(base, toEarth);
  const euler = new THREE.Euler().setFromQuaternion(quat);
  return [euler.x, euler.y, euler.z] as [number, number, number];
};

const eulerToQuat = (euler: [number, number, number]) => {
  const quat = new THREE.Quaternion().setFromEuler(new THREE.Euler(euler[0], euler[1], euler[2]));
  return [quat.w, quat.x, quat.y, quat.z] as [number, number, number, number];
};

const resolveOrbitTargetPose = (targetId: string) => {
  const obj = orbitSnapshot.objects.find(o => o.id === targetId);
  if (!obj) return undefined;
  const position = [...obj.position_m] as [number, number, number];
  const baseOrientation = obj.orientation ?? [0, 0, 0];
  const euler = obj.align_to_earth
    ? computeFacingEuler(position, obj.base_axis ?? [0, 0, -1], baseOrientation as [number, number, number])
    : (baseOrientation as [number, number, number]);
  const orientation = eulerToQuat(euler);
  return { frame: 'ECI' as const, position, orientation };
};

export function useMissionBuilder() {
  const [modelUrl, setModelUrl] = useState<string | null>(null);
  const [modelPath, setModelPath] = useState<string>('');
  const [loading, setLoading] = useState(false);
  
  // History-managed State for Path (Waypoints)
  const pathHistory = useHistory<[number, number, number][]>([]);
  const previewPath = pathHistory.state; // Alias for convenience
  const [isManualMode, setIsManualMode] = useState(false);

  const [stats, setStats] = useState<{ duration: number; length: number; points: number } | null>(null);
  
  // Mission Config State
  // Mission Config State
  const [startPosition, setStartPosition] = useState<[number, number, number]>([10, 0, 0]);
  const [startFrame, setStartFrame] = useState<'ECI' | 'LVLH'>('ECI');
  const [startTargetId, setStartTargetId] = useState<string | undefined>(undefined);
  const [startAngle, setStartAngle] = useState<[number, number, number]>([0, 0, 0]);
  const [referencePosition, setReferencePosition] = useState<[number, number, number]>([0, 0, 0]);
  const [referenceAngle, setReferenceAngle] = useState<[number, number, number]>([0, 0, 0]);
  const [obstacles, setObstacles] = useState<{ position: [number, number, number]; radius: number }[]>([]);
  
  // Interaction State
  const [selectedObjectId, setSelectedObjectId] = useState<SelectionType>(null);
  const [transformMode, setTransformMode] = useState<TransformMode>('translate');

  // Scan Config State
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
  const [editPointLimit, setEditPointLimit] = useState<number>(500);
  const [savePointMultiplier, setSavePointMultiplier] = useState<number>(10);

  // Unified Mission (V2)
  const [epoch, setEpoch] = useState<string>(new Date().toISOString());
  const [segments, setSegments] = useState<MissionSegment[]>([]);
  const [selectedSegmentIndex, setSelectedSegmentIndex] = useState<number | null>(null);
  const [splineControls, setSplineControls] = useState<SplineControl[]>([]);
  const [savedUnifiedMissions, setSavedUnifiedMissions] = useState<string[]>([]);

  const computePathLength = (path: [number, number, number][]) => {
      if (!path || path.length < 2) return 0;
      let total = 0;
      for (let i = 1; i < path.length; i++) {
          const a = path[i - 1];
          const b = path[i];
          const dx = b[0] - a[0];
          const dy = b[1] - a[1];
          const dz = b[2] - a[2];
          total += Math.sqrt(dx * dx + dy * dy + dz * dz);
      }
      return total;
  };
  const [selectedOrbitTargetId, setSelectedOrbitTargetId] = useState<string | null>(null);

  useEffect(() => {
    if (segments.length === 0) {
      setSegments([defaultScanSegment()]);
    }
  }, [segments.length]);

  useEffect(() => {
    trajectoryApi.listModels().then(setAvailableModels).catch((err) => {
      console.warn('Failed to load model list', err);
    });
    pathAssetsApi.list().then(setPathAssets).catch((err) => {
      console.warn('Failed to load path assets', err);
    });
  }, []);

  useEffect(() => {
    const unsub = telemetry.subscribe((data) => {
      if (isManualMode) return;
      const planned = data.planned_path;
      if (planned && planned.length > 0) {
        pathHistory.set(planned as [number, number, number][]);
      }
    });
    return () => {
      unsub();
    };
  }, [isManualMode, pathHistory]);

  useEffect(() => {
    if (previewPath.length < 2) return;
    const length = previewPath.reduce((acc, point, idx) => {
      if (idx === 0) return acc;
      const prev = previewPath[idx - 1];
      const dx = point[0] - prev[0];
      const dy = point[1] - prev[1];
      const dz = point[2] - prev[2];
      return acc + Math.sqrt(dx * dx + dy * dy + dz * dz);
    }, 0);
    const speed = config.speed_max > 0 ? config.speed_max : 0.1;
    setStats({
      duration: length / speed,
      length,
      points: previewPath.length,
    });
  }, [previewPath, config.speed_max]);

  const prevAxisRef = useRef(config.scan_axis);
  useEffect(() => {
    if (prevAxisRef.current === config.scan_axis) return;
    prevAxisRef.current = config.scan_axis;
    if (!config.obj_path || previewPath.length === 0) return;
    if (loading) return;
    handlePreview().catch((err) => {
      console.warn('Auto preview failed', err);
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [config.scan_axis]);

  // --- Actions ---

  const handleFileUpload = async (file: File) => {
      setLoading(true);
      try {
          const url = URL.createObjectURL(file);
          setModelUrl(url);
          const res = await trajectoryApi.uploadObject(file);
          setModelPath(res.path);
          setConfig(prev => ({ ...prev, obj_path: res.path }));
          trajectoryApi.listModels().then(setAvailableModels).catch(() => null);
      } catch (err) {
          console.error(err);
          alert("Upload failed");
      } finally {
          setLoading(false);
      }
  };

  const handlePreview = async () => {
      if (!config.obj_path) {
          alert("Please upload a model first");
          return;
      }
      setLoading(true);
      try {
          const previewConfig: MeshScanConfig = { ...config };
          if (levelSpacing > 0) {
              previewConfig.level_spacing = levelSpacing;
          }
          const res = await trajectoryApi.previewTrajectory(previewConfig);
          
          // New generation resets manual mode and history
          const editablePath = downsamplePath(res.path, editPointLimit);
          pathHistory.set(editablePath); 
          setIsManualMode(false);
          
          setStats({
              duration: res.estimated_duration,
              length: res.path_length,
              points: editablePath.length
          });
      } catch (err) {
          console.error(err);
          alert("Preview generation failed");
      } finally {
          setLoading(false);
      }
  };

  const selectModelPath = (path: string) => {
      if (!path) return;
      setModelPath(path);
      setConfig(prev => ({ ...prev, obj_path: path }));
      setModelUrl(`${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(path)}`);
      trajectoryApi.getModelBounds(path).then((bounds) => {
          const extent = Math.max(bounds.extents[0], bounds.extents[1], bounds.extents[2]);
          const distance = Math.max(extent * 2.5, 5);
          useCameraStore
              .getState()
              .requestFocus(
                  [bounds.center[0] * ORBIT_SCALE, bounds.center[1] * ORBIT_SCALE, bounds.center[2] * ORBIT_SCALE],
                  distance * ORBIT_SCALE
              );
      }).catch(() => null);
  };

  const refreshModelList = async () => {
      const models = await trajectoryApi.listModels();
      setAvailableModels(models);
      return models;
  };

  const refreshPathAssets = async () => {
      const assets = await pathAssetsApi.list();
      setPathAssets(assets);
      return assets;
  };

  const savePathAsset = async (name: string) => {
      const trimmed = name.trim();
      if (!trimmed) {
          alert('Please enter a path asset name');
          return;
      }
      if (!config.obj_path) {
          alert('Select an OBJ model first');
          return;
      }
      if (previewPath.length === 0) {
          alert('Generate or load a path before saving');
          return;
      }
      const densePath = resamplePath(previewPath, savePointMultiplier);
      const payload = {
          name: trimmed,
          obj_path: config.obj_path,
          path: densePath,
          open: true,
          relative_to_obj: true,
      };
      const saved = await pathAssetsApi.save(payload);
      await refreshPathAssets();
      return saved;
  };

  const loadPathAsset = async (assetId: string) => {
      const asset = await pathAssetsApi.get(assetId);
      if (asset.path && asset.path.length > 0) {
          const editablePath = downsamplePath(asset.path, editPointLimit);
          pathHistory.set(editablePath);
          setIsManualMode(true);
          const speed = config.speed_max > 0 ? config.speed_max : 0.1;
          setStats({
              duration: asset.path_length > 0 ? asset.path_length / speed : 0,
              length: asset.path_length,
              points: editablePath.length,
          });
      }
      if (asset.obj_path) {
          setModelPath(asset.obj_path);
          setConfig(prev => ({ ...prev, obj_path: asset.obj_path }));
          setModelUrl(`${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(asset.obj_path)}`);
      }
      return asset;
  };

  const handleRun = async (onSuccess?: () => void) => {
      // Launch directly to Live Dashboard (Single Process)
      setLoading(true);
      try {
          const validationError = validateScanSegments();
          if (validationError) {
            alert(validationError);
            return;
          }
          // 1. Push current definition to backend
          await pushUnifiedMission();
          
          // 2. Resume simulation
          await unifiedMissionApi.controlSimulation('resume');

          // 3. (Optional) Switch to Monitor mode? 
          // Not strictly required as Viewport handles mode based on parent.
          
          alert("Mission Launched! Switching to Live View.");
          if (onSuccess) onSuccess();
      } catch (err: any) {
          console.error(err);
          alert(`Failed to launch: ${err.message || err}`);
      } finally {
          setLoading(false);
      }
  };

  const addObstacle = (origin?: [number, number, number], offset: [number, number, number] = [5, 0, 0]) => {
      const base = origin ?? [0, 0, 0];
      const position: [number, number, number] = [
        base[0] + offset[0],
        base[1] + offset[1],
        base[2] + offset[2],
      ];
      setObstacles([...obstacles, { position, radius: 0.5 }]);
  };
  
  const removeObstacle = (idx: number) => {
      setObstacles(obstacles.filter((_, i) => i !== idx));
      if (selectedObjectId === `obstacle-${idx}`) setSelectedObjectId(null);
  };

  const updateObstacle = (idx: number, patch: Partial<{position: [number, number, number], radius: number}>) => {
      const newObs = [...obstacles];
      if (patch.position) newObs[idx].position = patch.position;
      if (patch.radius !== undefined) newObs[idx].radius = patch.radius;
      setObstacles(newObs);
  };

  // --- Manual Path Editing ---
  const handleWaypointMove = (idx: number, newPos: [number, number, number]) => {
      if (!isManualMode) setIsManualMode(true);
      if (idx === 0) return;

      if (!previewPath || previewPath.length === 0) return;
      const current = previewPath[idx];
      if (!current) return;
      const delta: [number, number, number] = [
          newPos[0] - current[0],
          newPos[1] - current[1],
          newPos[2] - current[2],
      ];

      // If no change, avoid extra work
      if (Math.abs(delta[0]) < 1e-9 && Math.abs(delta[1]) < 1e-9 && Math.abs(delta[2]) < 1e-9) {
          return;
      }

      const n = previewPath.length;
      if (n < 2) {
          const single = [...previewPath];
          single[idx] = newPos;
          pathHistory.updatePresent(single);
          return;
      }

      // Soft deformation along arc-length so nearby points stretch smoothly.
      // Use a bounded falloff to avoid long-range drift.
      const arc: number[] = new Array(n).fill(0);
      for (let i = 1; i < n; i++) {
          const a = previewPath[i - 1];
          const b = previewPath[i];
          const dx = b[0] - a[0];
          const dy = b[1] - a[1];
          const dz = b[2] - a[2];
          arc[i] = arc[i - 1] + Math.sqrt(dx * dx + dy * dy + dz * dz);
      }
      const totalLength = arc[n - 1];
      const avgSpacing = totalLength / Math.max(1, n - 1);
      const localStart = Math.max(0, idx - 3);
      const localEnd = Math.min(n - 2, idx + 2);
      let localSum = 0;
      let localCount = 0;
      for (let i = localStart; i <= localEnd; i++) {
          const seg = arc[i + 1] - arc[i];
          if (seg > 0) {
              localSum += seg;
              localCount += 1;
          }
      }
      const localSpacing = localCount > 0 ? localSum / localCount : avgSpacing;
      const radius = Math.max((localSpacing || avgSpacing || 1.0) * 6, localSpacing || avgSpacing || 1.0);
      const s0 = arc[idx];

      const nextPath = previewPath.map((p, i) => {
          if (i === 0) {
              return [p[0], p[1], p[2]] as [number, number, number];
          }
          const d = Math.abs(arc[i] - s0);
          const t = radius > 0 ? Math.max(0, 1 - d / radius) : 0;
          const w = t * t;
          return [
              p[0] + delta[0] * w,
              p[1] + delta[1] * w,
              p[2] + delta[2] * w,
          ] as [number, number, number];
      });
      
      // We use updatePresent for drag operations to avoid spamming history
      // Ideally onDragEnd we push to history. 
      // specific implementation triggers set() on drag end, updatePresent on drag
      pathHistory.updatePresent(nextPath);
  };

  const commitWaypointMove = () => {
      // Pushes current state to history stack (checkpoint)
      pathHistory.set([...previewPath]);
  };

  const addWaypoint = () => {
      if (previewPath.length === 0) {
          pathHistory.set([[0, 0, 0]]);
          setIsManualMode(true);
          return;
      }
      const selectedIdx =
          selectedObjectId && selectedObjectId.startsWith('waypoint-')
              ? parseInt(selectedObjectId.split('-')[1], 10)
              : null;
      let insertIndex = previewPath.length;
      let newPoint = previewPath[previewPath.length - 1];
      if (typeof selectedIdx === 'number' && !Number.isNaN(selectedIdx)) {
          if (selectedIdx >= 0 && selectedIdx < previewPath.length - 1) {
              const p0 = previewPath[selectedIdx];
              const p1 = previewPath[selectedIdx + 1];
              newPoint = [
                  (p0[0] + p1[0]) / 2,
                  (p0[1] + p1[1]) / 2,
                  (p0[2] + p1[2]) / 2,
              ];
              insertIndex = selectedIdx + 1;
          } else if (selectedIdx >= 0 && selectedIdx < previewPath.length) {
              newPoint = previewPath[selectedIdx];
              insertIndex = selectedIdx + 1;
          }
      }

      const nextPath = [...previewPath];
      nextPath.splice(insertIndex, 0, newPoint);
      pathHistory.set(nextPath);
      setIsManualMode(true);
  };

  const removeWaypoint = () => {
      if (previewPath.length <= 2) return;
      const selectedIdx =
          selectedObjectId && selectedObjectId.startsWith('waypoint-')
              ? parseInt(selectedObjectId.split('-')[1], 10)
              : null;
      if (typeof selectedIdx !== 'number' || Number.isNaN(selectedIdx)) return;
      if (selectedIdx === 0) return;
      if (selectedIdx < 0 || selectedIdx >= previewPath.length) return;
      const nextPath = previewPath.filter((_, i) => i !== selectedIdx);
      pathHistory.set(nextPath);
      setIsManualMode(true);
      setSelectedObjectId(null);
  };

  const removeWaypointAtIndex = (idx: number) => {
      if (previewPath.length <= 2) return;
      if (!Number.isFinite(idx) || idx < 0 || idx >= previewPath.length) return;
      if (idx === 0) return;
      const nextPath = previewPath.filter((_, i) => i !== idx);
      pathHistory.set(nextPath);
      setIsManualMode(true);
      if (selectedObjectId === `waypoint-${idx}`) {
          setSelectedObjectId(null);
      }
  };

  // --- Transform Helper ---
  
  const handleObjectTransform = (key: string, o: any) => {
      const pos: [number, number, number] = [o.position.x, o.position.y, o.position.z];
      const rot: [number, number, number] = [
          o.rotation.x * (180/Math.PI), 
          o.rotation.y * (180/Math.PI), 
          o.rotation.z * (180/Math.PI)
      ];

      if (key === 'satellite') {
          if(transformMode === 'translate') setStartPosition(pos);
          else setStartAngle(rot);
      } else if (key === 'reference') {
          if(transformMode === 'translate') setReferencePosition(pos);
          else setReferenceAngle(rot);
      } else if (key.startsWith('obstacle-')) {
          const idx = parseInt(key.split('-')[1]);
          const newObs = [...obstacles];
          newObs[idx].position = pos;
          setObstacles(newObs);
      } else if (key.startsWith('waypoint-')) {
          const idx = parseInt(key.split('-')[1]);
          handleWaypointMove(idx, pos);
      }
  };

  // --- Unified Mission Helpers ---

  const buildUnifiedMission = (): UnifiedMission => {
    // Resolve Start Pose
    let resolvedStartPose = {
        frame: startFrame,
        position: [...startPosition] as [number, number, number],
    };
    let resolvedStartTargetId = startTargetId;

    if (startFrame === 'LVLH' && startTargetId) {
         const targetObj = orbitSnapshot.objects.find(o => o.id === startTargetId);
         if (targetObj) {
             const absPos: [number, number, number] = [
                targetObj.position_m[0] + startPosition[0],
                targetObj.position_m[1] + startPosition[1],
                targetObj.position_m[2] + startPosition[2],
            ];
            resolvedStartPose = { frame: 'ECI', position: absPos };
            resolvedStartTargetId = undefined; // Resolved to absolute
         }
    }

    return {
      epoch,
      start_pose: resolvedStartPose,
      start_target_id: resolvedStartTargetId, // Should be undefined if resolved, but kept if ECI? No, clean up.

      segments: segments.map((seg) => {
        // Resolve Relative Transfers to Absolute ECI
        if (seg.type === 'transfer' && seg.end_pose.frame === 'LVLH' && seg.target_id) {
            const targetObj = orbitSnapshot.objects.find(o => o.id === seg.target_id);
            if (targetObj) {
                // Transform: TargetPosition (ECI) + RelativeOffset (LVLH approx as aligned for now, or just ECI offset)
                // Note: True LVLH rotation requires velocity state which we don't strictly have in this static snapshot.
                // For this simplified version (and consistency with Scan segments which often assume aligned frames or point-based),
                // we will treat "LVLH" here as "Relative Position in ECI frame centered on Target".
                // i.e., Absolute = Target + Offset.
                const absPos: [number, number, number] = [
                    targetObj.position_m[0] + seg.end_pose.position[0],
                    targetObj.position_m[1] + seg.end_pose.position[1],
                    targetObj.position_m[2] + seg.end_pose.position[2],
                ];
                
                return {
                    ...seg,
                    end_pose: {
                        frame: 'ECI',
                        position: absPos,
                        orientation: seg.end_pose.orientation
                    },
                    // We remove target_id so backend treats it as standard ECI point-to-point
                    target_id: undefined 
                } as TransferSegment;
            }
        }
        if (seg.type === 'scan' && seg.target_id) {
          const resolvedPose = resolveOrbitTargetPose(seg.target_id);
          if (resolvedPose) {
            return {
              ...seg,
              target_pose: resolvedPose,
            } as ScanSegment;
          }
        }
        return seg;
      }),
      obstacles: obstacles.map((o) => ({
        position: [...o.position] as [number, number, number],
        radius: o.radius,
      })),
      overrides: splineControls.length > 0 ? { spline_controls: splineControls } : undefined,
    };
  };

  const refreshUnifiedMissions = async () => {
    const res = await unifiedMissionApi.listSavedMissions();
    setSavedUnifiedMissions(res.missions);
  };

  const saveUnifiedMission = async (name: string) => {
    const mission = buildUnifiedMission();
    return unifiedMissionApi.saveMission(name, mission);
  };

  const saveEditedPathToAsset = async () => {
    if (!previewPath.length) return;

    const activeIdx =
      typeof selectedSegmentIndex === 'number'
        ? selectedSegmentIndex
        : null;
    let assetId: string | null = null;

    if (activeIdx !== null) {
      const seg = segments[activeIdx];
      if (seg && seg.type === 'scan' && seg.path_asset) {
        assetId = seg.path_asset;
      }
    }
    if (!assetId) {
      const firstScan = segments.find(
        (seg) => seg.type === 'scan' && (seg as ScanSegment).path_asset
      ) as ScanSegment | undefined;
      assetId = firstScan?.path_asset ?? null;
    }
    if (!assetId) return;

    let existing: Awaited<ReturnType<typeof pathAssetsApi.get>> | null = null;
    try {
      existing = await pathAssetsApi.get(assetId);
    } catch {
      existing = null;
    }

    const densePath = resamplePath(previewPath, savePointMultiplier);
    // Auto-rebase to relative if points look like absolute ECI coords.
    let finalPath = densePath;
    let relativeToObj = existing?.relative_to_obj ?? true;
    const maxAbs = densePath.reduce((acc, p) => {
      const v = Math.max(Math.abs(p[0]), Math.abs(p[1]), Math.abs(p[2]));
      return Math.max(acc, v);
    }, 0);
    if (maxAbs > 10000) {
      const activeSeg =
        (typeof selectedSegmentIndex === 'number' && segments[selectedSegmentIndex]) || null;
      const scanSeg =
        (activeSeg && activeSeg.type === 'scan' ? (activeSeg as ScanSegment) : null) ||
        (segments.find((seg) => seg.type === 'scan') as ScanSegment | undefined);
      const origin = scanSeg?.target_pose?.position;
      if (origin) {
        finalPath = densePath.map((p) => [
          p[0] - origin[0],
          p[1] - origin[1],
          p[2] - origin[2],
        ]) as [number, number, number][];
        relativeToObj = true;
      }
    }
    const payload = {
      name: existing?.name ?? assetId,
      obj_path: existing?.obj_path || config.obj_path || '',
      path: finalPath,
      open: existing?.open ?? true,
      relative_to_obj: relativeToObj,
      notes: existing?.notes ?? null,
    };

    await pathAssetsApi.save(payload);
  };

  const handleSaveUnifiedMission = async () => {
    const validationError = validateScanSegments();
    if (validationError) {
      alert(validationError);
      return;
    }
    const name = prompt('Enter mission name (e.g. Starlink_Scan_M01):');
    if (!name) return;
    try {
      await saveEditedPathToAsset();
      await saveUnifiedMission(name);
      await refreshUnifiedMissions();
      alert(`Mission saved: ${name}`);
    } catch (err: any) {
      console.error(err);
      alert(`Failed to save mission: ${err.message || err}`);
    }
  };

  const loadUnifiedMission = async (name: string) => {
    const mission = await unifiedMissionApi.loadMission(name);
    setEpoch(mission.epoch);
    const hydratedSegments = mission.segments.map((seg) => {
      if (seg.type === 'scan' && seg.target_id && !seg.target_pose) {
        const resolvedPose = resolveOrbitTargetPose(seg.target_id);
        if (resolvedPose) {
          return { ...seg, target_pose: resolvedPose } as ScanSegment;
        }
      }
      return seg;
    });
    setSegments(hydratedSegments);
    setSplineControls(mission.overrides?.spline_controls || []);
    setSelectedSegmentIndex(null);
    setStartPosition([...mission.start_pose.position] as [number, number, number]);
    if (mission.obstacles) {
      setObstacles(
        mission.obstacles.map((o) => ({
          position: [...o.position] as [number, number, number],
          radius: o.radius,
        }))
      );
    }
    const firstScan = hydratedSegments.find(seg => seg.type === 'scan') as ScanSegment | undefined;
    setSelectedOrbitTargetId(firstScan?.target_id ?? null);
  };

  const pushUnifiedMission = async () => {
    const mission = buildUnifiedMission();
    return unifiedMissionApi.setMission(mission);
  };

  const generateUnifiedPath = async () => {
    setLoading(true);
    try {
        const validationError = validateScanSegments();
        if (validationError) {
          alert(validationError);
          return;
        }
        setIsManualMode(false);
        const mission = buildUnifiedMission();
        const preview = await unifiedMissionApi.previewMission(mission);
        if (preview.path && preview.path.length > 0) {
          const editablePath = downsamplePath(preview.path, editPointLimit);
          pathHistory.set(editablePath);
          setStats({
            duration: preview.path_speed > 0 ? preview.path_length / preview.path_speed : 0,
            length: preview.path_length,
            points: editablePath.length,
          });
        } else {
          pathHistory.set([]);
          setStats(null);
        }
        return preview;
    } catch (err: any) {
        console.error("Preview Error:", err);
        alert(`Preview Failed: ${err.message || err}`);
        throw err;
    } finally {
        setLoading(false);
    }
  };

  const addTransferSegment = () => {
    setSegments(prev => {
      const next = [...prev, defaultTransferSegment()];
      setSelectedSegmentIndex(next.length - 1);
      return next;
    });
  };

  const addScanSegment = () => {
    setSegments(prev => {
      const next = [...prev, defaultScanSegment()];
      setSelectedSegmentIndex(next.length - 1);
      setSelectedOrbitTargetId(null);
      return next;
    });
  };

  const addHoldSegment = () => {
    setSegments(prev => {
      const next = [...prev, defaultHoldSegment()];
      setSelectedSegmentIndex(next.length - 1);
      return next;
    });
  };

  const removeSegment = (index: number) => {
    setSegments(prev => prev.filter((_, i) => i !== index));
    setSelectedSegmentIndex(prev => {
      if (prev === null) return null;
      if (prev === index) return null;
      if (prev > index) return prev - 1;
      return prev;
    });
  };

  const updateSegment = (index: number, next: MissionSegment) => {
    setSegments(prev => prev.map((seg, i) => (i === index ? next : seg)));
  };

  const applyPathAssetToSegment = (assetId: string, index?: number) => {
    setSegments(prev => {
      let targetIndex = index ?? selectedSegmentIndex ?? -1;
      if (targetIndex < 0 || !prev[targetIndex] || prev[targetIndex].type !== 'scan') {
        targetIndex = prev.findIndex(seg => seg.type === 'scan');
      }
      if (targetIndex < 0) return prev;
      const seg = prev[targetIndex] as ScanSegment;
      const next = prev.map((s, i) =>
        i === targetIndex ? { ...seg, path_asset: assetId } : s
      );
      setSelectedSegmentIndex(targetIndex);
      return next;
    });
  };

  const reorderSegments = (fromIndex: number, toIndex: number) => {
    setSegments(prev => {
        const next = [...prev];
        const [moved] = next.splice(fromIndex, 1);
        next.splice(toIndex, 0, moved);
        return next;
    });
    // Adjust selection if needed
    setSelectedSegmentIndex(prev => {
        if (prev === null) return null;
        if (prev === fromIndex) return toIndex;
        // If we moved something else, and it affected our index
        if (fromIndex < prev && toIndex >= prev) return prev - 1;
        if (fromIndex > prev && toIndex <= prev) return prev + 1;
        return prev;
    });
  };

  const addSplineControl = (position?: [number, number, number]) => {
    const nextControl: SplineControl = {
      position: position ? [...position] as [number, number, number] : [0, 0, 0],
      weight: 1.0
    };
    setSplineControls(prev => [...prev, nextControl]);
  };

  const updateSplineControl = (index: number, next: SplineControl) => {
    setSplineControls(prev => prev.map((c, i) => (i === index ? next : c)));
  };

  const removeSplineControl = (index: number) => {
    setSplineControls(prev => prev.filter((_, i) => i !== index));
  };

  const assignScanTarget = (targetId: string, targetPosition?: [number, number, number]) => {
    setSelectedOrbitTargetId(targetId);
    const resolvedPose = targetId ? resolveOrbitTargetPose(targetId) : undefined;
    setSegments(prev => {
      const applyPrefill = (seg: ScanSegment) => {
        const standoff = seg.scan.standoff > 0 ? seg.scan.standoff : 10;
        const overlap = Number.isFinite(seg.scan.overlap) ? seg.scan.overlap : 0.25;
        const fovDeg = Number.isFinite(seg.scan.fov_deg) ? seg.scan.fov_deg : 60;
        return {
          ...seg,
          target_id: targetId,
          target_pose: resolvedPose ?? (targetPosition
            ? { frame: 'ECI' as const, position: [...targetPosition] as [number, number, number] }
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
          .filter(idx => idx >= 0);
        if (scanIndices.length === 1) {
          targetIndex = scanIndices[0];
        }
      }

      if (targetIndex !== null && targetIndex >= 0) {
        const seg = prev[targetIndex] as ScanSegment;
        const next = prev.map((s, i) =>
          i === targetIndex ? applyPrefill(seg) : s
        );
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
      .filter(({ seg }) => seg.type === 'scan') as { seg: ScanSegment; idx: number }[];
    for (const { seg, idx } of scanSegments) {
      if (!seg.target_id) {
        setSelectedSegmentIndex(idx);
        return 'Scan segment requires a target object. Select one in the Inspector.';
      }
      if (!seg.path_asset) {
        setSelectedSegmentIndex(idx);
        return 'Scan segment requires a saved Path Asset. Create one in Scan Planner.';
      }
    }
    return null;
  };

  useEffect(() => {
    if (previewPath.length === 0) return;
    const nextPath = downsamplePath(previewPath, editPointLimit);
    if (nextPath.length === previewPath.length) return;
    pathHistory.set(nextPath);
    if (stats) {
      setStats({ ...stats, points: nextPath.length });
    }
    setSelectedObjectId(null);
  }, [editPointLimit]);

  useEffect(() => {
    if (!isManualMode) return;
    if (!previewPath || previewPath.length === 0) return;
    const length = computePathLength(previewPath);
    const speed = config.speed_max > 0 ? config.speed_max : 0.1;
    setStats({
      duration: speed > 0 ? length / speed : 0,
      length,
      points: previewPath.length,
    });
  }, [isManualMode, previewPath, config.speed_max]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== 'Delete' && event.key !== 'Backspace') return;
      const active = document.activeElement;
      if (active && (active.tagName === 'INPUT' || active.tagName === 'TEXTAREA')) return;
      if (selectedObjectId && selectedObjectId.startsWith('waypoint-')) {
        event.preventDefault();
        removeWaypoint();
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [selectedObjectId, previewPath]);

  return {
    state: {
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
        config,
        levelSpacing,
        availableModels,
        pathAssets,
        editPointLimit,
        savePointMultiplier,
        epoch,
        segments,
        selectedSegmentIndex,
        splineControls,
        savedUnifiedMissions,
        selectedOrbitTargetId,
        // History State
        canUndo: pathHistory.canUndo,
        canRedo: pathHistory.canRedo
    },
    setters: {
        setStartPosition,
        setStartFrame,
        setStartTargetId,
        setStartAngle,
        setReferencePosition,
        setReferenceAngle,
        setSelectedObjectId,
        setTransformMode,
        setConfig,
        setLevelSpacing,
        setEditPointLimit,
        setSavePointMultiplier,
        setEpoch,
        setSelectedSegmentIndex,
        setSegments
    },
    actions: {
        handleFileUpload,
        handlePreview,
        handleRun,
        handleSaveUnifiedMission,
        selectModelPath,
        refreshModelList,
        refreshPathAssets,
        savePathAsset,
        loadPathAsset,
        addObstacle,
        removeObstacle,
        updateObstacle,
        handleObjectTransform,
        setSelectedObjectId,
        setTransformMode,
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
        refreshUnifiedMissions,
        saveUnifiedMission,
        loadUnifiedMission,
        pushUnifiedMission,
        generateUnifiedPath,
        // History Actions
        undo: pathHistory.undo,
        redo: pathHistory.redo,
        handleWaypointMove,
        commitWaypointMove,
        addWaypoint,
        removeWaypoint,
        removeWaypointAtIndex,
        selectSegment: setSelectedSegmentIndex
    }
  };
}

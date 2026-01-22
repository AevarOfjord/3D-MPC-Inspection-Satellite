import { useEffect, useState } from 'react';
import { trajectoryApi, type MeshScanConfig } from '../api/trajectory';
import { unifiedMissionApi } from '../api/unifiedMissionApi';
import type {
  MissionSegment,
  TransferSegment,
  ScanSegment,
  HoldSegment,
  ScanConfig,
  SplineControl,
} from '../api/unifiedMission';
import { useHistory } from './useHistory';
import { telemetry } from '../services/telemetry';

export type TransformMode = 'translate' | 'rotate';
export type SelectionType = 'satellite' | 'reference' | `obstacle-${number}` | `waypoint-${number}` | null;

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

export function useMissionBuilder() {
  const [modelUrl, setModelUrl] = useState<string | null>(null);
  const [modelPath, setModelPath] = useState<string>('');
  const [loading, setLoading] = useState(false);
  
  // History-managed State for Path (Waypoints)
  const pathHistory = useHistory<[number, number, number][]>([]);
  const previewPath = pathHistory.state; // Alias for convenience
  const [isManualMode, setIsManualMode] = useState(false);

  const [stats, setStats] = useState<{ duration: number; length: number; points: number } | null>(null);
  const [savedMissionName, setSavedMissionName] = useState<string | null>(null);
  
  // Mission Config State
  const [startPosition, setStartPosition] = useState<[number, number, number]>([10, 0, 0]);
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
      scan_axis: 'Z'
  });
  const [levelSpacing, setLevelSpacing] = useState<number>(0.1);

  // Unified Mission (V2)
  const [epoch, setEpoch] = useState<string>(new Date().toISOString());
  const [segments, setSegments] = useState<MissionSegment[]>([]);
  const [selectedSegmentIndex, setSelectedSegmentIndex] = useState<number | null>(null);
  const [splineControls, setSplineControls] = useState<SplineControl[]>([]);
  const [savedUnifiedMissions, setSavedUnifiedMissions] = useState<string[]>([]);
  const [selectedOrbitTargetId, setSelectedOrbitTargetId] = useState<string | null>(null);

  useEffect(() => {
    if (segments.length === 0) {
      setSegments([defaultScanSegment()]);
    }
  }, [segments.length]);

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

  // --- Actions ---

  const handleFileUpload = async (file: File) => {
      setLoading(true);
      try {
          const url = URL.createObjectURL(file);
          setModelUrl(url);
          const res = await trajectoryApi.uploadObject(file);
          setModelPath(res.path);
          setConfig(prev => ({ ...prev, obj_path: res.path }));
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
          const previewConfig = { ...config, level_spacing: levelSpacing };
          const res = await trajectoryApi.previewTrajectory(previewConfig);
          
          // New generation resets manual mode and history
          pathHistory.set(res.path); 
          setIsManualMode(false);
          
          setStats({
              duration: res.estimated_duration,
              length: res.path_length,
              points: res.points
          });
      } catch (err) {
          console.error(err);
          alert("Preview generation failed");
      } finally {
          setLoading(false);
      }
  };

  const handleSave = async () => {
      const name = prompt("Enter mission name (e.g. Scan_Proxima):");
      if (!name) return;
      const missionPayload = {
          start_position: startPosition,
          end_position: referencePosition,
          end_orientation: referenceAngle,
          obstacles: obstacles.map(o => ({ position: o.position, radius: o.radius })),
          mesh_scan: { ...config, level_spacing: levelSpacing },
          // TODO: If isManualMode, we might need a different payload structure or flag
          // keeping as is for now, assuming backend regenerates unless we send explicit points.
          // For now, if manual, we heavily rely on backend accepting 'custom_path' or similar if implemented.
          // Since backend API wasn't changed, we assume saved config regenerates path. 
          // WARNING: Manual edits won't persist to backend re-generation without API change.
          // For this MVP, we will only visualize manual edits locally.
      };
      try {
          const res = await trajectoryApi.saveMission(name, missionPayload);
          setSavedMissionName(name);
          alert(`Mission saved: ${res.filename}`);
      } catch (err) {
          console.error(err);
          alert("Failed to save mission");
      }
  };

  const handleRun = async (onSuccess?: () => void) => {
      if (!savedMissionName) return;
      setLoading(true);
      try {
          const res = await trajectoryApi.runMission(savedMissionName);
          alert(`Mission started! Monitor in Dashboard.\nPID: ${res.pid}`);
          if (onSuccess) onSuccess();
      } catch (err) {
          console.error(err);
          alert(`Failed: ${err}`);
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
      
      const newPath = [...previewPath];
      newPath[idx] = newPos;
      
      // We use updatePresent for drag operations to avoid spamming history
      // Ideally onDragEnd we push to history. 
      // specific implementation triggers set() on drag end, updatePresent on drag
      pathHistory.updatePresent(newPath);
  };

  const commitWaypointMove = () => {
      // Pushes current state to history stack (checkpoint)
      pathHistory.set([...previewPath]);
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

  const buildUnifiedMission = (): UnifiedMission => ({
    epoch,
    start_pose: {
      frame: 'ECI',
      position: [...startPosition],
    },
    segments,
    obstacles: obstacles.map((o) => ({
      position: [...o.position] as [number, number, number],
      radius: o.radius,
    })),
    overrides: splineControls.length > 0 ? { spline_controls: splineControls } : undefined,
  });

  const refreshUnifiedMissions = async () => {
    const res = await unifiedMissionApi.listSavedMissions();
    setSavedUnifiedMissions(res.missions);
  };

  const saveUnifiedMission = async (name: string) => {
    const mission = buildUnifiedMission();
    return unifiedMissionApi.saveMission(name, mission);
  };

  const loadUnifiedMission = async (name: string) => {
    const mission = await unifiedMissionApi.loadMission(name);
    setEpoch(mission.epoch);
    setSegments(mission.segments);
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
    const firstScan = mission.segments.find(seg => seg.type === 'scan') as ScanSegment | undefined;
    setSelectedOrbitTargetId(firstScan?.target_id ?? null);
  };

  const pushUnifiedMission = async () => {
    const mission = buildUnifiedMission();
    return unifiedMissionApi.setMission(mission);
  };

  const generateUnifiedPath = async () => {
    setIsManualMode(false);
    const mission = buildUnifiedMission();
    const preview = await unifiedMissionApi.previewMission(mission);
    if (preview.path && preview.path.length > 0) {
      pathHistory.set(preview.path);
      setStats({
        duration: preview.path_speed > 0 ? preview.path_length / preview.path_speed : 0,
        length: preview.path_length,
        points: preview.path.length,
      });
    } else {
      pathHistory.set([]);
      setStats(null);
    }
    return preview;
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
    setSegments(prev => {
      const applyPrefill = (seg: ScanSegment) => {
        const standoff = seg.scan.standoff > 0 ? seg.scan.standoff : 10;
        const overlap = Number.isFinite(seg.scan.overlap) ? seg.scan.overlap : 0.25;
        const fovDeg = Number.isFinite(seg.scan.fov_deg) ? seg.scan.fov_deg : 60;
        return {
          ...seg,
          target_id: targetId,
          target_pose: targetPosition
            ? { frame: 'ECI', position: [...targetPosition] as [number, number, number] }
            : seg.target_pose,
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

  return {
    state: {
        modelUrl,
        modelPath,
        loading,
        previewPath,
        isManualMode,
        stats,
        savedMissionName,
        startPosition,
        startAngle,
        referencePosition,
        referenceAngle,
        obstacles,
        selectedObjectId,
        transformMode,
        config,
        levelSpacing,
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
        setStartAngle,
        setReferencePosition,
        setReferenceAngle,
        setSelectedObjectId,
        setTransformMode,
        setConfig,
        setLevelSpacing,
        setEpoch,
        setSelectedSegmentIndex,
        setSegments
    },
    actions: {
        handleFileUpload,
        handlePreview,
        handleSave,
        handleRun,
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
        commitWaypointMove
    }
  };
}

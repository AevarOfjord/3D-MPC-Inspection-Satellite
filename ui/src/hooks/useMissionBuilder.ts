import { useState } from 'react';
import { trajectoryApi, type MeshScanConfig } from '../api/trajectory';

export type TransformMode = 'translate' | 'rotate';
export type SelectionType = 'satellite' | 'target' | `obstacle-${number}` | null;

export function useMissionBuilder() {
  const [modelUrl, setModelUrl] = useState<string | null>(null);
  const [modelPath, setModelPath] = useState<string>('');
  const [loading, setLoading] = useState(false);
  const [previewPath, setPreviewPath] = useState<[number, number, number][]>([]);
  const [stats, setStats] = useState<{ duration: number; length: number; points: number } | null>(null);
  const [savedMissionName, setSavedMissionName] = useState<string | null>(null);
  
  // Mission Config State
  const [startPosition, setStartPosition] = useState<[number, number, number]>([10, 0, 0]);
  const [startAngle, setStartAngle] = useState<[number, number, number]>([0, 0, 0]);
  const [objectPosition, setObjectPosition] = useState<[number, number, number]>([0, 0, 0]);
  const [objectAngle, setObjectAngle] = useState<[number, number, number]>([0, 0, 0]);
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
          setPreviewPath(res.path);
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
          target_position: objectPosition,
          target_orientation: objectAngle,
          obstacles: obstacles.map(o => ({ position: o.position, radius: o.radius })),
          mesh_scan: { ...config, level_spacing: levelSpacing }
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

  const addObstacle = () => setObstacles([...obstacles, { position: [5, 0, 0], radius: 0.5 }]);
  
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
      } else if (key === 'target') {
          if(transformMode === 'translate') setObjectPosition(pos);
          else setObjectAngle(rot);
      } else if (key.startsWith('obstacle-')) {
          const idx = parseInt(key.split('-')[1]);
          const newObs = [...obstacles];
          newObs[idx].position = pos;
          setObstacles(newObs);
      }
  };

  return {
    state: {
        modelUrl,
        modelPath,
        loading,
        previewPath,
        stats,
        savedMissionName,
        startPosition,
        startAngle,
        objectPosition,
        objectAngle,
        obstacles,
        selectedObjectId,
        transformMode,
        config,
        levelSpacing
    },
    setters: {
        setStartPosition,
        setStartAngle,
        setObjectPosition,
        setObjectAngle,
        setSelectedObjectId,
        setTransformMode,
        setConfig,
        setLevelSpacing
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
        // Expose setters needed for Viewport actions
        setSelectedObjectId,
        setTransformMode
    }
  };
}

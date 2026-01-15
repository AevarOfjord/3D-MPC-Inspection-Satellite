import { useState, useEffect, useMemo, useRef } from 'react';
import { missionApi, type MissionConfig } from '../api/mission';
import { useMissionStore } from '../store/missionStore';
import { Settings, Play, Plus, Trash2 } from 'lucide-react';
import { useUiStore } from '../store/uiStore';
import { useCameraStore } from '../store/cameraStore';
import { useTelemetryStore } from '../store/telemetryStore';

interface MissionWaypoint {
  position: [number, number, number];
  orientationDeg: [number, number, number];
}

const missionOptions: Array<{
  id: string;
  label: string;
  waypoints: MissionWaypoint[];
}> = [
  {
    id: 'ptp_x',
    label: 'Point: (0,0,0) → (1,0,0)',
    waypoints: [
      { position: [0, 0, 0], orientationDeg: [0, 0, 0] },
      { position: [1, 0, 0], orientationDeg: [0, 0, 0] },
    ],
  },
  {
    id: 'ptp_diag',
    label: 'Point: (0,0,0) → (1,1,1) @ 45°',
    waypoints: [
      { position: [0, 0, 0], orientationDeg: [0, 0, 0] },
      { position: [1, 1, 1], orientationDeg: [45, 45, 45] },
    ],
  },
  {
    id: 'square_flat',
    label: 'Square (z=0): (1,-1) → (1,1) → (-1,1) → (-1,-1)',
    waypoints: [
      { position: [1, -1, 0], orientationDeg: [0, 0, 0] },
      { position: [1, 1, 0], orientationDeg: [0, 0, 0] },
      { position: [-1, 1, 0], orientationDeg: [0, 0, 0] },
      { position: [-1, -1, 0], orientationDeg: [0, 0, 0] },
    ],
  },
  {
    id: 'square_mix',
    label: 'Square (z mix): (1,-1,1) → (1,1,0) → (-1,1,1) → (-1,-1,0)',
    waypoints: [
      { position: [1, -1, 1], orientationDeg: [0, 0, 0] },
      { position: [1, 1, 0], orientationDeg: [0, 0, 0] },
      { position: [-1, 1, 1], orientationDeg: [0, 0, 0] },
      { position: [-1, -1, 0], orientationDeg: [0, 0, 0] },
    ],
  },
];

const degToRad = (deg: number) => (deg * Math.PI) / 180;

export function Editor() {
  const [isOpen, setIsOpen] = useState(false);
  const [loading, setLoading] = useState(false);
  const [presets, setPresets] = useState<{ name: string; config: MissionConfig }[]>([]);
  const [presetName, setPresetName] = useState('');
  const [selectedPreset, setSelectedPreset] = useState<string>('');
  const [selectedMissionOption, setSelectedMissionOption] = useState<string>('');
  const [sequenceQueue, setSequenceQueue] = useState<MissionWaypoint[]>([]);
  const [sequenceRunning, setSequenceRunning] = useState(false);
  const advancingRef = useRef(false);
  const fileInputRef = useRef<HTMLInputElement | null>(null);
  
  const { 
      config, 
      updateStartPos,
      updateTargetPos,
      updateTargetOri,
      addObstacle, 
      removeObstacle, 
      updateObstacle,
      setEditing,
      setConfig,
      lastRunConfig,
      setLastRunConfig,
      selectedObstacleIndex,
      setSelectedObstacleIndex,
      transformAxis,
      setTransformAxis,
      transformSnap,
      setTransformSnap
  } = useMissionStore();
  const setMissionError = useUiStore(s => s.setMissionError);
  const requestFocus = useCameraStore(s => s.requestFocus);
  const latest = useTelemetryStore(s => s.latest);

  const warnings = useMemo(() => {
    const issues: string[] = [];
    const start = config.start_position;
    const target = config.target_position;

    config.obstacles.forEach((obs, i) => {
      if (obs.radius <= 0) {
        issues.push(`Obstacle ${i + 1}: radius must be > 0`);
      }
      const dxs = obs.position[0] - start[0];
      const dys = obs.position[1] - start[1];
      const dzs = obs.position[2] - start[2];
      const distStart = Math.sqrt(dxs * dxs + dys * dys + dzs * dzs);
      if (distStart < obs.radius) {
        issues.push(`Obstacle ${i + 1}: start position inside obstacle`);
      }
      const dxt = obs.position[0] - target[0];
      const dyt = obs.position[1] - target[1];
      const dzt = obs.position[2] - target[2];
      const distTarget = Math.sqrt(dxt * dxt + dyt * dyt + dzt * dzt);
      if (distTarget < obs.radius) {
        issues.push(`Obstacle ${i + 1}: target position inside obstacle`);
      }
    });

    for (let i = 0; i < config.obstacles.length; i += 1) {
      for (let j = i + 1; j < config.obstacles.length; j += 1) {
        const a = config.obstacles[i];
        const b = config.obstacles[j];
        const dx = a.position[0] - b.position[0];
        const dy = a.position[1] - b.position[1];
        const dz = a.position[2] - b.position[2];
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < a.radius + b.radius) {
          issues.push(`Obstacles ${i + 1} and ${j + 1} overlap`);
        }
      }
    }

    return issues;
  }, [config]);

  const selectedOption = useMemo(
    () => missionOptions.find(option => option.id === selectedMissionOption) || null,
    [selectedMissionOption]
  );

  useEffect(() => {
    const raw = localStorage.getItem('mission_presets_v1');
    if (raw) {
      try {
        setPresets(JSON.parse(raw));
      } catch {
        setPresets([]);
      }
    }
  }, []);

  useEffect(() => {
    if (!sequenceRunning || !latest) return;
    if (sequenceQueue.length === 0) {
      setSequenceRunning(false);
      return;
    }
    if (advancingRef.current) return;

    const posOk = (latest.pos_error ?? 0) <= 0.1;
    const angOk = ((latest.ang_error ?? 0) * (180 / Math.PI)) <= 2.0;
    if (!posOk || !angOk) return;

    const next = sequenceQueue[0];
    const rest = sequenceQueue.slice(1);
    const nextConfig: MissionConfig = {
      ...config,
      start_position: [...latest.position] as [number, number, number],
      target_position: [...next.position],
      target_orientation: next.orientationDeg.map(degToRad) as [number, number, number],
    };

    advancingRef.current = true;
    missionApi.updateMission(nextConfig)
      .then(() => {
        setConfig(nextConfig);
        setSequenceQueue(rest);
        setSequenceRunning(rest.length > 0);
      })
      .catch((error) => {
        console.error(error);
        setMissionError('Failed to advance mission sequence');
        setSequenceRunning(false);
      })
      .finally(() => {
        advancingRef.current = false;
      });
  }, [config, latest, sequenceQueue, sequenceRunning, setConfig, setMissionError]);

  const persistPresets = (next: { name: string; config: MissionConfig }[]) => {
    setPresets(next);
    localStorage.setItem('mission_presets_v1', JSON.stringify(next));
  };

  const handleSavePreset = () => {
    const name = presetName.trim();
    if (!name) return;
    const existing = presets.filter(p => p.name !== name);
    const next = [...existing, { name, config }];
    persistPresets(next);
    setSelectedPreset(name);
  };

  const handleLoadPreset = () => {
    const preset = presets.find(p => p.name === selectedPreset);
    if (preset) {
      setConfig(preset.config);
    }
  };

  const handleDeletePreset = () => {
    if (!selectedPreset) return;
    const next = presets.filter(p => p.name !== selectedPreset);
    persistPresets(next);
    setSelectedPreset('');
  };

  const handleExport = () => {
    const blob = new Blob([JSON.stringify(config, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'mission-config.json';
    link.click();
    URL.revokeObjectURL(url);
  };

  const handleImportClick = () => {
    fileInputRef.current?.click();
  };

  const handleImport = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;
    try {
      const text = await file.text();
      const parsed = JSON.parse(text);
      setConfig(parsed);
    } catch (error) {
      setMissionError('Failed to import mission config');
      console.error(error);
    } finally {
      event.target.value = '';
    }
  };

  // Sync isEditing with isOpen
  useEffect(() => {
      setEditing(isOpen);
  }, [isOpen, setEditing]);

  const handleSubmit = async () => {
    setLoading(true);
    try {
      await missionApi.updateMission(config);
      setLastRunConfig(config);
      setMissionError(null);
      if (selectedOption) {
        const queue = selectedOption.waypoints.slice(2);
        setSequenceQueue(queue);
        setSequenceRunning(queue.length > 0);
      } else {
        setSequenceQueue([]);
        setSequenceRunning(false);
      }
    } catch (error) {
      console.error(error);
      setMissionError('Failed to update mission');
    } finally {
      setLoading(false);
    }
  };

  const handleMissionOptionApply = (optionId: string) => {
    if (!optionId) {
      setSelectedMissionOption('');
      setSequenceQueue([]);
      setSequenceRunning(false);
      return;
    }
    const option = missionOptions.find(item => item.id === optionId);
    if (!option) return;
    const start = option.waypoints[0];
    const target = option.waypoints[1];
    const nextConfig: MissionConfig = {
      ...config,
      start_position: [...start.position],
      target_position: [...target.position],
      target_orientation: target.orientationDeg.map(degToRad) as [number, number, number],
    };
    setConfig(nextConfig);
    setSelectedMissionOption(optionId);
    setSequenceQueue([]);
    setSequenceRunning(false);
  };

  return (
    <>
      {/* Toggle Button */}
      <button 
        onClick={() => setIsOpen(!isOpen)}
        className="absolute top-4 right-4 z-20 p-2 bg-gray-800 rounded-lg hover:bg-gray-700 transition-colors border border-gray-600"
      >
        <Settings size={20} className={isOpen ? "text-blue-400" : "text-white"} />
      </button>

      {/* Editor Sidebar */}
      <div className={`absolute top-0 right-0 h-full w-80 bg-gray-900/95 backdrop-blur-md border-l border-gray-700 transform transition-transform duration-300 z-10 flex flex-col ${isOpen ? 'translate-x-0' : 'translate-x-full'}`}>
        <div className="p-4 border-b border-gray-700 flex justify-between items-center mt-12">
           <h2 className="font-bold text-lg">Mission Editor</h2>
        </div>

        <div className="flex-1 overflow-y-auto p-4 space-y-6">
           {warnings.length > 0 && (
             <div className="bg-yellow-500/10 border border-yellow-600/40 text-yellow-200 text-xs rounded-lg p-3 space-y-1">
               <div className="font-semibold uppercase tracking-wider text-[10px]">Validation Warnings</div>
               {warnings.map((warning) => (
                 <div key={warning}>{warning}</div>
               ))}
             </div>
           )}

           {/* Presets */}
           <div className="space-y-2">
              <label className="text-xs font-semibold text-gray-400 uppercase tracking-wider">Mission Presets</label>
              <div className="flex gap-2">
                 <input
                   type="text"
                   value={presetName}
                   onChange={(e) => setPresetName(e.target.value)}
                   placeholder="Preset name"
                   className="flex-1 bg-gray-800 border border-gray-700 rounded p-1 text-sm text-center focus:border-blue-500 outline-none"
                 />
                 <button
                   onClick={handleSavePreset}
                   className="px-2 py-1 text-xs bg-gray-800 hover:bg-gray-700 rounded border border-gray-600"
                 >
                   Save
                 </button>
              </div>
              <div className="flex gap-2">
                 <select
                   value={selectedPreset}
                   onChange={(e) => setSelectedPreset(e.target.value)}
                   className="flex-1 bg-gray-800 border border-gray-700 rounded p-1 text-sm focus:border-blue-500 outline-none"
                 >
                   <option value="">Select preset</option>
                   {presets.map((preset) => (
                     <option key={preset.name} value={preset.name}>{preset.name}</option>
                   ))}
                 </select>
                 <button
                   onClick={handleLoadPreset}
                   className="px-2 py-1 text-xs bg-gray-800 hover:bg-gray-700 rounded border border-gray-600"
                 >
                   Load
                 </button>
                 <button
                   onClick={handleDeletePreset}
                   className="px-2 py-1 text-xs bg-gray-800 hover:bg-gray-700 rounded border border-gray-600"
                 >
                   Delete
                 </button>
              </div>
              <div className="flex gap-2">
                 <button
                   onClick={handleExport}
                   className="flex-1 px-2 py-1 text-xs bg-gray-800 hover:bg-gray-700 rounded border border-gray-600"
                 >
                   Export JSON
                 </button>
                 <button
                   onClick={handleImportClick}
                   className="flex-1 px-2 py-1 text-xs bg-gray-800 hover:bg-gray-700 rounded border border-gray-600"
                 >
                   Import JSON
                 </button>
                 <input
                   ref={fileInputRef}
                   type="file"
                   accept="application/json"
                   onChange={handleImport}
                   className="hidden"
                 />
              </div>
           </div>

           {/* Mission Options */}
           <div className="space-y-2">
             <label className="text-xs font-semibold text-gray-400 uppercase tracking-wider">Mission Options</label>
             <div className="flex gap-2">
               <select
                 value={selectedMissionOption}
                 onChange={(e) => handleMissionOptionApply(e.target.value)}
                 className="flex-1 bg-gray-800 border border-gray-700 rounded p-1 text-sm focus:border-blue-500 outline-none"
               >
                 <option value="">Select mission</option>
                 {missionOptions.map(option => (
                   <option key={option.id} value={option.id}>{option.label}</option>
                 ))}
               </select>
             </div>
             {selectedOption && (
               <div className="text-[10px] text-gray-500">
                 {selectedOption.waypoints.length} waypoints loaded.
               </div>
             )}
             {sequenceRunning && (
               <div className="text-[10px] text-blue-300">
                 Mission sequence active: {sequenceQueue.length + 1} remaining.
               </div>
             )}
           </div>

           {/* Start Position */}
           <div className="space-y-2">
              <label className="text-xs font-semibold text-gray-400 uppercase tracking-wider">Start Position (m)</label>
              <div className="grid grid-cols-3 gap-2">
                 {['x', 'y', 'z'].map((label, i) => (
                    <div key={label} className="flex flex-col">
                       <input 
                         type="number" 
                         step="0.1"
                         value={config.start_position[i]}
                         onChange={(e) => updateStartPos(i, parseFloat(e.target.value) || 0)}
                         className="bg-gray-800 border border-gray-700 rounded p-1 text-sm text-center focus:border-blue-500 outline-none"
                       />
                       <span className="text-[10px] text-gray-500 text-center mt-1 uppercase">{label}</span>
                    </div>
                 ))}
              </div>
           </div>

           {/* Target Position */}
           <div className="space-y-2">
              <label className="text-xs font-semibold text-gray-400 uppercase tracking-wider">Target Position (m)</label>
              <div className="grid grid-cols-3 gap-2">
                 {['x', 'y', 'z'].map((label, i) => (
                    <div key={label} className="flex flex-col">
                       <input 
                         type="number" 
                         step="0.1"
                         value={config.target_position[i]}
                         onChange={(e) => updateTargetPos(i, parseFloat(e.target.value) || 0)}
                         className="bg-gray-800 border border-gray-700 rounded p-1 text-sm text-center focus:border-blue-500 outline-none"
                       />
                       <span className="text-[10px] text-gray-500 text-center mt-1 uppercase">{label}</span>
                    </div>
                 ))}
              </div>
           </div>

           {/* Target Orientation */}
           <div className="space-y-2">
              <label className="text-xs font-semibold text-gray-400 uppercase tracking-wider">Target Rotation (deg)</label>
              <div className="grid grid-cols-3 gap-2">
                 {['r', 'p', 'y'].map((label, i) => (
                    <div key={label} className="flex flex-col">
                       <input 
                         type="number" 
                         step="1"
                         value={Math.round(config.target_orientation[i] * (180/Math.PI))}
                         onChange={(e) => {
                             const deg = parseFloat(e.target.value) || 0;
                             updateTargetOri(i, deg * (Math.PI/180));
                         }}
                         className="bg-gray-800 border border-gray-700 rounded p-1 text-sm text-center focus:border-blue-500 outline-none"
                       />
                       <span className="text-[10px] text-gray-500 text-center mt-1 uppercase">{label === 'r' ? 'Roll' : label === 'p' ? 'Pitch' : 'Yaw'}</span>
                    </div>
                 ))}
              </div>
           </div>

           {/* Obstacles */}
           <div className="space-y-4">
              <div className="flex justify-between items-center">
                <label className="text-xs font-semibold text-gray-400 uppercase tracking-wider">Obstacles</label>
                <button 
                  onClick={addObstacle}
                  className="p-1 bg-gray-800 hover:bg-gray-700 rounded border border-gray-600"
                  title="Add Obstacle"
                >
                   <Plus size={14} />
                </button>
              </div>

              <div className="flex flex-wrap gap-2">
                <div className="text-[10px] text-gray-500 uppercase tracking-wider self-center">Axis Lock</div>
                {(['free', 'x', 'y', 'z'] as const).map(axis => (
                  <button
                    key={axis}
                    onClick={() => setTransformAxis(axis)}
                    className={`px-2 py-1 text-[10px] uppercase rounded border ${
                      transformAxis === axis ? 'border-blue-500 text-blue-300' : 'border-gray-700 text-gray-400'
                    }`}
                  >
                    {axis}
                  </button>
                ))}
                <div className="ml-auto flex items-center gap-2">
                  <span className="text-[10px] text-gray-500 uppercase tracking-wider">Snap</span>
                  <button
                    onClick={() => setTransformSnap(transformSnap ? null : 0.5)}
                    className={`px-2 py-1 text-[10px] uppercase rounded border ${
                      transformSnap ? 'border-green-500 text-green-300' : 'border-gray-700 text-gray-400'
                    }`}
                  >
                    {transformSnap ? `${transformSnap}m` : 'off'}
                  </button>
                </div>
              </div>
              
              <div className="space-y-3">
                 {config.obstacles.map((obs, i) => (
                    <div
                      key={i}
                      className={`bg-gray-800/50 p-3 rounded-lg border relative group cursor-pointer ${
                        selectedObstacleIndex === i ? 'border-blue-500/80 shadow-[0_0_0_1px_rgba(59,130,246,0.4)]' : 'border-gray-700'
                      }`}
                      onClick={() => {
                        setSelectedObstacleIndex(i);
                        requestFocus(obs.position);
                      }}
                    >
                       <button 
                         onClick={(event) => {
                           event.stopPropagation();
                           removeObstacle(i);
                         }}
                         className="absolute top-2 right-2 text-gray-500 hover:text-red-400 opacity-0 group-hover:opacity-100 transition-opacity"
                       >
                          <Trash2 size={14} />
                       </button>
                       
                       <div className="text-xs text-blue-400 mb-2 font-mono">#{i+1} SPHERE</div>
                       
                       <div className="grid grid-cols-2 gap-2 mb-2">
                          <div className="col-span-2">
                             <label className="text-[10px] text-gray-500">Position (x,y,z)</label>
                             <div className="grid grid-cols-3 gap-1">
                                {[0,1,2].map(idx => (
                                   <input 
                                     key={idx}
                                     type="number"
                                     step="0.5"
                                     value={obs.position[idx]}
                                     onChange={(e) => updateObstacle(i, 'position', parseFloat(e.target.value) || 0, idx)}
                                     className="bg-gray-900 border border-gray-700 rounded px-1 py-0.5 text-xs w-full"
                                   />
                                ))}
                             </div>
                          </div>
                       </div>
                       
                       <div>
                          <label className="text-[10px] text-gray-500">Radius (m)</label>
                          <input 
                            type="number"
                            step="0.1"
                            value={obs.radius}
                            onChange={(e) => updateObstacle(i, 'radius', parseFloat(e.target.value) || 0)}
                            className="bg-gray-900 border border-gray-700 rounded px-1 py-0.5 text-xs w-full"
                          />
                       </div>
                    </div>
                 ))}
                 {config.obstacles.length === 0 && (
                    <div className="text-sm text-gray-500 text-center py-4 italic">No obstacles</div>
                 )}
              </div>
           </div>
        </div>

        <div className="p-4 border-t border-gray-700 bg-gray-900 space-y-2">
           <button
             onClick={() => lastRunConfig && setConfig(lastRunConfig)}
             disabled={!lastRunConfig}
             className="w-full bg-gray-800 hover:bg-gray-700 disabled:opacity-50 disabled:cursor-not-allowed text-white font-semibold py-2 px-4 rounded-lg transition-colors"
           >
             Reset to Last Run
           </button>
           <button
             onClick={handleSubmit}
             disabled={loading}
             className="w-full bg-blue-600 hover:bg-blue-500 disabled:opacity-50 disabled:cursor-not-allowed text-white font-semibold py-2 px-4 rounded-lg flex items-center justify-center gap-2 transition-colors"
           >
             {loading ? <div className="w-4 h-4 border-2 border-white/30 border-t-white rounded-full animate-spin" /> : <Play size={16} fill="currentColor" />}
             Run Simulation
           </button>
        </div>
      </div>
    </>
  );
}

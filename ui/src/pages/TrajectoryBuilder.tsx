import { useState, Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Line, TransformControls } from '@react-three/drei';
import { trajectoryApi, type MeshScanConfig } from '../api/trajectory';
import { useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import * as THREE from 'three';
import { HudPanel, HudButton, HudInput, HudSection } from '../components/HudComponents';
import { Play, Save, Eye, MousePointer2, Move3d, Rotate3d, Plus, Trash2, Upload } from 'lucide-react';

// --- 3D Components ---

function Model({ url }: { url: string }) {
  const obj = useLoader(OBJLoader, url);
  return <primitive object={obj} />;
}

function TrajectoryPath({ 
    points, 
    onHover 
}: { 
    points: [number, number, number][]; 
    onHover: (point: [number, number, number] | null) => void;
}) {
    const [highlightPoint, setHighlightPoint] = useState<[number, number, number] | null>(null);
    
    if (!points || points.length === 0) return null;
    const vectors = points.map(p => new THREE.Vector3(...p));
    const curve = new THREE.CatmullRomCurve3(vectors);
    
    const findClosestPoint = (hitPoint: THREE.Vector3): [number, number, number] => {
        let closestDist = Infinity;
        let closestPoint = points[0];
        for (const p of points) {
            const dist = hitPoint.distanceTo(new THREE.Vector3(...p));
            if (dist < closestDist) {
                closestDist = dist;
                closestPoint = p;
            }
        }
        return closestPoint;
    };
    
    const handlePointerMove = (e: any) => {
        e.stopPropagation();
        const closest = findClosestPoint(e.point);
        setHighlightPoint(closest);
        onHover(closest);
    };
    
    const handlePointerOut = () => {
        setHighlightPoint(null);
        onHover(null);
    };
    
    return (
        <group>
            <Line points={vectors} color="#06b6d4" lineWidth={2} transparent opacity={0.6} />
            <mesh 
                onPointerMove={handlePointerMove}
                onPointerOut={handlePointerOut}
                onPointerDown={(e) => e.stopPropagation()}
                onClick={(e) => e.stopPropagation()}
            >
                <tubeGeometry args={[curve, 64, 0.2, 8, false]} />
                <meshBasicMaterial transparent opacity={0} />
            </mesh>
            {highlightPoint && (
                <mesh position={highlightPoint}>
                    <sphereGeometry args={[0.1, 16, 16]} />
                    <meshBasicMaterial color="#22d3ee" toneMapped={false} />
                    <pointLight color="#22d3ee" intensity={2} distance={2} />
                </mesh>
            )}
        </group>
    );
}

function SatellitePreview({ position, rotation }: { position: [number, number, number]; rotation: [number, number, number] }) {
    const euler = new THREE.Euler(
        (rotation[0] * Math.PI) / 180,
        (rotation[1] * Math.PI) / 180,
        (rotation[2] * Math.PI) / 180
    );
    return (
        <group position={position} rotation={euler}>
            <mesh>
                <boxGeometry args={[0.5, 0.5, 0.5]} />
                <meshStandardMaterial color="#fbbf24" metalness={0.8} roughness={0.2} />
            </mesh>
            <axesHelper args={[1]} />
        </group>
    );
}

// --- Main Builder Component ---

export function TrajectoryBuilder({ onBack }: { onBack: () => void }) {
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
  const [selectedObjectId, setSelectedObjectId] = useState<string | null>(null); // 'satellite', 'target', 'obstacle-{i}'
  const [transformMode, setTransformMode] = useState<'translate' | 'rotate'>('translate');
  const [hoveredPoint, setHoveredPoint] = useState<[number, number, number] | null>(null);

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

  // --- Handlers ---

  const handleFileUpload = async (e: React.ChangeEvent<HTMLInputElement>) => {
      const file = e.target.files?.[0];
      if (!file) return;
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

  const handleRun = async () => {
      if (!savedMissionName) return;
      setLoading(true);
      try {
          const res = await trajectoryApi.runMission(savedMissionName);
          alert(`Mission started! Monitor in Dashboard.\nPID: ${res.pid}`);
          onBack(); // Go back to dashboard
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

  // --- Transform Handlers ---
  
  const handleObjectTransform = (key: string, e: any) => {
      // e.target.object is the transformed mesh
      if(!e.target?.object) return;
      const o = e.target.object;
      
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

  return (
    <div className="flex h-screen bg-slate-950 text-white overflow-hidden">
      
      {/* 3D Viewport */}
      <div className="flex-1 relative cursor-crosshair">
          <Canvas camera={{ position: [8, 8, 8], fov: 50 }}>
            <color attach="background" args={['#050510']} />
            <fog attach="fog" args={['#050510', 10, 50]} />
            <ambientLight intensity={0.5} />
            <pointLight position={[10, 10, 10]} intensity={1} />
            
            <OrbitControls makeDefault enabled={!selectedObjectId} />
            <Grid infiniteGrid sectionColor="#444" cellColor="#222" fadeDistance={30} />

            <Suspense fallback={null}>
                <group>
                    {/* Satellite */}
                    <TransformControls 
                        object={undefined} 
                        mode={transformMode}
                        enabled={selectedObjectId === 'satellite'}
                        showX={selectedObjectId === 'satellite'}
                        showY={selectedObjectId === 'satellite'}
                        showZ={selectedObjectId === 'satellite'}
                        onObjectChange={(e) => handleObjectTransform('satellite', e)}
                    >
                        <group onClick={(e) => { e.stopPropagation(); setSelectedObjectId('satellite'); }}>
                            <SatellitePreview position={startPosition} rotation={startAngle} />
                        </group>
                    </TransformControls>

                    {/* Target Object */}
                    <TransformControls
                        mode={transformMode}
                        enabled={selectedObjectId === 'target'}
                        showX={selectedObjectId === 'target'}
                        showY={selectedObjectId === 'target'}
                        showZ={selectedObjectId === 'target'}
                        onObjectChange={(e) => handleObjectTransform('target', e)}
                    >
                         <group 
                            position={objectPosition} 
                            rotation={[objectAngle[0]*Math.PI/180, objectAngle[1]*Math.PI/180, objectAngle[2]*Math.PI/180]}
                            onClick={(e) => { e.stopPropagation(); setSelectedObjectId('target'); }}
                         >
                            {modelUrl ? <Model url={modelUrl} /> : (
                                <mesh>
                                    <boxGeometry args={[1, 1, 1]} />
                                    <meshStandardMaterial color="#64748b" wireframe />
                                </mesh>
                            )}
                            <axesHelper args={[2]} />
                         </group>
                    </TransformControls>

                    {/* Obstacles */}
                    {obstacles.map((obs, i) => (
                        <TransformControls
                            key={i}
                            mode="translate" // Obstacles usually just positional
                            enabled={selectedObjectId === `obstacle-${i}`}
                            showX={selectedObjectId === `obstacle-${i}`}
                            showY={selectedObjectId === `obstacle-${i}`}
                            showZ={selectedObjectId === `obstacle-${i}`}
                            onObjectChange={(e) => handleObjectTransform(`obstacle-${i}`, e)}
                        >
                            <mesh 
                                position={obs.position} 
                                onClick={(e) => { e.stopPropagation(); setSelectedObjectId(`obstacle-${i}`); }}
                            >
                                <sphereGeometry args={[obs.radius, 16, 16]} />
                                <meshStandardMaterial color="#ef4444" transparent opacity={0.4} wireframe />
                            </mesh>
                        </TransformControls>
                    ))}

                    <TrajectoryPath points={previewPath} onHover={setHoveredPoint} />
                </group>
            </Suspense>
          </Canvas>
          
          {/* Overlay Controls */}
          <div className="absolute top-4 left-4 bg-slate-900/80 backdrop-blur rounded p-2 flex gap-2 border border-slate-700">
             <button 
                onClick={() => setTransformMode('translate')}
                className={`p-2 rounded ${transformMode === 'translate' ? 'bg-cyan-500/20 text-cyan-400' : 'text-slate-400 hover:text-white'}`}
                title="Move (Translate)"
             >
                <Move3d size={20} />
             </button>
             <button 
                onClick={() => setTransformMode('rotate')}
                className={`p-2 rounded ${transformMode === 'rotate' ? 'bg-cyan-500/20 text-cyan-400' : 'text-slate-400 hover:text-white'}`}
                title="Rotate"
             >
                <Rotate3d size={20} />
             </button>
             <div className="w-px bg-slate-700 mx-1" />
             <button 
                onClick={() => setSelectedObjectId(null)}
                className="p-2 text-slate-400 hover:text-white"
                title="Deselect All"
             >
                <MousePointer2 size={20} />
             </button>
          </div>

          {hoveredPoint && (
            <div className="absolute bottom-4 left-4 pointer-events-none">
                <HudPanel className="text-xs font-mono">
                    <div className="text-cyan-400 font-bold mb-1">WAYPOINT</div>
                    <div>X: {hoveredPoint[0].toFixed(2)}</div>
                    <div>Y: {hoveredPoint[1].toFixed(2)}</div>
                    <div>Z: {hoveredPoint[2].toFixed(2)}</div>
                </HudPanel>
            </div>
          )}
      </div>

      {/* Sidebar */}
      <div className="w-96 bg-slate-950 border-l border-slate-800 flex flex-col shadow-2xl z-10">
         <div className="p-4 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
             <h2 className="font-bold text-lg tracking-wider text-white">MISSION EDITOR</h2>
             <HudButton size="sm" variant="ghost" onClick={onBack}>EXIT</HudButton>
         </div>
         
         <div className="flex-1 overflow-y-auto p-4 space-y-4 custom-scrollbar">
             
             {/* Object Upload */}
             <div className="border border-dashed border-slate-700 rounded-lg p-6 text-center hover:border-cyan-500/50 hover:bg-cyan-500/5 transition-all group">
                 <input type="file" onChange={handleFileUpload} accept=".obj" className="hidden" id="obj-upload"/>
                 <label htmlFor="obj-upload" className="cursor-pointer block">
                     <Upload className="mx-auto mb-2 text-slate-500 group-hover:text-cyan-400" />
                     <span className="text-sm font-bold text-slate-400 group-hover:text-cyan-200">UPLOAD TARGET (.OBJ)</span>
                 </label>
             </div>
             {modelPath && (
                 <div className="text-xs bg-green-500/10 text-green-400 px-2 py-1 rounded border border-green-500/20 truncate font-mono">
                    LOADED: {modelPath.split('/').pop()}
                 </div>
             )}

             <HudSection title="Mission Config" defaultOpen={true}>
                 <div className="space-y-4">
                     <div>
                         <div className="text-xs text-cyan-400 font-bold mb-2 flex items-center gap-2">
                             <div className="w-2 h-2 bg-cyan-500 rounded-full" /> SATELLITE START
                         </div>
                         <div className="pl-2 border-l border-slate-800 space-y-2">
                             <Vec3Input label="Offset" value={startPosition} onChange={setStartPosition} />
                             <Vec3Input label="Rotation" value={startAngle} onChange={setStartAngle} unit="°" />
                         </div>
                     </div>
                     <div>
                         <div className="text-xs text-cyan-400 font-bold mb-2 flex items-center gap-2">
                             <div className="w-2 h-2 bg-slate-500 rounded-full" /> TARGET OBJECT
                         </div>
                         <div className="pl-2 border-l border-slate-800 space-y-2">
                             <Vec3Input label="Position" value={objectPosition} onChange={setObjectPosition} />
                             <Vec3Input label="Rotation" value={objectAngle} onChange={setObjectAngle} unit="°" />
                         </div>
                     </div>
                 </div>
             </HudSection>

             <HudSection title="Scanner Settings">
                 <div className="grid grid-cols-2 gap-3">
                     <HudInput label="Standoff (m)" value={config.standoff} type="number" step={0.1} onChange={v => setConfig({...config, standoff: v})} />
                     <HudInput label="Level Spacing (m)" value={levelSpacing} type="number" step={0.05} onChange={setLevelSpacing} />
                     <HudInput label="Points/Ring" value={config.points_per_circle} type="number" step={4} onChange={v => setConfig({...config, points_per_circle: v})} />
                     <HudInput label="Speed (m/s)" value={config.speed_max} type="number" step={0.05} onChange={v => setConfig({...config, speed_max: v})} />
                 </div>
                 <div className="mt-3">
                     <label className="text-[10px] font-bold uppercase text-slate-400 block mb-1">Scan Axis</label>
                     <div className="flex bg-slate-900 rounded p-1 border border-slate-700">
                         {['X', 'Y', 'Z'].map(axis => (
                             <button
                                key={axis}
                                onClick={() => setConfig({...config, scan_axis: axis as any})}
                                className={`flex-1 py-1 text-xs font-bold rounded ${config.scan_axis === axis ? 'bg-cyan-500/20 text-cyan-400' : 'text-slate-500 hover:text-white'}`}
                             >
                                 {axis}
                             </button>
                         ))}
                     </div>
                 </div>
             </HudSection>

             <HudSection title="Obstacles">
                 <div className="space-y-3">
                     {obstacles.map((obs, i) => (
                         <div key={i} className={`bg-slate-900/50 p-2 rounded border ${selectedObjectId === `obstacle-${i}` ? 'border-cyan-500/50' : 'border-slate-800'}`}>
                             <div className="flex justify-between items-center mb-2">
                                 <span className="text-xs font-bold text-slate-400">OBSTACLE {i+1}</span>
                                 <button onClick={() => removeObstacle(i)} className="text-red-400 hover:text-red-300"><Trash2 size={14} /></button>
                             </div>
                             <Vec3Input label="Pos" value={obs.position} onChange={v => {
                                 const newObs = [...obstacles];
                                 newObs[i].position = v;
                                 setObstacles(newObs);
                             }} />
                             <div className="mt-2">
                                 <HudInput label="Radius" value={obs.radius} type="number" step={0.1} onChange={v => {
                                     const newObs = [...obstacles];
                                     newObs[i].radius = v;
                                     setObstacles(newObs);
                                 }} />
                             </div>
                         </div>
                     ))}
                     <HudButton variant="secondary" size="sm" onClick={addObstacle} className="w-full">
                         <Plus size={14} /> ADD OBSTACLE
                     </HudButton>
                 </div>
             </HudSection>
         </div>

         <div className="p-4 bg-slate-900/80 border-t border-slate-800 space-y-2">
             <HudButton variant="secondary" size="md" className="w-full" onClick={handlePreview} disabled={!modelPath || loading}>
                 <Eye size={16} /> {loading ? "PROCESSING..." : "PREVIEW TRAJECTORY"}
             </HudButton>

             {stats && (
                 <div className="grid grid-cols-3 gap-1 text-[10px] font-mono text-cyan-300 bg-cyan-950/30 p-2 rounded border border-cyan-900/50">
                     <div className="text-center">PTS: {stats.points}</div>
                     <div className="text-center">LEN: {stats.length.toFixed(1)}m</div>
                     <div className="text-center">TIME: {stats.duration.toFixed(1)}s</div>
                 </div>
             )}
             
             <div className="flex gap-2">
                 <HudButton variant="primary" className="flex-1" onClick={handleSave} disabled={!stats}>
                     <Save size={16} /> SAVE
                 </HudButton>
                 <HudButton variant="danger" className="flex-1" onClick={handleRun} disabled={!savedMissionName}>
                     <Play size={16} /> RUN
                 </HudButton>
             </div>
         </div>
      </div>
    </div>
  );
}

// --- Helpers ---
function Vec3Input({ label, value, onChange, unit }: { label: string, value: [number, number, number], onChange: (v: [number, number, number]) => void, unit?: string }) {
    return (
        <div className="space-y-1">
            <div className="flex justify-between">
                <span className="text-[10px] font-bold text-slate-500 uppercase">{label}</span>
                {unit && <span className="text-[10px] text-slate-600">{unit}</span>}
            </div>
            <div className="grid grid-cols-3 gap-1">
                <HudInput value={value[0]} type="number" step={0.1} onChange={v => onChange([v, value[1], value[2]])} />
                <HudInput value={value[1]} type="number" step={0.1} onChange={v => onChange([value[0], v, value[2]])} />
                <HudInput value={value[2]} type="number" step={0.1} onChange={v => onChange([value[0], value[1], v])} />
            </div>
        </div>
    );
}

import { useState, Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Line, Bounds } from '@react-three/drei';
import { trajectoryApi, type MeshScanConfig } from '../api/trajectory';
import { useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import * as THREE from 'three';

// Component to render uploaded OBJ
function Model({ url }: { url: string }) {
  const obj = useLoader(OBJLoader, url);
  return <primitive object={obj} />;
}

// Component to render the path with hover detection
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
    
    // Create a curve for the tube
    const curve = new THREE.CatmullRomCurve3(vectors);
    
    // Find closest point on path to a given 3D position
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
            {/* Visible path line */}
            <Line points={vectors} color="cyan" lineWidth={2} />
            
            {/* Invisible thick tube for hover detection */}
            <mesh 
                onPointerMove={handlePointerMove}
                onPointerOut={handlePointerOut}
                onPointerDown={(e) => e.stopPropagation()}
                onClick={(e) => e.stopPropagation()}
            >
                <tubeGeometry args={[curve, 64, 0.1, 8, false]} />
                <meshBasicMaterial transparent opacity={0} />
            </mesh>

            
            {/* Highlight marker at closest point */}
            {highlightPoint && (
                <mesh position={highlightPoint}>
                    <sphereGeometry args={[0.05, 16, 16]} />
                    <meshBasicMaterial color="red" />
                </mesh>
            )}
        </group>
    );
}


// Component to render obstacles
function Obstacles({ obstacles }: { obstacles: { position: [number, number, number]; radius: number }[] }) {
    return (
        <>
            {obstacles.map((obs, i) => (
                <mesh key={i} position={obs.position}>
                    <sphereGeometry args={[obs.radius, 16, 16]} />
                    <meshStandardMaterial color="red" transparent opacity={0.5} wireframe />
                </mesh>
            ))}
        </>
    );
}

// Component to show satellite at start position
function SatellitePreview({ position, angle }: { position: [number, number, number]; angle: [number, number, number] }) {
    // Convert degrees to radians
    const rotation: [number, number, number] = [
        (angle[0] * Math.PI) / 180,
        (angle[1] * Math.PI) / 180,
        (angle[2] * Math.PI) / 180
    ];
    
    return (
        <group position={position} rotation={rotation}>
            {/* Main Bus - Gold Foil */}
            <mesh>
                <boxGeometry args={[0.3, 0.3, 0.3]} />
                <meshStandardMaterial color="#D4AF37" metalness={0.7} roughness={0.3} />
            </mesh>
            {/* Axes helper */}
            <axesHelper args={[0.5]} />
        </group>
    );
}

// Collapsible Section Component
function Section({ title, children, defaultOpen = true }: { title: string; children: React.ReactNode; defaultOpen?: boolean }) {
    const [isOpen, setIsOpen] = useState(defaultOpen);
    return (
        <div className="border border-gray-700 rounded-lg overflow-hidden">
            <button 
                onClick={() => setIsOpen(!isOpen)}
                className="w-full px-3 py-2 bg-gray-800 text-left text-sm font-bold uppercase text-gray-400 hover:bg-gray-750 flex justify-between items-center"
            >
                {title}
                <span>{isOpen ? '▼' : '▶'}</span>
            </button>
            {isOpen && <div className="p-3 space-y-3">{children}</div>}
        </div>
    );
}

// Vector3 Input Component
function Vec3Input({ label, value, onChange, unit = "m" }: { 
    label: string; 
    value: [number, number, number]; 
    onChange: (v: [number, number, number]) => void;
    unit?: string;
}) {
    return (
        <div>
            <label className="text-xs uppercase text-gray-500 font-bold block mb-1">{label} ({unit})</label>
            <div className="grid grid-cols-3 gap-1">
                {['X', 'Y', 'Z'].map((axis, i) => (
                    <input
                        key={axis}
                        type="number"
                        step="0.1"
                        value={value[i]}
                        onChange={e => {
                            const newVal = [...value] as [number, number, number];
                            newVal[i] = parseFloat(e.target.value) || 0;
                            onChange(newVal);
                        }}
                        placeholder={axis}
                        className="bg-gray-900 border border-gray-700 rounded px-2 py-1 text-sm text-center"
                    />
                ))}
            </div>
        </div>
    );
}

// Number Input Component
function NumberInput({ label, value, onChange, step = 0.1, min, max, unit }: {
    label: string;
    value: number;
    onChange: (v: number) => void;
    step?: number;
    min?: number;
    max?: number;
    unit?: string;
}) {
    return (
        <div>
            <label className="text-xs uppercase text-gray-500 font-bold">{label}{unit ? ` (${unit})` : ''}</label>
            <input 
                type="number" 
                step={step} 
                min={min}
                max={max}
                value={value} 
                onChange={e => onChange(parseFloat(e.target.value) || 0)}
                className="w-full bg-gray-900 border border-gray-700 rounded px-2 py-1 text-sm"
            />
        </div>
    );
}

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
  
  // Level Spacing mode (instead of count)
  const [levelSpacing, setLevelSpacing] = useState<number>(0.1); // meters between levels

  const handleFileUpload = async (e: React.ChangeEvent<HTMLInputElement>) => {
      const file = e.target.files?.[0];
      if (!file) return;

      setLoading(true);
      try {
          const url = URL.createObjectURL(file);
          setModelUrl(url);

          const res = await trajectoryApi.uploadObject(file);
          console.log("Uploaded:", res);
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
          // Use level_spacing if provided (API will compute levels)
          const previewConfig = {
              ...config,
              level_spacing: levelSpacing
          };
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
      const name = prompt("Enter mission name (e.g. Scan_Satellite_01):");
      if (!name) return;

      const missionPayload = {
          start_position: startPosition,
          target_position: objectPosition,
          target_orientation: objectAngle,
          obstacles: obstacles.map(o => ({ position: o.position, radius: o.radius })),
          mesh_scan: {
              ...config,
              level_spacing: levelSpacing
          }
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
      if (!savedMissionName) {
          alert("Please save the mission first");
          return;
      }
      
      setLoading(true);
      try {
          const res = await trajectoryApi.runMission(savedMissionName);
          alert(`Mission started!\n\nPID: ${res.pid}\nFile: ${res.mission_file}\n\nThe simulation is running in the background. Check the terminal for progress.`);
      } catch (err) {
          console.error(err);
          alert(`Failed to run mission: ${err}`);
      } finally {
          setLoading(false);
      }
  };


  const addObstacle = () => {
      setObstacles([...obstacles, { position: [0, 0, 0], radius: 0.5 }]);
  };

  const removeObstacle = (index: number) => {
      setObstacles(obstacles.filter((_, i) => i !== index));
  };

  const updateObstacle = (index: number, field: 'position' | 'radius', value: [number, number, number] | number) => {
      const newObstacles = [...obstacles];
      if (field === 'position') {
          newObstacles[index].position = value as [number, number, number];
      } else {
          newObstacles[index].radius = value as number;
      }
      setObstacles(newObstacles);
  };

  return (
    <div className="flex h-screen bg-gray-900 text-white">
      {/* Sidebar */}
      <div className="w-96 bg-gray-800 p-4 border-r border-gray-700 flex flex-col gap-3 overflow-y-auto">
        <div className="flex items-center gap-2 mb-2">
             <button onClick={onBack} className="text-gray-400 hover:text-white">← Back</button>
             <h2 className="font-bold text-lg">Mission Control</h2>
        </div>

        {/* Object Upload */}
        <div className="border-2 border-dashed border-gray-600 rounded-lg p-3 text-center hover:border-blue-500 transition-colors">
            <input type="file" onChange={handleFileUpload} accept=".obj" className="hidden" id="obj-upload"/>
            <label htmlFor="obj-upload" className="cursor-pointer block text-sm">
                {loading ? "Uploading..." : "📦 Upload Target Object (.OBJ)"}
            </label>
        </div>
        {modelPath && <div className="text-xs text-green-400 truncate">✓ {modelPath.split('/').pop()}</div>}

        {/* Satellite Start */}
        <Section title="Satellite Start" defaultOpen={true}>
            <Vec3Input label="Position" value={startPosition} onChange={setStartPosition} unit="m" />
            <Vec3Input label="Orientation" value={startAngle} onChange={setStartAngle} unit="deg" />
        </Section>

        {/* Target Object */}
        <Section title="Target Object" defaultOpen={true}>
            <Vec3Input label="Position" value={objectPosition} onChange={setObjectPosition} unit="m" />
            <Vec3Input label="Orientation" value={objectAngle} onChange={setObjectAngle} unit="deg" />
        </Section>

        {/* Scan Parameters */}
        <Section title="Scan Parameters" defaultOpen={true}>
            <NumberInput label="Standoff Distance" value={config.standoff} onChange={v => setConfig({...config, standoff: v})} unit="m" step={0.1} />
            <NumberInput label="Level Spacing" value={levelSpacing} onChange={setLevelSpacing} unit="m" step={0.05} />
            <NumberInput label="Points per Ring" value={config.points_per_circle} onChange={v => setConfig({...config, points_per_circle: v})} step={1} />
            <NumberInput label="Travel Speed" value={config.speed_max} onChange={v => setConfig({...config, speed_max: v, speed_min: v * 0.25})} unit="m/s" step={0.01} />

            <div>
               <label className="text-xs uppercase text-gray-500 font-bold">Scan Axis</label>
               <select value={config.scan_axis || "Z"}
                       onChange={e => setConfig({...config, scan_axis: e.target.value as 'X'|'Y'|'Z'})}
                       className="w-full bg-gray-900 border border-gray-700 rounded px-2 py-1 text-sm">
                   <option value="X">X Axis</option>
                   <option value="Y">Y Axis</option>
                   <option value="Z">Z Axis</option>
               </select>
            </div>
        </Section>

        {/* Obstacles */}
        <Section title="Obstacles" defaultOpen={false}>
            {obstacles.map((obs, i) => (
                <div key={i} className="bg-gray-900 p-2 rounded space-y-2">
                    <div className="flex justify-between items-center">
                        <span className="text-xs text-gray-400">Obstacle {i + 1}</span>
                        <button onClick={() => removeObstacle(i)} className="text-red-400 text-xs hover:text-red-300">Remove</button>
                    </div>
                    <Vec3Input label="Position" value={obs.position} onChange={v => updateObstacle(i, 'position', v)} />
                    <NumberInput label="Radius" value={obs.radius} onChange={v => updateObstacle(i, 'radius', v)} unit="m" step={0.1} />
                </div>
            ))}
            <button onClick={addObstacle} className="w-full py-2 border border-dashed border-gray-600 rounded text-gray-400 hover:text-white hover:border-gray-500 text-sm">
                + Add Obstacle
            </button>
        </Section>

        {/* Actions */}
        <div className="space-y-2 mt-auto pt-4 border-t border-gray-700">
            <button onClick={handlePreview} disabled={!modelPath || loading}
                className="w-full bg-blue-600 hover:bg-blue-500 text-white font-bold py-2 rounded disabled:opacity-50">
                {loading ? "Processing..." : "👁 Preview Trajectory"}
            </button>

            {stats && (
                <div className="bg-gray-900 p-2 rounded text-xs space-y-1 font-mono">
                    <div className="flex justify-between"><span>Points:</span> <span className="text-blue-400">{stats.points}</span></div>
                    <div className="flex justify-between"><span>Length:</span> <span className="text-blue-400">{stats.length.toFixed(2)}m</span></div>
                    <div className="flex justify-between"><span>Est. Time:</span> <span className="text-blue-400">{stats.duration.toFixed(1)}s</span></div>
                </div>
            )}

            <button onClick={handleSave} disabled={!stats}
                className="w-full bg-green-700 hover:bg-green-600 text-white font-bold py-2 rounded disabled:opacity-50">
                💾 Save Mission
            </button>
            
            <button onClick={handleRun} disabled={!savedMissionName}
                className="w-full bg-orange-600 hover:bg-orange-500 text-white font-bold py-2 rounded disabled:opacity-50">
                🚀 Run Mission
            </button>
        </div>
      </div>

      {/* 3D Viewport */}
      <div className="flex-1 relative bg-black">
         <Canvas camera={{ position: [5, 5, 5], fov: 50 }}>
            <color attach="background" args={['#111']} />
            <ambientLight intensity={0.5} />
            <pointLight position={[10, 10, 10]} intensity={1} />
            <OrbitControls makeDefault />
            <Grid infiniteGrid sectionColor="#444" cellColor="#222" fadeDistance={30} />
             
             <Suspense fallback={null}>
                <Bounds fit clip observe margin={1.2}>
                   <group>
                      <SatellitePreview position={startPosition} angle={startAngle} />
                      {modelUrl && <Model url={modelUrl} />}
                      <TrajectoryPath points={previewPath} onHover={setHoveredPoint} />
                      <Obstacles obstacles={obstacles} />
                   </group>
                </Bounds>

             </Suspense>
          </Canvas>
         
         <div className="absolute top-4 left-4 pointer-events-none">
            <h1 className="text-xl font-bold opacity-50">MISSION CONTROL</h1>
         </div>
         
         {/* Hover Tooltip */}
         {hoveredPoint && (
            <div className="absolute bottom-4 left-4 bg-gray-900/90 border border-cyan-500 rounded px-3 py-2 font-mono text-sm pointer-events-none">
               <div className="text-cyan-400 font-bold mb-1">Position</div>
               <div className="grid grid-cols-3 gap-3">
                  <span>X: <span className="text-white">{hoveredPoint[0].toFixed(3)}m</span></span>
                  <span>Y: <span className="text-white">{hoveredPoint[1].toFixed(3)}m</span></span>
                  <span>Z: <span className="text-white">{hoveredPoint[2].toFixed(3)}m</span></span>
               </div>
            </div>
         )}
      </div>

    </div>
  );
}

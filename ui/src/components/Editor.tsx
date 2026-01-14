import { useState } from 'react';
import { missionApi } from '../api/mission';
import type { MissionConfig, Obstacle } from '../api/mission';
import { Settings, Play, Plus, Trash2 } from 'lucide-react';

export function Editor() {
  const [isOpen, setIsOpen] = useState(false);
  const [config, setConfig] = useState<MissionConfig>({
    start_position: [10.0, 0.0, 0.0],
    obstacles: []
  });
  const [loading, setLoading] = useState(false);

  const handleStartPosChange = (index: number, value: string) => {
    const newPos = [...config.start_position] as [number, number, number];
    newPos[index] = parseFloat(value) || 0;
    setConfig({ ...config, start_position: newPos });
  };

  const addObstacle = () => {
    setConfig({
      ...config,
      obstacles: [...config.obstacles, { position: [5.0, 0.0, 0.0], radius: 1.0 }]
    });
  };

  const removeObstacle = (index: number) => {
    setConfig({
      ...config,
      obstacles: config.obstacles.filter((_, i) => i !== index)
    });
  };

  const updateObstacle = (index: number, field: keyof Obstacle, value: any, subIndex?: number) => {
    const newObstacles = [...config.obstacles];
    if (field === 'position' && typeof subIndex === 'number') {
        newObstacles[index].position[subIndex] = parseFloat(value) || 0;
    } else if (field === 'radius') {
        newObstacles[index].radius = parseFloat(value) || 0;
    }
    setConfig({ ...config, obstacles: newObstacles });
  };

  const handleSubmit = async () => {
    setLoading(true);
    try {
      await missionApi.updateMission(config);
      // Optional: Add toast notification
    } catch (error) {
      console.error(error);
      alert('Failed to update mission');
    } finally {
      setLoading(false);
    }
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
                         onChange={(e) => handleStartPosChange(i, e.target.value)}
                         className="bg-gray-800 border border-gray-700 rounded p-1 text-sm text-center focus:border-blue-500 outline-none"
                       />
                       <span className="text-[10px] text-gray-500 text-center mt-1 uppercase">{label}</span>
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
              
              <div className="space-y-3">
                 {config.obstacles.map((obs, i) => (
                    <div key={i} className="bg-gray-800/50 p-3 rounded-lg border border-gray-700 relative group">
                       <button 
                         onClick={() => removeObstacle(i)}
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
                                     onChange={(e) => updateObstacle(i, 'position', e.target.value, idx)}
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
                            onChange={(e) => updateObstacle(i, 'radius', e.target.value)}
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

        <div className="p-4 border-t border-gray-700 bg-gray-900">
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

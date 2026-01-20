import { Upload, Trash2, Plus, Eye, Save, Play } from 'lucide-react';
import { HudButton, HudInput, HudSection } from './HudComponents';
import type { useMissionBuilder } from '../hooks/useMissionBuilder';

type BuilderHook = ReturnType<typeof useMissionBuilder>;

interface BuilderSidebarProps {
    builder: BuilderHook;
    onExit?: () => void;
}

export function BuilderSidebar({ builder, onExit }: BuilderSidebarProps) {
    const { state, setters, actions } = builder;
    
    // Helper handlers
    const onFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.files?.[0]) actions.handleFileUpload(e.target.files[0]);
    };

    return (
        <div className="w-96 bg-slate-950 border-r border-slate-800 flex flex-col shadow-2xl z-20 h-full">
            <div className="p-4 border-b border-slate-800 flex justify-between items-center bg-slate-900/50">
                <h2 className="font-bold text-lg tracking-wider text-white">MISSION EDITOR</h2>
            </div>
            
            <div className="flex-1 overflow-y-auto p-4 space-y-4 custom-scrollbar">
                
                {/* Object Upload */}
                <div className="border border-dashed border-slate-700 rounded-lg p-6 text-center hover:border-cyan-500/50 hover:bg-cyan-500/5 transition-all group">
                    <input type="file" onChange={onFileChange} accept=".obj" className="hidden" id="obj-upload"/>
                    <label htmlFor="obj-upload" className="cursor-pointer block">
                        <Upload className="mx-auto mb-2 text-slate-500 group-hover:text-cyan-400" />
                        <span className="text-sm font-bold text-slate-400 group-hover:text-cyan-200">UPLOAD TARGET (.OBJ)</span>
                    </label>
                </div>
                {state.modelPath && (
                    <div className="text-xs bg-green-500/10 text-green-400 px-2 py-1 rounded border border-green-500/20 truncate font-mono">
                       LOADED: {state.modelPath.split('/').pop()}
                    </div>
                )}

                <HudSection title="Mission Config" defaultOpen={true}>
                    <div className="space-y-4">
                        <div>
                            <div className="text-xs text-cyan-400 font-bold mb-2 flex items-center gap-2">
                                <div className="w-2 h-2 bg-cyan-500 rounded-full" /> SATELLITE START
                            </div>
                            <div className="pl-2 border-l border-slate-800 space-y-2">
                                <Vec3Input label="Offset" value={state.startPosition} onChange={setters.setStartPosition} />
                                <Vec3Input label="Rotation" value={state.startAngle} onChange={setters.setStartAngle} unit="°" />
                            </div>
                        </div>
                        <div>
                            <div className="text-xs text-cyan-400 font-bold mb-2 flex items-center gap-2">
                                <div className="w-2 h-2 bg-slate-500 rounded-full" /> TARGET OBJECT
                            </div>
                            <div className="pl-2 border-l border-slate-800 space-y-2">
                                <Vec3Input label="Position" value={state.objectPosition} onChange={setters.setObjectPosition} />
                                <Vec3Input label="Rotation" value={state.objectAngle} onChange={setters.setObjectAngle} unit="°" />
                            </div>
                        </div>
                    </div>
                </HudSection>

                <HudSection title="Scanner Settings">
                    <div className="grid grid-cols-2 gap-3">
                        <HudInput label="Standoff (m)" value={state.config.standoff} type="number" step={0.1} onChange={v => setters.setConfig({...state.config, standoff: v})} />
                        <HudInput label="Level Spacing (m)" value={state.levelSpacing} type="number" step={0.05} onChange={setters.setLevelSpacing} />
                        <HudInput label="Points/Ring" value={state.config.points_per_circle} type="number" step={4} onChange={v => setters.setConfig({...state.config, points_per_circle: v})} />
                        <HudInput label="Speed (m/s)" value={state.config.speed_max} type="number" step={0.05} onChange={v => setters.setConfig({...state.config, speed_max: v})} />
                    </div>
                    <div className="mt-3">
                        <label className="text-[10px] font-bold uppercase text-slate-400 block mb-1">Scan Axis</label>
                        <div className="flex bg-slate-900 rounded p-1 border border-slate-700">
                            {['X', 'Y', 'Z'].map(axis => (
                                <button
                                   key={axis}
                                   onClick={() => setters.setConfig({...state.config, scan_axis: axis as any})}
                                   className={`flex-1 py-1 text-xs font-bold rounded ${state.config.scan_axis === axis ? 'bg-cyan-500/20 text-cyan-400' : 'text-slate-500 hover:text-white'}`}
                                >
                                    {axis}
                                </button>
                            ))}
                        </div>
                    </div>
                </HudSection>

                <HudSection title="Obstacles">
                    <div className="space-y-3">
                        {state.obstacles.map((obs, i) => (
                            <div key={i} className={`bg-slate-900/50 p-2 rounded border ${state.selectedObjectId === `obstacle-${i}` ? 'border-cyan-500/50' : 'border-slate-800'}`}>
                                <div className="flex justify-between items-center mb-2">
                                    <span className="text-xs font-bold text-slate-400">OBSTACLE {i+1}</span>
                                    <button onClick={() => actions.removeObstacle(i)} className="text-red-400 hover:text-red-300"><Trash2 size={14} /></button>
                                </div>
                                <Vec3Input label="Pos" value={obs.position} onChange={v => actions.updateObstacle(i, { position: v })} />
                                <div className="mt-2">
                                    <HudInput label="Radius" value={obs.radius} type="number" step={0.1} onChange={v => actions.updateObstacle(i, { radius: v })} />
                                </div>
                            </div>
                        ))}
                        <HudButton variant="secondary" size="sm" onClick={actions.addObstacle} className="w-full">
                            <Plus size={14} /> ADD OBSTACLE
                        </HudButton>
                    </div>
                </HudSection>
            </div>

            <div className="p-4 bg-slate-900/80 border-t border-slate-800 space-y-2">
                <HudButton variant="secondary" size="md" className="w-full" onClick={actions.handlePreview} disabled={!state.modelPath || state.loading}>
                    <Eye size={16} /> {state.loading ? "PROCESSING..." : "PREVIEW TRAJECTORY"}
                </HudButton>

                {state.stats && (
                    <div className="grid grid-cols-3 gap-1 text-[10px] font-mono text-cyan-300 bg-cyan-950/30 p-2 rounded border border-cyan-900/50">
                        <div className="text-center">PTS: {state.stats.points}</div>
                        <div className="text-center">LEN: {state.stats.length.toFixed(1)}m</div>
                        <div className="text-center">TIME: {state.stats.duration.toFixed(1)}s</div>
                    </div>
                )}
                
                <div className="flex gap-2">
                    <HudButton variant="primary" className="flex-1" onClick={actions.handleSave} disabled={!state.stats}>
                        <Save size={16} /> SAVE
                    </HudButton>
                    <HudButton variant="danger" className="flex-1" onClick={() => actions.handleRun(onExit)} disabled={!state.savedMissionName}>
                        <Play size={16} /> RUN
                    </HudButton>
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

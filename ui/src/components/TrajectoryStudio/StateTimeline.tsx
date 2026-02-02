import React, { useMemo } from 'react';
import { Activity, Clock, Zap } from 'lucide-react';
import { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { ResponsiveContainer, AreaChart, Area, XAxis, YAxis, Tooltip, ReferenceLine } from 'recharts';

interface StateTimelineProps {
    builder: ReturnType<typeof useMissionBuilder>;
}

export function StateTimeline({ builder }: StateTimelineProps) {
    const { state } = builder;
    
    // Convert path points to chart data
    // Assuming uniform time steps for now or using index as proxy if time not available
    // Ideally we getting full trajectory state with timestamps.
    const chartData = useMemo(() => {
        if (!state.previewPath) return [];
        return state.previewPath.map((p, i) => {
            // Mock velocity magnitude if not available (delta from previous)
            const prev = state.previewPath[Math.max(0, i - 1)];
            const dist = Math.sqrt(
                Math.pow(p[0] - prev[0], 2) + 
                Math.pow(p[1] - prev[1], 2) + 
                Math.pow(p[2] - prev[2], 2)
            );
            // Assuming simplified DT=1 for visualization if real DT unknown, 
            // but effectively this visualizes "Step Magnitude".
            return {
                index: i,
                velocity: dist * 10, // scaling for viz
                x: p[0],
                y: p[1],
                z: p[2]
            };
        });
    }, [state.previewPath]);

    const stats = state.stats;

    return (
        <div className="h-full bg-slate-950/90 border-t border-slate-800 flex flex-col">
            {/* Toolbar */}
            <div className="h-8 border-b border-slate-800 flex items-center px-4 gap-6 bg-slate-900/50">
                <div className="flex items-center gap-2 text-xs font-mono text-slate-400">
                    <Clock size={12} className="text-cyan-500" />
                    <span>DURATION: <span className="text-slate-200">{stats?.duration.toFixed(1) || '0.0'}s</span></span>
                </div>
                <div className="flex items-center gap-2 text-xs font-mono text-slate-400">
                    <Activity size={12} className="text-green-500" />
                    <span>LENGTH: <span className="text-slate-200">{stats?.length.toFixed(1) || '0.0'}m</span></span>
                </div>
                <div className="flex items-center gap-2 text-xs font-mono text-slate-400">
                    <Zap size={12} className="text-orange-500" />
                    <span>POINTS: <span className="text-slate-200">{stats?.points || '0'}</span></span>
                </div>
            </div>

            {/* Chart Area */}
            <div className="flex-1 relative w-full">
                <ResponsiveContainer width="100%" height="100%">
                    <AreaChart data={chartData} margin={{ top: 5, right: 0, left: 0, bottom: 0 }}>
                        <defs>
                            <linearGradient id="colorVel" x1="0" y1="0" x2="0" y2="1">
                                <stop offset="5%" stopColor="#06b6d4" stopOpacity={0.3} />
                                <stop offset="95%" stopColor="#06b6d4" stopOpacity={0} />
                            </linearGradient>
                        </defs>
                        <Tooltip 
                            contentStyle={{ backgroundColor: '#0f172a', borderColor: '#1e293b', fontSize: '12px' }}
                            itemStyle={{ color: '#94a3b8' }}
                            labelStyle={{ color: '#cbd5e1' }}
                        />
                        <Area 
                            type="monotone" 
                            dataKey="velocity" 
                            stroke="#06b6d4" 
                            fillOpacity={1} 
                            fill="url(#colorVel)" 
                            strokeWidth={2}
                        />
                        {/* We could add reference lines for segments here */}
                    </AreaChart>
                </ResponsiveContainer>
                
                {chartData.length === 0 && (
                    <div className="absolute inset-0 flex items-center justify-center text-slate-600 font-mono text-sm pointer-events-none">
                        NO TRAJECTORY DATA
                    </div>
                )}
            </div>
        </div>
    );
}

import { useEffect, useState, useRef } from 'react';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, CartesianGrid } from 'recharts';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { Activity } from 'lucide-react';
import { useMissionStore } from '../store/missionStore';

const MAX_HISTORY = 300; // Keep 300 points. At 0.1s interval = 30 seconds history.

interface ChartDataPoint {
  time: number;
  posError: number;
  angError: number;
  velocity: number;
  solveTime: number;
}

export function TelemetryCharts() {
  const [history, setHistory] = useState<ChartDataPoint[]>([]);
  const lastTimeRef = useRef<number>(0);
  const isEditing = useMissionStore(s => s.isEditing);

  useEffect(() => {
    const unsubscribe = telemetry.subscribe((data) => {
      // Throttle updates to ~10Hz (every 0.1s sim time)
      if (Math.abs(data.time - lastTimeRef.current) < 0.1) return;
      lastTimeRef.current = data.time;

      setHistory(prev => {
        const velocityMag = Math.sqrt(
          data.velocity[0] ** 2 + data.velocity[1] ** 2 + data.velocity[2] ** 2
        );
        
        const newPoint: ChartDataPoint = {
          time: Number(data.time.toFixed(1)),
          posError: data.pos_error || 0,
          angError: (data.ang_error || 0) * (180 / Math.PI), // deg
          velocity: velocityMag,
          solveTime: (data.solve_time || 0) * 1000 // ms
        };

        const newHistory = [...prev, newPoint];
        if (newHistory.length > MAX_HISTORY) {
          return newHistory.slice(newHistory.length - MAX_HISTORY);
        }
        return newHistory;
      });
    });

    return () => { unsubscribe(); };
  }, []);

  return (
    <div className={`absolute bottom-0 left-0 h-48 bg-black/80 backdrop-blur-md border-t border-white/10 flex p-4 pl-8 gap-4 z-20 transition-all duration-300 ${isEditing ? 'right-80' : 'right-0'}`}>
      
      {/* Position Error Chart */}
      <div className="flex-1 min-w-[300px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-blue-400" />
          POSITION ERROR (m)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={history}>
               <CartesianGrid strokeDasharray="3 3" stroke="#333" />
               <XAxis 
                  dataKey="time" 
                  stroke="#666" 
                  fontSize={10} 
                  tickFormatter={(val) => `${val}s`}
                  interval="preserveStartEnd"
                  minTickGap={30}
               />
               <YAxis domain={['auto', 'auto']} stroke="#666" fontSize={10} width={50} />
               <Tooltip 
                 contentStyle={{ backgroundColor: '#111', border: '1px solid #333' }}
                 itemStyle={{ fontSize: '12px' }}
                 labelFormatter={(label) => `Time: ${label}s`}
               />
               <Line type="monotone" dataKey="posError" stroke="#60a5fa" strokeWidth={2} dot={false} isAnimationActive={false} />
             </LineChart>
           </ResponsiveContainer>
        </div>
      </div>

      {/* Angle Error Chart */}
      <div className="flex-1 min-w-[300px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-purple-400" />
          ANGLE ERROR (deg)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={history}>
               <CartesianGrid strokeDasharray="3 3" stroke="#333" />
               <XAxis 
                  dataKey="time" 
                  stroke="#666" 
                  fontSize={10} 
                  tickFormatter={(val) => `${val}s`}
                  interval="preserveStartEnd" 
                  minTickGap={30}
               />
               <YAxis domain={['auto', 'auto']} stroke="#666" fontSize={10} width={50} />
               <Tooltip 
                 contentStyle={{ backgroundColor: '#111', border: '1px solid #333' }}
                 itemStyle={{ fontSize: '12px' }}
                 labelFormatter={(label) => `Time: ${label}s`}
               />
               <Line type="monotone" dataKey="angError" stroke="#c084fc" strokeWidth={2} dot={false} isAnimationActive={false} />
             </LineChart>
           </ResponsiveContainer>
        </div>
      </div>

       {/* Velocity Chart */}
       <div className="flex-1 min-w-[300px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-green-400" />
          VELOCITY (m/s)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={history}>
               <CartesianGrid strokeDasharray="3 3" stroke="#333" />
               <XAxis 
                  dataKey="time" 
                  stroke="#666" 
                  fontSize={10} 
                  tickFormatter={(val) => `${val}s`}
                  interval="preserveStartEnd" 
                  minTickGap={30}
               />
               <YAxis domain={['auto', 'auto']} stroke="#666" fontSize={10} width={50} />
               <Tooltip 
                 contentStyle={{ backgroundColor: '#111', border: '1px solid #333' }}
                 itemStyle={{ fontSize: '12px' }}
                 labelFormatter={(label) => `Time: ${label}s`}
               />
               <Line type="monotone" dataKey="velocity" stroke="#4ade80" strokeWidth={2} dot={false} isAnimationActive={false} />
             </LineChart>
           </ResponsiveContainer>
        </div>
      </div>

       {/* Solve Time Chart */}
       <div className="flex-1 min-w-[300px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-yellow-400" />
          SOLVE TIME (ms)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={history}>
               <CartesianGrid strokeDasharray="3 3" stroke="#333" />
               <XAxis 
                  dataKey="time" 
                  stroke="#666" 
                  fontSize={10} 
                  tickFormatter={(val) => `${val}s`}
                  interval="preserveStartEnd" 
                  minTickGap={30}
               />
               <YAxis domain={['auto', 'auto']} stroke="#666" fontSize={10} width={50} />
               <Tooltip 
                 contentStyle={{ backgroundColor: '#111', border: '1px solid #333' }}
                 itemStyle={{ fontSize: '12px' }}
                 labelFormatter={(label) => `Time: ${label}s`}
               />
               <Line type="monotone" dataKey="solveTime" stroke="#facc15" strokeWidth={2} dot={false} isAnimationActive={false} />
             </LineChart>
           </ResponsiveContainer>
        </div>
      </div>

    </div>
  );
}

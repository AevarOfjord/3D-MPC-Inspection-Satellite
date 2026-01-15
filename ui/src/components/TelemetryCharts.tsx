import { useMemo, useState } from 'react';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, CartesianGrid, ReferenceLine } from 'recharts';
import { Activity } from 'lucide-react';
import { useMissionStore } from '../store/missionStore';
import { useTelemetryStore } from '../store/telemetryStore';

interface ChartDataPoint {
  time: number;
  posError: number;
  angError: number;
  velocity: number;
  solveTime: number;
}

export function TelemetryCharts() {
  const history = useTelemetryStore(s => s.history);
  const isEditing = useMissionStore(s => s.isEditing);
  const [timeWindow, setTimeWindow] = useState(30);
  const [visible, setVisible] = useState({
    pos: true,
    ang: true,
    vel: true,
    solve: true,
  });

  const chartData = useMemo(() => {
    const points: ChartDataPoint[] = [];
    let lastTime = -Infinity;
    history.forEach((data) => {
      if (Math.abs(data.time - lastTime) < 0.1) return;
      lastTime = data.time;
      const velocityMag = Math.sqrt(
        data.velocity[0] ** 2 + data.velocity[1] ** 2 + data.velocity[2] ** 2
      );
      points.push({
        time: Number(data.time.toFixed(1)),
        posError: data.pos_error || 0,
        angError: (data.ang_error || 0) * (180 / Math.PI),
        velocity: velocityMag,
        solveTime: (data.solve_time || 0) * 1000,
      });
    });

    if (points.length === 0) return [];
    const latestTime = points[points.length - 1].time;
    if (timeWindow === 0) return points;
    return points.filter((p) => p.time >= latestTime - timeWindow);
  }, [history, timeWindow]);

  return (
    <div className={`absolute bottom-0 left-0 h-56 bg-black/80 backdrop-blur-md border-t border-white/10 flex flex-col p-4 pl-8 gap-3 z-20 transition-all duration-300 ${isEditing ? 'right-80' : 'right-0'}`}>
      <div className="flex items-center justify-between text-xs text-gray-400">
        <div className="flex items-center gap-2">
          <span className="uppercase tracking-wider">Window</span>
          {[10, 30, 120, 0].map((range) => (
            <button
              key={range}
              onClick={() => setTimeWindow(range)}
              className={`px-2 py-1 rounded border text-[10px] uppercase ${
                timeWindow === range ? 'border-blue-500 text-blue-300' : 'border-gray-700 text-gray-500'
              }`}
            >
              {range === 0 ? 'all' : `${range}s`}
            </button>
          ))}
        </div>
        <div className="flex items-center gap-2">
          {(['pos', 'ang', 'vel', 'solve'] as const).map((key) => (
            <button
              key={key}
              onClick={() => setVisible((prev) => ({ ...prev, [key]: !prev[key] }))}
              className={`px-2 py-1 rounded border text-[10px] uppercase ${
                visible[key] ? 'border-green-500 text-green-300' : 'border-gray-700 text-gray-500'
              }`}
            >
              {key}
            </button>
          ))}
        </div>
      </div>

      <div className="flex flex-1 gap-4">
      
      {/* Position Error Chart */}
      {visible.pos && (
      <div className="flex-1 min-w-[240px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-blue-400" />
          POSITION ERROR (m)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={chartData}>
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
               <ReferenceLine y={0.1} stroke="#22c55e" strokeDasharray="4 4" />
               <ReferenceLine y={0.5} stroke="#ef4444" strokeDasharray="4 4" />
               <Line type="monotone" dataKey="posError" stroke="#60a5fa" strokeWidth={2} dot={false} isAnimationActive={false} />
             </LineChart>
           </ResponsiveContainer>
        </div>
      </div>
      )}

      {/* Angle Error Chart */}
      {visible.ang && (
      <div className="flex-1 min-w-[240px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-purple-400" />
          ANGLE ERROR (deg)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={chartData}>
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
               <ReferenceLine y={1} stroke="#22c55e" strokeDasharray="4 4" />
               <ReferenceLine y={5} stroke="#ef4444" strokeDasharray="4 4" />
               <Line type="monotone" dataKey="angError" stroke="#c084fc" strokeWidth={2} dot={false} isAnimationActive={false} />
             </LineChart>
           </ResponsiveContainer>
        </div>
      </div>
      )}

       {/* Velocity Chart */}
       {visible.vel && (
       <div className="flex-1 min-w-[240px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-green-400" />
          VELOCITY (m/s)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={chartData}>
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
      )}

       {/* Solve Time Chart */}
       {visible.solve && (
       <div className="flex-1 min-w-[240px] flex flex-col">
        <div className="text-xs font-bold text-gray-400 mb-2 flex items-center gap-2">
          <Activity size={12} className="text-yellow-400" />
          SOLVE TIME (ms)
        </div>
        <div className="flex-1 w-full">
           <ResponsiveContainer width="100%" height="100%">
             <LineChart data={chartData}>
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
               <ReferenceLine y={20} stroke="#22c55e" strokeDasharray="4 4" />
               <ReferenceLine y={40} stroke="#ef4444" strokeDasharray="4 4" />
               <Line type="monotone" dataKey="solveTime" stroke="#facc15" strokeWidth={2} dot={false} isAnimationActive={false} />
             </LineChart>
           </ResponsiveContainer>
        </div>
      </div>
      )}

      </div>
    </div>
  );
}

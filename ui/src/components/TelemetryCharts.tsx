import { useMemo, useState, useDeferredValue } from 'react';
import { LineChart, Line, XAxis, YAxis, Tooltip, ResponsiveContainer, CartesianGrid, ReferenceLine } from 'recharts';
import { Activity, ChevronUp, ChevronDown } from 'lucide-react';
import { useTelemetryStore } from '../store/telemetryStore';

export function TelemetryCharts() {
  const rawHistory = useTelemetryStore(s => s.history);
  const history = useDeferredValue(rawHistory);
  const [timeWindow, setTimeWindow] = useState(30);
  const [expanded, setExpanded] = useState(false);
  const [visible, setVisible] = useState({
    pos: true,
    ang: true,
    vel: true,
    solve: true,
  });

  const chartData = useMemo(() => {
    if (history.length === 0) return [];
    if (timeWindow === 0) return history;
    const latestTime = history[history.length - 1].time;
    return history.filter((p) => p.time >= latestTime - timeWindow);
  }, [history, timeWindow]);

  const visibleKeys = (['pos', 'ang', 'vel', 'solve'] as const).filter((k) => visible[k]);

  return (
    <div
      className={`absolute bottom-0 left-0 right-0 z-20 transition-all duration-300 ${
        expanded ? 'h-56' : 'h-8'
      }`}
    >
      {/* Toggle tab — always visible */}
      <button
        onClick={() => setExpanded((v) => !v)}
        className="absolute top-0 left-0 right-0 h-8 bg-black/70 backdrop-blur-sm border-t border-white/10 flex items-center justify-between px-4 text-xs text-gray-400 hover:text-gray-200 transition-colors cursor-pointer select-none"
      >
        <div className="flex items-center gap-3">
          {expanded ? <ChevronDown size={12} /> : <ChevronUp size={12} />}
          <span className="uppercase tracking-wider font-semibold">Charts</span>
          <span className="text-gray-600">
            {visibleKeys.map((k) => k.toUpperCase()).join(' · ')}
          </span>
        </div>
        {expanded && (
          <div className="flex items-center gap-2" onClick={(e) => e.stopPropagation()}>
            <span className="uppercase tracking-wider text-[10px] mr-1">Window</span>
            {[10, 30, 120, 0].map((range) => (
              <button
                key={range}
                onClick={() => setTimeWindow(range)}
                className={`px-2 py-0.5 rounded border text-[10px] uppercase ${
                  timeWindow === range ? 'border-blue-500 text-blue-300' : 'border-gray-700 text-gray-500 hover:border-gray-500'
                }`}
              >
                {range === 0 ? 'all' : `${range}s`}
              </button>
            ))}
            <div className="w-px h-3 bg-gray-700 mx-1" />
            {(['pos', 'ang', 'vel', 'solve'] as const).map((key) => (
              <button
                key={key}
                onClick={() => setVisible((prev) => ({ ...prev, [key]: !prev[key] }))}
                className={`px-2 py-0.5 rounded border text-[10px] uppercase ${
                  visible[key] ? 'border-green-500 text-green-300' : 'border-gray-700 text-gray-500'
                }`}
              >
                {key}
              </button>
            ))}
          </div>
        )}
      </button>

      {/* Chart content — only rendered when expanded */}
      {expanded && (
        <div className="absolute top-8 left-0 right-0 bottom-0 bg-black/80 backdrop-blur-md border-t border-white/5 flex px-4 pb-3 pt-2 gap-4 min-h-0">

          {visible.pos && (
            <ChartPane title="POSITION ERROR (m)" color="#60a5fa" dataKey="posError" icon="text-blue-400"
              refLines={[{ y: 0.1, color: '#22c55e' }, { y: 0.5, color: '#ef4444' }]}
              data={chartData} />
          )}
          {visible.ang && (
            <ChartPane title="ANGLE ERROR (deg)" color="#c084fc" dataKey="angError" icon="text-purple-400"
              refLines={[{ y: 1, color: '#22c55e' }, { y: 5, color: '#ef4444' }]}
              data={chartData} />
          )}
          {visible.vel && (
            <ChartPane title="VELOCITY (m/s)" color="#4ade80" dataKey="velocity" icon="text-green-400"
              refLines={[]}
              data={chartData} />
          )}
          {visible.solve && (
            <ChartPane title="SOLVE TIME (ms)" color="#facc15" dataKey="solveTime" icon="text-yellow-400"
              refLines={[{ y: 20, color: '#22c55e' }, { y: 40, color: '#ef4444' }]}
              data={chartData} />
          )}
        </div>
      )}
    </div>
  );
}

function ChartPane({
  title,
  color,
  dataKey,
  icon,
  refLines,
  data,
}: {
  title: string;
  color: string;
  dataKey: string;
  icon: string;
  refLines: { y: number; color: string }[];
  data: { time: number; [key: string]: number }[];
}) {
  return (
    <div className="flex-1 min-w-[200px] min-h-0 flex flex-col">
      <div className={`text-[10px] font-bold text-gray-400 mb-1 flex items-center gap-1.5`}>
        <Activity size={11} className={icon} />
        {title}
      </div>
      <div className="flex-1 min-h-0">
        <ResponsiveContainer width="100%" height="100%" minWidth={0}>
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="3 3" stroke="#222" />
            <XAxis dataKey="time" stroke="#555" fontSize={9} tickFormatter={(v) => `${v}s`}
              interval="preserveStartEnd" minTickGap={30} />
            <YAxis domain={['auto', 'auto']} stroke="#555" fontSize={9} width={42} />
            <Tooltip contentStyle={{ backgroundColor: '#111', border: '1px solid #333', fontSize: 11 }}
              labelFormatter={(l) => `${l}s`} />
            {refLines.map((r) => (
              <ReferenceLine key={r.y} y={r.y} stroke={r.color} strokeDasharray="4 4" />
            ))}
            <Line type="monotone" dataKey={dataKey} stroke={color} strokeWidth={1.5}
              dot={false} isAnimationActive={false} />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}

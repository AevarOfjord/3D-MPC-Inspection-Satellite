import { useEffect, useState } from 'react';
import { Quaternion, Euler } from 'three';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { Activity } from 'lucide-react';
import { HudPanel } from './HudComponents';

export function Overlay() {
  const [data, setData] = useState<TelemetryData | null>(null);
  const [attitude, setAttitude] = useState<[number, number, number]>([0, 0, 0]);

  useEffect(() => {
    const unsubscribe = telemetry.subscribe(d => {
       setData(d);
       if (d) {
          const q = new Quaternion(d.quaternion[1], d.quaternion[2], d.quaternion[3], d.quaternion[0]);
          const e = new Euler().setFromQuaternion(q, 'ZYX');
          setAttitude([
             e.x * (180/Math.PI),
             e.y * (180/Math.PI),
             e.z * (180/Math.PI)
          ]);
       }
    });
    return () => { unsubscribe(); };
  }, []);

  if (!data) return null;

  const {
    position,
    velocity,
    thrusters,
    rw_torque,
    solve_time = 0,
    pos_error = 0,
    ang_error = 0,
    angular_velocity = [0, 0, 0],
    reference_position = [0, 0, 0],
  } = data;

  const speed = Math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2);
  const distance = Math.sqrt(position[0]**2 + position[1]**2 + position[2]**2);
  const angErrorDeg = ang_error * (180 / Math.PI);
  const delta = [
    reference_position[0] - position[0],
    reference_position[1] - position[1],
    reference_position[2] - position[2],
  ];

  return (
    <div className="absolute inset-0 pointer-events-none p-4 flex flex-col justify-between items-start z-10">
      
      {/* Top Left: Telemetry Panel */}
      <div className="pointer-events-auto flex flex-col gap-2">
        <HudPanel title="TELEMETRY" className="min-w-[240px]">
            <div className="flex flex-col gap-1 font-mono text-xs">
                 {/* Header */}
                 <div className="grid grid-cols-[30px_1fr_1fr_1fr_25px] gap-1 text-center text-slate-500 text-[10px] mb-1">
                    <div></div>
                    <div>X</div>
                    <div>Y</div>
                    <div>Z</div>
                    <div></div>
                 </div>

                 <DataRow label="POS" values={position} unit="m" />
                 <DataRow label="VEL" values={velocity} unit="m/s" />
                 <DataRow label="ERR" values={delta} unit="m" colorClass="text-orange-200" />
                 <DataRow label="ROT" values={attitude} unit="deg" colorClass="text-yellow-200" />
                 <DataRow label="SPIN" values={[angular_velocity[0] * 180 / Math.PI, angular_velocity[1] * 180 / Math.PI, angular_velocity[2] * 180 / Math.PI]} unit="°/s" colorClass="text-cyan-200" />

                 <div className="mt-3 pt-2 border-t border-slate-700/50 flex justify-between px-1">
                    <span className="text-slate-400 font-bold text-[10px]">RANGE</span>
                    <span className="text-cyan-300 font-bold font-mono">{distance.toFixed(2)} m</span>
                 </div>
                 <div className="flex justify-between px-1">
                    <span className="text-slate-400 font-bold text-[10px]">SPEED</span>
                    <span className="text-cyan-300 font-bold font-mono">{speed.toFixed(3)} m/s</span>
                 </div>
            </div>
        </HudPanel>

        <HudPanel title="CONTROLLER" className="min-w-[240px]">
             <div className="space-y-2 font-mono text-xs">
                <div className="flex justify-between items-center">
                   <div className="flex items-center gap-2">
                       <Activity size={14} className="text-yellow-500" />
                       <span className="text-slate-400 font-bold text-[10px]">SOLVE TIME</span>
                   </div>
                   <span className={`${solve_time < 20 ? 'text-green-400' : solve_time < 40 ? 'text-yellow-400' : 'text-red-400'}`}>
                     {(solve_time * 1000).toFixed(1)} ms
                   </span>
                </div>
                
                <div className="h-px bg-slate-700/50" />

                <div className="flex justify-between">
                   <span className="text-slate-400 font-bold text-[10px]">POS ERROR</span>
                   <span className={pos_error < 0.1 ? 'text-green-400' : 'text-slate-200'}>{pos_error.toFixed(3)} m</span>
                </div>
                <div className="flex justify-between">
                   <span className="text-slate-400 font-bold text-[10px]">ANG ERROR</span>
                   <span className={angErrorDeg < 1.0 ? 'text-green-400' : 'text-slate-200'}>{angErrorDeg.toFixed(1)}°</span>
                </div>
             </div>
        </HudPanel>
        
        <HudPanel title="ACTUATORS" className="min-w-[240px]">
             <div className="flex gap-4 justify-between">
                 {/* Thrusters */}
                 <div className="flex flex-col items-center gap-2">
                     <span className="text-[9px] font-bold text-slate-500 tracking-wider">THRUSTERS</span>
                     <div className="flex gap-1">
                        {thrusters.slice(0, 6).map((val, i) => (
                           <ThrusterBar key={i} value={val} label={['+X', '-X', '+Y', '-Y', '+Z', '-Z'][i]} />
                        ))}
                     </div>
                 </div>

                 {/* RW */}
                 <div className="flex flex-col items-center gap-2">
                     <span className="text-[9px] font-bold text-slate-500 tracking-wider">REACTION WHEELS</span>
                     <div className="flex gap-1">
                        {(rw_torque.length > 0 ? rw_torque : [0,0,0]).map((val, i) => (
                           <ReactionWheelBar key={i} value={val} label={['X', 'Y', 'Z'][i]} />
                        ))}
                     </div>
                 </div>
             </div>
        </HudPanel>
      </div>

    </div>
  );
}

// --- Subcomponents ---

function DataRow({ label, values, unit, colorClass = "text-slate-200" }: { label: string, values: number[] | Float32Array, unit: string, colorClass?: string }) {
    const fmt = (n: number) => (n >= 0 ? '+' : '') + n.toFixed(2);
    return (
        <div className="grid grid-cols-[30px_1fr_1fr_1fr_25px] gap-1 items-center hover:bg-white/5 rounded transition-colors">
            <span className="text-slate-500 font-bold text-[10px]">{label}</span>
            <span className={`text-right bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[0])}</span>
            <span className={`text-right bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[1])}</span>
            <span className={`text-right bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[2])}</span>
            <span className="text-slate-600 pl-1 text-[9px]">{unit}</span>
        </div>
    );
}

function ThrusterBar({ value, label }: { value: number, label: string }) {
    const active = value > 0.01;
    return (
        <div className="flex flex-col items-center gap-1 group">
             <div className="relative w-2 h-8 bg-slate-800 rounded-sm overflow-hidden">
                 <div 
                    className={`absolute bottom-0 left-0 right-0 transition-all duration-100 ${active ? 'bg-orange-500 shadow-[0_0_8px_rgba(249,115,22,0.8)]' : 'bg-slate-700'}`}
                    style={{ height: `${Math.min(value * 100, 100)}%` }}
                 />
             </div>
             <span className={`text-[8px] font-mono ${active ? 'text-orange-400 font-bold' : 'text-slate-600'}`}>{label}</span>
        </div>
    );
}

function ReactionWheelBar({ value, label }: { value: number, label: string }) {
    const active = Math.abs(value) > 0.00001;
    const heightPct = Math.min(Math.abs(value) * 500, 100); // Scale for visibility
    return (
        <div className="flex flex-col items-center gap-1">
             <div className="relative w-2 h-8 bg-slate-800 rounded-sm overflow-hidden">
                 <div 
                    className={`absolute bottom-0 left-0 right-0 transition-all duration-100 ${active ? 'bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.8)]' : 'bg-slate-700'}`}
                    style={{ height: `${heightPct}%` }}
                 />
             </div>
             <span className={`text-[8px] font-mono ${active ? 'text-green-400 font-bold' : 'text-slate-600'}`}>{label}</span>
        </div>
    );
}

import { useEffect, useState } from 'react';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { Cpu, Radio, Activity } from 'lucide-react';

export function Overlay() {
  const [data, setData] = useState<TelemetryData | null>(null);

  useEffect(() => {
    const unsubscribe = telemetry.subscribe(setData);
    return () => { unsubscribe(); };
  }, []);

  if (!data) return null;

  const { position, velocity, thrusters, rw_torque, solve_time = 0, pos_error = 0, ang_error = 0 } = data;
  const speed = Math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2);
  const distance = Math.sqrt(position[0]**2 + position[1]**2 + position[2]**2);
  const angErrorDeg = ang_error * (180 / Math.PI);
  
  // Thruster status (assuming 12 thrusters, paired?)
  // Simple indicator: Grey for off, Green for active
  
  return (
    <div className="absolute inset-0 pointer-events-none p-4 flex flex-col justify-between">
      {/* Top Left: Mission Status */}
      <div className="flex flex-col gap-2 pointer-events-auto items-start">
        <div className="bg-black/60 backdrop-blur-md rounded-lg p-3 border border-white/10 text-white min-w-[200px]">
           <div className="flex items-center gap-2 mb-2 border-b border-white/10 pb-1">
             <Radio size={16} className="text-green-400" />
             <span className="font-bold text-sm tracking-wider">TELEMETRY</span>
           </div>
           
           <div className="space-y-1 font-mono text-xs">
              <div className="flex justify-between">
                 <span className="text-gray-400">POS</span>
                 <span>[{position[0].toFixed(2)}, {position[1].toFixed(2)}, {position[2].toFixed(2)}] m</span>
              </div>
              <div className="flex justify-between">
                 <span className="text-gray-400">VEL</span>
                 <span>[{velocity[0].toFixed(3)}, {velocity[1].toFixed(3)}, {velocity[2].toFixed(3)}] m/s</span>
              </div>
              <div className="mt-2 pt-1 border-t border-white/10 flex justify-between">
                 <span className="text-gray-400">RANGE</span>
                 <span className="text-blue-300">{distance.toFixed(2)} m</span>
              </div>
              <div className="flex justify-between">
                 <span className="text-gray-400">SPEED</span>
                 <span className="text-blue-300">{speed.toFixed(3)} m/s</span>
              </div>
           </div>
        </div>

        {/* Controller Stats */}
        <div className="bg-black/60 backdrop-blur-md rounded-lg p-3 border border-white/10 text-white min-w-[200px]">
           <div className="flex items-center gap-2 mb-2 border-b border-white/10 pb-1">
             <Activity size={16} className="text-yellow-400" />
             <span className="font-bold text-sm tracking-wider">CONTROLLER</span>
           </div>
           
           <div className="space-y-1 font-mono text-xs">
              <div className="flex justify-between">
                 <span className="text-gray-400">SOLVE</span>
                 <span className={`${solve_time < 20 ? 'text-green-400' : solve_time < 40 ? 'text-yellow-400' : 'text-red-400'}`}>
                   {(solve_time * 1000).toFixed(1)} ms
                 </span>
              </div>
              <div className="flex justify-between">
                 <span className="text-gray-400">POS ERR</span>
                 <span className={pos_error < 0.1 ? 'text-green-400' : 'text-white'}>{pos_error.toFixed(3)} m</span>
              </div>
              <div className="flex justify-between">
                 <span className="text-gray-400">ANG ERR</span>
                 <span className={angErrorDeg < 1.0 ? 'text-green-400' : 'text-white'}>{angErrorDeg.toFixed(1)}°</span>
              </div>
           </div>
        </div>
      </div>
      
      {/* Bottom Center: Thrusters */}
      <div className="absolute bottom-52 left-1/2 -translate-x-1/2 transition-all duration-300">
         <div className="bg-black/60 backdrop-blur-md rounded-lg p-2 border border-white/10 flex gap-2">
            {thrusters.slice(0, 6).map((active, i) => (
               <div key={i} className="flex flex-col items-center gap-1">
                  <div 
                    className={`w-3 h-8 rounded-sm transition-all duration-100 ${active > 0.1 ? 'bg-orange-500 shadow-[0_0_8px_rgba(249,115,22,0.8)] scale-y-110' : 'bg-gray-700'}`}
                    title={`Thruster ${i}`}
                  />
                  <span className="text-[9px] font-mono text-gray-400">
                    {['+X', '-X', '+Y', '-Y', '+Z', '-Z'][i] || i}
                  </span>
                  <span className="text-[8px] font-mono text-gray-500 h-3">
                     {active > 0.01 ? active.toFixed(1) : ''}
                  </span>
               </div>
            ))}
         </div>
      </div>
      
      {/* Bottom Right: Reaction Wheels */}
      <div className="absolute bottom-52 right-6 transition-all duration-300">
         <div className="bg-black/60 backdrop-blur-md rounded-lg p-2 border border-white/10 flex gap-2 items-center">
            <span className="text-[10px] font-bold text-gray-500 mr-1 self-start mt-1">RW</span>
            {(rw_torque.length > 0 ? rw_torque : [0,0,0]).map((torque, i) => (
               <div key={i} className="flex flex-col items-center gap-1">
                  <div 
                    className={`w-3 h-8 rounded-sm transition-all duration-100 ${Math.abs(torque) > 0.00001 ? 'bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.8)] scale-y-110' : 'bg-gray-700'}`}
                    title={`RW ${i}`}
                  />
                  <span className="text-[9px] font-mono text-gray-400">
                    {['X', 'Y', 'Z'][i] || i}
                  </span>
                   <span className="text-[8px] font-mono text-gray-500 h-3">
                     {Math.abs(torque) > 0.00001 ? torque.toFixed(3) : '0.0'}
                  </span>
               </div>
            ))}
         </div>
      </div>
    </div>
  );
}

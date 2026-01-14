import { useEffect, useState } from 'react';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { Cpu, Radio } from 'lucide-react';

export function Overlay() {
  const [data, setData] = useState<TelemetryData | null>(null);

  useEffect(() => {
    const unsubscribe = telemetry.subscribe(setData);
    return () => { unsubscribe(); };
  }, []);

  if (!data) return null;

  const { position, velocity, thrusters, rw_torque } = data;
  const speed = Math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2);
  const distance = Math.sqrt(position[0]**2 + position[1]**2 + position[2]**2);
  
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
      </div>
      
      {/* Bottom Center: Thrusters */}
      <div className="absolute bottom-6 left-1/2 -translate-x-1/2">
         <div className="bg-black/60 backdrop-blur-md rounded-lg p-2 border border-white/10 flex gap-2">
            {thrusters.map((active, i) => (
               <div 
                 key={i} 
                 className={`w-3 h-8 rounded-sm transition-all duration-100 ${active > 0.1 ? 'bg-orange-500 shadow-[0_0_8px_rgba(249,115,22,0.8)] scale-y-110' : 'bg-gray-700'}`}
                 title={`Thruster ${i}`}
               />
            ))}
         </div>
      </div>
      
      {/* Bottom Right: Reaction Wheels */}
      <div className="absolute bottom-6 right-6">
         <div className="bg-black/60 backdrop-blur-md rounded-lg p-3 border border-white/10 text-white min-w-[180px]">
            <div className="flex items-center gap-2 mb-2 pb-1 border-b border-white/10">
               <Cpu size={16} className="text-purple-400" />
               <span className="font-bold text-sm tracking-wider">RW SYSTEMS</span>
            </div>
            <div className="space-y-2">
               {rw_torque.map((torque, i) => (
                  <div key={i} className="flex items-center gap-2 text-xs font-mono">
                     <span className="w-4 text-gray-400">W{i+1}</span>
                     <div className="flex-1 h-2 bg-gray-700 rounded-full overflow-hidden">
                        {/* Torque Bar from center? Or 0 to max? Assuming bidirectional */}
                        {/* Let's visualize magnitude for now. Normalize to assumed max 0.05 Nm */}
                        <div 
                          className="h-full bg-purple-500 transition-all duration-100" 
                          style={{ width: `${Math.min(Math.abs(torque)/0.05 * 100, 100)}%` }}
                        />
                     </div>
                     <span className="w-10 text-right">{torque.toFixed(3)}</span>
                  </div>
               ))}
            </div>
         </div>
      </div>
    </div>
  );
}

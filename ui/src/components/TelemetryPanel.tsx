import { useEffect, useState } from 'react';
import { Quaternion, Euler } from 'three';
import { telemetry } from '../services/telemetry';
import type { TelemetryData } from '../services/telemetry';
import { HudPanel } from './HudComponents';

function DataRow({
  label,
  values,
  unit,
  colorClass = 'text-slate-200',
}: {
  label: string;
  values: number[] | Float32Array;
  unit: string;
  colorClass?: string;
}) {
  const fmt = (n: number) => (n >= 0 ? '+' : '') + n.toFixed(2);
  return (
    <div className="grid grid-cols-[30px_1fr_1fr_1fr_25px] gap-1 items-center hover:bg-white/5 rounded transition-colors">
      <span className="text-slate-500 font-bold text-[10px]">{label}</span>
      <span className={`text-right tabular-nums bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[0])}</span>
      <span className={`text-right tabular-nums bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[1])}</span>
      <span className={`text-right tabular-nums bg-slate-900/50 rounded px-1 ${colorClass}`}>{fmt(values[2])}</span>
      <span className="text-slate-600 pl-1 text-[9px]">{unit}</span>
    </div>
  );
}

export function TelemetryPanel() {
  const [data, setData] = useState<TelemetryData | null>(null);
  const [attitude, setAttitude] = useState<[number, number, number]>([0, 0, 0]);

  useEffect(() => {
    return telemetry.subscribe((d) => {
      setData(d);
      if (d.orientation_unwrapped_deg?.length === 3) {
        setAttitude([
          d.orientation_unwrapped_deg[0],
          d.orientation_unwrapped_deg[1],
          d.orientation_unwrapped_deg[2],
        ]);
        return;
      }
      const q = new Quaternion(d.quaternion[1], d.quaternion[2], d.quaternion[3], d.quaternion[0]);
      const e = new Euler().setFromQuaternion(q, 'ZYX');
      const yawDeg = Number.isFinite(d.yaw_unwrapped_deg ?? NaN)
        ? (d.yaw_unwrapped_deg as number)
        : e.z * (180 / Math.PI);
      setAttitude([e.x * (180 / Math.PI), e.y * (180 / Math.PI), yawDeg]);
    });
  }, []);

  if (!data) return null;

  const { position, velocity, angular_velocity = [0, 0, 0], reference_position = [0, 0, 0] } = data;
  const speed = Math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2);
  const distance = Math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2);
  const delta: [number, number, number] = [
    reference_position[0] - position[0],
    reference_position[1] - position[1],
    reference_position[2] - position[2],
  ];
  const spinDeg: [number, number, number] = [
    angular_velocity[0] * 180 / Math.PI,
    angular_velocity[1] * 180 / Math.PI,
    angular_velocity[2] * 180 / Math.PI,
  ];

  return (
    <HudPanel title="TELEMETRY" live className="min-w-[240px]">
      <div className="flex flex-col gap-1 font-mono text-xs">
        <div className="grid grid-cols-[30px_1fr_1fr_1fr_25px] gap-1 text-center text-slate-500 text-[10px] mb-1">
          <div />
          <div>X</div>
          <div>Y</div>
          <div>Z</div>
          <div />
        </div>
        <DataRow label="POS" values={position} unit="m" />
        <DataRow label="VEL" values={velocity} unit="m/s" />
        <DataRow label="ERR" values={delta} unit="m" colorClass="text-orange-200" />
        <DataRow label="ROT" values={attitude} unit="deg" colorClass="text-yellow-200" />
        {data.euler_unreliable && (
          <div className="text-[10px] text-amber-300 px-1">
            Euler roll/yaw near singularity (|pitch| &gt; 85°); yaw shown unwrapped.
          </div>
        )}
        <DataRow label="SPIN" values={spinDeg} unit="°/s" colorClass="text-cyan-200" />

        <div className="mt-3 pt-2 border-t border-slate-700/50 flex justify-between px-1">
          <span className="text-slate-400 font-bold text-[10px]">RANGE</span>
          <span className="text-cyan-300 font-bold tabular-nums">{distance.toFixed(2)} m</span>
        </div>
        <div className="flex justify-between px-1">
          <span className="text-slate-400 font-bold text-[10px]">SPEED</span>
          <span className="text-cyan-300 font-bold tabular-nums">{speed.toFixed(3)} m/s</span>
        </div>
      </div>
    </HudPanel>
  );
}

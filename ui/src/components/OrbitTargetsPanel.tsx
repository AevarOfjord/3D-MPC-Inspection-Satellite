import { useMemo } from 'react';
import { Eye, EyeOff } from 'lucide-react';
import { orbitSnapshot, ORBIT_SCALE } from '../data/orbitSnapshot';

interface OrbitTargetsPanelProps {
  selectedTargetId?: string | null;
  orbitVisibility?: Record<string, boolean>;
  onSelectTarget?: (
    targetId: string,
    positionMeters: [number, number, number],
    positionScene: [number, number, number]
  ) => void;
  onToggleOrbit?: (targetId: string) => void;
  ownSatellite?: {
    id?: string;
    name?: string;
    positionScene: [number, number, number];
    positionMeters?: [number, number, number];
  };
  onFocusTarget?: (targetId: string, positionScene: [number, number, number], focusDistance?: number) => void;
  solarBodies?: {
    id: string;
    name: string;
    type: string;
    radiusScene?: number;
    positionScene: [number, number, number];
    positionMeters?: [number, number, number];
  }[];
}

const formatPosition = (pos: [number, number, number]) =>
  `[${pos.map((v) => v.toFixed(0)).join(', ')}] m`;

export function OrbitTargetsPanel({
  selectedTargetId,
  orbitVisibility,
  onSelectTarget,
  onToggleOrbit,
  ownSatellite,
  onFocusTarget,
  solarBodies,
}: OrbitTargetsPanelProps) {
  const targets = useMemo(
    () =>
      orbitSnapshot.objects.map((obj) => ({
        ...obj,
        scenePosition: [
          obj.position_m[0] * ORBIT_SCALE,
          obj.position_m[1] * ORBIT_SCALE,
          obj.position_m[2] * ORBIT_SCALE,
        ] as [number, number, number],
      })),
    []
  );

  return (
    <div className="fixed right-6 top-20 z-40 w-64 rounded border border-cyan-400/30 bg-slate-900/90 backdrop-blur shadow-[0_0_30px_rgba(15,23,42,0.65)] pointer-events-auto">
      <div className="px-3 py-2 border-b border-slate-800 text-[10px] uppercase tracking-widest text-slate-300">
        Orbit Targets
      </div>
      <div className="max-h-[70vh] overflow-y-auto custom-scrollbar">
        {ownSatellite && (
          <button
            onClick={() => onFocusTarget?.(ownSatellite.id ?? 'SATELLITE', ownSatellite.positionScene)}
            className="w-full text-left px-3 py-2 border-b border-slate-900 transition-colors hover:bg-slate-900/60 text-slate-200"
          >
            <div className="flex items-center justify-between text-xs font-semibold">
              <span className="truncate">{ownSatellite.name ?? 'Satellite'}</span>
              <span className="text-[10px] uppercase text-slate-400">vehicle</span>
            </div>
            <div className="mt-1 text-[10px] text-slate-400">
              {formatPosition(
                ownSatellite.positionMeters ?? [
                  ownSatellite.positionScene[0] / ORBIT_SCALE,
                  ownSatellite.positionScene[1] / ORBIT_SCALE,
                  ownSatellite.positionScene[2] / ORBIT_SCALE,
                ]
              )}
            </div>
          </button>
        )}
        {targets.map((obj) => {
          const isSelected = selectedTargetId === obj.id;
          const orbitOn = orbitVisibility?.[obj.id] ?? false;
          const focusDistance = obj.real_span_m ? Math.max(obj.real_span_m * 5, 10) : undefined;
          return (
            <div
              key={obj.id}
              role="button"
              tabIndex={0}
              onClick={() => {
                onSelectTarget?.(obj.id, obj.position_m, obj.scenePosition);
                onFocusTarget?.(obj.id, obj.scenePosition, focusDistance);
              }}
              onKeyDown={(event) => {
                if (event.key === 'Enter' || event.key === ' ') {
                  event.preventDefault();
                  onSelectTarget?.(obj.id, obj.position_m, obj.scenePosition);
                  onFocusTarget?.(obj.id, obj.scenePosition, focusDistance);
                }
              }}
              className={`w-full text-left px-3 py-2 border-b border-slate-900 transition-colors cursor-pointer ${
                isSelected ? 'bg-cyan-500/15 text-cyan-200' : 'hover:bg-slate-900/60 text-slate-200'
              }`}
            >
              <div className="flex items-center justify-between gap-2 text-xs font-semibold">
                <span className="truncate">{obj.name}</span>
                <div className="flex items-center gap-2">
                  <span className="text-[10px] uppercase text-slate-400">{obj.type}</span>
                  <button
                    type="button"
                    onClick={(event) => {
                      event.stopPropagation();
                      onToggleOrbit?.(obj.id);
                    }}
                    className={`p-1 rounded border ${
                      orbitOn
                        ? 'border-cyan-400/40 text-cyan-200 bg-cyan-500/10'
                        : 'border-slate-700 text-slate-400 hover:text-slate-200'
                    }`}
                    title={orbitOn ? 'Hide orbit' : 'Show orbit'}
                  >
                    {orbitOn ? <Eye size={12} /> : <EyeOff size={12} />}
                  </button>
                </div>
              </div>
              <div className="mt-1 text-[10px] text-slate-400">
                {formatPosition(obj.position_m)}
              </div>
            </div>
          );
        })}
        {targets.length === 0 && (
          <div className="px-3 py-3 text-xs text-slate-500">No orbit targets.</div>
        )}
        {solarBodies && solarBodies.length > 0 && (
          <div className="px-3 py-2 border-t border-slate-800 text-[10px] uppercase tracking-widest text-slate-300">
            Solar System
          </div>
        )}
        {solarBodies?.map((body) => (
          <button
            key={body.id}
            onClick={() =>
              onFocusTarget?.(
                body.id,
                body.positionScene,
                body.radiusScene ? Math.max(body.radiusScene * 3, 5) : undefined
              )
            }
            className="w-full text-left px-3 py-2 border-b border-slate-900 transition-colors hover:bg-slate-900/60 text-slate-200"
          >
            <div className="flex items-center justify-between text-xs font-semibold">
              <span className="truncate">{body.name}</span>
              <span className="text-[10px] uppercase text-slate-400">{body.type}</span>
            </div>
            <div className="mt-1 text-[10px] text-slate-400">
              {formatPosition(
                body.positionMeters ?? [
                  body.positionScene[0] / ORBIT_SCALE,
                  body.positionScene[1] / ORBIT_SCALE,
                  body.positionScene[2] / ORBIT_SCALE,
                ]
              )}
            </div>
          </button>
        ))}
      </div>
    </div>
  );
}

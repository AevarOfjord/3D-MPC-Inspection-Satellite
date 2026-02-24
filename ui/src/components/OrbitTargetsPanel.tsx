import { useMemo, useState } from 'react';
import { orbitSnapshot, ORBIT_SCALE } from '../data/orbitSnapshot';
import { HudPanel } from './HudComponents';

interface OrbitTargetsPanelProps {
  selectedTargetId?: string | null;
  className?: string;
  onSelectTarget?: (
    targetId: string,
    positionMeters: [number, number, number],
    positionScene: [number, number, number]
  ) => void;
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
  selectedTargetId: selectedTargetIdProp,
  className,
  onSelectTarget,
  ownSatellite,
  onFocusTarget,
  solarBodies,
}: OrbitTargetsPanelProps) {
  const [selectedTargetId, setSelectedTargetId] = useState(selectedTargetIdProp ?? null);

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

  // When used standalone (inside Overlay), className is undefined — no wrapper div needed.
  // When used with a className (legacy placement), wrap in a positioned div.
  const inner = (
    <HudPanel title="ORBITAL TARGETS" className={className ? '' : 'min-w-[240px]'}>
      <div className="flex flex-col gap-0 font-mono text-xs -mx-3 -mb-3">
        {ownSatellite && (
          <button
            onClick={() => onFocusTarget?.(ownSatellite.id ?? 'SATELLITE', ownSatellite.positionScene)}
            className="w-full text-left px-3 py-2 border-b border-slate-800/60 transition-colors hover:bg-white/5"
          >
            <div className="flex items-center justify-between">
              <span className="text-slate-200 font-semibold truncate">{ownSatellite.name ?? 'Satellite'}</span>
              <span className="text-[10px] uppercase text-slate-500 ml-2 flex-shrink-0">VEHICLE</span>
            </div>
            <div className="mt-0.5 text-[10px] text-slate-500">
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
          const focusDistance = obj.real_span_m ? Math.max(obj.real_span_m * 5, 10) : undefined;
          return (
            <div
              key={obj.id}
              role="button"
              tabIndex={0}
              onClick={() => {
                setSelectedTargetId(obj.id);
                onSelectTarget?.(obj.id, obj.position_m, obj.scenePosition);
                onFocusTarget?.(obj.id, obj.scenePosition, focusDistance);
              }}
              onKeyDown={(event) => {
                if (event.key === 'Enter' || event.key === ' ') {
                  event.preventDefault();
                  setSelectedTargetId(obj.id);
                  onSelectTarget?.(obj.id, obj.position_m, obj.scenePosition);
                  onFocusTarget?.(obj.id, obj.scenePosition, focusDistance);
                }
              }}
              className={`w-full text-left px-3 py-2 border-b border-slate-800/60 transition-colors cursor-pointer ${
                isSelected ? 'bg-cyan-500/15 text-cyan-200' : 'hover:bg-white/5 text-slate-200'
              }`}
            >
              <div className="flex items-center justify-between gap-2">
                <span className="font-semibold truncate">{obj.name}</span>
                <span className="text-[10px] uppercase text-slate-500 flex-shrink-0">{obj.type}</span>
              </div>
              <div className="mt-0.5 text-[10px] text-slate-500">
                {formatPosition(obj.position_m)}
              </div>
            </div>
          );
        })}
        {targets.length === 0 && (
          <div className="px-3 py-3 text-slate-500">No orbit targets.</div>
        )}
        {solarBodies && solarBodies.length > 0 && (
          <div className="px-3 py-2 border-t border-slate-800/60 text-[10px] uppercase tracking-widest text-slate-500">
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
            className="w-full text-left px-3 py-2 border-b border-slate-800/60 transition-colors hover:bg-white/5 text-slate-200"
          >
            <div className="flex items-center justify-between">
              <span className="font-semibold truncate">{body.name}</span>
              <span className="text-[10px] uppercase text-slate-500 flex-shrink-0">{body.type}</span>
            </div>
            <div className="mt-0.5 text-[10px] text-slate-500">
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
    </HudPanel>
  );

  if (className) {
    return <div className={`${className} z-40 pointer-events-auto`}>{inner}</div>;
  }
  return inner;
}

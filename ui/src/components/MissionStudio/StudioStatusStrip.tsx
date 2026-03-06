import { useMemo } from 'react';
import { useStudioStore } from './useStudioStore';
import { getStudioRouteDiagnostics } from './studioRouteDiagnostics';

function StatusPill({
  label,
  value,
  tone,
}: {
  label: string;
  value: string;
  tone: 'neutral' | 'info' | 'warn' | 'good' | 'bad';
}) {
  const toneClass =
    tone === 'good'
      ? 'border-emerald-500/40 bg-emerald-950/70 text-emerald-200'
      : tone === 'bad'
        ? 'border-red-500/40 bg-red-950/60 text-red-200'
        : tone === 'warn'
          ? 'border-amber-500/40 bg-amber-950/60 text-amber-200'
          : tone === 'info'
            ? 'border-cyan-500/40 bg-cyan-950/60 text-cyan-100'
            : 'border-slate-700 bg-slate-950/75 text-slate-300';
  return (
    <div className={`rounded-full border px-2.5 py-1 ${toneClass}`}>
      <span className="text-[9px] uppercase tracking-[0.14em] text-slate-400">
        {label}
      </span>
      <span className="ml-1.5 text-[11px] font-semibold">{value}</span>
    </div>
  );
}

export function StudioStatusStrip() {
  const referenceObjectPath = useStudioStore((s) => s.referenceObjectPath);
  const paths = useStudioStore((s) => s.paths);
  const wires = useStudioStore((s) => s.wires);
  const holds = useStudioStore((s) => s.holds);
  const points = useStudioStore((s) => s.points);
  const assembly = useStudioStore((s) => s.assembly);
  const validationReport = useStudioStore((s) => s.validationReport);
  const missionName = useStudioStore((s) => s.missionName);

  const diagnostics = useMemo(
    () =>
      getStudioRouteDiagnostics({
        referenceObjectPath,
        paths,
        wires,
        holds,
        points,
        assembly,
      }),
    [referenceObjectPath, paths, wires, holds, points, assembly]
  );

  const routeTone =
    diagnostics.status === 'executable'
      ? 'good'
      : diagnostics.status === 'invalid'
        ? 'bad'
        : diagnostics.status === 'empty'
          ? 'neutral'
          : 'warn';
  const validationTone = validationReport?.valid
    ? 'good'
    : validationReport
      ? 'bad'
      : 'neutral';
  const saveTone =
    diagnostics.executable && missionName.trim().length > 0 ? 'good' : 'warn';

  return (
    <div className="rounded-2xl border border-white/10 bg-slate-950/78 px-3 py-2 shadow-[0_10px_30px_rgba(2,6,23,0.45)] backdrop-blur-md">
      <div className="flex flex-wrap items-center gap-2">
        <StatusPill
          label="Target"
          value={diagnostics.targetMode === 'object' ? 'Object' : 'Local'}
          tone={diagnostics.targetMode === 'object' ? 'info' : 'neutral'}
        />
        <StatusPill
          label="Paths"
          value={`${diagnostics.validPathCount}/${diagnostics.totalPathCount}`}
          tone={
            diagnostics.validPathCount > 0 && diagnostics.invalidPathIds.length === 0
              ? 'good'
              : diagnostics.totalPathCount > 0
                ? 'warn'
                : 'neutral'
          }
        />
        <StatusPill
          label="Connections"
          value={`${diagnostics.totalWireCount}`}
          tone={
            diagnostics.totalWireCount === 0
              ? 'neutral'
              : diagnostics.invalidWireIds.length === 0
                ? 'info'
                : 'bad'
          }
        />
        <StatusPill
          label="Route"
          value={
            diagnostics.status === 'executable'
              ? 'Executable'
              : diagnostics.status === 'invalid'
                ? 'Invalid'
                : diagnostics.status === 'empty'
                  ? 'Empty'
                  : 'Incomplete'
          }
          tone={routeTone}
        />
        <StatusPill
          label="Validation"
          value={validationReport?.valid ? 'Passed' : validationReport ? 'Issues' : 'Pending'}
          tone={validationTone}
        />
        <StatusPill
          label="Save"
          value={
            diagnostics.executable && missionName.trim().length > 0
              ? 'Ready'
              : 'Blocked'
          }
          tone={saveTone}
        />
      </div>
      <div className="mt-2 text-[11px] text-slate-300">{diagnostics.nextAction}</div>
    </div>
  );
}

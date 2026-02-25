import { useState, useEffect } from 'react';
import { Trash2, Save, CheckCircle } from 'lucide-react';
import { useStudioStore } from './useStudioStore';
import { compileStudioMission } from './compileStudioMission';

function SegmentRow({ index }: { index: number }) {
  const segments = useStudioStore((s) => s.segments);
  const scanPasses = useStudioStore((s) => s.scanPasses);
  const holds = useStudioStore((s) => s.holds);
  const removeSegment = useStudioStore((s) => s.removeSegment);
  const seg = segments[index];
  if (!seg) return null;

  let icon = '●';
  let label = seg.type;
  let badge: string | null = null;

  if (seg.type === 'start') { icon = '🛰'; label = 'Start Position'; }
  if (seg.type === 'scan' && seg.scanId) {
    const pass = scanPasses.find((p) => p.id === seg.scanId);
    icon = '🔄';
    label = 'Scan Pass';
    badge = pass?.axis ?? null;
  }
  if (seg.type === 'transfer') { icon = '↗'; label = 'Transfer'; }
  if (seg.type === 'hold' && seg.holdId) {
    const hold = holds.find((h) => h.id === seg.holdId);
    icon = '⏸';
    label = `Hold ${hold?.duration.toFixed(1) ?? '?'}s`;
  }

  return (
    <div className="flex items-center gap-2 px-3 py-2 rounded-lg border border-slate-800 hover:border-slate-700 bg-slate-900/40 group">
      <span className="text-[10px] text-slate-500 w-5 shrink-0 tabular-nums">{index + 1}</span>
      <span className="text-sm">{icon}</span>
      <span className="flex-1 text-xs text-slate-200 font-medium truncate">{label}</span>
      {badge && (
        <span className="text-[10px] px-1.5 py-0.5 rounded border border-cyan-800 text-cyan-300 bg-cyan-950/40">
          {badge}
        </span>
      )}
      {seg.type !== 'start' && (
        <button
          type="button"
          onClick={() => removeSegment(seg.id)}
          className="opacity-0 group-hover:opacity-100 text-slate-500 hover:text-red-400 transition-opacity"
        >
          <Trash2 size={12} />
        </button>
      )}
    </div>
  );
}

export function MissionStudioRightPanel() {
  const segments = useStudioStore((s) => s.segments);
  const scanPasses = useStudioStore((s) => s.scanPasses);
  const missionName = useStudioStore((s) => s.missionName);
  const setMissionName = useStudioStore((s) => s.setMissionName);
  const validationBusy = useStudioStore((s) => s.validationBusy);
  const setValidationBusy = useStudioStore((s) => s.setValidationBusy);
  const saveBusy = useStudioStore((s) => s.saveBusy);
  const setSaveBusy = useStudioStore((s) => s.setSaveBusy);

  const [saveSuccess, setSaveSuccess] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [validateResult, setValidateResult] = useState<{ ok: boolean; message: string } | null>(null);

  useEffect(() => {
    if (missionName.trim().length > 0) return;
    if (scanPasses.length === 0) return;
    const ts = new Date().toISOString().slice(0, 16).replace(/[-:T]/g, '');
    setMissionName(`Studio_${scanPasses.length}pass_${ts}`);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scanPasses.length]);

  const totalWaypoints = scanPasses.reduce((acc, p) => acc + p.waypoints.length, 0);

  const handleValidate = async () => {
    setValidationBusy(true);
    setValidateResult(null);
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      const { unifiedMissionApi } = await import('../../api/unifiedMissionApi');
      const report = await unifiedMissionApi.validateMission(mission);
      setValidateResult({ ok: report.valid, message: report.valid ? 'Validation passed' : `${report.summary?.errors ?? '?'} error(s)` });
    } catch (e) {
      setValidateResult({ ok: false, message: String(e) });
    } finally {
      setValidationBusy(false);
    }
  };

  const handleSave = async () => {
    setSaveBusy(true);
    setSaveSuccess(false);
    setSaveError(null);
    try {
      const mission = compileStudioMission(useStudioStore.getState());
      const { unifiedMissionApi } = await import('../../api/unifiedMissionApi');
      await unifiedMissionApi.saveMission(missionName || 'Untitled Studio Mission', mission);
      setSaveSuccess(true);
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (e) {
      setSaveError(String(e));
    } finally {
      setSaveBusy(false);
    }
  };

  const canSave = segments.length > 0 && missionName.trim().length > 0;

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="px-3 py-2 text-[10px] font-bold uppercase tracking-[0.16em] text-slate-500 border-b border-slate-800/60 flex items-center justify-between">
        <span>Mission Assembly</span>
        <span className="text-slate-600 tabular-nums">{segments.length} seg · {totalWaypoints} pts</span>
      </div>

      {/* Segment list */}
      <div className="flex-1 overflow-y-auto p-3 flex flex-col gap-1.5">
        {segments.length === 0 ? (
          <div className="text-xs text-slate-600 text-center py-8">
            Add segments using the left panel
          </div>
        ) : (
          segments.map((_, i) => <SegmentRow key={i} index={i} />)
        )}
      </div>

      {/* Validation result */}
      {validateResult && (
        <div className="px-3 py-2 border-t border-slate-800/60">
          <div className={`text-xs font-semibold ${validateResult.ok ? 'text-emerald-400' : 'text-amber-400'}`}>
            {validateResult.ok ? '✓' : '✗'} {validateResult.message}
          </div>
        </div>
      )}
      {saveError && (
        <div className="px-3 py-1 text-[10px] text-red-400">{saveError}</div>
      )}

      {/* Footer actions */}
      <div className="p-3 border-t border-slate-800/60 flex flex-col gap-2">
        <input
          className="w-full bg-black/40 border border-slate-700 rounded-lg px-2.5 py-1.5 text-xs text-slate-100 placeholder:text-slate-600 focus:outline-none focus:border-cyan-700"
          placeholder="Mission name..."
          value={missionName}
          onChange={(e) => setMissionName(e.target.value)}
        />
        <button
          type="button"
          onClick={() => void handleValidate()}
          disabled={validationBusy}
          className="w-full py-2 rounded-lg border border-slate-700 bg-slate-800 text-slate-200 text-xs font-semibold disabled:opacity-50 hover:border-slate-600 transition-all"
        >
          {validationBusy ? 'Validating...' : 'Validate'}
        </button>
        <button
          type="button"
          onClick={() => void handleSave()}
          disabled={!canSave || saveBusy}
          className="w-full py-2 rounded-lg border border-emerald-700 bg-emerald-900/40 text-emerald-100 text-xs font-semibold disabled:opacity-40 flex items-center justify-center gap-1.5 hover:bg-emerald-900/60 transition-all"
        >
          {saveSuccess ? <><CheckCircle size={13} /> Saved!</> : saveBusy ? 'Saving...' : <><Save size={13} /> Save Mission</>}
        </button>
      </div>
    </div>
  );
}

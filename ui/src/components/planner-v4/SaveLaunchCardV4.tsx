import { Play, Rocket, Save, WandSparkles } from 'lucide-react';

import type { ScanSegment } from '../../api/unifiedMission';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { isSaveLaunchReady } from '../../utils/plannerValidation';
import { Panel } from '../ui-v4/Panel';
import { InlineBanner } from '../ui-v4/InlineBanner';
import { StatusPill } from '../ui-v4/StatusPill';

interface SaveLaunchCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

function suggestMissionName(current: string, selectedTargetId: string | null, segmentsCount: number): string {
  const timestamp = new Date()
    .toISOString()
    .slice(0, 16)
    .replaceAll('-', '')
    .replaceAll(':', '')
    .replace('T', '');
  const target = selectedTargetId || 'Target';
  if (current.trim().length > 0 && !current.startsWith('Mission_')) {
    return current;
  }
  return `${target}_M${segmentsCount}_${timestamp}`;
}

export function SaveLaunchCardV4({ builder }: SaveLaunchCardV4Props) {
  const { state, setters, actions } = builder;
  const report = state.validationReport;
  const ready = isSaveLaunchReady(report);

  const scanSegments = state.segments.filter((segment) => segment.type === 'scan') as ScanSegment[];
  const checklist = [
    {
      label: 'At least one segment added',
      done: state.segments.length > 0,
    },
    {
      label: 'All scan segments have path assets',
      done: scanSegments.length > 0 && scanSegments.every((segment) => Boolean(segment.path_asset)),
    },
    {
      label: 'Validation passed',
      done: Boolean(report?.valid),
    },
  ];

  const suggestedName = suggestMissionName(
    state.missionName,
    state.selectedOrbitTargetId,
    state.segments.length
  );

  return (
    <Panel
      title="Step 5 · Save Mission"
      subtitle="Preflight validation, naming helper, save, then optional launch"
      actions={
        ready ? <StatusPill tone="success">Ready</StatusPill> : <StatusPill tone="warning">Blocked</StatusPill>
      }
    >
      <div id="coachmark-save_launch" className="space-y-3">
        {ready ? (
          <InlineBanner tone="success" title="Preflight Pass">
            Mission is valid. You can save and launch.
          </InlineBanner>
        ) : (
          <InlineBanner tone="warning" title="Preflight Incomplete">
            Complete the checklist items below before save/launch.
          </InlineBanner>
        )}

        <div className="v4-subtle-panel p-3 space-y-2">
          <div className="text-xs font-semibold text-[color:var(--v4-text-2)] uppercase tracking-[0.12em]">
            Preflight Checklist
          </div>
          {checklist.map((item) => (
            <div key={item.label} className="flex items-center justify-between text-xs">
              <span className="text-[color:var(--v4-text-2)]">{item.label}</span>
              <StatusPill tone={item.done ? 'success' : 'warning'}>{item.done ? 'done' : 'pending'}</StatusPill>
            </div>
          ))}
          {report ? (
            <div className="pt-1 text-[11px] text-[color:var(--v4-text-3)]">
              Validation summary: errors={report.summary.errors}, warnings={report.summary.warnings}
            </div>
          ) : null}
        </div>

        <div className="v4-subtle-panel p-3 space-y-2">
          <div className="text-xs font-semibold text-[color:var(--v4-text-2)] uppercase tracking-[0.12em]">
            Naming Helper
          </div>
          <input
            className="v4-field"
            value={state.missionName}
            onChange={(event) => setters.setMissionName(event.target.value)}
          />
          <button
            type="button"
            onClick={() => setters.setMissionName(suggestedName)}
            className="v4-focus v4-button px-2 py-1.5 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)] flex items-center gap-1"
          >
            <WandSparkles size={12} /> Use Suggested Name
          </button>
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => void actions.validateUnifiedMission()}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            Validate
          </button>
          <button
            type="button"
            onClick={() => void actions.generateUnifiedPath()}
            className="v4-focus v4-button px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            Preview Path
          </button>
          <button
            type="button"
            onClick={() => void actions.handleSaveUnifiedMission()}
            disabled={!ready}
            className="v4-focus v4-button px-3 py-2 bg-emerald-900/35 border-emerald-700 text-emerald-100 flex items-center justify-center gap-1"
          >
            <Save size={13} /> Save
          </button>
          <button
            type="button"
            onClick={() => void actions.handleRun()}
            disabled={!ready}
            className="v4-focus v4-button px-3 py-2 bg-blue-900/35 border-blue-700 text-blue-100 flex items-center justify-center gap-1"
          >
            <Play size={13} /> Launch Now
          </button>
        </div>

        <div className="text-[11px] text-[color:var(--v4-text-3)] flex items-center gap-1">
          <Rocket size={12} />
          Save and launch behavior uses existing v2 backend flow with no API changes.
        </div>
      </div>
    </Panel>
  );
}

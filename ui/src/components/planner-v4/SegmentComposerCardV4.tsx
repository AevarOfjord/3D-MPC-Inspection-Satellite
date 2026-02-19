import { ArrowDown, ArrowUp, Plus, Route, Trash2 } from 'lucide-react';

import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { mapIssuePathToPlannerStep } from '../../utils/plannerValidation';
import { Panel } from '../ui-v4/Panel';
import { InlineBanner } from '../ui-v4/InlineBanner';
import { StatusPill } from '../ui-v4/StatusPill';

interface SegmentComposerCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
  emphasizeConstraints?: boolean;
}

function segmentSummary(segment: ReturnType<typeof useMissionBuilder>['state']['segments'][number]): string {
  if (segment.type === 'transfer') {
    return `To [${segment.end_pose.position.map((value) => value.toFixed(1)).join(', ')}]`;
  }
  if (segment.type === 'scan') {
    return segment.path_asset
      ? `Path: ${segment.path_asset}`
      : `Target: ${segment.target_id || 'not set'}`;
  }
  return `Duration: ${segment.duration.toFixed(1)} s`;
}

function isCoreTransferSegment(
  segment: ReturnType<typeof useMissionBuilder>['state']['segments'][number],
  index: number,
  state: ReturnType<typeof useMissionBuilder>['state']
): boolean {
  if (segment.type !== 'transfer') return false;
  const titledIndex = state.segments.findIndex(
    (item) => item.type === 'transfer' && item.title === 'Transfer To Path'
  );
  if (titledIndex >= 0) return index === titledIndex;
  const firstTransfer = state.segments.findIndex((item) => item.type === 'transfer');
  return firstTransfer === index;
}

function segmentDisplayName(
  segment: ReturnType<typeof useMissionBuilder>['state']['segments'][number],
  index: number,
  state: ReturnType<typeof useMissionBuilder>['state']
): string {
  if (segment.type === 'scan') return 'Scan';
  if (isCoreTransferSegment(segment, index, state)) return 'Transfer To Path';
  if (segment.type === 'transfer') return 'Transfer';
  return 'Hold';
}

export function SegmentComposerCardV4({ builder, emphasizeConstraints = false }: SegmentComposerCardV4Props) {
  const { state, actions } = builder;

  const issues = state.validationReport?.issues ?? [];
  const startIssueCount = issues.filter((issue) => mapIssuePathToPlannerStep(issue.path) === 'target').length;

  const issueCountBySegment = state.segments.map((_, index) =>
    issues.filter((issue) => issue.path.includes(`segments[${index}]`)).length
  );

  return (
    <Panel
      title="Segment Composer"
      subtitle="Core mission segments + optional Transfer/Hold"
      actions={<StatusPill tone="info">{state.segments.length} Segments</StatusPill>}
    >
      <div className="space-y-3">
        {emphasizeConstraints ? (
          <InlineBanner tone="info" title="Constraint Focus">
            Select a segment below to edit speed, acceleration, and angular limits.
          </InlineBanner>
        ) : null}

        <button
          type="button"
          onClick={() => actions.selectSegment(-1)}
          className={`v4-focus w-full rounded-[10px] border px-3 py-2 text-left ${
            state.selectedSegmentIndex === -1
              ? 'border-cyan-500/80 bg-cyan-900/25'
              : 'border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)] hover:border-cyan-700/70'
          }`}
        >
          <div className="flex items-center justify-between gap-2">
            <div>
              <div className="text-[10px] uppercase tracking-[0.13em] text-emerald-300">Start</div>
              <div className="text-xs text-[color:var(--v4-text-2)]">
                {state.startFrame}
                {state.startFrame === 'LVLH' && state.startTargetId ? ` @ ${state.startTargetId}` : ''} ·
                [{state.startPosition.map((value) => value.toFixed(1)).join(', ')}]
              </div>
            </div>
            {startIssueCount > 0 ? <StatusPill tone="warning">{startIssueCount} Issues</StatusPill> : null}
          </div>
        </button>

        <div className="space-y-2 max-h-[20rem] overflow-y-auto custom-scrollbar pr-1">
          {state.segments.length === 0 ? (
            <InlineBanner tone="warning" title="No Segments Yet">
              Add at least one segment before validating or launching.
            </InlineBanner>
          ) : (
            state.segments.map((segment, index) => {
              const selected = state.selectedSegmentIndex === index;
              const issueCount = issueCountBySegment[index] ?? 0;
              const lockedCore = segment.type === 'scan' || isCoreTransferSegment(segment, index, state);
              const canMoveUp = !lockedCore && index > 0;
              const canMoveDown = !lockedCore && index < state.segments.length - 1;
              return (
                <div
                  key={segment.segment_id}
                  className={`rounded-[10px] border px-3 py-2 ${
                    selected
                      ? 'border-cyan-500/80 bg-cyan-900/25'
                      : 'border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)]'
                  }`}
                >
                  <button
                    type="button"
                    onClick={() => actions.selectSegment(index)}
                    className="v4-focus w-full text-left"
                  >
                    <div className="flex items-center justify-between gap-2">
                      <div className="min-w-0">
                        <div className="text-[11px] uppercase tracking-[0.12em] text-[color:var(--v4-text-3)]">
                          {index + 1}. {segmentDisplayName(segment, index, state)}
                        </div>
                        <div className="text-xs text-[color:var(--v4-text-2)] truncate">
                          {segmentSummary(segment)}
                        </div>
                      </div>
                      <div className="flex items-center gap-1">
                        {issueCount > 0 ? <StatusPill tone="warning">{issueCount}</StatusPill> : null}
                      </div>
                    </div>
                  </button>
                  <div className="mt-2 flex items-center gap-1">
                    <button
                      type="button"
                      onClick={() => actions.reorderSegments(index, Math.max(0, index - 1))}
                      disabled={!canMoveUp}
                      className="v4-focus v4-button px-2 py-1 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
                    >
                      <ArrowUp size={12} />
                    </button>
                    <button
                      type="button"
                      onClick={() => actions.reorderSegments(index, Math.min(state.segments.length - 1, index + 1))}
                      disabled={!canMoveDown}
                      className="v4-focus v4-button px-2 py-1 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
                    >
                      <ArrowDown size={12} />
                    </button>
                    {lockedCore ? (
                      <div className="ml-auto text-[10px] uppercase tracking-[0.12em] text-[color:var(--v4-text-3)] px-2">
                        Core
                      </div>
                    ) : (
                      <button
                        type="button"
                        onClick={() => actions.removeSegment(index)}
                        className="v4-focus v4-button ml-auto px-2 py-1 bg-red-900/35 border-red-700/70 text-red-200"
                      >
                        <Trash2 size={12} />
                      </button>
                    )}
                  </div>
                </div>
              );
            })
          )}
        </div>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => actions.addTransferSegment()}
            className="v4-focus v4-button px-2 py-2 bg-blue-900/35 border-blue-700/70 text-blue-100 flex items-center justify-center gap-1"
          >
            <Route size={12} /> Transfer
          </button>
          <button
            type="button"
            onClick={() => actions.addHoldSegment()}
            className="v4-focus v4-button px-2 py-2 bg-amber-900/35 border-amber-700/70 text-amber-100 flex items-center justify-center gap-1"
          >
            <Plus size={12} /> Hold
          </button>
        </div>
      </div>
    </Panel>
  );
}

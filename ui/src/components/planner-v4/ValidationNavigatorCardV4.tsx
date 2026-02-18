import type { ValidationIssueV2 } from '../../api/unifiedMissionApi';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { mapIssuePathToPlannerStep } from '../../utils/plannerValidation';
import { Panel } from '../ui-v4/Panel';
import { InlineBanner } from '../ui-v4/InlineBanner';
import { StatusPill } from '../ui-v4/StatusPill';

interface ValidationNavigatorCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

type Severity = 'error' | 'warning' | 'info';

const SEVERITIES: Severity[] = ['error', 'warning', 'info'];

const STEP_LABEL: Record<ReturnType<typeof mapIssuePathToPlannerStep>, string> = {
  target: 'Target',
  segments: 'Segments',
  scan_definition: 'Scan Definition',
  constraints: 'Constraints',
  validate: 'Validate',
  save_launch: 'Save/Launch',
};

const severityTone: Record<Severity, 'danger' | 'warning' | 'info'> = {
  error: 'danger',
  warning: 'warning',
  info: 'info',
};

function groupIssuesBySeverity(issues: ValidationIssueV2[]) {
  return {
    error: issues.filter((issue) => issue.severity === 'error'),
    warning: issues.filter((issue) => issue.severity === 'warning'),
    info: issues.filter((issue) => issue.severity === 'info'),
  };
}

export function ValidationNavigatorCardV4({ builder }: ValidationNavigatorCardV4Props) {
  const { state, actions } = builder;
  const report = state.validationReport;
  const grouped = groupIssuesBySeverity(report?.issues ?? []);

  const jumpToIssue = (issue: ValidationIssueV2) => {
    const step = mapIssuePathToPlannerStep(issue.path);
    const segmentMatch = /segments\[(\d+)\]/.exec(issue.path);
    if (segmentMatch) {
      const index = Number.parseInt(segmentMatch[1], 10);
      if (!Number.isNaN(index)) actions.selectSegment(index);
    }
    actions.setAuthoringStep(step);
  };

  return (
    <Panel
      title="Step 5 · Validate"
      subtitle="Grouped issue navigator with click-to-field guidance"
      actions={
        <button
          type="button"
          onClick={() => void actions.validateUnifiedMission()}
          disabled={state.validationBusy}
          className="v4-focus v4-button px-2 py-1.5 bg-cyan-900/35 border-cyan-700 text-cyan-100"
        >
          {state.validationBusy ? 'Validating...' : 'Re-run'}
        </button>
      }
    >
      <div id="coachmark-validation" className="space-y-3">
        {!report ? (
          <InlineBanner tone="info" title="Run Validation">
            Validation report is empty. Run validation to inspect issues before save/launch.
          </InlineBanner>
        ) : report.valid ? (
          <InlineBanner tone="success" title="Validation Pass">
            Mission is valid and ready for Save/Launch.
          </InlineBanner>
        ) : (
          <InlineBanner tone="warning" title="Validation Requires Attention">
            Resolve all errors before saving or launching.
          </InlineBanner>
        )}

        {report ? (
          <div className="flex items-center gap-2 text-xs">
            <StatusPill tone="danger">Errors: {report.summary.errors}</StatusPill>
            <StatusPill tone="warning">Warnings: {report.summary.warnings}</StatusPill>
            <StatusPill tone="info">Info: {report.summary.info}</StatusPill>
          </div>
        ) : null}

        {SEVERITIES.map((severity) => {
          const issues = grouped[severity];
          if (issues.length === 0) return null;
          return (
            <details key={severity} open={severity !== 'info'} className="v4-subtle-panel p-3">
              <summary className="cursor-pointer text-xs uppercase tracking-[0.14em] font-semibold text-[color:var(--v4-text-2)]">
                {severity} ({issues.length})
              </summary>
              <div className="mt-3 space-y-2 max-h-[18rem] overflow-y-auto custom-scrollbar pr-1">
                {issues.map((issue, idx) => (
                  <button
                    key={`${issue.code}-${idx}`}
                    type="button"
                    onClick={() => jumpToIssue(issue)}
                    className="v4-focus w-full text-left rounded-[10px] border border-[color:var(--v4-border)] bg-[color:var(--v4-surface-1)] px-3 py-2 hover:border-cyan-700/70"
                  >
                    <div className="flex items-center justify-between gap-2">
                      <div className="text-[11px] font-semibold uppercase tracking-[0.12em]">
                        {issue.code}
                      </div>
                      <StatusPill tone={severityTone[severity]}>{STEP_LABEL[mapIssuePathToPlannerStep(issue.path)]}</StatusPill>
                    </div>
                    <div className="text-xs mt-1 text-[color:var(--v4-text-2)]">{issue.message}</div>
                    <div className="text-[10px] mt-1 text-[color:var(--v4-text-3)]">Path: {issue.path}</div>
                    {issue.suggestion ? (
                      <div className="text-[10px] mt-1 text-cyan-200">Suggestion: {issue.suggestion}</div>
                    ) : null}
                  </button>
                ))}
              </div>
            </details>
          );
        })}
      </div>
    </Panel>
  );
}

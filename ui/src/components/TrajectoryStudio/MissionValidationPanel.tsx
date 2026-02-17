import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import type { ValidationIssueV2 } from '../../api/unifiedMissionApi';

interface MissionValidationPanelProps {
  builder: ReturnType<typeof useMissionBuilder>;
}

const severityClass: Record<'error' | 'warning' | 'info', string> = {
  error: 'border-red-500/40 bg-red-950/30 text-red-100',
  warning: 'border-amber-500/40 bg-amber-950/30 text-amber-100',
  info: 'border-slate-600 bg-slate-900/40 text-slate-100',
};

export function MissionValidationPanel({ builder }: MissionValidationPanelProps) {
  const { state, actions } = builder;
  const report = state.validationReport;
  const issues: ValidationIssueV2[] = report?.issues ?? [];

  const navigateToIssue = (path: string) => {
    const match = /segments\[(\d+)\]/.exec(path);
    if (!match) {
      actions.setAuthoringStep('target');
      return;
    }
    const index = Number.parseInt(match[1], 10);
    if (!Number.isNaN(index)) {
      actions.selectSegment(index);
      actions.setAuthoringStep('constraints');
    }
  };

  return (
    <div className="w-[30rem] bg-slate-950/90 backdrop-blur-md border border-slate-800 rounded-lg shadow-2xl p-3">
      <div className="flex items-center justify-between mb-2">
        <h3 className="text-sm font-semibold text-slate-100">Validation</h3>
        <button
          type="button"
          onClick={() => void actions.validateUnifiedMission()}
          className="px-2 py-1 text-xs border border-cyan-700 rounded text-cyan-200 hover:bg-cyan-900/30"
          disabled={state.validationBusy}
        >
          {state.validationBusy ? 'Validating...' : 'Re-run'}
        </button>
      </div>
      {report ? (
        <div className="text-xs text-slate-300 mb-2">
          valid={String(report.valid)} | errors={report.summary.errors} | warnings={report.summary.warnings}
        </div>
      ) : (
        <div className="text-xs text-slate-400 mb-2">Run validation to see issues.</div>
      )}
      <div className="max-h-72 overflow-y-auto space-y-2 custom-scrollbar">
        {issues.length === 0 ? (
          <div className="text-xs text-slate-400">No issues detected.</div>
        ) : (
          issues.map((issue: ValidationIssueV2, idx: number) => (
            <button
              key={`${issue.code}-${idx}`}
              type="button"
              onClick={() => navigateToIssue(issue.path)}
              className={`w-full text-left rounded border px-2 py-2 ${severityClass[issue.severity]}`}
            >
              <div className="text-[11px] font-semibold uppercase tracking-wide">
                {issue.severity} · {issue.code}
              </div>
              <div className="text-xs mt-1">{issue.message}</div>
              <div className="text-[11px] opacity-80 mt-1">Path: {issue.path}</div>
              {issue.suggestion ? (
                <div className="text-[11px] opacity-90 mt-1">Fix: {issue.suggestion}</div>
              ) : null}
            </button>
          ))
        )}
      </div>
    </div>
  );
}

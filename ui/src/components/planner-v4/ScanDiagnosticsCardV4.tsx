import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { mapIssuePathToPlannerStep } from '../../utils/plannerValidation';
import { Panel } from '../ui-v4/Panel';
import { InlineBanner } from '../ui-v4/InlineBanner';

interface ScanDiagnosticsCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

export function ScanDiagnosticsCardV4({ builder }: ScanDiagnosticsCardV4Props) {
  const { state } = builder;
  const diagnostics = state.compilePreviewState;
  const scanDefinitionIssues = (state.validationReport?.issues ?? []).filter(
    (issue) => mapIssuePathToPlannerStep(issue.path) === 'scan_definition'
  );

  return (
    <Panel title="Diagnostics" subtitle="Preview and scan-definition quality signals">
      <div className="space-y-3">
        {scanDefinitionIssues.length > 0 ? (
          <InlineBanner tone="warning" title="Validation Issues">
            {scanDefinitionIssues.length} scan-definition issue(s) detected. Open Validate and jump to the affected fields.
          </InlineBanner>
        ) : (
          <InlineBanner tone="success" title="Validation">
            No scan-definition issues detected.
          </InlineBanner>
        )}

        {diagnostics ? (
          <div className="v4-subtle-panel p-3 text-xs text-[color:var(--v4-text-2)] space-y-1.5">
            <div>Path points: {diagnostics.points}</div>
            <div>Length: {diagnostics.path_length.toFixed(2)} m</div>
            <div>Estimated duration: {diagnostics.estimated_duration.toFixed(1)} s</div>
            <div>
              Min clearance:{' '}
              {diagnostics.diagnostics.min_clearance_m == null
                ? 'n/a'
                : `${diagnostics.diagnostics.min_clearance_m.toFixed(3)} m`}
            </div>
            <div>Collision points: {diagnostics.diagnostics.collision_points_count}</div>
            {diagnostics.diagnostics.warnings.length > 0 ? (
              <InlineBanner tone="warning" title="Warnings" className="mt-2">
                {diagnostics.diagnostics.warnings.join(' | ')}
              </InlineBanner>
            ) : null}
          </div>
        ) : (
          <div className="text-xs text-[color:var(--v4-text-3)]">Run preview to populate diagnostics.</div>
        )}
      </div>
    </Panel>
  );
}

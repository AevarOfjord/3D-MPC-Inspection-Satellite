import type { TelemetryData } from '../../services/telemetry';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { Overlay } from '../Overlay';
import { TelemetryCharts } from '../TelemetryCharts';
import { UnifiedViewport } from '../UnifiedViewport';

interface ViewerModeViewProps {
  viewMode: 'free' | 'chase' | 'top';
  builder: ReturnType<typeof useMissionBuilder>;
  latestTelemetry: TelemetryData | null;
}

export function ViewerModeView({ viewMode, builder }: ViewerModeViewProps) {
  return (
    <div className="flex-1 relative">
      <UnifiedViewport
        mode="viewer"
        viewMode={viewMode}
        builderState={builder.state}
        builderActions={builder.actions}
      />
      <Overlay />
      <TelemetryCharts />
    </div>
  );
}

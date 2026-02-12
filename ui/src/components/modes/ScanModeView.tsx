import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { UnifiedViewport } from '../UnifiedViewport';
import { TrajectoryStudioLayout } from '../TrajectoryStudio/TrajectoryStudioLayout';

interface ScanModeViewProps {
  viewMode: 'free' | 'chase' | 'top';
  builder: ReturnType<typeof useMissionBuilder>;
}

export function ScanModeView({ viewMode, builder }: ScanModeViewProps) {
  return (
    <TrajectoryStudioLayout
      builder={builder}
      showPathStudio={true}
      showGeneratorStack={false}
      showTimeline={false}
      showInspector={false}
      viewport={
        <div className="absolute inset-0 z-0">
          <UnifiedViewport
            mode="scan"
            viewMode={viewMode}
            builderState={builder.state}
            builderActions={builder.actions}
          />
        </div>
      }
    />
  );
}


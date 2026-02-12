import { Suspense } from 'react';
import { useCameraStore } from '../../store/cameraStore';
import type { TelemetryData } from '../../services/telemetry';
import { ORBIT_SCALE } from '../../data/orbitSnapshot';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { Overlay } from '../Overlay';
import { OrbitTargetsPanel } from '../OrbitTargetsPanel';
import { TelemetryCharts } from '../TelemetryCharts';
import { UnifiedViewport } from '../UnifiedViewport';

interface ViewerModeViewProps {
  viewMode: 'free' | 'chase' | 'top';
  builder: ReturnType<typeof useMissionBuilder>;
  latestTelemetry: TelemetryData | null;
}

export function ViewerModeView({ viewMode, builder, latestTelemetry }: ViewerModeViewProps) {
  return (
    <div className="flex-1 relative">
      <UnifiedViewport
        mode="viewer"
        viewMode={viewMode}
        builderState={builder.state}
        builderActions={builder.actions}
      />
      <Suspense fallback={null}>
        <OrbitTargetsPanel
          className="fixed right-6 top-1/2 -translate-y-1/2"
          selectedTargetId={null}
          ownSatellite={{
            id: 'SATELLITE',
            name: 'Your Satellite',
            positionScene: latestTelemetry
              ? [
                  latestTelemetry.position[0] * ORBIT_SCALE,
                  latestTelemetry.position[1] * ORBIT_SCALE,
                  latestTelemetry.position[2] * ORBIT_SCALE,
                ]
              : [0, 0, 0],
            positionMeters: latestTelemetry?.position,
          }}
          onFocusTarget={(_targetId, positionScene, focusDistance) => {
            useCameraStore.getState().requestFocus(positionScene, focusDistance);
          }}
        />
      </Suspense>

      <Overlay />
      <TelemetryCharts />
    </div>
  );
}

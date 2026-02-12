import { Suspense } from 'react';
import { useCameraStore } from '../../store/cameraStore';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { ORBIT_SCALE, orbitSnapshot } from '../../data/orbitSnapshot';
import { UnifiedViewport } from '../UnifiedViewport';
import { OrbitTargetsPanel } from '../OrbitTargetsPanel';
import { TrajectoryStudioLayout } from '../TrajectoryStudio/TrajectoryStudioLayout';

interface MissionModeViewProps {
  viewMode: 'free' | 'chase' | 'top';
  builder: ReturnType<typeof useMissionBuilder>;
}

export function MissionModeView({ viewMode, builder }: MissionModeViewProps) {
  return (
    <TrajectoryStudioLayout
      builder={builder}
      showPathStudio={false}
      showGeneratorStack={true}
      showTimeline={true}
      showInspector={true}
      viewport={
        <div className="absolute inset-0 z-0">
          <UnifiedViewport
            mode="mission"
            viewMode={viewMode}
            builderState={builder.state}
            builderActions={builder.actions}
          />
          <Suspense fallback={null}>
            <OrbitTargetsPanel
              selectedTargetId={builder.state.selectedOrbitTargetId}
              ownSatellite={{
                id: 'SATELLITE',
                name: 'Your Satellite',
                positionScene: [
                  builder.state.startPosition[0] * ORBIT_SCALE,
                  builder.state.startPosition[1] * ORBIT_SCALE,
                  builder.state.startPosition[2] * ORBIT_SCALE,
                ],
                positionMeters: [
                  builder.state.startPosition[0],
                  builder.state.startPosition[1],
                  builder.state.startPosition[2],
                ],
              }}
              solarBodies={[]}
              onFocusTarget={(targetId, _positionScene, focusDistance) => {
                const originTargetId = builder.state.selectedOrbitTargetId || builder.state.startTargetId;
                const originObj = originTargetId
                  ? orbitSnapshot.objects.find((o) => o.id === originTargetId)
                  : null;
                const targetObj = orbitSnapshot.objects.find((o) => o.id === targetId);
                if (!targetObj) return;
                const origin = originObj?.position_m ?? [0, 0, 0];
                const scenePos: [number, number, number] = [
                  (targetObj.position_m[0] - origin[0]) * ORBIT_SCALE,
                  (targetObj.position_m[1] - origin[1]) * ORBIT_SCALE,
                  (targetObj.position_m[2] - origin[2]) * ORBIT_SCALE,
                ];
                useCameraStore.getState().requestFocus(scenePos, focusDistance);
              }}
            />
          </Suspense>
        </div>
      }
    />
  );
}


import { TelemetryPanel } from './TelemetryPanel';
import { ControllerActuatorPanel } from './ControllerActuatorPanel';
import { OrbitTargetsPanel } from './OrbitTargetsPanel';
import { useTelemetryStore } from '../store/telemetryStore';
import { useCameraStore } from '../store/cameraStore';
import { ORBIT_SCALE } from '../data/orbitSnapshot';

export function Overlay() {
  const latestTelemetry = useTelemetryStore(s => s.latest);
  const requestFocus = useCameraStore(s => s.requestFocus);

  const ownSatellite = {
    id: 'SATELLITE',
    name: 'Your Satellite',
    positionScene: latestTelemetry
      ? [
          latestTelemetry.position[0] * ORBIT_SCALE,
          latestTelemetry.position[1] * ORBIT_SCALE,
          latestTelemetry.position[2] * ORBIT_SCALE,
        ] as [number, number, number]
      : [0, 0, 0] as [number, number, number],
    positionMeters: latestTelemetry?.position,
  };

  return (
    <div className="absolute inset-0 pointer-events-none p-4 z-10">
      {/* Left column: Telemetry → Controller/Actuators → Orbital Targets */}
      <div className="absolute top-4 left-4 pointer-events-auto flex flex-col gap-2">
        <TelemetryPanel />
        <ControllerActuatorPanel />
        <OrbitTargetsPanel
          ownSatellite={ownSatellite}
          onFocusTarget={(_id, positionScene, focusDistance) => {
            requestFocus(positionScene, focusDistance);
          }}
        />
      </div>
    </div>
  );
}

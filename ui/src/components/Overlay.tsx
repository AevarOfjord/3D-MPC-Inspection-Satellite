import { TelemetryPanel } from './TelemetryPanel';
import { ControllerActuatorPanel } from './ControllerActuatorPanel';

export function Overlay() {
  return (
    <div className="absolute inset-0 pointer-events-none p-4 z-10">
      {/* Top-left: Telemetry */}
      <div className="absolute top-4 left-4 pointer-events-auto">
        <TelemetryPanel />
      </div>

      {/* Top-right: Controller + Actuators */}
      <div className="absolute top-4 right-4 pointer-events-auto">
        <ControllerActuatorPanel />
      </div>
    </div>
  );
}

import { useStudioStore } from './useStudioStore';
import { SatellitePreview } from '../viewport/SatellitePreview';

export function SatelliteStartNode() {
  const satelliteStart = useStudioStore((s) => s.satelliteStart);

  return (
    <SatellitePreview position={satelliteStart} rotation={[0, 0, 0]} />
  );
}

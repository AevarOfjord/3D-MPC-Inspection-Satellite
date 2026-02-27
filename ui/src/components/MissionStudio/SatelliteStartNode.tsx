import { useStudioStore } from './useStudioStore';
import { SatellitePreview } from '../viewport/SatellitePreview';

export function SatelliteStartNode() {
  const assembly = useStudioStore((s) => s.assembly);
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);
  const satelliteStart = useStudioStore((s) => s.satelliteStart);
  const selectedAssembly = selectedAssemblyId ? assembly.find((a) => a.id === selectedAssemblyId) ?? null : null;
  const visible = selectedAssembly == null || selectedAssembly.type === 'place_satellite';

  if (!visible) return null;

  return (
    <SatellitePreview position={satelliteStart} rotation={[0, 0, 0]} />
  );
}

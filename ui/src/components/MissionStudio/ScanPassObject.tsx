import { useMemo, useEffect } from 'react';
import * as THREE from 'three';
import { useStudioStore } from './useStudioStore';
import { generateSpiral } from './useSpiralGenerator';

interface ScanPassObjectProps {
  scanId: string;
}

export function ScanPassObject({ scanId }: ScanPassObjectProps) {
  const pass = useStudioStore((s) => s.scanPasses.find((p) => p.id === scanId));
  const updateScanPass = useStudioStore((s) => s.updateScanPass);
  const selectedScanId = useStudioStore((s) => s.selectedScanId);

  // eslint-disable-next-line react-hooks/exhaustive-deps
  useEffect(() => {
    if (!pass) return;
    const waypoints = generateSpiral({
      axis: pass.axis,
      planeAOffset: pass.planeAOffset,
      planeBOffset: pass.planeBOffset,
      crossSection: pass.crossSection,
      levelHeight: pass.levelHeight,
    });
    updateScanPass(pass.id, { waypoints });
  // We intentionally serialize crossSection to detect changes
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pass?.axis, pass?.planeAOffset, pass?.planeBOffset, pass?.levelHeight,
      // eslint-disable-next-line react-hooks/exhaustive-deps
      JSON.stringify(pass?.crossSection)]);

  const lineGeometry = useMemo(() => {
    if (!pass || pass.waypoints.length < 2) return null;
    const points = pass.waypoints.map(([x, y, z]) => new THREE.Vector3(x, y, z));
    return new THREE.BufferGeometry().setFromPoints(points);
  }, [pass?.waypoints]);

  const startPos = pass?.waypoints[0] ?? null;
  const endPos = pass && pass.waypoints.length > 0 ? pass.waypoints[pass.waypoints.length - 1] : null;
  const isSelected = selectedScanId === scanId;
  const color = pass?.color ?? '#22d3ee';

  if (!pass || !lineGeometry) return null;

  return (
    <group>
      <line geometry={lineGeometry}>
        <lineBasicMaterial color={color} linewidth={isSelected ? 2 : 1} opacity={isSelected ? 1 : 0.7} transparent />
      </line>

      {startPos && (
        <mesh position={startPos} onClick={() => useStudioStore.getState().selectScanPass(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#22d3ee" />
        </mesh>
      )}
      {endPos && (
        <mesh position={endPos} onClick={() => useStudioStore.getState().selectScanPass(scanId)}>
          <sphereGeometry args={[0.3, 16, 16]} />
          <meshBasicMaterial color="#a78bfa" />
        </mesh>
      )}

      <PlaneIndicator axis={pass.axis} offset={pass.planeAOffset} color={color} />
      <PlaneIndicator axis={pass.axis} offset={pass.planeBOffset} color={color} />
    </group>
  );
}

function PlaneIndicator({ axis, offset, color }: { axis: 'X' | 'Y' | 'Z'; offset: number; color: string }) {
  const pos: [number, number, number] =
    axis === 'X' ? [offset, 0, 0] :
    axis === 'Y' ? [0, offset, 0] :
                   [0, 0, offset];
  const rot: [number, number, number] =
    axis === 'X' ? [0, 0, Math.PI / 2] :
    axis === 'Y' ? [0, 0, 0] :
                   [Math.PI / 2, 0, 0];
  return (
    <mesh position={pos} rotation={rot}>
      <ringGeometry args={[4.5, 5, 32]} />
      <meshBasicMaterial color={color} opacity={0.3} transparent side={THREE.DoubleSide} />
    </mesh>
  );
}

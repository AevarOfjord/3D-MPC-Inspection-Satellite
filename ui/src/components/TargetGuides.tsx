import { useEffect, useMemo, useState } from 'react';
import { Line } from '@react-three/drei';
import { Euler, Quaternion } from 'three';
import { telemetry } from '../services/telemetry';

export function TargetGuides() {
  const [targetPos, setTargetPos] = useState<[number, number, number] | null>(null);
  const [targetOri, setTargetOri] = useState<[number, number, number] | null>(null);
  const [currentPos, setCurrentPos] = useState<[number, number, number] | null>(null);

  useEffect(() => {
    const unsub = telemetry.subscribe((data) => {
      setTargetPos(data.target_position);
      setTargetOri(data.target_orientation || [0, 0, 0]);
      setCurrentPos(data.position);
    });
    return () => { unsub(); };
  }, []);

  if (!targetPos || !targetOri || !currentPos) return null;

  const targetQuat = useMemo(() => {
    const euler = new Euler(targetOri[0], targetOri[1], targetOri[2], 'XYZ');
    return new Quaternion().setFromEuler(euler);
  }, [targetOri]);

  return (
    <>
      <group position={targetPos} quaternion={targetQuat}>
        <mesh>
          <boxGeometry args={[0.3, 0.3, 0.3]} />
          <meshBasicMaterial color="#ff9b45" wireframe transparent opacity={0.6} />
        </mesh>
        <axesHelper args={[0.4]} />
      </group>

      <Line
        points={[currentPos, targetPos]}
        color="#ff9b45"
        lineWidth={1}
        dashed
        dashScale={2}
        dashSize={0.3}
        gapSize={0.2}
      />
    </>
  );
}

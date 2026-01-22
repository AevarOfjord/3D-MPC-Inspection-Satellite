import { useEffect, useState } from 'react';
import { Line } from '@react-three/drei';
import { telemetry } from '../services/telemetry';

export function ReferenceGuides() {
  const [referencePos, setReferencePos] = useState<[number, number, number] | null>(null);
  const [referenceQuat, setReferenceQuat] = useState<[number, number, number, number] | null>(null);
  const [currentPos, setCurrentPos] = useState<[number, number, number] | null>(null);

  useEffect(() => {
    const unsub = telemetry.subscribe((data) => {
      if (!data) return;
      // Strict Validation
      if (!data.reference_position || !data.position) return;
      if (data.reference_position.some(v => !Number.isFinite(v))) return;
      if (data.position.some(v => !Number.isFinite(v))) return;
      
      setReferencePos(data.reference_position);
      setReferenceQuat(data.reference_quaternion ?? [1, 0, 0, 0]);
      setCurrentPos(data.position);
    });
    return () => { unsub(); };
  }, []);

  if (!referencePos || !currentPos || !referenceQuat) return null;
  const [w, x, y, z] = referenceQuat;

  return (
    <>
      <group position={referencePos} quaternion={[x, y, z, w]}>
        <mesh>
          <boxGeometry args={[0.3, 0.3, 0.3]} />
          <meshBasicMaterial color="#ff9b45" wireframe transparent opacity={0.6} />
        </mesh>
        <axesHelper args={[0.4]} />
      </group>

      <Line
        points={[currentPos, referencePos]}
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

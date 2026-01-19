import { useEffect, useState } from 'react';
import { Line } from '@react-three/drei';
import { Vector3 } from 'three';
import { telemetry } from '../services/telemetry';

import { TargetMarker } from './Earth';

export function PlannedPath() {
  const [path, setPath] = useState<Vector3[]>([]);

  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       if (d.planned_path && d.planned_path.length > 0) {
           setPath(d.planned_path.map(p => new Vector3(...p)));
       } else {
           setPath([]);
       }
    });
    return () => { unsub(); };
  }, []);

  if (path.length < 2) return null;

  const lastPoint = path[path.length - 1];

  return (
    <group>
        <Line
          points={path}       // Array of Vector3
          color="yellow"      // Default
          lineWidth={2}       // In pixels (default)
          dashed={true}       // Default
          dashScale={2}       // Default
          dashSize={1}        // Default
          gapSize={0.5}       // Default
        />
        <TargetMarker 
            position={[lastPoint.x, lastPoint.y, lastPoint.z]} 
            color="#4ade80" // Green-400 equivalent or similar bright green
        />
    </group>
  );
}

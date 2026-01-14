import { useEffect, useState } from 'react';
import { Line } from '@react-three/drei';
import { Vector3 } from 'three';
import { telemetry } from '../services/telemetry';

const MAX_POINTS = 2000;
const MIN_DISTANCE = 0.05; // Meters. Don't add point if haven't moved much.

export function Trajectory() {
  const [points, setPoints] = useState<Vector3[]>([]);

  useEffect(() => {
    // Subscribe to telemetry
    const unsubscribe = telemetry.subscribe((data) => {
      const [x, y, z] = data.position;
      
      // Convert to visual coordinates (1/1000 scale, Z-up to Y-up)
      const vizPos = new Vector3(x / 1000, z / 1000, -y / 1000);

      setPoints(prev => {
        const last = prev.length > 0 ? prev[prev.length - 1] : null;
        if (last && last.distanceTo(vizPos) < MIN_DISTANCE) {
          return prev;
        }
        
        const newPoints = [...prev, vizPos];
        if (newPoints.length > MAX_POINTS) {
          return newPoints.slice(newPoints.length - MAX_POINTS);
        }
        return newPoints;
      });
    });

    return () => { unsubscribe(); };
  }, []);

  if (points.length < 2) return null;

  return (
    <Line
      points={points}
      color="cyan"
      lineWidth={1}
      opacity={0.5}
      transparent
    />
  );
}

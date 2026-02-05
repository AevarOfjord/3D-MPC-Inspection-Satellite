// PlannedPath.tsx
import { useEffect, useState, useMemo } from 'react';
import { Vector3, CatmullRomCurve3 } from 'three';
import { Line } from '@react-three/drei';
import { telemetry } from '../services/telemetry';
import { ReferenceMarker } from './Earth';

interface PlannedPathProps {
  origin?: [number, number, number];
}

const MIN_DISTANCE = 1e-4; // Slightly larger epsilon for spline stability

export function PlannedPath({ origin = [0, 0, 0] }: PlannedPathProps) {
  const [path, setPath] = useState<Vector3[]>([]);
  const originVec = useMemo(() => new Vector3(...origin), [origin[0], origin[1], origin[2]]);

  // Store path in World Coordinates (ECI)
  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       if (d.planned_path && d.planned_path.length > 0) {
           const rawPoints = d.planned_path.map(p => new Vector3(...p));
           // Filter coincident points
           const uniquePoints: Vector3[] = [];
           for (const p of rawPoints) {
             if (uniquePoints.length === 0 || uniquePoints[uniquePoints.length - 1].distanceTo(p) >= MIN_DISTANCE) {
                uniquePoints.push(p);
             }
           }
           setPath(uniquePoints);
       } else if (d.planned_path && d.planned_path.length === 0) {
           setPath([]);
       }
    });
    return () => { unsub(); };
  }, []);

  // Compute View Coordinates and Spline
  const displayPath = useMemo(() => {
    if (path.length < 2) return [];
    
    // Project world points to view space
    const viewPoints = path.map(p => p.clone().sub(originVec));
    
    // Apply smoothing for better visualization of sparse waypoints
    // We only have ~800 points for a complex spiral, so native lines look jagged.
    // CatmullRom restores the "Mission Planner" smooth look.
    const curve = new CatmullRomCurve3(viewPoints, false, 'centripetal');
    
    // Adaptive sample count: roughly 10 samples per segment
    const sampleCount = Math.max(viewPoints.length * 10, 200);
    return curve.getPoints(sampleCount);
  }, [path, originVec]);

  if (displayPath.length < 2) return null;

  const lastPoint = displayPath[displayPath.length - 1];

  return (
    <>
        <Line
            points={displayPath}
            color="yellow"
            lineWidth={2}
            opacity={0.8}
            transparent
            dashed={false}
        />
        <ReferenceMarker 
            position={[lastPoint.x, lastPoint.y, lastPoint.z]} 
            color="#4ade80" 
        />
    </>
  );
}

import { useEffect, useState, useMemo } from 'react';
import { Line } from '@react-three/drei';
import { Vector3 } from 'three';
import { telemetry } from '../services/telemetry';
import { useTelemetryStore } from '../store/telemetryStore';

const MAX_POINTS = 2000;
const MIN_DISTANCE = 0.05; // Meters. Don't add point if haven't moved much.

export function Trajectory() {
  const [livePoints, setLivePoints] = useState<Vector3[]>([]);
  
  // Get playback data and index from store for proper trajectory during playback scrubbing
  const playbackData = useTelemetryStore(s => s.playbackData);
  const playbackIndex = useTelemetryStore(s => s.playbackIndex);
  
  // Compute playback trajectory from all points up to current index
  const playbackPoints = useMemo(() => {
    if (!playbackData.length) return [];
    
    const endIdx = Math.min(playbackIndex + 1, playbackData.length);
    const points: Vector3[] = [];
    
    for (let i = 0; i < endIdx; i++) {
      const data = playbackData[i];
      if (!data || !data.position) continue;
      const [x, y, z] = data.position;
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      
      const vizPos = new Vector3(x, y, z);
      
      // Apply distance filter to reduce point count
      if (points.length > 0) {
        const last = points[points.length - 1];
        if (last.distanceTo(vizPos) < MIN_DISTANCE) continue;
      }
      
      points.push(vizPos);
    }
    
    // Limit to MAX_POINTS
    if (points.length > MAX_POINTS) {
      return points.slice(points.length - MAX_POINTS);
    }
    return points;
  }, [playbackData, playbackIndex]);

  // Live mode: Subscribe to telemetry for real-time updates
  useEffect(() => {
    const unsubscribe = telemetry.subscribe((data) => {
      // Only accumulate live points if NOT in playback mode
      if (playbackData.length > 0) return;
      
      if (!data || !data.position) return;
      const [x, y, z] = data.position;
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return;
      
      const vizPos = new Vector3(x, y, z);

      setLivePoints(prev => {
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
  }, [playbackData.length]);

  // Use playback points if in playback mode, otherwise use live points
  const points = playbackData.length > 0 ? playbackPoints : livePoints;

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

// Trajectory.tsx
import { useEffect, useState, useMemo, useRef, useLayoutEffect } from 'react';
import { Vector3, BufferGeometry, Float32BufferAttribute, BufferAttribute, DynamicDrawUsage } from 'three';
import { telemetry } from '../services/telemetry';
import { useTelemetryStore } from '../store/telemetryStore';

const MAX_POINTS = 1000000;
const MIN_DISTANCE = 1e-5; // Meters

interface TrajectoryProps {
  origin?: [number, number, number];
}

export function Trajectory({ origin = [0, 0, 0] }: TrajectoryProps) {
  const [livePoints, setLivePoints] = useState<Vector3[]>([]);
  const geometryRef = useRef<BufferGeometry>(null);
  
  // Pre-allocate buffer for maximum performance and to avoid allocation warnings
  const [positionBuffer] = useState(() => new Float32Array(MAX_POINTS * 3));
  
  const playbackData = useTelemetryStore(s => s.playbackData);
  const playbackIndex = useTelemetryStore(s => s.playbackIndex);
  
  const originVec = useMemo(() => new Vector3(...origin), [origin[0], origin[1], origin[2]]);

  // Compute playback trajectory (View Coordinates)
  const playbackPoints = useMemo(() => {
    if (!playbackData.length) return [];
    
    const endIdx = Math.min(playbackIndex + 1, playbackData.length);
    const points: Vector3[] = [];
    
    // Optimization: If > 100k points, we might want to skip this recalculation every frame?
    // But for < 100k it's fine. For 1M it might be laggy. 
    // Given the request for precision, we do it.
    
    const viewX = originVec.x;
    const viewY = originVec.y;
    const viewZ = originVec.z;

    for (let i = 0; i < endIdx; i++) {
        const data = playbackData[i];
        if (!data || !data.position) continue;
        
        // Manual subtraction for speed
        const px = data.position[0] - viewX;
        const py = data.position[1] - viewY;
        const pz = data.position[2] - viewZ;
        
        // Simple distance filter check against last point
        if (points.length > 0) {
            const last = points[points.length - 1];
            const dx = px - last.x;
            const dy = py - last.y;
            const dz = pz - last.z;
            if ((dx*dx + dy*dy + dz*dz) < MIN_DISTANCE*MIN_DISTANCE) continue;
        }
        
        points.push(new Vector3(px, py, pz));
    }
    
    if (points.length > MAX_POINTS) {
        return points.slice(points.length - MAX_POINTS);
    }
    return points;
  }, [playbackData, playbackIndex, originVec]);

  // Live mode: Subscribe and store World Coordinates
  useEffect(() => {
    const unsubscribe = telemetry.subscribe((data) => {
      if (playbackData.length > 0) return;
      if (!data || !data.position) return;
      
      // Store Raw World Position
      const pos = new Vector3(data.position[0], data.position[1], data.position[2]);

      setLivePoints(prev => {
        const last = prev.length > 0 ? prev[prev.length - 1] : null;
        if (last && last.distanceTo(pos) < MIN_DISTANCE) return prev;
        
        const newPoints = [...prev, pos];
        if (newPoints.length > MAX_POINTS) {
          return newPoints.slice(newPoints.length - MAX_POINTS);
        }
        return newPoints;
      });
    });

    return () => { unsubscribe(); };
  }, [playbackData.length]);

  // Combine and Project
  const displayPoints = useMemo(() => {
    if (playbackData.length > 0) return playbackPoints;
    
    // Project Live Points to View Space
    // This is fast enough for typical live data counts
    return livePoints.map(p => p.clone().sub(originVec));
  }, [playbackData.length, playbackPoints, livePoints, originVec]);

  // Update Geometry using BufferAttribute directly
  useLayoutEffect(() => {
    if (!geometryRef.current) return;
    const geo = geometryRef.current;
    
    // Initialize attribute if needed
    if (!geo.getAttribute('position')) {
        const attr = new BufferAttribute(positionBuffer, 3);
        attr.setUsage(DynamicDrawUsage);
        geo.setAttribute('position', attr);
    }
    
    const posAttr = geo.getAttribute('position') as BufferAttribute;
    const count = displayPoints.length;
    
    // Update buffer data
    for (let i = 0; i < count; i++) {
        const p = displayPoints[i];
        posAttr.setXYZ(i, p.x, p.y, p.z);
    }
    
    // Important: Tell GPU providing range changed
    posAttr.needsUpdate = true;
    geo.setDrawRange(0, count);
    
    if (count > 0) {
        geo.computeBoundingSphere();
    }
  }, [displayPoints, positionBuffer]);

  return (
    <line>
      <bufferGeometry ref={geometryRef} />
      <lineBasicMaterial color="cyan" linewidth={1} opacity={0.6} transparent />
    </line>
  );
}

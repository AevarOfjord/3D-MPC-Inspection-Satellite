import { useMemo } from 'react';
import * as THREE from 'three';

interface ConstraintVisualizerProps {
    points: [number, number, number][];
    maxCurvature?: number; // 1/radius
}

export function ConstraintVisualizer({ points, maxCurvature = 1.0 }: ConstraintVisualizerProps) {
    if (!points || points.length < 3) return null;

    return (
        <group></group>
    );
}

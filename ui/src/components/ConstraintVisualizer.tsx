import { useMemo } from 'react';
import * as THREE from 'three';

interface ConstraintVisualizerProps {
    points: [number, number, number][];
    maxCurvature?: number; // 1/radius
}

export function ConstraintVisualizer({ points, maxCurvature = 1.0 }: ConstraintVisualizerProps) {
    // Analyze path for curvature
    const analysis = useMemo(() => {
        const violations: THREE.Vector3[][] = [];
        let totalLinearDist = 0;
        let totalAngularDist = 0; // Rough approx
        for (let i = 0; i < points.length - 2; i++) {
            const p0 = new THREE.Vector3(...points[i]);
            const p1 = new THREE.Vector3(...points[i+1]);
            const p2 = new THREE.Vector3(...points[i+2]);

            // Curvature (k) approx = 2 * sin(angle) / |p0-p2| for equal segments,
            // or simpler: deviation angle / segment length.
            const v1 = new THREE.Vector3().subVectors(p1, p0);
            const v2 = new THREE.Vector3().subVectors(p2, p1);
            const len1 = v1.length();

            totalLinearDist += len1;

            if (len1 > 0.001 && v2.length() > 0.001) {
                const angle = v1.angleTo(v2);
                const curvature = angle / len1;

                totalAngularDist += angle;

                if (curvature > maxCurvature) {
                    violations.push([p0, p1, p2]);
                }
            }
        }

        return { violations, totalLinearDist, totalAngularDist };
    }, [points, maxCurvature]);

    if (!points || points.length < 3) return null;

    return (
        <group>
            {/* Disabled to prevent Z-fighting with EditableTrajectory */}
            {/*
            {analysis.violations.map((segment, i) => (
                <Line key={i} points={segment} color="red" lineWidth={4} transparent opacity={0.8} />
            ))}
            */}
        </group>
    );
}

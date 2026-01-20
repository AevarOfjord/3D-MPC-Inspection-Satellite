import { useMemo } from 'react';
import { Line } from '@react-three/drei';
import * as THREE from 'three';

interface ConstraintVisualizerProps {
    points: [number, number, number][];
    maxCurvature?: number; // 1/radius
}

export function ConstraintVisualizer({ points, maxCurvature = 1.0 }: ConstraintVisualizerProps) {
    if (!points || points.length < 3) return null;

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

    return (
        <group>
            {/* Render Red segments for violations */}
            {analysis.violations.map((segment, i) => (
                <Line key={i} points={segment} color="red" lineWidth={4} transparent opacity={0.8} />
            ))}
        </group>
    );
}

// Separate component for the Side Panel readouts
export function ConstraintReadout({ points }: { points: [number, number, number][] }) {
    const analysis = useMemo(() => {
        let linear = 0, angular = 0;
        for (let i = 0; i < points.length - 1; i++) {
            const p0 = new THREE.Vector3(...points[i]);
            const p1 = new THREE.Vector3(...points[i+1]);
            linear += p0.distanceTo(p1);
            // Angular approx (sum of turn angles)
            if (i < points.length - 2) {
                 const p2 = new THREE.Vector3(...points[i+2]);
                 const v1 = p1.clone().sub(p0);
                 const v2 = p2.clone().sub(p1);
                 angular += v1.angleTo(v2);
            }
        }
        return { linear, angular };
    }, [points]);
    
    // Fake DeltaV calculation: DeltaV = Linear + (Angular * MomentArm_Factor)
    const dv = (analysis.linear * 0.2 + analysis.angular * 0.5).toFixed(2);
    const maxFuel = 100;
    const usage = Math.min((parseFloat(dv) / maxFuel) * 100, 100);

    return (
        <div className="space-y-2">
            <div className="flex justify-between text-[10px] text-slate-400 font-bold uppercase">
                <span>Est. Delta-V</span>
                <span>{dv} m/s</span>
            </div>
            <div className="h-2 bg-slate-800 rounded-full overflow-hidden border border-slate-700">
                <div 
                    className={`h-full transition-all duration-300 ${usage > 90 ? 'bg-red-500' : 'bg-cyan-500'}`} 
                    style={{ width: `${usage}%` }} 
                />
            </div>
        </div>
    );
}

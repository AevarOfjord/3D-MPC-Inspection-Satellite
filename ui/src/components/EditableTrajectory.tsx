import { useState } from 'react';
import { Line, TransformControls, useCursor } from '@react-three/drei';
import * as THREE from 'three';
import type { useMissionBuilder } from '../hooks/useMissionBuilder';

interface EditableTrajectoryProps {
    points: [number, number, number][];
    onHover?: (point: [number, number, number] | null) => void;
    builderActions: ReturnType<typeof useMissionBuilder>['actions'];
    selectedId: string | null;
}

export function EditableTrajectory({ points, onHover, builderActions, selectedId }: EditableTrajectoryProps) {
    const [highlightIndex, setHighlightIndex] = useState<number | null>(null);
    useCursor(typeof highlightIndex === 'number');

    if (!points || points.length === 0) return null;
    const vectors = points.map(p => new THREE.Vector3(...p));

    return (
        <group>
            {/* The Path Line */}
            <Line points={vectors} color="#06b6d4" lineWidth={2} transparent={false} opacity={1} frustumCulled={false} renderOrder={10} />

            {/* Interactive Waypoints (Invisible until hovered or selected) */}
            {vectors.map((vec, i) => (
                <group key={i} position={vec}>
                    <mesh
                        onPointerOver={(e) => { e.stopPropagation(); setHighlightIndex(i); onHover?.(points[i]); }}
                        onPointerOut={() => { setHighlightIndex(null); onHover?.(null); }}
                        onClick={(e) => { 
                            e.stopPropagation(); 
                            builderActions.setSelectedObjectId(`waypoint-${i}`); 
                        }}
                    >
                        <sphereGeometry args={[0.15, 8, 8]} />
                        <meshBasicMaterial 
                            color={selectedId === `waypoint-${i}` ? "#ffff00" : (highlightIndex === i ? "#22d3ee" : "#22d3ee")} 
                            transparent 
                            opacity={selectedId === `waypoint-${i}` || highlightIndex === i ? 0.8 : 0.0} 
                        />
                    </mesh>
                    
                    {/* Visual Dot for every 10th point to give structure without clutter */}
                    {/* {i % 10 === 0 && <mesh scale={0.5}><sphereGeometry args={[0.1]} /><meshBasicMaterial color="#22d3ee" opacity={0.3} transparent /></mesh>} */}
                </group>
            ))}

            {/* Transform Controls for Selected Waypoint */}
            {selectedId && selectedId.startsWith('waypoint-') && (
                (() => {
                    const idx = parseInt(selectedId.split('-')[1]);
                    if (idx >= 0 && idx < points.length) {
                         // We render a phantom target at the waypoint position for the controls to attach to
                         return (
                            <TransformControls
                                mode="translate"
                                object={undefined}
                                position={vectors[idx]}
                                onObjectChange={(e) => builderActions.handleObjectTransform(selectedId, e)}
                                onMouseUp={() => builderActions.commitWaypointMove()} // Save history on drag release
                            >
                                <mesh visible={false} /> 
                            </TransformControls>
                         );
                    }
                    return null;
                })()
            )}
        </group>
    );
}

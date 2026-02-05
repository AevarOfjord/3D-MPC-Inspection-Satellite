import { useState, useMemo } from 'react';
import { Line, useCursor, TransformControls } from '@react-three/drei';
import * as THREE from 'three';
import type { useMissionBuilder } from '../hooks/useMissionBuilder';
import { resamplePath } from '../utils/pathResample';

interface EditableTrajectoryProps {
    points: [number, number, number][];
    onHover?: (point: [number, number, number] | null) => void;
    builderActions: ReturnType<typeof useMissionBuilder>['actions'];
    selectedId: string | null;
    sceneScale?: number;
    sceneOrigin?: [number, number, number];
    fixedAnchor?: [number, number, number];
}

export function EditableTrajectory({ points, onHover, builderActions, selectedId, sceneScale = 1, sceneOrigin = [0, 0, 0], fixedAnchor }: EditableTrajectoryProps) {
    const [highlightIndex, setHighlightIndex] = useState<number | null>(null);
    useCursor(typeof highlightIndex === 'number');

    const safePoints = points ?? [];
    const smoothPoints = useMemo(() => resamplePath(safePoints, 10), [safePoints]);
    const linePoints = smoothPoints.length >= 2 ? smoothPoints : safePoints;

    if (!safePoints || safePoints.length === 0) return null;
    const vectors = safePoints.map(p => new THREE.Vector3(...p));
    const markerRadius = Math.max(0.000003, 0.03 * sceneScale);
    const highlightRadius = markerRadius * 2.2;
    const selectedIndex =
        selectedId && selectedId.startsWith('waypoint-')
            ? parseInt(selectedId.split('-')[1], 10)
            : null;

    return (
        <group>
            {fixedAnchor && (
                <group position={new THREE.Vector3(...fixedAnchor)}>
                    <mesh>
                        <sphereGeometry args={[markerRadius * 1.6, 12, 12]} />
                        <meshBasicMaterial color="#facc15" wireframe transparent opacity={0.8} />
                    </mesh>
                    <mesh>
                        <sphereGeometry args={[markerRadius * 0.6, 12, 12]} />
                        <meshBasicMaterial color="#facc15" transparent opacity={0.5} />
                    </mesh>
                </group>
            )}
            {/* Smoothed preview line for real-time shape feedback */}
            <Line
                points={linePoints}
                color="#22d3ee"
                lineWidth={2}
                opacity={0.95}
                transparent
                depthTest={false}
                depthWrite={false}
            />

            {/* Interactive Waypoints (Invisible until hovered or selected) */}
            {vectors.map((vec, i) => (
                <group key={i} position={vec}>
                    <mesh
                        onPointerOver={(e) => { e.stopPropagation(); setHighlightIndex(i); onHover?.(points[i]); }}
                        onPointerOut={() => { setHighlightIndex(null); onHover?.(null); }}
                        onClick={(e) => { 
                            e.stopPropagation(); 
                            if (i === 0) return;
                            if (e.shiftKey || e.altKey) {
                                builderActions.removeWaypointAtIndex?.(i);
                                return;
                            }
                            builderActions.setSelectedObjectId(`waypoint-${i}`); 
                        }}
                    >
                        <sphereGeometry args={[markerRadius, 8, 8]} />
                        <meshBasicMaterial
                            color="#22d3ee"
                            transparent
                            opacity={0.15}
                        />
                    </mesh>

                    {/* Hover/selected highlight sphere */}
                    {(highlightIndex === i || selectedId === `waypoint-${i}`) && (
                        <mesh>
                            <sphereGeometry args={[highlightRadius, 12, 12]} />
                            <meshBasicMaterial
                                color={selectedId === `waypoint-${i}` ? "#ffff00" : "#22d3ee"}
                                wireframe
                                transparent
                                opacity={0.6}
                            />
                        </mesh>
                    )}
                    
                    {/* Visual Dot for every 10th point to give structure without clutter */}
                    {/* {i % 10 === 0 && <mesh scale={0.5}><sphereGeometry args={[0.1]} /><meshBasicMaterial color="#22d3ee" opacity={0.3} transparent /></mesh>} */}
                </group>
            ))}

            {/* Waypoint drag controls */}
            {typeof selectedIndex === 'number' && selectedIndex > 0 && selectedIndex < vectors.length && (
                <TransformControls
                    mode="translate"
                    position={[vectors[selectedIndex].x, vectors[selectedIndex].y, vectors[selectedIndex].z]}
                    showX
                    showY
                    showZ
                    onObjectChange={(e: any) => {
                        const obj = e?.target?.object as THREE.Object3D;
                        if (!obj) return;
                        const base = sceneOrigin ?? [0, 0, 0];
                        const next: [number, number, number] = [
                            obj.position.x / sceneScale + base[0],
                            obj.position.y / sceneScale + base[1],
                            obj.position.z / sceneScale + base[2],
                        ];
                        builderActions.handleWaypointMove(selectedIndex, next);
                    }}
                    onMouseUp={() => builderActions.commitWaypointMove()}
                />
            )}
        </group>
    );
}

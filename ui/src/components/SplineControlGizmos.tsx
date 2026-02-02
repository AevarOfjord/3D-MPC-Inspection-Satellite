import { TransformControls, Html } from '@react-three/drei';
import * as THREE from 'three';
import type { SplineControl } from '../api/unifiedMission';

interface SplineControlGizmosProps {
    controls: SplineControl[];
    onUpdate: (index: number, next: SplineControl) => void;
    onSelect?: (index: number) => void;
    selectedId?: string | null;
}

export function SplineControlGizmos({ controls, onUpdate, onSelect, selectedId }: SplineControlGizmosProps) {

    return (
        <group>
            {controls.map((control, idx) => {
                const isSelected = selectedId === `spline-${idx}`;
                
                return (
                    <group key={idx}>
                        <TransformControls
                            object={new THREE.Object3D()} // Dummy object, we control position directly via props
                            position={[control.position[0], control.position[1], control.position[2]]}
                            mode="translate"
                            // If selected, show gizmo. If not, maybe just show a marker?
                            // Actually, TransformControls is the gizmo. We can show it always or only when selected.
                            // Let's show it always for now but maybe smaller/transparent if not selected?
                            // TransformControls doesn't support transparency well.
                            // Better UX: Show a sphere for all, and show TransformControls ONLY for the selected one.
                            enabled={isSelected}
                            showX={isSelected}
                            showY={isSelected}
                            showZ={isSelected}
                            onObjectChange={(e: any) => {
                                const obj = e?.target?.object as THREE.Object3D;
                                if (!obj) return;
                                const pos = obj.position;
                                onUpdate(idx, {
                                    ...control,
                                    position: [pos.x, pos.y, pos.z],
                                });
                            }}
                        />
                        
                        {/* Always visible marker */}
                        <mesh
                            position={[control.position[0], control.position[1], control.position[2]]}
                            onClick={(e) => {
                                e.stopPropagation();
                                onSelect?.(idx);
                            }}
                        >
                            <sphereGeometry args={[isSelected ? 0.3 : 0.2, 16, 16]} />
                            <meshBasicMaterial color={isSelected ? "#f59e0b" : "#fbbf24"} transparent opacity={0.8} />
                        </mesh>
                        
                        {/* Label */}
                        <Html position={[control.position[0], control.position[1], control.position[2]]}>
                            <div
                                style={{
                                    transform: 'translate3d(-50%, -150%, 0)',
                                    color: isSelected ? '#f59e0b' : '#fbbf24',
                                    fontWeight: 'bold',
                                    fontSize: '10px',
                                    whiteSpace: 'nowrap',
                                    textShadow: '0 1px 2px black',
                                    pointerEvents: 'none',
                                    userSelect: 'none',
                                }}
                            >
                                C{idx + 1}
                            </div>
                        </Html>
                    </group>
                );
            })}
        </group>
    );
}

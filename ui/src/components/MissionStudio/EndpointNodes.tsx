import { useRef, useCallback } from 'react';
import * as THREE from 'three';
import { useThree, useFrame } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

function resolveNodePosition(nodeId: string, state: ReturnType<typeof useStudioStore.getState>): [number, number, number] | null {
  if (nodeId === 'satellite:start') return state.satelliteStart;
  const parts = nodeId.split(':');
  if (parts.length < 3 || parts[0] !== 'path') return null;
  const pathId = parts[1];
  const endpoint = parts[2];
  const path = state.paths.find((p) => p.id === pathId);
  if (!path || path.waypoints.length === 0) return null;
  return endpoint === 'start' ? path.waypoints[0] : path.waypoints[path.waypoints.length - 1];
}

export function EndpointNodes() {
  const paths = useStudioStore((s) => s.paths);
  const satelliteStart = useStudioStore((s) => s.satelliteStart);
  const wireDrag = useStudioStore((s) => s.wireDrag);
  const activeTool = useStudioStore((s) => s.activeTool);
  const setWireDrag = useStudioStore((s) => s.setWireDrag);
  const addWire = useStudioStore((s) => s.addWire);
  const { camera, gl } = useThree();
  const raycaster = useRef(new THREE.Raycaster());
  const dragLineRef = useRef<THREE.Line>(null);

  useFrame(() => {
    if (wireDrag.phase !== 'dragging' || !dragLineRef.current) return;
    const state = useStudioStore.getState();
    const srcPos = resolveNodePosition(wireDrag.sourceNodeId, state);
    if (!srcPos) return;
    const points = [new THREE.Vector3(...srcPos), new THREE.Vector3(...wireDrag.cursorWorld)];
    (dragLineRef.current.geometry as THREE.BufferGeometry).setFromPoints(points);
  });

  const handlePointerMove = useCallback((e: PointerEvent) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging') return;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(plane, intersection);
    setWireDrag({ phase: 'dragging', sourceNodeId: drag.sourceNodeId, cursorWorld: [intersection.x, intersection.y, intersection.z] });
  }, [camera, gl, setWireDrag]);

  const startDrag = useCallback((nodeId: string) => {
    if (activeTool !== 'connect') return;
    setWireDrag({ phase: 'dragging', sourceNodeId: nodeId, cursorWorld: [0, 0, 0] });
    gl.domElement.addEventListener('pointermove', handlePointerMove);
    gl.domElement.addEventListener('pointerup', () => {
      gl.domElement.removeEventListener('pointermove', handlePointerMove);
      if (useStudioStore.getState().wireDrag.phase === 'dragging') setWireDrag({ phase: 'idle' });
    }, { once: true });
  }, [activeTool, gl, setWireDrag, handlePointerMove]);

  const completeDrag = useCallback((targetNodeId: string) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging' || drag.sourceNodeId === targetNodeId) {
      setWireDrag({ phase: 'idle' });
      return;
    }
    addWire({ id: `wire-${Date.now()}`, fromNodeId: drag.sourceNodeId, toNodeId: targetNodeId });
    setWireDrag({ phase: 'idle' });
  }, [addWire, setWireDrag]);

  if (activeTool !== 'connect') return null;

  return (
    <group>
      {wireDrag.phase === 'dragging' && (
        // @ts-expect-error r3f ref typing
        <line ref={dragLineRef}>
          <bufferGeometry />
          <lineDashedMaterial color="#22d3ee" dashSize={0.3} gapSize={0.2} linewidth={1} />
        </line>
      )}

      <EndpointSphere
        position={satelliteStart}
        color="#ffffff"
        pulse
        onPointerDown={() => startDrag('satellite:start')}
        onPointerUp={() => completeDrag('satellite:start')}
      />

      {paths.map((path) => {
        if (path.waypoints.length === 0) return null;
        const startId = `path:${path.id}:start`;
        const endId = `path:${path.id}:end`;
        return (
          <group key={path.id}>
            <EndpointSphere
              position={path.waypoints[0]}
              color="#22d3ee"
              pulse
              onPointerDown={() => startDrag(startId)}
              onPointerUp={() => completeDrag(startId)}
            />
            <EndpointSphere
              position={path.waypoints[path.waypoints.length - 1]}
              color="#a78bfa"
              pulse
              onPointerDown={() => startDrag(endId)}
              onPointerUp={() => completeDrag(endId)}
            />
          </group>
        );
      })}
    </group>
  );
}

function EndpointSphere({
  position,
  color,
  pulse,
  onPointerDown,
  onPointerUp,
}: {
  position: [number, number, number];
  color: string;
  pulse: boolean;
  onPointerDown: () => void;
  onPointerUp: () => void;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  useFrame(({ clock }) => {
    if (!meshRef.current || !pulse) return;
    const s = 1 + 0.2 * Math.sin(clock.getElapsedTime() * 4);
    meshRef.current.scale.setScalar(s);
  });
  return (
    <mesh ref={meshRef} position={position} onPointerDown={onPointerDown} onPointerUp={onPointerUp}>
      <sphereGeometry args={[0.45, 16, 16]} />
      <meshBasicMaterial color={color} />
    </mesh>
  );
}

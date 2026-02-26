import { useRef, useCallback, useEffect } from 'react';
import * as THREE from 'three';
import { useThree, useFrame } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

type NodePositionState = Pick<ReturnType<typeof useStudioStore.getState>, 'satelliteStart' | 'paths'>;

function resolveNodePosition(nodeId: string, state: NodePositionState): [number, number, number] | null {
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
  const wires = useStudioStore((s) => s.wires);
  const wireDrag = useStudioStore((s) => s.wireDrag);
  const activeTool = useStudioStore((s) => s.activeTool);
  const setWireDrag = useStudioStore((s) => s.setWireDrag);
  const addWire = useStudioStore((s) => s.addWire);
  const { camera, gl } = useThree();
  const raycaster = useRef(new THREE.Raycaster());
  const dragLineRef = useRef<THREE.Line>(null);
  const nodeState = { paths, satelliteStart };

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
  }, [activeTool, gl, setWireDrag, handlePointerMove]);

  const completeDrag = useCallback((targetNodeId: string) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging' || drag.sourceNodeId === targetNodeId) {
      setWireDrag({ phase: 'idle' });
      gl.domElement.removeEventListener('pointermove', handlePointerMove);
      return;
    }
    addWire({ id: `wire-${Date.now()}`, fromNodeId: drag.sourceNodeId, toNodeId: targetNodeId });
    setWireDrag({ phase: 'idle' });
    gl.domElement.removeEventListener('pointermove', handlePointerMove);
  }, [addWire, setWireDrag, gl, handlePointerMove]);

  const handleNodeClick = useCallback((nodeId: string) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging') {
      startDrag(nodeId);
      return;
    }
    completeDrag(nodeId);
  }, [startDrag, completeDrag]);

  useEffect(() => {
    if (activeTool === 'connect') return;
    gl.domElement.removeEventListener('pointermove', handlePointerMove);
    if (useStudioStore.getState().wireDrag.phase === 'dragging') {
      setWireDrag({ phase: 'idle' });
    }
  }, [activeTool, gl, handlePointerMove, setWireDrag]);

  return (
    <group>
      {wires.map((wire) => {
        const src = resolveNodePosition(wire.fromNodeId, nodeState);
        const dst = resolveNodePosition(wire.toNodeId, nodeState);
        if (!src || !dst) return null;
        const geom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...src), new THREE.Vector3(...dst)]);
        return (
          <line key={wire.id} geometry={geom}>
            <lineBasicMaterial color="#f59e0b" opacity={0.95} transparent />
          </line>
        );
      })}

      {activeTool === 'connect' && (
        <>
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
        onClick={() => handleNodeClick('satellite:start')}
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
              onClick={() => handleNodeClick(startId)}
            />
            <EndpointSphere
              position={path.waypoints[path.waypoints.length - 1]}
              color="#a78bfa"
              pulse
              onClick={() => handleNodeClick(endId)}
            />
          </group>
        );
      })}
        </>
      )}
    </group>
  );
}

function EndpointSphere({
  position,
  color,
  pulse,
  onClick,
}: {
  position: [number, number, number];
  color: string;
  pulse: boolean;
  onClick: () => void;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  useFrame(({ clock }) => {
    if (!meshRef.current || !pulse) return;
    const s = 1 + 0.2 * Math.sin(clock.getElapsedTime() * 4);
    meshRef.current.scale.setScalar(s);
  });
  return (
    <mesh
      ref={meshRef}
      position={position}
      onClick={(e) => {
        e.stopPropagation();
        onClick();
      }}
    >
      <sphereGeometry args={[0.45, 16, 16]} />
      <meshBasicMaterial color={color} />
    </mesh>
  );
}

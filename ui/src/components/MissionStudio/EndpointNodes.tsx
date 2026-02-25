import { useRef, useCallback } from 'react';
import * as THREE from 'three';
import { useThree, useFrame } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

export function EndpointNodes() {
  const scanPasses = useStudioStore((s) => s.scanPasses);
  const wireDrag = useStudioStore((s) => s.wireDrag);
  const setWireDrag = useStudioStore((s) => s.setWireDrag);
  const addWire = useStudioStore((s) => s.addWire);
  const { camera, gl } = useThree();
  const raycaster = useRef(new THREE.Raycaster());
  const dragLineRef = useRef<THREE.Line>(null);

  useFrame(() => {
    if (wireDrag.phase !== 'dragging' || !dragLineRef.current) return;
    const cursor = wireDrag.cursorWorld;
    const [srcScanId, srcEndpoint] = wireDrag.sourceNodeId.split(':');
    const srcPass = scanPasses.find((p) => p.id === srcScanId);
    if (!srcPass || srcPass.waypoints.length === 0) return;
    const srcPos = srcEndpoint === 'start' ? srcPass.waypoints[0] : srcPass.waypoints[srcPass.waypoints.length - 1];
    const points = [new THREE.Vector3(...srcPos), new THREE.Vector3(...cursor)];
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
    if (intersection) {
      setWireDrag({ ...drag, cursorWorld: [intersection.x, intersection.y, intersection.z] });
    }
  }, [camera, gl, setWireDrag]);

  const startDrag = useCallback((nodeId: string) => {
    setWireDrag({ phase: 'dragging', sourceNodeId: nodeId, cursorWorld: [0, 0, 0] });
    gl.domElement.addEventListener('pointermove', handlePointerMove);
    gl.domElement.addEventListener('pointerup', () => {
      gl.domElement.removeEventListener('pointermove', handlePointerMove);
      if (useStudioStore.getState().wireDrag.phase === 'dragging') {
        setWireDrag({ phase: 'idle' });
      }
    }, { once: true });
  }, [gl, setWireDrag, handlePointerMove]);

  const completeDrag = useCallback((targetNodeId: string) => {
    const drag = useStudioStore.getState().wireDrag;
    if (drag.phase !== 'dragging') return;
    if (drag.sourceNodeId === targetNodeId) { setWireDrag({ phase: 'idle' }); return; }
    const wireId = `wire-${Date.now()}`;
    addWire({ id: wireId, fromNodeId: drag.sourceNodeId, toNodeId: targetNodeId });
    setWireDrag({ phase: 'idle' });
  }, [addWire, setWireDrag]);

  return (
    <group>
      {wireDrag.phase === 'dragging' && (
        // @ts-expect-error – line ref type
        <line ref={dragLineRef}>
          <bufferGeometry />
          <lineDashedMaterial color="#22d3ee" dashSize={0.3} gapSize={0.2} linewidth={1} />
        </line>
      )}

      {scanPasses.map((pass) => {
        if (pass.waypoints.length === 0) return null;
        const startPos = pass.waypoints[0];
        const endPos = pass.waypoints[pass.waypoints.length - 1];
        const startId = `${pass.id}:start`;
        const endId = `${pass.id}:end`;
        const isDragging = wireDrag.phase === 'dragging';

        return (
          <group key={pass.id}>
            <EndpointSphere
              position={startPos}
              color="#22d3ee"
              pulse={isDragging}
              onPointerDown={() => startDrag(startId)}
              onPointerUp={() => completeDrag(startId)}
            />
            <EndpointSphere
              position={endPos}
              color="#a78bfa"
              pulse={isDragging}
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
  position, color, pulse, onPointerDown, onPointerUp,
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
      <sphereGeometry args={[0.5, 16, 16]} />
      <meshBasicMaterial color={color} />
    </mesh>
  );
}

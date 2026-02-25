import { useRef, useCallback } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

interface WaypointNudgerProps {
  scanId: string;
}

export function WaypointNudger({ scanId }: WaypointNudgerProps) {
  const waypoints = useStudioStore((s) => s.scanPasses.find((p) => p.id === scanId)?.waypoints ?? []);
  const { camera, gl } = useThree();
  const dragging = useRef<{ index: number; plane: THREE.Plane } | null>(null);
  const raycaster = useRef(new THREE.Raycaster());

  const onPointerDown = useCallback((index: number, e: { stopPropagation: () => void; pointerId: number }) => {
    e.stopPropagation();
    const wp = waypoints[index];
    const normal = new THREE.Vector3().subVectors(camera.position, new THREE.Vector3(...wp)).normalize();
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, new THREE.Vector3(...wp));
    dragging.current = { index, plane };
    gl.domElement.setPointerCapture(e.pointerId);
  }, [waypoints, camera, gl]);

  const onPointerMove = useCallback((e: { clientX: number; clientY: number }) => {
    if (!dragging.current) return;
    const { index, plane } = dragging.current;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(plane, intersection);
    if (!intersection) return;
    const wp = waypoints[index];
    const delta: [number, number, number] = [
      intersection.x - wp[0],
      intersection.y - wp[1],
      intersection.z - wp[2],
    ];
    useStudioStore.getState().applyNudge(scanId, index, delta);
  }, [waypoints, scanId, camera, gl]);

  const onPointerUp = useCallback((e: { pointerId: number }) => {
    dragging.current = null;
    gl.domElement.releasePointerCapture(e.pointerId);
  }, [gl]);

  const stride = Math.max(1, Math.floor(waypoints.length / 40));

  return (
    <group onPointerMove={onPointerMove} onPointerUp={onPointerUp}>
      {waypoints.map((wp, i) => {
        if (i % stride !== 0) return null;
        return (
          <mesh
            key={i}
            position={wp}
            onPointerDown={(e) => onPointerDown(i, e)}
          >
            <sphereGeometry args={[0.12, 8, 8]} />
            <meshBasicMaterial color="white" opacity={0.7} transparent />
          </mesh>
        );
      })}
    </group>
  );
}

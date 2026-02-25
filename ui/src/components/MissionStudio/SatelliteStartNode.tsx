import { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

export function SatelliteStartNode() {
  const satelliteStart = useStudioStore((s) => s.satelliteStart);
  const setSatelliteStart = useStudioStore((s) => s.setSatelliteStart);
  const { camera, gl } = useThree();
  const dragging = useRef(false);
  const dragPlane = useRef(new THREE.Plane());
  const raycaster = useRef(new THREE.Raycaster());
  const arrowGroupRef = useRef<THREE.Group>(null);

  // Imperatively build ArrowHelper to avoid JSX constructor issues
  useEffect(() => {
    if (!arrowGroupRef.current) return;
    // Clear previous children
    while (arrowGroupRef.current.children.length > 0) {
      arrowGroupRef.current.remove(arrowGroupRef.current.children[0]);
    }
    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      2,
      0xff4444,
      0.4,
      0.3,
    );
    arrowGroupRef.current.add(arrow);
  }, []);

  const onPointerDown = (e: { stopPropagation: () => void; pointerId: number }) => {
    e.stopPropagation();
    dragging.current = true;
    dragPlane.current.setFromNormalAndCoplanarPoint(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(...satelliteStart)
    );
    gl.domElement.setPointerCapture(e.pointerId);
  };

  const onPointerMove = (e: { clientX: number; clientY: number }) => {
    if (!dragging.current) return;
    const rect = gl.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.current.setFromCamera(ndc, camera);
    const intersection = new THREE.Vector3();
    raycaster.current.ray.intersectPlane(dragPlane.current, intersection);
    if (intersection) setSatelliteStart([intersection.x, satelliteStart[1], intersection.z]);
  };

  const onPointerUp = (e: { pointerId: number }) => {
    dragging.current = false;
    gl.domElement.releasePointerCapture(e.pointerId);
  };

  return (
    <group
      position={satelliteStart}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
    >
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.8, 1.0, 32]} />
        <meshBasicMaterial color="white" side={THREE.DoubleSide} />
      </mesh>
      <group ref={arrowGroupRef} />
      <mesh>
        <sphereGeometry args={[0.2, 8, 8]} />
        <meshBasicMaterial color="white" />
      </mesh>
    </group>
  );
}

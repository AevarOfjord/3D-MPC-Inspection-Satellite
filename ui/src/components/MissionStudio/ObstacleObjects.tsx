import { useRef } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useStudioStore } from './useStudioStore';

export function ObstacleObjects() {
  const obstacles = useStudioStore((s) => s.obstacles);
  const updateObstacle = useStudioStore((s) => s.updateObstacle);

  return (
    <>
      {obstacles.map((obs) => (
        <DraggableObstacle
          key={obs.id}
          id={obs.id}
          position={obs.position}
          radius={obs.radius}
          onMove={(pos) => updateObstacle(obs.id, { position: pos })}
        />
      ))}
    </>
  );
}

function DraggableObstacle({
  id: _id, position, radius, onMove,
}: {
  id: string;
  position: [number, number, number];
  radius: number;
  onMove: (pos: [number, number, number]) => void;
}) {
  const dragging = useRef(false);
  const dragPlane = useRef(new THREE.Plane());
  const raycaster = useRef(new THREE.Raycaster());
  const { camera, gl } = useThree();

  const onPointerDown = (e: { stopPropagation: () => void; pointerId: number }) => {
    e.stopPropagation();
    dragging.current = true;
    dragPlane.current.setFromNormalAndCoplanarPoint(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(...position)
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
    if (intersection) onMove([intersection.x, position[1], intersection.z]);
  };

  const onPointerUp = (e: { pointerId: number }) => {
    dragging.current = false;
    gl.domElement.releasePointerCapture(e.pointerId);
  };

  return (
    <mesh
      position={position}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
    >
      <sphereGeometry args={[radius, 24, 24]} />
      <meshBasicMaterial color="#ef4444" opacity={0.25} transparent />
    </mesh>
  );
}

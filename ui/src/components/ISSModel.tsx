import { useMemo } from 'react';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';
import { ORBIT_SCALE } from '../data/orbitSnapshot';

interface ISSModelProps {
  position: [number, number, number];
  orientation: [number, number, number];
  scale?: number;
  realSpanMeters?: number;
}

export function ISSModel({ position, orientation, scale = 1, realSpanMeters }: ISSModelProps) {
  const gltf = useGLTF('/OBJ_files/ISS/ISS.glb');
  const clonedObj = useMemo(() => {
    const clone = gltf.scene.clone();
    clone.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mesh = child as THREE.Mesh;
        if (!mesh.material || (mesh.material as THREE.Material).name === '') {
          mesh.material = new THREE.MeshStandardMaterial({
            color: '#cbd5f5',
            metalness: 0.7,
            roughness: 0.4,
          });
        }
        mesh.castShadow = true;
        mesh.receiveShadow = true;
      }
    });
    return clone;
  }, [gltf.scene]);

  const resolvedScale = useMemo(() => {
    if (!realSpanMeters) return scale;
    const box = new THREE.Box3().setFromObject(gltf.scene);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    if (!Number.isFinite(maxDim) || maxDim <= 0) return scale;
    const targetSpan = realSpanMeters * ORBIT_SCALE;
    return (targetSpan / maxDim) * scale;
  }, [gltf.scene, realSpanMeters, scale]);

  return (
    <primitive
      object={clonedObj}
      position={position}
      rotation={orientation}
      scale={[resolvedScale, resolvedScale, resolvedScale]}
    />
  );
}

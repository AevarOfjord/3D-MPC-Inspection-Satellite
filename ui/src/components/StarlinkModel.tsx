import { useMemo } from 'react';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';
import { ORBIT_SCALE } from '../data/orbitSnapshot';

interface StarlinkModelProps {
  position: [number, number, number];
  orientation: [number, number, number];
  scale?: number;
  realSpanMeters?: number;
}

export function StarlinkModel({ 
  position, 
  orientation,
  scale = 1,
  realSpanMeters
}: StarlinkModelProps) {
  const gltf = useGLTF('/OBJ_files/Starlink/starlink.glb');

  // Clone and apply fallback material if needed
  const clonedObj = useMemo(() => {
    const clone = gltf.scene.clone();
    
    // Apply metallic material to all meshes (as fallback or enhancement)
    clone.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mesh = child as THREE.Mesh;
        // If no material was loaded from MTL, apply our own
        if (!mesh.material || (mesh.material as THREE.Material).name === '') {
          mesh.material = new THREE.MeshStandardMaterial({
            color: '#c0c0c0',
            metalness: 0.8,
            roughness: 0.3,
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

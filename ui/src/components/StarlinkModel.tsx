import { useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';
import { useMemo } from 'react';
import * as THREE from 'three';

interface StarlinkModelProps {
  position: [number, number, number];
  orientation: [number, number, number];
  scale?: number;
}

export function StarlinkModel({ 
  position, 
  orientation,
  scale = 1 
}: StarlinkModelProps) {
  // Try to load MTL first, then OBJ
  let materials: MTLLoader.MaterialCreator | null = null;
  
  try {
    materials = useLoader(MTLLoader, '/OBJ_files/starlink.mtl');
    materials.preload();
  } catch {
    // MTL loading failed, will use fallback material
    materials = null;
  }
  
  // Load OBJ with or without materials
  const obj = useLoader(
    OBJLoader,
    '/OBJ_files/starlink.obj',
    (loader) => {
      if (materials) {
        loader.setMaterials(materials);
      }
    }
  );
  
  // Clone and apply fallback material if needed
  const clonedObj = useMemo(() => {
    const clone = obj.clone();
    
    // Apply metallic material to all meshes (as fallback or enhancement)
    clone.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mesh = child as THREE.Mesh;
        // If no material was loaded from MTL, apply our own
        if (!materials || !mesh.material || (mesh.material as THREE.Material).name === '') {
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
  }, [obj, materials]);
  
  return (
    <primitive
      object={clonedObj}
      position={position}
      rotation={orientation}
      scale={[scale, scale, scale]}
    />
  );
}

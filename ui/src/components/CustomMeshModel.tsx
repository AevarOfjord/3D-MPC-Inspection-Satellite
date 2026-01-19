import { useLoader } from '@react-three/fiber';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { useMemo } from 'react';
import * as THREE from 'three';
import { API_BASE_URL } from '../config/endpoints';

interface CustomMeshModelProps {
  objPath: string;
  position: [number, number, number];
  orientation: [number, number, number];
  scale?: number;
}

export function CustomMeshModel({ 
  objPath, 
  position, 
  orientation,
  scale = 1 
}: CustomMeshModelProps) {
  // Construct URL to serve the model via the backend API
  const modelUrl = `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(objPath)}`;
  
  console.log('[CustomMeshModel] Loading model from:', modelUrl);
  
  // Load OBJ
  const obj = useLoader(OBJLoader, modelUrl);

  
  // Clone and apply material
  const clonedObj = useMemo(() => {
    const clone = obj.clone();
    
    clone.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mesh = child as THREE.Mesh;
        mesh.material = new THREE.MeshStandardMaterial({
          color: '#808080',
          metalness: 0.4,
          roughness: 0.6,
          transparent: true,
          opacity: 0.7,
        });
        mesh.castShadow = true;
        mesh.receiveShadow = true;
      }
    });
    
    return clone;
  }, [obj]);
  
  return (
    <primitive
      object={clonedObj}
      position={position}
      rotation={orientation}
      scale={[scale, scale, scale]}
    />
  );
}

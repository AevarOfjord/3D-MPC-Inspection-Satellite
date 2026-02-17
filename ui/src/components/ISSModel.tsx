import { useMemo } from 'react';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';
import { ORBIT_SCALE } from '../data/orbitSnapshot';
import { API_BASE_URL } from '../config/endpoints';

interface ISSModelProps {
  position: [number, number, number];
  orientation: [number, number, number];
  scale?: number;
  realSpanMeters?: number;
}

export function ISSModel({ position, orientation, scale = 1, realSpanMeters }: ISSModelProps) {
  const modelUrl = `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent('assets/model_files/ISS/ISS.glb')}`;
  const gltf = useGLTF(modelUrl);
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

  const modelMetrics = useMemo(() => {
    const box = new THREE.Box3().setFromObject(gltf.scene);
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);
    const maxDim = Math.max(size.x, size.y, size.z);
    return { maxDim, center };
  }, [gltf.scene]);

  const resolvedScale = useMemo(() => {
    if (!realSpanMeters) return scale;
    if (!Number.isFinite(modelMetrics.maxDim) || modelMetrics.maxDim <= 0) return scale;
    const targetSpan = realSpanMeters * ORBIT_SCALE;
    return (targetSpan / modelMetrics.maxDim) * scale;
  }, [modelMetrics.maxDim, realSpanMeters, scale]);

  const centerOffset = useMemo(
    () => [-modelMetrics.center.x, -modelMetrics.center.y, -modelMetrics.center.z] as [number, number, number],
    [modelMetrics.center]
  );

  return (
    <group position={position} rotation={orientation} scale={[resolvedScale, resolvedScale, resolvedScale]}>
      <primitive object={clonedObj} position={centerOffset} />
    </group>
  );
}

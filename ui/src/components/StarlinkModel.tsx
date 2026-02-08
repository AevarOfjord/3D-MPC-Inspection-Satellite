import { useMemo } from 'react';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';
import { ORBIT_SCALE } from '../data/orbitSnapshot';

interface StarlinkModelProps {
  position: [number, number, number];
  orientation: [number, number, number];
  scale?: number;
  realSpanMeters?: number;
  pivot?: 'center' | 'minY' | 'maxY' | 'centroid' | 'origin';
}

export function StarlinkModel({
  position,
  orientation,
  scale = 1,
  realSpanMeters,
  pivot = 'center',
}: StarlinkModelProps) {
  const gltf = useGLTF('/models/Starlink/starlink.glb');

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

  const modelMetrics = useMemo(() => {
    // Ensure the clone has up-to-date matrices before measurement
    clonedObj.updateWorldMatrix(true, true);

    const box = new THREE.Box3().setFromObject(clonedObj);
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();

    box.getSize(size);
    box.getCenter(center);

    // Fallback if box is empty
    if (box.isEmpty()) {
      return {
        maxDim: 0,
        center: new THREE.Vector3(0, 0, 0),
        min: new THREE.Vector3(0, 0, 0),
        max: new THREE.Vector3(0, 0, 0)
      };
    }

    const maxDim = Math.max(size.x, size.y, size.z);
    return { maxDim, center, min: box.min, max: box.max };
  }, [clonedObj]);

  const resolvedScale = useMemo(() => {
    if (!realSpanMeters) return scale;
    if (!Number.isFinite(modelMetrics.maxDim) || modelMetrics.maxDim <= 0) return scale;
    const targetSpan = realSpanMeters * ORBIT_SCALE;
    return (targetSpan / modelMetrics.maxDim) * scale;
  }, [modelMetrics.maxDim, realSpanMeters, scale]);

  const centerOffset = useMemo(() => {
    if (pivot === 'origin') {
      return [0, 0, 0] as [number, number, number];
    }
    if (pivot === 'minY') {
      return [-modelMetrics.center.x, -modelMetrics.min.y, -modelMetrics.center.z] as [number, number, number];
    }
    if (pivot === 'maxY') {
      return [-modelMetrics.center.x, -modelMetrics.max.y, -modelMetrics.center.z] as [number, number, number];
    }
    return [-modelMetrics.center.x, -modelMetrics.center.y, -modelMetrics.center.z] as [number, number, number];
  }, [modelMetrics, pivot]);

  return (
    <group position={position} rotation={orientation} scale={[resolvedScale, resolvedScale, resolvedScale]}>
      <primitive object={clonedObj} position={centerOffset} />
    </group>
  );
}

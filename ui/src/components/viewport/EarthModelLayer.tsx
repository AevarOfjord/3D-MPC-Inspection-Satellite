import { useMemo } from 'react';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';

interface EarthModelLayerProps {
  earthRadius: number;
}

export function EarthModelLayer({ earthRadius }: EarthModelLayerProps) {
  const earthGltf = useGLTF('/model_files/Earth/Earth.glb');
  const earthScale = useMemo(() => {
    const box = new THREE.Box3().setFromObject(earthGltf.scene);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    if (!Number.isFinite(maxDim) || maxDim <= 0) return 1;
    return (earthRadius * 2) / maxDim;
  }, [earthGltf, earthRadius]);

  return <primitive object={earthGltf.scene} scale={[earthScale, earthScale, earthScale]} />;
}


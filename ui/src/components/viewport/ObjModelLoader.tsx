import { useState, useEffect, Suspense } from 'react';
import * as THREE from 'three';

import { API_BASE_URL } from '../../config/endpoints';
import { StarlinkModel } from '../StarlinkModel';
import { ISSModel } from '../ISSModel';

export function ObjWithMtl({ objPath }: { objPath: string }) {
  const [object, setObject] = useState<THREE.Object3D | null>(null);

  useEffect(() => {
    if (!objPath) {
      setObject(null);
      return;
    }

    let cancelled = false;
    const objUrl = `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(objPath)}`;
    const mtlPath = objPath.replace(/\.obj$/i, '.mtl');
    const mtlUrl = `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(mtlPath)}`;

    const loadModel = async () => {
      const [{ OBJLoader }, { MTLLoader }] = await Promise.all([
        import('three/examples/jsm/loaders/OBJLoader.js'),
        import('three/examples/jsm/loaders/MTLLoader.js'),
      ]);
      if (cancelled) return;

      const objLoader = new OBJLoader();
      const applyFallback = () => {
        objLoader.load(objUrl, (obj) => {
          if (cancelled) return;
          obj.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
              const mesh = child as THREE.Mesh;
              mesh.material = new THREE.MeshStandardMaterial({
                color: '#8b8b8b',
                metalness: 0.2,
                roughness: 0.7,
              });
            }
          });
          setObject(obj);
        });
      };

      const mtlLoader = new MTLLoader();
      mtlLoader.load(
        mtlUrl,
        (materials) => {
          if (cancelled) return;
          materials.preload();
          objLoader.setMaterials(materials);
          objLoader.load(objUrl, (obj) => {
            if (cancelled) return;
            setObject(obj);
          });
        },
        undefined,
        () => applyFallback()
      );
    };

    void loadModel();

    return () => {
      cancelled = true;
    };
  }, [objPath]);

  if (!object) return null;
  return <primitive object={object} />;
}

export function resolvePreviewModel(modelPath?: string) {
  if (!modelPath) return null;
  const lower = modelPath.toLowerCase();
  if (lower.includes('starlink')) {
    return (
      <StarlinkModel
        position={[0, 0, 0]}
        orientation={[0, 0, 0]}
        realSpanMeters={11}
        scale={1}
        pivot="origin"
      />
    );
  }
  if (lower.includes('iss')) {
    return (
      <ISSModel
        position={[0, 0, 0]}
        orientation={[0, 0, 0]}
        realSpanMeters={109}
        scale={1}
      />
    );
  }
  return null;
}

export function computeFacingEuler(
  position: [number, number, number],
  baseAxis: [number, number, number] = [0, 0, -1],
  fallback: [number, number, number] = [0, 0, 0]
) {
  const toEarth = new THREE.Vector3(-position[0], -position[1], -position[2]);
  if (toEarth.lengthSq() < 1e-8) return fallback;
  toEarth.normalize();
  const base = new THREE.Vector3(baseAxis[0], baseAxis[1], baseAxis[2]);
  if (base.lengthSq() < 1e-8) return fallback;
  base.normalize();
  const quat = new THREE.Quaternion().setFromUnitVectors(base, toEarth);
  const euler = new THREE.Euler().setFromQuaternion(quat);
  return [euler.x, euler.y, euler.z] as [number, number, number];
}

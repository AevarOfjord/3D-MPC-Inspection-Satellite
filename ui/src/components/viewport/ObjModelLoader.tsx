import { useState, useEffect, Suspense } from 'react';
import * as THREE from 'three';

import { API_BASE_URL } from '../../config/endpoints';

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

import { useState, useEffect, Suspense } from 'react';
import * as THREE from 'three';

import { API_BASE_URL } from '../../config/endpoints';

function toServeUrl(modelPath: string): string {
  return `${API_BASE_URL}/api/models/serve?path=${encodeURIComponent(modelPath)}`;
}

function modelDirectory(modelPath: string): string {
  const normalized = modelPath.replace(/\\/g, '/');
  const idx = normalized.lastIndexOf('/');
  return idx >= 0 ? normalized.slice(0, idx) : '';
}

function normalizeAssetRef(rawRef: string): string {
  let ref = (rawRef || '').trim().replace(/^["']|["']$/g, '');
  if (!ref) return ref;
  ref = ref.replace(/\\/g, '/');
  // Some exported MTLs contain absolute system paths; keep only basename.
  if (/^[a-zA-Z]:\//.test(ref) || ref.startsWith('/')) {
    const parts = ref.split('/').filter(Boolean);
    return parts[parts.length - 1] || ref;
  }
  ref = ref.replace(/^\.\/+/, '');
  return ref;
}

function resolveAssetPath(baseObjPath: string, rawRef: string): string {
  const cleaned = normalizeAssetRef(rawRef);
  if (!cleaned) return cleaned;
  if (/^(https?:|data:|blob:)/i.test(cleaned)) return cleaned;
  const baseDir = modelDirectory(baseObjPath);
  return baseDir ? `${baseDir}/${cleaned}` : cleaned;
}

export function ObjWithMtl({ objPath }: { objPath: string }) {
  const [object, setObject] = useState<THREE.Object3D | null>(null);

  useEffect(() => {
    if (!objPath) {
      setObject(null);
      return;
    }

    let cancelled = false;
    const objUrl = toServeUrl(objPath);
    const mtlPath = objPath.replace(/\.obj$/i, '.mtl');
    const mtlUrl = toServeUrl(mtlPath);

    const loadModel = async () => {
      const [{ OBJLoader }, { MTLLoader }] = await Promise.all([
        import('three/examples/jsm/loaders/OBJLoader.js'),
        import('three/examples/jsm/loaders/MTLLoader.js'),
      ]);
      if (cancelled) return;

      const manager = new THREE.LoadingManager();
      manager.setURLModifier((url) => {
        if (/^(https?:|data:|blob:)/i.test(url)) return url;
        return toServeUrl(resolveAssetPath(objPath, url));
      });

      const objLoader = new OBJLoader(manager);
      const applyFallback = () => {
        objLoader.load(objUrl, (obj) => {
          if (cancelled) return;
          obj.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
              const mesh = child as THREE.Mesh;
              const hasVertexColors = !!mesh.geometry?.getAttribute('color');
              mesh.material = new THREE.MeshStandardMaterial({
                color: '#aeb6c2',
                metalness: 0.2,
                roughness: 0.7,
                vertexColors: hasVertexColors,
              });
              mesh.castShadow = true;
              mesh.receiveShadow = true;
            }
          });
          setObject(obj);
        });
      };

      const mtlLoader = new MTLLoader(manager);
      mtlLoader.load(
        mtlUrl,
        (materials) => {
          if (cancelled) return;
          materials.preload();
          objLoader.setMaterials(materials);
          objLoader.load(objUrl, (obj) => {
            if (cancelled) return;
            obj.traverse((child) => {
              if ((child as THREE.Mesh).isMesh) {
                const mesh = child as THREE.Mesh;
                mesh.castShadow = true;
                mesh.receiveShadow = true;
              }
            });
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

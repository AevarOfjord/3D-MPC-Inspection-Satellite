import { useState, useEffect, Suspense } from 'react';
import * as THREE from 'three';

import { telemetry } from '../../services/telemetry';
import type { TelemetryData } from '../../services/telemetry';
import { StarlinkModel } from '../StarlinkModel';
import { CustomMeshModel } from '../CustomMeshModel';
import { ErrorBoundary } from '../ErrorBoundary';
import { lazy } from 'react';

const ReferenceMarker = lazy(() =>
  import('../Earth').then((m) => ({ default: m.ReferenceMarker }))
);

export function LiveObstaclesRender() {
  const [params, setParams] = useState<{
    obstacles: TelemetryData['obstacles'],
    referencePos: TelemetryData['reference_position'],
    referenceOri: TelemetryData['reference_orientation'],
    referenceQuat?: TelemetryData['reference_quaternion'],
    scanObject?: TelemetryData['scan_object']
  } | null>(null);

  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       if (!d || !d.reference_position) return;
       setParams({
         obstacles: d.obstacles || [],
         referencePos: d.reference_position,
         referenceOri: d.reference_orientation || [0,0,0],
         referenceQuat: d.reference_quaternion,
         scanObject: d.scan_object
       });
    });
    return () => { unsub(); };
  }, []);

  if (!params) return null;

  return (
    <group>
      <Suspense fallback={null}>
        <ReferenceMarker
          position={params.referencePos}
          orientation={params.referenceOri}
          quaternion={params.referenceQuat}
        />
      </Suspense>
      {params.scanObject && params.scanObject.type === 'cylinder' && (
        <group
          position={new THREE.Vector3(...params.scanObject.position)}
          rotation={params.scanObject.orientation as [number, number, number]}
        >
          <mesh rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry
              args={[
                params.scanObject.radius,
                params.scanObject.radius,
                params.scanObject.height,
                32,
              ]}
            />
            <meshStandardMaterial color="#ff4444" transparent opacity={0.3} wireframe />
          </mesh>
        </group>
      )}
      {params.scanObject && params.scanObject.type === 'starlink' && (
        <Suspense fallback={null}>
          <StarlinkModel
            position={params.scanObject.position}
            orientation={params.scanObject.orientation}
          />
        </Suspense>
      )}
      {params.scanObject && params.scanObject.type === 'mesh' && params.scanObject.obj_path && (
        <ErrorBoundary fallback={null}>
          <Suspense fallback={null}>
            <CustomMeshModel
              objPath={params.scanObject.obj_path}
              position={params.scanObject.position}
              orientation={params.scanObject.orientation}
            />
          </Suspense>
        </ErrorBoundary>
      )}

      {(params.obstacles ?? []).map((obs, i) => (
        <mesh key={i} position={new THREE.Vector3(...obs.position)}>
          <sphereGeometry args={[obs.radius, 32, 32]} />
          <meshStandardMaterial color="#ff4444" transparent opacity={0.3} wireframe />
        </mesh>
      ))}
    </group>
  );
}

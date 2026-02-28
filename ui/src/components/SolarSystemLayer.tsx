import { Suspense, useMemo } from 'react';
import { useGLTF, Line } from '@react-three/drei';
import * as THREE from 'three';
import { solarSystemBodies, SOLAR_SCALE, AU_M, getSolarBodyPosition } from '../data/solarSystemSnapshot';

interface PlanetModelProps {
  url: string;
  radiusMeters: number;
  position: [number, number, number];
}

function PlanetModel({ url, radiusMeters, position }: PlanetModelProps) {
  const gltf = useGLTF(url);
  const clonedObj = useMemo(() => gltf.scene.clone(), [gltf.scene]);
  const scale = useMemo(() => {
    const box = new THREE.Box3().setFromObject(gltf.scene);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    if (!Number.isFinite(maxDim) || maxDim <= 0) return 1;
    const targetDiameter = radiusMeters * SOLAR_SCALE * 2;
    return targetDiameter / maxDim;
  }, [gltf.scene, radiusMeters]);

  return (
    <primitive
      object={clonedObj}
      position={position}
      scale={[scale, scale, scale]}
    />
  );
}

const buildOrbitPoints = (radius: number, centerX: number, segments = 180) => {
  const points: [number, number, number][] = [];
  for (let i = 0; i <= segments; i += 1) {
    const t = (i / segments) * Math.PI * 2;
    points.push([centerX + Math.cos(t) * radius, 0, Math.sin(t) * radius]);
  }
  return points;
};

export function SolarSystemLayer() {
  const bodies = useMemo(
    () =>
      solarSystemBodies.map((body) => ({
        ...body,
        position: getSolarBodyPosition(body),
      })),
    []
  );
  const sunCenterX = -AU_M * SOLAR_SCALE;

  return (
    <group>
      {bodies
        .filter((body) => body.type === 'planet' || body.type === 'moon')
        .map((body) => {
          const orbitRadius = body.orbit_au * AU_M * SOLAR_SCALE;

          let centerX = sunCenterX;
          if (body.parentId) {
            const parent = bodies.find((b) => b.id === body.parentId);
            if (parent && parent.position) {
              // Parent's X position is the center of the orbit
              centerX = parent.position[0];
            }
          }

          return (
            <group key={`${body.id}-orbit`}>
              <Line
                points={buildOrbitPoints(orbitRadius, centerX, 240)}
                color={body.type === 'moon' ? '#94a3b8' : '#64748b'}
                lineWidth={body.type === 'moon' ? 1 : 2}
                transparent
                opacity={0.55}
                depthWrite={false}
                frustumCulled={false}
                renderOrder={5}
              />
            </group>
          );
        })}
      {bodies.map((body) => {
        if (body.glb) {
          return (
            <Suspense key={body.id} fallback={null}>
              <PlanetModel url={body.glb} radiusMeters={body.radius_m} position={body.position} />
            </Suspense>
          );
        }
        if (body.type === 'sun') {
          return (
            <mesh key={body.id} position={body.position}>
              <sphereGeometry args={[body.radius_m * SOLAR_SCALE, 64, 64]} />
              <meshStandardMaterial color="#fbbf24" emissive="#fbbf24" emissiveIntensity={1.2} />
              <pointLight intensity={2.5} distance={body.radius_m * SOLAR_SCALE * 50} />
            </mesh>
          );
        }
        return null;
      })}
    </group>
  );
}

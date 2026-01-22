import { Suspense, useMemo, useState } from 'react';
import { useGLTF, Line, Html, useCursor } from '@react-three/drei';
import * as THREE from 'three';
import { solarSystemBodies, SOLAR_SCALE, AU_M, getSolarBodyPosition } from '../data/solarSystemSnapshot';
import { useCameraStore } from '../store/cameraStore';

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
  const requestFocus = useCameraStore(s => s.requestFocus);
  const [hoveredId, setHoveredId] = useState<string | null>(null);
  const [hoveredPoint, setHoveredPoint] = useState<[number, number, number] | null>(null);
  const bodies = useMemo(
    () =>
      solarSystemBodies.map((body) => ({
        ...body,
        position: getSolarBodyPosition(body),
      })),
    []
  );
  const sunCenterX = -AU_M * SOLAR_SCALE;
  useCursor(!!hoveredId);

  return (
    <group>
      {bodies
        .filter((body) => body.type === 'planet')
        .map((body) => {
          const orbitRadius = body.orbit_au * AU_M * SOLAR_SCALE;
          const pickWidth = Math.max(orbitRadius * 0.03, 0.2);
          return (
            <group key={`${body.id}-orbit`}>
              <Line
                points={buildOrbitPoints(orbitRadius, sunCenterX, 240)}
                color="#64748b"
                lineWidth={2}
                transparent
                opacity={0.55}
                depthWrite={false}
                frustumCulled={false}
                renderOrder={5}
              />
              <mesh
                position={[sunCenterX, 0, 0]}
                rotation={[Math.PI / 2, 0, 0]}
                renderOrder={10}
                frustumCulled={false}
                onPointerOver={(event) => {
                  event.stopPropagation();
                  setHoveredId(body.id);
                  setHoveredPoint([event.point.x, event.point.y, event.point.z]);
                }}
                onPointerMove={(event) => {
                  event.stopPropagation();
                  setHoveredPoint([event.point.x, event.point.y, event.point.z]);
                }}
                onPointerOut={(event) => {
                  event.stopPropagation();
                  setHoveredId((prev) => (prev === body.id ? null : prev));
                  setHoveredPoint(null);
                }}
                onClick={(event) => {
                  event.stopPropagation();
                  const radiusScene = body.radius_m * SOLAR_SCALE;
                  requestFocus(body.position, Math.max(radiusScene * 3, 5));
                }}
              >
                <torusGeometry args={[orbitRadius, pickWidth, 12, 96]} />
                <meshBasicMaterial transparent opacity={0} depthWrite={false} depthTest={false} />
              </mesh>
            </group>
          );
        })}
      {hoveredId && hoveredPoint && (
        (() => {
          const body = bodies.find((b) => b.id === hoveredId);
          if (!body) return null;
          return (
            <Html position={hoveredPoint} center>
              <div className="px-2 py-1 rounded bg-slate-900/80 border border-slate-700 text-[10px] uppercase tracking-widest text-slate-200">
                {body.name} Orbit
              </div>
            </Html>
          );
        })()
      )}
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

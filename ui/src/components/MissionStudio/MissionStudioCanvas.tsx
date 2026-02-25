import { Suspense, useRef, useEffect } from 'react';
import { Canvas, useLoader } from '@react-three/fiber';
import { OrbitControls, GizmoHelper, GizmoViewport, Grid } from '@react-three/drei';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import * as THREE from 'three';
import { useStudioStore } from './useStudioStore';
import { ScanPassObject } from './ScanPassObject';
import { WaypointNudger } from './WaypointNudger';
import { EndpointNodes } from './EndpointNodes';
import { SatelliteStartNode } from './SatelliteStartNode';
import { ObstacleObjects } from './ObstacleObjects';

function ObjModel({ url }: { url: string }) {
  const obj = useLoader(OBJLoader, url);
  const groupRef = useRef<THREE.Group>(null);
  const setModelBoundingBox = useStudioStore((s) => s.setModelBoundingBox);

  useEffect(() => {
    if (!groupRef.current) return;
    const box = new THREE.Box3().setFromObject(groupRef.current);
    const min = box.min.toArray() as [number, number, number];
    const max = box.max.toArray() as [number, number, number];
    setModelBoundingBox({ min, max });
  }, [obj, setModelBoundingBox]);

  return (
    <group ref={groupRef}>
      <primitive object={obj} />
    </group>
  );
}

function StudioGrid() {
  return (
    <Grid
      args={[100, 100]}
      cellSize={1}
      cellThickness={0.5}
      cellColor="#1e3a5f"
      sectionSize={10}
      sectionThickness={1}
      sectionColor="#2a4f7a"
      fadeDistance={80}
      fadeStrength={1}
      infiniteGrid
    />
  );
}

function SceneContents() {
  const modelUrl = useStudioStore((s) => s.modelUrl);
  const scanPasses = useStudioStore((s) => s.scanPasses);
  const selectedScanId = useStudioStore((s) => s.selectedScanId);

  return (
    <>
      <ambientLight intensity={0.6} />
      <directionalLight position={[10, 20, 10]} intensity={1.2} />

      <StudioGrid />

      {modelUrl && (
        <Suspense fallback={null}>
          <ObjModel url={modelUrl} />
        </Suspense>
      )}

      {scanPasses.map((p) => <ScanPassObject key={p.id} scanId={p.id} />)}
      {selectedScanId && <WaypointNudger scanId={selectedScanId} />}
      <EndpointNodes />
      <SatelliteStartNode />
      <ObstacleObjects />

      <OrbitControls makeDefault />
      <GizmoHelper alignment="bottom-right" margin={[60, 60]}>
        <GizmoViewport labelColor="white" axisHeadScale={0.9} />
      </GizmoHelper>
    </>
  );
}

export function MissionStudioCanvas() {
  return (
    <Canvas
      camera={{ position: [0, 15, 30], fov: 50, near: 0.01, far: 10000 }}
      gl={{ antialias: true, alpha: false }}
      style={{ background: '#070b14', width: '100%', height: '100%' }}
    >
      <color attach="background" args={['#070b14']} />
      <SceneContents />
    </Canvas>
  );
}

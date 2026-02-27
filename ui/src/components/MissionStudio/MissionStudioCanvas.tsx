import { Suspense, useRef, useEffect, useMemo, Component, type ReactNode } from 'react';
import { Canvas, useLoader } from '@react-three/fiber';
import { OrbitControls, GizmoHelper, GizmoViewport, Grid } from '@react-three/drei';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import * as THREE from 'three';
import { useStudioStore } from './useStudioStore';
import { ScanPassObject } from './ScanPassObject';
import { EndpointNodes } from './EndpointNodes';
import { SatelliteStartNode } from './SatelliteStartNode';
import { ObstacleObjects } from './ObstacleObjects';
import { EllipseHandles } from './EllipseHandles';
import { PointObjects } from './PointObjects';

function ObjModel({ url }: { url: string }) {
  const obj = useLoader(OBJLoader, url);
  const groupRef = useRef<THREE.Group>(null);
  const setModelBoundingBox = useStudioStore((s) => s.setModelBoundingBox);

  useEffect(() => {
    if (!groupRef.current) return;
    const box = new THREE.Box3().setFromObject(groupRef.current);
    setModelBoundingBox({
      min: box.min.toArray() as [number, number, number],
      max: box.max.toArray() as [number, number, number],
    });
  }, [obj, setModelBoundingBox]);

  return (
    <group ref={groupRef}>
      <primitive object={obj} />
    </group>
  );
}

class ModelErrorBoundary extends Component<{ children: ReactNode; onError: () => void }, { hasError: boolean }> {
  constructor(props: { children: ReactNode; onError: () => void }) {
    super(props);
    this.state = { hasError: false };
  }
  static getDerivedStateFromError() {
    return { hasError: true };
  }
  componentDidCatch() {
    this.props.onError();
  }
  render() {
    return this.state.hasError ? null : this.props.children;
  }
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

function SceneContents({ onModelError }: { onModelError: () => void }) {
  const modelUrl = useStudioStore((s) => s.modelUrl);
  const paths = useStudioStore((s) => s.paths);
  const wires = useStudioStore((s) => s.wires);
  const holds = useStudioStore((s) => s.holds);
  const assembly = useStudioStore((s) => s.assembly);
  const selectedAssemblyId = useStudioStore((s) => s.selectedAssemblyId);
  const selectedPathId = useStudioStore((s) => s.selectedPathId);
  const activeTool = useStudioStore((s) => s.activeTool);
  const selectedAssembly = useMemo(
    () => (selectedAssemblyId ? assembly.find((item) => item.id === selectedAssemblyId) ?? null : null),
    [assembly, selectedAssemblyId]
  );

  const focusedPathId = useMemo(() => {
    if (!selectedAssembly) return null;
    if (selectedAssembly.type === 'create_path') return selectedAssembly.pathId ?? null;
    if (selectedAssembly.type === 'hold') {
      const hold = holds.find((item) => item.id === selectedAssembly.holdId);
      return hold?.pathId ?? null;
    }
    return null;
  }, [holds, selectedAssembly]);

  const pathsToRender = useMemo(() => {
    if (!selectedAssembly) return paths;
    if (!focusedPathId) return [];
    return paths.filter((path) => path.id === focusedPathId);
  }, [focusedPathId, paths, selectedAssembly]);

  const visibleWireIds = useMemo(() => {
    if (!selectedAssembly) return null;
    if (selectedAssembly.type !== 'connect') return [] as string[];
    return selectedAssembly.wireId ? [selectedAssembly.wireId] : [];
  }, [selectedAssembly]);

  const connectNodeFilter = useMemo(() => {
    if (!selectedAssembly || selectedAssembly.type !== 'connect') return null;
    const wire = wires.find((item) => item.id === selectedAssembly.wireId);
    if (!wire) return [] as string[];
    return [wire.fromNodeId, wire.toNodeId];
  }, [selectedAssembly, wires]);

  const handleBackgroundClick = () => {
    if (selectedPathId) {
      useStudioStore.getState().setSelectedHandle(selectedPathId, null);
    }
  };

  return (
    <>
      <ambientLight intensity={0.6} />
      <directionalLight position={[10, 20, 10]} intensity={1.2} />

      <StudioGrid />

      <mesh visible={false} onClick={handleBackgroundClick}>
        <sphereGeometry args={[500, 8, 8]} />
        <meshBasicMaterial side={THREE.BackSide} />
      </mesh>

      {modelUrl && (
        <ModelErrorBoundary key={modelUrl} onError={onModelError}>
          <Suspense fallback={null}>
            <ObjModel url={modelUrl} />
          </Suspense>
        </ModelErrorBoundary>
      )}

      {pathsToRender.map((p) => (
        <ScanPassObject key={p.id} scanId={p.id} />
      ))}
      {selectedPathId &&
        activeTool === 'create_path' &&
        pathsToRender.some((path) => path.id === selectedPathId) && <EllipseHandles scanId={selectedPathId} />}
      <EndpointNodes visibleWireIds={visibleWireIds} connectNodeFilter={connectNodeFilter} />
      <SatelliteStartNode />
      <ObstacleObjects />
      <PointObjects />

      <OrbitControls makeDefault />
      <GizmoHelper alignment="bottom-right" margin={[60, 60]}>
        <GizmoViewport labelColor="white" axisHeadScale={0.9} />
      </GizmoHelper>
    </>
  );
}

export function MissionStudioCanvas() {
  const setModelUrl = useStudioStore((s) => s.setModelUrl);
  const setReferenceObjectPath = useStudioStore((s) => s.setReferenceObjectPath);
  const setWelcomeDismissed = useStudioStore((s) => s.setWelcomeDismissed);

  const handleModelError = () => {
    setReferenceObjectPath(null);
    setModelUrl(null);
    setWelcomeDismissed(false);
  };

  return (
    <Canvas
      camera={{ position: [0, 15, 30], fov: 50, near: 0.01, far: 10000 }}
      gl={{ antialias: true, alpha: false }}
      style={{ background: '#070b14', width: '100%', height: '100%' }}
    >
      <color attach="background" args={['#070b14']} />
      <SceneContents onModelError={handleModelError} />
    </Canvas>
  );
}

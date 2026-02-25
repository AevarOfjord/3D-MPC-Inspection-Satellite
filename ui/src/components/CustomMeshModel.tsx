import { ObjWithMtl } from './viewport/ObjModelLoader';

interface CustomMeshModelProps {
  objPath: string;
  position: [number, number, number];
  orientation: [number, number, number];
  scale?: number;
}

export function CustomMeshModel({
  objPath,
  position,
  orientation,
  scale = 1
}: CustomMeshModelProps) {
  return (
    <group position={position} rotation={orientation} scale={[scale, scale, scale]}>
      <ObjWithMtl objPath={objPath} />
    </group>
  );
}

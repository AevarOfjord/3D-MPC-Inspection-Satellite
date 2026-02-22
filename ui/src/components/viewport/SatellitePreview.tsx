import * as THREE from 'three';
import { Text } from '@react-three/drei';

export function SatellitePreview({ position, rotation }: { position: [number, number, number]; rotation: [number, number, number] }) {
    const euler = new THREE.Euler(
        (rotation[0] * Math.PI) / 180,
        (rotation[1] * Math.PI) / 180,
        (rotation[2] * Math.PI) / 180
    );
    return (
        <group position={position} rotation={euler}>
            <mesh>
                <boxGeometry args={[0.3, 0.3, 0.3]} />
                <meshStandardMaterial attach="material-0" side={THREE.DoubleSide} color="#ff3b30" emissive="#7f1d1d" emissiveIntensity={0.25} metalness={0.45} roughness={0.25} />
                <meshStandardMaterial attach="material-1" side={THREE.DoubleSide} color="#ff8a80" emissive="#7f1d1d" emissiveIntensity={0.15} metalness={0.35} roughness={0.35} />
                <meshStandardMaterial attach="material-2" side={THREE.DoubleSide} color="#39ff14" emissive="#14532d" emissiveIntensity={0.25} metalness={0.45} roughness={0.25} />
                <meshStandardMaterial attach="material-3" side={THREE.DoubleSide} color="#86efac" emissive="#14532d" emissiveIntensity={0.15} metalness={0.35} roughness={0.35} />
                <meshStandardMaterial attach="material-4" side={THREE.DoubleSide} color="#00c2ff" emissive="#1e3a8a" emissiveIntensity={0.25} metalness={0.45} roughness={0.25} />
                <meshStandardMaterial attach="material-5" side={THREE.DoubleSide} color="#93c5fd" emissive="#1e3a8a" emissiveIntensity={0.15} metalness={0.35} roughness={0.35} />
            </mesh>
            <Text position={[0.155, 0, 0]} rotation={[0, Math.PI / 2, 0]} fontSize={0.055} color="#ffffff" anchorX="center" anchorY="middle">
              +X
            </Text>
            <Text position={[-0.155, 0, 0]} rotation={[0, -Math.PI / 2, 0]} fontSize={0.055} color="#ffffff" anchorX="center" anchorY="middle">
              -X
            </Text>
            <Text position={[0, 0.155, 0]} rotation={[-Math.PI / 2, 0, 0]} fontSize={0.055} color="#ffffff" anchorX="center" anchorY="middle">
              +Y
            </Text>
            <Text position={[0, -0.155, 0]} rotation={[Math.PI / 2, 0, 0]} fontSize={0.055} color="#ffffff" anchorX="center" anchorY="middle">
              -Y
            </Text>
            <Text position={[0, 0, 0.155]} fontSize={0.055} color="#ffffff" anchorX="center" anchorY="middle">
              +Z
            </Text>
            <Text position={[0, 0, -0.155]} rotation={[0, Math.PI, 0]} fontSize={0.055} color="#ffffff" anchorX="center" anchorY="middle">
              -Z
            </Text>
        </group>
    );
}

export function ReferenceModelFallback() {
  return (
    <mesh>
      <sphereGeometry args={[0.4, 16, 16]} />
      <meshStandardMaterial color="#60a5fa" wireframe opacity={0.75} transparent />
    </mesh>
  );
}

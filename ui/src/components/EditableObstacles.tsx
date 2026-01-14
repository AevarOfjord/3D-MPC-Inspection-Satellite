import { TransformControls } from '@react-three/drei';
import { useMissionStore } from '../store/missionStore';

export function EditableObstacles() {
  const { config, updateObstacle, isEditing } = useMissionStore();

  if (!isEditing) return null;

  return (
    <group>
      {config.obstacles.map((obs, i) => (
        <TransformControls
          key={i}
          mode="translate"
          position={[obs.position[0], obs.position[1], obs.position[2]]}
          onObjectChange={(e: any) => {
             if (e.target.object) {
                 const { x, y, z } = e.target.object.position;
                 // Update store (this might be spammy, but gives live feedback)
                 // We could optimize by only syncing on dragEnd if needed.
                 // For now, let's try direct updates. Note: this might cause jitter if re-render resets Pivot.
                 // TransformControls usually handles its own target during drag.
                 // But if we update props 'position', it might reset.
                 // A common pattern is to update store, but TransformControls keeps its internal state?
                 // Or use 'onMouseUp' to commit?
                 
                 // Let's rely on standard binding. R3F TransformControls is robust.
                 updateObstacle(i, 'position', x, 0); // x
                 updateObstacle(i, 'position', y, 1); // y
                 updateObstacle(i, 'position', z, 2); // z
             }
          }}
        >
          <mesh>
            <sphereGeometry args={[obs.radius, 32, 32]} />
            <meshStandardMaterial color="#00ffff" transparent opacity={0.5} wireframe />
          </mesh>
        </TransformControls>
      ))}
    </group>
  );
}

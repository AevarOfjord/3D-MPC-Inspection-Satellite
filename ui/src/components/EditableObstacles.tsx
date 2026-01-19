import { TransformControls, Html } from '@react-three/drei';
import { useMissionStore } from '../store/missionStore';

export function EditableObstacles() {
  const {
    config,
    updateObstacle,
    isEditing,
    selectedObstacleIndex,
    setSelectedObstacleIndex,
    transformAxis,
    transformSnap,
  } = useMissionStore();

  if (!isEditing) return null;

  return (
    <group>
      {config.obstacles.map((obs, i) => (
        <group key={i}>
          {selectedObstacleIndex === i ? (
            <TransformControls
              mode="translate"
              position={[obs.position[0], obs.position[1], obs.position[2]]}
              showX={transformAxis === 'free' || transformAxis === 'x'}
              showY={transformAxis === 'free' || transformAxis === 'y'}
              showZ={transformAxis === 'free' || transformAxis === 'z'}
              translationSnap={transformSnap ?? undefined}
              onObjectChange={(e: any) => {
                if (e.target.object) {
                  const { x, y, z } = e.target.object.position;
                  updateObstacle(i, 'position', x, 0);
                  updateObstacle(i, 'position', y, 1);
                  updateObstacle(i, 'position', z, 2);
                }
              }}
            >
              <group>
                <mesh
                  onClick={(event) => {
                    event.stopPropagation();
                    setSelectedObstacleIndex(i);
                  }}
                >
                  <sphereGeometry args={[obs.radius, 32, 32]} />
                  <meshStandardMaterial color="#00ffff" transparent opacity={0.5} wireframe />
                </mesh>
                <Html position={[0, obs.radius + 0.1, 0]}>
                  <div
                    className="px-2 py-0.5 rounded bg-cyan-500/80 text-black text-[10px] font-bold"
                    style={{ pointerEvents: 'none' }}
                  >
                    #{i + 1}
                  </div>
                </Html>
              </group>
            </TransformControls>
          ) : (
            <group position={[obs.position[0], obs.position[1], obs.position[2]]}>
              <mesh
                onClick={(event) => {
                  event.stopPropagation();
                  setSelectedObstacleIndex(i);
                }}
              >
                <sphereGeometry args={[obs.radius, 32, 32]} />
                <meshStandardMaterial color="#66e6ff" transparent opacity={0.25} wireframe />
              </mesh>
              <Html position={[0, obs.radius + 0.1, 0]}>
                <div
                  className="px-2 py-0.5 rounded bg-cyan-500/60 text-black text-[10px] font-bold"
                  style={{ pointerEvents: 'none' }}
                >
                  #{i + 1}
                </div>
              </Html>
            </group>
          )}
        </group>
      ))}
    </group>
  );
}

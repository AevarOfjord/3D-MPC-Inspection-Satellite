import { useEffect, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import { Vector3, Quaternion } from 'three';
import { telemetry } from '../services/telemetry';
import { useCameraStore } from '../store/cameraStore';
import { EARTH_RADIUS_M, ORBIT_SCALE } from '../data/orbitSnapshot';

type CameraMode = 'free' | 'chase' | 'top';

interface CameraManagerProps {
  mode: CameraMode;
  origin?: [number, number, number];
}

const WORLD_UP = new Vector3(0, 0, 1);
const TOP_VIEW_SCREEN_UP = new Vector3(0, 1, 0);

// Body-frame corner direction for chase cam: -X, -Y, +Z corner
const CHASE_BODY_CORNER = new Vector3(-1, -1, 1).normalize();
// Body-frame +Z axis — rotated to world space each frame to keep it screen-up
const CHASE_BODY_UP = new Vector3(0, 0, 1);
// Distance from satellite centre to camera (metres, same scale as scene)
const CHASE_DISTANCE = 5;

const isValidVector = (v: [number, number, number] | Vector3) => {
  if (v instanceof Vector3) {
    return Number.isFinite(v.x) && Number.isFinite(v.y) && Number.isFinite(v.z);
  }
  return Number.isFinite(v[0]) && Number.isFinite(v[1]) && Number.isFinite(v[2]);
};

const CHASE_ZOOM_SPEED = 0.1;   // fraction of current distance per scroll tick
const CHASE_MIN_DIST  = 0.5;    // metres
const CHASE_MAX_DIST  = 500;    // metres

export function CameraManager({ mode, origin = [0, 0, 0] }: CameraManagerProps) {
  const { camera, controls } = useThree();
  const satPosRef = useRef(new Vector3());
  const satQuatRef = useRef(new Quaternion());
  const originRef = useRef(new Vector3(origin[0], origin[1], origin[2]));
  const chaseDistance = useCameraStore(s => s.chaseDistance);
  const setChaseDistance = useCameraStore(s => s.setChaseDistance);
  const focusTarget = useCameraStore(s => s.focusTarget);
  const focusDistance = useCameraStore(s => s.focusDistance);
  const focusNonce = useCameraStore(s => s.focusNonce);
  const viewPreset = useCameraStore(s => s.viewPreset);
  const viewNonce = useCameraStore(s => s.viewNonce);
  const fallbackDistance = EARTH_RADIUS_M * 2.5 * ORBIT_SCALE;
  const baseDistance = Number.isFinite(focusDistance ?? NaN) ? (focusDistance as number) : fallbackDistance;

  useEffect(() => {
    originRef.current.set(origin[0], origin[1], origin[2]);
  }, [origin[0], origin[1], origin[2]]);

  // Scroll-to-zoom for chase mode
  useEffect(() => {
    const onWheel = (e: WheelEvent) => {
      if (mode !== 'chase') return;
      e.preventDefault();
      // deltaY > 0 → scroll down → zoom out (increase distance)
      const factor = 1 + Math.sign(e.deltaY) * CHASE_ZOOM_SPEED;
      setChaseDistance(Math.min(
        CHASE_MAX_DIST,
        Math.max(CHASE_MIN_DIST, chaseDistance * factor)
      ));
    };
    window.addEventListener('wheel', onWheel, { passive: false });
    return () => window.removeEventListener('wheel', onWheel);
  }, [chaseDistance, mode, setChaseDistance]);

  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       if (!d || !d.position || !d.quaternion) return;
       // Valid Check
       if (!isValidVector(d.position)) return;
       if (d.quaternion.some(v => !Number.isFinite(v))) return;

       satPosRef.current.set(
         d.position[0] - originRef.current.x,
         d.position[1] - originRef.current.y,
         d.position[2] - originRef.current.z
       );
       const [w, x, y, z] = d.quaternion;
       satQuatRef.current.set(x, y, z, w);
    });
    return () => { unsub(); };
  }, []);

  // Handle Mode Switching transitions
  useEffect(() => {
    if (mode === 'top') {
       // Move to Top Down
       camera.position.set(0, 0, baseDistance);
       camera.lookAt(0, 0, 0);
       camera.up.copy(TOP_VIEW_SCREEN_UP); // Keep screen-up along +Y for top-down view.
       if (controls) {
           (controls as any).enabled = true;
           (controls as any).target.set(0, 0, 0);
           (controls as any).update();
       }
    } else if (mode === 'chase') {
        camera.up.copy(WORLD_UP);
        setChaseDistance(CHASE_DISTANCE);
        // Disable orbit controls — camera is programmatically locked to the corner
        if (controls) {
            (controls as any).enabled = false;
        }
    } else {
        camera.up.copy(WORLD_UP);
        if (controls) {
            (controls as any).enabled = true;
        }
    }
  }, [baseDistance, mode, camera, controls, setChaseDistance]);

  useEffect(() => {
    if (!focusTarget) return;
    const target = new Vector3(
      focusTarget[0] - originRef.current.x,
      focusTarget[1] - originRef.current.y,
      focusTarget[2] - originRef.current.z
    );
    const distance = Number.isFinite(focusDistance ?? NaN) ? (focusDistance as number) : 2.5;
    const offset = new Vector3(1, 1, 0.8).normalize().multiplyScalar(distance);
    camera.position.copy(target.clone().add(offset));
    camera.lookAt(target);
    if (controls) {
      (controls as any).target.copy(target);
      (controls as any).update();
    }
  }, [
    camera,
    controls,
    focusDistance,
    focusNonce,
    focusTarget,
    origin[0],
    origin[1],
    origin[2],
  ]);

  useEffect(() => {
    if (!viewPreset) return;
    const target = focusTarget
      ? new Vector3(
          focusTarget[0] - originRef.current.x,
          focusTarget[1] - originRef.current.y,
          focusTarget[2] - originRef.current.z
        )
      : new Vector3(0, 0, 0);
    let offset: Vector3;
    let up = WORLD_UP;

    switch (viewPreset) {
      case 'top':
        offset = new Vector3(0, 0, baseDistance);
        up = TOP_VIEW_SCREEN_UP;
        break;
      case 'front':
        offset = new Vector3(0, baseDistance, 0);
        break;
      case 'back':
        offset = new Vector3(0, -baseDistance, 0);
        break;
      case 'left':
        offset = new Vector3(-baseDistance, 0, 0);
        break;
      case 'right':
        offset = new Vector3(baseDistance, 0, 0);
        break;
      case 'iso':
      default:
        offset = new Vector3(baseDistance, baseDistance, baseDistance);
        break;
    }

    camera.up.copy(up);
    camera.position.copy(target.clone().add(offset));
    camera.lookAt(target);
    if (controls) {
      (controls as any).target.copy(target);
      (controls as any).update();
    }
  }, [
    camera,
    controls,
    focusTarget,
    viewNonce,
    viewPreset,
    baseDistance,
    origin[0],
    origin[1],
    origin[2],
  ]);

  useFrame(() => {
    if (mode === 'chase') {
        // Fixed-corner chase: lock camera to the -X/-Y/+Z body-frame corner,
        // rotating with the satellite so the viewpoint is always from that corner.
        const satPos = satPosRef.current.clone();
        const satQuat = satQuatRef.current.clone();

        // Rotate the body-frame corner direction into world space
        const worldCorner = CHASE_BODY_CORNER.clone().applyQuaternion(satQuat);

        // Position camera along that world-space direction from the satellite centre
        const camPos = satPos.clone().addScaledVector(worldCorner, chaseDistance);
        camera.position.copy(camPos);

        // Use satellite body +Z rotated to world space as camera up,
        // so +Z always appears pointing up on screen
        const worldUp = CHASE_BODY_UP.clone().applyQuaternion(satQuat);
        camera.up.copy(worldUp);

        // Point camera at the satellite centre
        camera.lookAt(satPos);
        camera.updateMatrixWorld();

        // Keep controls target in sync (for zoom/pan when controls re-enabled)
        if (controls) {
            (controls as any).target.copy(satPos);
        }
    } else if (mode === 'top') {
         // Keep looking at 0,0,0? Or follow sat from top?
         // Let's just fix it for now to overview.
    }
  });

  return null;
}

import { useEffect, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import { Vector3, Quaternion } from 'three';
import { telemetry } from '../services/telemetry';
import { useCameraStore } from '../store/cameraStore';

type CameraMode = 'free' | 'chase' | 'top';

interface CameraManagerProps {
  mode: CameraMode;
}

const WORLD_UP = new Vector3(0, 0, 1);
const TOP_VIEW_SCREEN_UP = new Vector3(0, 1, 0);

export function CameraManager({ mode }: CameraManagerProps) {
  const { camera, controls } = useThree();
  const satPosRef = useRef(new Vector3());
  const satQuatRef = useRef(new Quaternion());
  const focusTarget = useCameraStore(s => s.focusTarget);
  const focusNonce = useCameraStore(s => s.focusNonce);
  const viewPreset = useCameraStore(s => s.viewPreset);
  const viewNonce = useCameraStore(s => s.viewNonce);
  
  useEffect(() => {
    const unsub = telemetry.subscribe(d => {
       satPosRef.current.set(d.position[0], d.position[1], d.position[2]);
       const [w, x, y, z] = d.quaternion;
       satQuatRef.current.set(x, y, z, w);
    });
    return () => { unsub(); };
  }, []);

  // Handle Mode Switching transitions
  useEffect(() => {
    if (mode === 'top') {
       // Move to Top Down
       camera.position.set(0, 0, 15);
       camera.lookAt(0, 0, 0);
       camera.up.copy(TOP_VIEW_SCREEN_UP); // Keep screen-up along +Y for top-down view.
       if (controls) {
           (controls as any).target.set(0, 0, 0);
           (controls as any).update();
       }
    } else {
        camera.up.copy(WORLD_UP);
    }
  }, [mode, camera, controls]);

  useEffect(() => {
    if (!focusTarget) return;
    const target = new Vector3(...focusTarget);
    const offset = new Vector3(2.5, 2.5, 2.0);
    camera.position.copy(target.clone().add(offset));
    camera.lookAt(target);
    if (controls) {
      (controls as any).target.copy(target);
      (controls as any).update();
    }
  }, [camera, controls, focusNonce, focusTarget]);

  useEffect(() => {
    if (!viewPreset) return;
    const target = focusTarget ? new Vector3(...focusTarget) : new Vector3(0, 0, 0);
    let offset: Vector3;
    let up = WORLD_UP;

    switch (viewPreset) {
      case 'top':
        offset = new Vector3(0, 0, 10);
        up = TOP_VIEW_SCREEN_UP;
        break;
      case 'front':
        offset = new Vector3(0, 10, 0);
        break;
      case 'back':
        offset = new Vector3(0, -10, 0);
        break;
      case 'left':
        offset = new Vector3(-10, 0, 0);
        break;
      case 'right':
        offset = new Vector3(10, 0, 0);
        break;
      case 'iso':
      default:
        offset = new Vector3(6, 6, 6);
        break;
    }

    camera.up.copy(up);
    camera.position.copy(target.clone().add(offset));
    camera.lookAt(target);
    if (controls) {
      (controls as any).target.copy(target);
      (controls as any).update();
    }
  }, [camera, controls, focusTarget, viewNonce, viewPreset]);

  useFrame(() => {
    if (mode === 'chase') {
        // Simple Chase: Position camera behind satellite
        // Offset in local space: [-2, 0, 1] (Behind and slightly up +Z)
        const offset = new Vector3(-2, 0, 1);
        offset.applyQuaternion(satQuatRef.current);
        const targetPos = satPosRef.current.clone();
        
        const camPos = targetPos.clone().add(offset);
        
        // Smooth lerp could go here, but strict follow for now
        camera.position.lerp(camPos, 0.1);
        camera.lookAt(targetPos);
        
        if (controls) {
            (controls as any).target.copy(targetPos);
            (controls as any).update();
        }
    } else if (mode === 'top') {
         // Keep looking at 0,0,0? Or follow sat from top?
         // Let's just fix it for now to overview. 
    }
  });

  return null;
}

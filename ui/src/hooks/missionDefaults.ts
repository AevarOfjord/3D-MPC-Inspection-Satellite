import * as THREE from 'three';

import type {
  HoldSegment,
  ScanConfig,
  ScanSegment,
  TransferSegment,
} from '../api/unifiedMission';
import { orbitSnapshot } from '../data/orbitSnapshot';

let segmentCounter = 0;
let missionCounter = 0;

export const nextSegmentId = (prefix: string) => {
  segmentCounter += 1;
  return `${prefix}_${segmentCounter.toString().padStart(4, '0')}`;
};

export const nextMissionId = () => {
  missionCounter += 1;
  return `mission_${missionCounter.toString().padStart(6, '0')}`;
};

export const defaultTransferSegment = (): TransferSegment => ({
  segment_id: nextSegmentId('transfer'),
  title: 'Transfer',
  notes: null,
  type: 'transfer',
  end_pose: { frame: 'LVLH', position: [0, 0, 0] },
  constraints: { speed_max: 0.25, accel_max: 0.05, angular_rate_max: 0.1 },
});

export const defaultTransferToPathSegment = (): TransferSegment => ({
  segment_id: nextSegmentId('transfer'),
  title: 'Transfer To Path',
  notes: null,
  type: 'transfer',
  end_pose: { frame: 'LVLH', position: [0, 0, 0] },
  constraints: { speed_max: 0.25, accel_max: 0.05, angular_rate_max: 0.1 },
});

const defaultScanConfig = (): ScanConfig => ({
  frame: 'LVLH',
  axis: '+Z',
  standoff: 10,
  overlap: 0.25,
  fov_deg: 60,
  pitch: null,
  revolutions: 4,
  direction: 'CW',
  sensor_axis: '+Y',
  pattern: 'spiral',
});

export const defaultScanSegment = (): ScanSegment => ({
  segment_id: nextSegmentId('scan'),
  title: null,
  notes: null,
  type: 'scan',
  target_id: '',
  scan: defaultScanConfig(),
  constraints: { speed_max: 0.2, accel_max: 0.03, angular_rate_max: 0.08 },
});

export const defaultHoldSegment = (): HoldSegment => ({
  segment_id: nextSegmentId('hold'),
  title: null,
  notes: null,
  type: 'hold',
  duration: 0,
  constraints: { speed_max: 0.1 },
});

const computeFacingEuler = (
  position: [number, number, number],
  baseAxis: [number, number, number] = [0, 0, -1],
  fallback: [number, number, number] = [0, 0, 0]
) => {
  const toEarth = new THREE.Vector3(-position[0], -position[1], -position[2]);
  if (toEarth.lengthSq() < 1e-8) return fallback;
  toEarth.normalize();
  const base = new THREE.Vector3(baseAxis[0], baseAxis[1], baseAxis[2]);
  if (base.lengthSq() < 1e-8) return fallback;
  base.normalize();
  const quat = new THREE.Quaternion().setFromUnitVectors(base, toEarth);
  const euler = new THREE.Euler().setFromQuaternion(quat);
  return [euler.x, euler.y, euler.z] as [number, number, number];
};

const eulerToQuat = (euler: [number, number, number]) => {
  const quat = new THREE.Quaternion().setFromEuler(new THREE.Euler(euler[0], euler[1], euler[2]));
  return [quat.w, quat.x, quat.y, quat.z] as [number, number, number, number];
};

export const resolveOrbitTargetPose = (targetId: string) => {
  const obj = orbitSnapshot.objects.find((o) => o.id === targetId);
  if (!obj) return undefined;
  const position = [...obj.position_m] as [number, number, number];
  const baseOrientation = obj.orientation ?? [0, 0, 0];
  const euler = obj.align_to_earth
    ? computeFacingEuler(position, obj.base_axis ?? [0, 0, -1], baseOrientation as [number, number, number])
    : (baseOrientation as [number, number, number]);
  const orientation = eulerToQuat(euler);
  return { frame: 'ECI' as const, position, orientation };
};

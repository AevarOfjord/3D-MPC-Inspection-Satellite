import * as THREE from 'three';
import type { BodyAxis } from '../../types/scanProject';

/**
 * Build auto-positioned Bézier control points for connector curves.
 */
export const buildAutoConnectorControls = (
  start: [number, number, number],
  end: [number, number, number]
): { control1: [number, number, number]; control2: [number, number, number] } => {
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const dz = end[2] - start[2];
  const dist = Math.hypot(dx, dy, dz);
  if (dist < 1e-6) {
    return {
      control1: [...start],
      control2: [...end],
    };
  }
  const dir: [number, number, number] = [dx / dist, dy / dist, dz / dist];
  const up: [number, number, number] = Math.abs(dir[2]) < 0.9 ? [0, 0, 1] : [0, 1, 0];
  const cx = dir[1] * up[2] - dir[2] * up[1];
  const cy = dir[2] * up[0] - dir[0] * up[2];
  const cz = dir[0] * up[1] - dir[1] * up[0];
  const clen = Math.hypot(cx, cy, cz);
  const side: [number, number, number] = clen > 1e-6 ? [cx / clen, cy / clen, cz / clen] : [0, 0, 0];
  const bulge = Math.min(Math.max(dist * 0.25, 0.15), 2.0);
  return {
    control1: [
      start[0] + dx * 0.33 + side[0] * bulge,
      start[1] + dy * 0.33 + side[1] * bulge,
      start[2] + dz * 0.33 + side[2] * bulge,
    ],
    control2: [
      start[0] + dx * 0.66 + side[0] * bulge,
      start[1] + dy * 0.66 + side[1] * bulge,
      start[2] + dz * 0.66 + side[2] * bulge,
    ],
  };
};

/**
 * Resolve a body axis label ('X', 'Y', 'Z') to a world-space direction
 * given the reference body orientation.
 */
export function resolveBodyAxisVector(
  axis: BodyAxis,
  referenceAngle: [number, number, number]
): [number, number, number] {
  const basis: [number, number, number] =
    axis === 'X' ? [1, 0, 0] : axis === 'Y' ? [0, 1, 0] : [0, 0, 1];
  const e = new THREE.Euler(
    (referenceAngle[0] * Math.PI) / 180,
    (referenceAngle[1] * Math.PI) / 180,
    (referenceAngle[2] * Math.PI) / 180
  );
  const v = new THREE.Vector3(basis[0], basis[1], basis[2]).applyEuler(e).normalize();
  return [v.x, v.y, v.z];
}

/**
 * Resolve the normal, U, and V axes for a scan plane given the body axis
 * and reference angle.
 */
export function resolveScanFrameAxes(
  axis: BodyAxis,
  referenceAngle: [number, number, number]
): {
  normal: [number, number, number];
  uAxis: [number, number, number];
  vAxis: [number, number, number];
} {
  const basisNormal: [number, number, number] =
    axis === 'X' ? [1, 0, 0] : axis === 'Y' ? [0, 1, 0] : [0, 0, 1];
  const basisU: [number, number, number] =
    axis === 'X' ? [0, 1, 0] : axis === 'Y' ? [1, 0, 0] : [1, 0, 0];
  const basisV: [number, number, number] =
    axis === 'X' ? [0, 0, 1] : axis === 'Y' ? [0, 0, 1] : [0, 1, 0];

  const e = new THREE.Euler(
    (referenceAngle[0] * Math.PI) / 180,
    (referenceAngle[1] * Math.PI) / 180,
    (referenceAngle[2] * Math.PI) / 180
  );
  const normal = new THREE.Vector3(...basisNormal).applyEuler(e).normalize();
  const u = new THREE.Vector3(...basisU).applyEuler(e).normalize();
  const v = new THREE.Vector3(...basisV).applyEuler(e).normalize();
  return {
    normal: [normal.x, normal.y, normal.z],
    uAxis: [u.x, u.y, u.z],
    vAxis: [v.x, v.y, v.z],
  };
}

/**
 * Project a point onto a line defined by an axis direction through an origin point.
 */
export function projectPointToAxisThrough(
  point: [number, number, number],
  axis: [number, number, number],
  origin: [number, number, number]
): [number, number, number] {
  const rel: [number, number, number] = [
    point[0] - origin[0],
    point[1] - origin[1],
    point[2] - origin[2],
  ];
  const t = rel[0] * axis[0] + rel[1] * axis[1] + rel[2] * axis[2];
  return [
    origin[0] + axis[0] * t,
    origin[1] + axis[1] * t,
    origin[2] + axis[2] * t,
  ];
}

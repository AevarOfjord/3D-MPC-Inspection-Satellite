import * as THREE from 'three';

function removeNearDuplicates(points: [number, number, number][], eps = 1e-6): [number, number, number][] {
  if (points.length <= 1) return points;
  const out: [number, number, number][] = [points[0]];
  for (let i = 1; i < points.length; i += 1) {
    const a = out[out.length - 1];
    const b = points[i];
    if (Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) > eps) out.push(b);
  }
  return out;
}

export function sampleCatmullRomBySpacing(
  points: [number, number, number][],
  spacing: number
): [number, number, number][] {
  if (!points || points.length < 2) return points;
  const clean = removeNearDuplicates(points);
  if (clean.length < 2) return [clean[0]];
  if (clean.length === 2) {
    const dx = clean[1][0] - clean[0][0];
    const dy = clean[1][1] - clean[0][1];
    const dz = clean[1][2] - clean[0][2];
    const dist = Math.hypot(dx, dy, dz);
    const steps = Math.max(1, Math.ceil(dist / Math.max(1e-3, spacing)));
    const out: [number, number, number][] = [];
    for (let i = 0; i <= steps; i += 1) {
      const t = i / steps;
      out.push([
        clean[0][0] + dx * t,
        clean[0][1] + dy * t,
        clean[0][2] + dz * t,
      ]);
    }
    return out;
  }

  const safeSpacing = Math.max(1e-3, spacing);
  const vectors = clean.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
  const curve = new THREE.CatmullRomCurve3(vectors, false, 'centripetal', 0.5);
  const length = curve.getLength();
  const steps = Math.max(8, Math.ceil(length / safeSpacing));
  const spaced = curve.getSpacedPoints(steps);
  return spaced.map((v) => [v.x, v.y, v.z]);
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

function normalize(v: [number, number, number]): [number, number, number] {
  const n = Math.hypot(v[0], v[1], v[2]);
  if (n <= 1e-9) return [0, 0, 0];
  return [v[0] / n, v[1] / n, v[2] / n];
}

function dot(a: [number, number, number], b: [number, number, number]): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function fairCorners(
  points: [number, number, number][],
  minAngleDeg = 145,
  iterations = 2
): [number, number, number][] {
  if (!points || points.length < 3) return points;
  let cur = points.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
  const minA = clamp(minAngleDeg, 90, 179);
  const iters = Math.max(1, Math.floor(iterations));
  for (let k = 0; k < iters; k += 1) {
    const next = cur.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
    for (let i = 1; i < cur.length - 1; i += 1) {
      const a = cur[i - 1];
      const b = cur[i];
      const c = cur[i + 1];
      const v1 = normalize([a[0] - b[0], a[1] - b[1], a[2] - b[2]]);
      const v2 = normalize([c[0] - b[0], c[1] - b[1], c[2] - b[2]]);
      const ang = Math.acos(clamp(dot(v1, v2), -1, 1)) * (180 / Math.PI);
      if (ang >= minA) continue;
      const t = clamp((minA - ang) / Math.max(1e-6, minA), 0, 1);
      const alpha = 0.55 * t;
      const mid: [number, number, number] = [
        0.5 * (a[0] + c[0]),
        0.5 * (a[1] + c[1]),
        0.5 * (a[2] + c[2]),
      ];
      next[i] = [
        b[0] + alpha * (mid[0] - b[0]),
        b[1] + alpha * (mid[1] - b[1]),
        b[2] + alpha * (mid[2] - b[2]),
      ];
    }
    cur = next;
  }
  return cur;
}

export interface SmoothOptions {
  angleThresholdDeg?: number;
  strength?: number;
  iterations?: number;
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

export function smoothSharpCorners(
  points: [number, number, number][],
  opts: SmoothOptions = {}
): [number, number, number][] {
  if (!points || points.length < 3) return points;
  const angleThresholdDeg = opts.angleThresholdDeg ?? 150;
  const strength = clamp(opts.strength ?? 0.45, 0, 1);
  const iterations = Math.max(1, Math.floor(opts.iterations ?? 2));

  let current = points.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
  for (let it = 0; it < iterations; it += 1) {
    const next = current.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
    for (let i = 1; i < current.length - 1; i += 1) {
      const a = current[i - 1];
      const b = current[i];
      const c = current[i + 1];
      const v1 = normalize([a[0] - b[0], a[1] - b[1], a[2] - b[2]]);
      const v2 = normalize([c[0] - b[0], c[1] - b[1], c[2] - b[2]]);
      const angle = Math.acos(clamp(dot(v1, v2), -1, 1)) * (180 / Math.PI);
      if (angle >= angleThresholdDeg) continue;
      const t = clamp((angleThresholdDeg - angle) / Math.max(1e-6, angleThresholdDeg), 0, 1);
      const alpha = strength * t;
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
    current = next;
  }
  return current;
}

export function smoothPolylineChaikin(
  points: [number, number, number][],
  iterations = 2
): [number, number, number][] {
  if (!points || points.length < 3) return points;
  let current = points.map((p) => [p[0], p[1], p[2]] as [number, number, number]);
  const iters = Math.max(1, Math.floor(iterations));
  for (let k = 0; k < iters; k += 1) {
    const next: [number, number, number][] = [current[0]];
    for (let i = 0; i < current.length - 1; i += 1) {
      const p0 = current[i];
      const p1 = current[i + 1];
      const q: [number, number, number] = [
        0.75 * p0[0] + 0.25 * p1[0],
        0.75 * p0[1] + 0.25 * p1[1],
        0.75 * p0[2] + 0.25 * p1[2],
      ];
      const r: [number, number, number] = [
        0.25 * p0[0] + 0.75 * p1[0],
        0.25 * p0[1] + 0.75 * p1[1],
        0.25 * p0[2] + 0.75 * p1[2],
      ];
      next.push(q, r);
    }
    next.push(current[current.length - 1]);
    current = next;
  }
  return current;
}

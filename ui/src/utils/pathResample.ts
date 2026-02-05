import { CatmullRomCurve3, Vector3 } from 'three';

export type Vec3 = [number, number, number];

export function resamplePath(points: Vec3[], multiplier = 10): Vec3[] {
  if (!points || points.length < 2 || multiplier <= 1) {
    return points ? [...points] : [];
  }

  const targetCount = Math.max(2, Math.round(points.length * multiplier));

  if (points.length === 2) {
    const [a, b] = points;
    const out: Vec3[] = [];
    const denom = Math.max(1, targetCount - 1);
    for (let i = 0; i < targetCount; i++) {
      const t = i / denom;
      out.push([
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
      ]);
    }
    return out;
  }

  const curve = new CatmullRomCurve3(points.map((p) => new Vector3(...p)), false, 'centripetal');
  const divisions = Math.max(1, targetCount - 1);
  return curve.getPoints(divisions).map((p) => [p.x, p.y, p.z]);
}

import { useCallback, useRef } from 'react';
import { API_BASE_URL } from '../../config/endpoints';
import { useStudioStore } from './useStudioStore';
import type { StudioPath } from './useStudioStore';

async function fetchScanPath(path: StudioPath): Promise<[number, number, number][]> {
  const body = {
    axis_seed: path.axisSeed,
    plane_a: path.planeA,
    plane_b: path.planeB,
    ellipse: {
      radius_x: path.ellipse.radiusX,
      radius_y: path.ellipse.radiusY,
    },
    level_spacing_m: path.levelSpacing,
    point_density_scale: path.waypointDensity,
  };
  const res = await fetch(`${API_BASE_URL}/api/models/generate_scan_path`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`generate_scan_path failed: ${res.status}`);
  const data = (await res.json()) as { waypoints: [number, number, number][] };
  return data.waypoints;
}

function isFiniteVec3(p: [number, number, number]): boolean {
  return Number.isFinite(p[0]) && Number.isFinite(p[1]) && Number.isFinite(p[2]);
}

function normalizeQuat(q: [number, number, number, number]): [number, number, number, number] {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n <= 1e-9) return [1, 0, 0, 0];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

function quatToMatrix(q: [number, number, number, number]): number[][] {
  const [w, x, y, z] = normalizeQuat(q);
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

function axisFrame(_axisSeed: StudioPath['axisSeed']): { u: [number, number, number]; v: [number, number, number] } {
  // Local in-plane basis for plane geometry (+Z normal).
  return { u: [1, 0, 0], v: [0, 1, 0] };
}

function mulMatVec(m: number[][], v: [number, number, number]): [number, number, number] {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

function dot(a: [number, number, number], b: [number, number, number]): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function norm(v: [number, number, number]): number {
  return Math.hypot(v[0], v[1], v[2]);
}

function normalize(v: [number, number, number], fallback: [number, number, number]): [number, number, number] {
  const n = norm(v);
  if (n <= 1e-9) return fallback;
  return [v[0] / n, v[1] / n, v[2] / n];
}

function sub(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function add(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(v: [number, number, number], s: number): [number, number, number] {
  return [v[0] * s, v[1] * s, v[2] * s];
}

function cross(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function buildFallbackPath(path: StudioPath): [number, number, number][] {
  const a = path.planeA.position;
  const b = path.planeB.position;
  const qa = path.planeA.orientation;
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const dz = b[2] - a[2];
  const span = Math.max(1e-6, Math.hypot(dx, dy, dz));
  const nAxis = normalize([dx, dy, dz] as [number, number, number], [0, 0, 1]);
  const turns = Math.max(1, span / Math.max(0.05, path.levelSpacing));
  const pointsPerTurn = 32 * Math.max(0.25, Math.min(25, path.waypointDensity ?? 1));
  const total = Math.max(8, Math.ceil(turns * pointsPerTurn));
  const base = axisFrame(path.axisSeed);
  const rotA = quatToMatrix(qa);
  const uSeed = mulMatVec(rotA, base.u);
  const uProj = sub(uSeed, scale(nAxis, dot(uSeed, nAxis)));
  let u = normalize(uProj, [1, 0, 0]);
  if (Math.abs(dot(u, nAxis)) > 0.9) {
    const alt = Math.abs(nAxis[2]) < 0.9 ? ([0, 0, 1] as [number, number, number]) : ([1, 0, 0] as [number, number, number]);
    u = normalize(cross(alt, nAxis), [1, 0, 0]);
  }
  const v = normalize(cross(nAxis, u), [0, 1, 0]);
  const out: [number, number, number][] = [];
  for (let i = 0; i <= total; i += 1) {
    const t = i / Math.max(1, total);
    const c: [number, number, number] = [a[0] + dx * t, a[1] + dy * t, a[2] + dz * t];
    const ang = 2 * Math.PI * turns * t;
    const ex = path.ellipse.radiusX * Math.cos(ang);
    const ey = path.ellipse.radiusY * Math.sin(ang);
    const off = add(scale(u, ex), scale(v, ey));
    out.push([
      c[0] + off[0],
      c[1] + off[1],
      c[2] + off[2],
    ] as [number, number, number]);
  }
  return out;
}

export function useRegenerateWaypoints() {
  const timerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const regenerate = useCallback((pathId: string, debounceMs = 0) => {
    if (timerRef.current !== null) clearTimeout(timerRef.current);
    timerRef.current = setTimeout(() => {
      const path = useStudioStore.getState().paths.find((p) => p.id === pathId);
      if (!path) return;
      void fetchScanPath(path)
        .then((waypoints) => {
          const valid = (waypoints || []).filter((p) => Array.isArray(p) && p.length === 3 && isFiniteVec3(p as [number, number, number])) as [number, number, number][];
          if (valid.length >= 2) {
            useStudioStore.getState().setWaypointsFromBackend(pathId, valid);
            return;
          }
          useStudioStore.getState().setWaypointsFromBackend(pathId, buildFallbackPath(path));
        })
        .catch(() => {
          // Keep Studio responsive even if backend generation fails.
          useStudioStore.getState().setWaypointsFromBackend(pathId, buildFallbackPath(path));
        });
    }, debounceMs);
  }, []);

  return regenerate;
}

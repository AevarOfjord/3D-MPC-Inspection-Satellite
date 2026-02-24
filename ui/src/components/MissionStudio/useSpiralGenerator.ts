interface SpiralParams {
  axis: 'X' | 'Y' | 'Z';
  planeAOffset: number;
  planeBOffset: number;
  crossSection: [number, number][];
  levelHeight: number;
}

export function generateSpiral(params: SpiralParams): [number, number, number][] {
  const { axis, planeAOffset, planeBOffset, crossSection, levelHeight } = params;
  const gap = planeBOffset - planeAOffset;
  if (Math.abs(gap) < 0.001 || levelHeight <= 0) return [];

  const turns = Math.abs(gap) / levelHeight;
  const pointsPerTurn = 32;
  const totalPoints = Math.max(4, Math.round(turns * pointsPerTurn));

  const perimeterPoints = buildPerimeterSamples(crossSection, pointsPerTurn);

  const waypoints: [number, number, number][] = [];
  for (let i = 0; i <= totalPoints; i++) {
    const t = i / totalPoints;
    const along = planeAOffset + gap * t;
    const ringIndex = Math.floor((t * turns * pointsPerTurn) % pointsPerTurn);
    const [u, v] = perimeterPoints[ringIndex % perimeterPoints.length];

    let x = 0, y = 0, z = 0;
    if (axis === 'Z') { x = u; y = v; z = along; }
    else if (axis === 'X') { y = u; z = v; x = along; }
    else { x = u; z = v; y = along; }

    waypoints.push([x, y, z]);
  }

  return waypoints;
}

function buildPerimeterSamples(polygon: [number, number][], count: number): [number, number][] {
  const n = polygon.length;
  let totalLen = 0;
  const segLengths: number[] = [];
  for (let i = 0; i < n; i++) {
    const a = polygon[i];
    const b = polygon[(i + 1) % n];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    segLengths.push(len);
    totalLen += len;
  }

  const samples: [number, number][] = [];
  for (let s = 0; s < count; s++) {
    const target = (s / count) * totalLen;
    let acc = 0;
    for (let i = 0; i < n; i++) {
      const next = acc + segLengths[i];
      if (target <= next || i === n - 1) {
        const t = segLengths[i] > 0 ? (target - acc) / segLengths[i] : 0;
        const a = polygon[i];
        const b = polygon[(i + 1) % n];
        samples.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
        break;
      }
      acc = next;
    }
  }
  return samples;
}

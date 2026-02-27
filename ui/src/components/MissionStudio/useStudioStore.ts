import { create } from 'zustand';
import { fairCorners } from './splineUtils';

export type StudioTool =
  | 'place_satellite'
  | 'create_path'
  | 'edit'
  | 'connect'
  | 'hold'
  | 'obstacle'
  | 'point'
  | null;

export type StudioAxisSeed = 'X' | 'Y' | 'Z';

export interface PlanePose {
  position: [number, number, number];
  orientation: [number, number, number, number]; // [w,x,y,z]
}

export interface EllipseShape {
  radiusX: number;
  radiusY: number;
}

export interface StudioPath {
  id: string;
  axisSeed: StudioAxisSeed;
  planeA: PlanePose;
  planeB: PlanePose;
  ellipse: EllipseShape;
  levelSpacing: number;
  waypointDensity: number;
  densityScope: 'total' | 'snippet';
  densitySnippetRange: [number, number] | null;
  isLocallyEdited: boolean;
  waypoints: [number, number, number][];
  color: string;
  selectedHandleId: 'rx_pos' | 'rx_neg' | 'ry_pos' | 'ry_neg' | null;
}

export interface TransferWire {
  id: string;
  fromNodeId: string;
  toNodeId: string;
  waypoints?: [number, number, number][];
  constraintMode?: 'constrained' | 'free';
}

export interface HoldMarker {
  id: string;
  pathId: string;
  waypointIndex: number;
  duration: number;
}

export interface StudioObstacle {
  id: string;
  position: [number, number, number];
  radius: number;
}

export interface StudioPoint {
  id: string;
  position: [number, number, number];
}

export type StudioAssemblyType =
  | 'place_satellite'
  | 'create_path'
  | 'connect'
  | 'hold'
  | 'obstacle'
  | 'point';

export interface StudioAssemblyItem {
  id: string;
  type: StudioAssemblyType;
  pathId?: string;
  wireId?: string;
  holdId?: string;
  obstacleId?: string;
  pointId?: string;
}

export type WireDragState =
  | { phase: 'idle' }
  | { phase: 'dragging'; sourceNodeId: string; cursorWorld: [number, number, number] };

export interface StudioState {
  modelUrl: string | null;
  referenceObjectPath: string | null;
  modelBoundingBox: { min: [number, number, number]; max: [number, number, number] } | null;

  activeTool: StudioTool;
  pathEditMode: 'translate' | 'rotate' | 'edit';
  editMode: 'stretch' | 'add' | 'delete' | 'density';

  satelliteStart: [number, number, number];
  paths: StudioPath[];
  wires: TransferWire[];
  holds: HoldMarker[];
  obstacles: StudioObstacle[];
  points: StudioPoint[];

  assembly: StudioAssemblyItem[];

  selectedPathId: string | null;
  selectedAssemblyId: string | null;
  wireDrag: WireDragState;

  validationReport: null;
  validationBusy: boolean;
  saveBusy: boolean;
  missionName: string;

  welcomeDismissed: boolean;

  setModelUrl: (url: string | null) => void;
  setReferenceObjectPath: (path: string | null) => void;
  setModelBoundingBox: (bb: StudioState['modelBoundingBox']) => void;
  setActiveTool: (tool: StudioTool) => void;
  setPathEditMode: (mode: 'translate' | 'rotate' | 'edit') => void;
  setEditMode: (mode: 'stretch' | 'add' | 'delete' | 'density') => void;

  setSatelliteStart: (pos: [number, number, number]) => void;

  addPath: (axisSeed: StudioAxisSeed) => string;
  updatePath: (id: string, updates: Partial<StudioPath>) => void;
  updatePathPlane: (id: string, plane: 'planeA' | 'planeB', pose: Partial<PlanePose>) => void;
  updatePathEllipse: (id: string, ellipse: Partial<EllipseShape>) => void;
  removePath: (id: string) => void;
  selectPath: (id: string | null) => void;
  setSelectedHandle: (pathId: string, handleId: StudioPath['selectedHandleId']) => void;
  setWaypointsFromBackend: (pathId: string, waypoints: [number, number, number][]) => void;
  setPathWaypointsManual: (pathId: string, waypoints: [number, number, number][]) => void;

  addWire: (wire: TransferWire) => void;
  removeWire: (id: string) => void;
  setWireWaypoints: (id: string, waypoints: [number, number, number][]) => void;
  setWireConstraintMode: (id: string, mode: 'constrained' | 'free') => void;

  addHold: (hold: HoldMarker) => void;
  updateHold: (id: string, updates: Partial<Pick<HoldMarker, 'duration'>>) => void;
  removeHold: (id: string) => void;

  addObstacle: () => void;
  updateObstacle: (id: string, updates: Partial<Pick<StudioObstacle, 'position' | 'radius'>>) => void;
  removeObstacle: (id: string) => void;

  addPoint: () => void;
  updatePoint: (id: string, updates: Partial<Pick<StudioPoint, 'position'>>) => void;
  removePoint: (id: string) => void;

  setWireDrag: (state: WireDragState) => void;
  setSelectedAssemblyId: (id: string | null) => void;

  setValidationReport: (report: null) => void;
  setValidationBusy: (busy: boolean) => void;
  setSaveBusy: (busy: boolean) => void;
  setMissionName: (name: string) => void;
  setWelcomeDismissed: (dismissed: boolean) => void;
}

let obstacleCounter = 0;
let pointCounter = 0;
let assemblyCounter = 0;

const PATH_COLORS = ['#22d3ee', '#a78bfa', '#fb923c', '#4ade80', '#f472b6', '#facc15'];

function defaultPlanes(axisSeed: StudioAxisSeed): { planeA: PlanePose; planeB: PlanePose } {
  const zero: [number, number, number] = [0, 0, 0];
  // Plane geometry normal is +Z. Orient plane A so +Z points along selected axis,
  // and plane B 180deg opposite so they face each other by default.
  const qAByAxis: Record<StudioAxisSeed, [number, number, number, number]> = {
    Z: [1, 0, 0, 0],
    X: [Math.SQRT1_2, 0, Math.SQRT1_2, 0], // +90deg about Y: +Z -> +X
    Y: [Math.SQRT1_2, -Math.SQRT1_2, 0, 0], // -90deg about X: +Z -> +Y
  };
  const qA = qAByAxis[axisSeed];
  const qFlipLocalY: [number, number, number, number] = [0, 0, 1, 0]; // 180deg about local Y
  const mul = (
    qa: [number, number, number, number],
    qb: [number, number, number, number]
  ): [number, number, number, number] => {
    const [aw, ax, ay, az] = qa;
    const [bw, bx, by, bz] = qb;
    return [
      aw * bw - ax * bx - ay * by - az * bz,
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
    ];
  };
  const qB = mul(qA, qFlipLocalY);
  if (axisSeed === 'X') {
    return {
      planeA: { position: [-5, 0, 0], orientation: qA },
      planeB: { position: [5, 0, 0], orientation: qB },
    };
  }
  if (axisSeed === 'Y') {
    return {
      planeA: { position: [0, -5, 0], orientation: qA },
      planeB: { position: [0, 5, 0], orientation: qB },
    };
  }
  return {
    planeA: { position: [zero[0], zero[1], -5], orientation: qA },
    planeB: { position: [zero[0], zero[1], 5], orientation: qB },
  };
}

export const useStudioStore = create<StudioState>((set, get) => ({
  modelUrl: null,
  referenceObjectPath: null,
  modelBoundingBox: null,

  activeTool: null,
  pathEditMode: 'translate',
  editMode: 'stretch',

  satelliteStart: [0, 0, 20],
  paths: [],
  wires: [],
  holds: [],
  obstacles: [],
  points: [],

  assembly: [],

  selectedPathId: null,
  selectedAssemblyId: null,
  wireDrag: { phase: 'idle' },

  validationReport: null,
  validationBusy: false,
  saveBusy: false,
  missionName: '',

  welcomeDismissed: false,

  setModelUrl: (url) => set({ modelUrl: url }),
  setReferenceObjectPath: (path) => set({ referenceObjectPath: path }),
  setModelBoundingBox: (bb) => set({ modelBoundingBox: bb }),
  setActiveTool: (tool) => set({ activeTool: tool }),
  setPathEditMode: (mode) => set({ pathEditMode: mode }),
  setEditMode: (mode) => set({ editMode: mode }),

  setSatelliteStart: (pos) => {
    set((s) => ({
      satelliteStart: pos,
      assembly: s.assembly.some((a) => a.type === 'place_satellite')
        ? s.assembly
        : [...s.assembly, { id: `asm-${++assemblyCounter}`, type: 'place_satellite' }],
    }));
  },

  addPath: (axisSeed) => {
    const id = `path-${Date.now()}`;
    const { planeA, planeB } = defaultPlanes(axisSeed);
    const color = PATH_COLORS[get().paths.length % PATH_COLORS.length];
    const itemId = `asm-${++assemblyCounter}`;
    set((s) => ({
      paths: [
        ...s.paths,
        {
          id,
          axisSeed,
          planeA,
          planeB,
          ellipse: { radiusX: 5, radiusY: 5 },
          levelSpacing: 0.5,
          waypointDensity: 1,
          densityScope: 'total',
          densitySnippetRange: null,
          isLocallyEdited: false,
          waypoints: [],
          color,
          selectedHandleId: null,
        },
      ],
      selectedPathId: id,
      assembly: [...s.assembly, { id: itemId, type: 'create_path', pathId: id }],
    }));
    return id;
  },

  updatePath: (id, updates) =>
    set((s) => ({ paths: s.paths.map((p) => (p.id === id ? { ...p, ...updates } : p)) })),

  updatePathPlane: (id, plane, pose) =>
    set((s) => ({
      paths: s.paths.map((p) => {
        if (p.id !== id) return p;
        return {
          ...p,
          isLocallyEdited: false,
          [plane]: {
            ...p[plane],
            ...pose,
          },
        };
      }),
    })),

  updatePathEllipse: (id, ellipse) =>
    set((s) => ({
      paths: s.paths.map((p) =>
        p.id === id
          ? {
              ...p,
              isLocallyEdited: false,
              ellipse: {
                radiusX: Math.max(0.1, ellipse.radiusX ?? p.ellipse.radiusX),
                radiusY: Math.max(0.1, ellipse.radiusY ?? p.ellipse.radiusY),
              },
            }
          : p
      ),
    })),

  removePath: (id) =>
    set((s) => ({
      paths: s.paths.filter((p) => p.id !== id),
      wires: s.wires.filter((w) => !w.fromNodeId.includes(`path:${id}:`) && !w.toNodeId.includes(`path:${id}:`)),
      holds: s.holds.filter((h) => h.pathId !== id),
      assembly: s.assembly.filter((a) => a.pathId !== id),
      selectedPathId: s.selectedPathId === id ? null : s.selectedPathId,
      selectedAssemblyId:
        s.selectedAssemblyId && s.assembly.some((a) => a.id === s.selectedAssemblyId && a.pathId === id)
          ? null
          : s.selectedAssemblyId,
    })),

  selectPath: (id) => set({ selectedPathId: id }),

  setSelectedHandle: (pathId, handleId) =>
    set((s) => ({
      paths: s.paths.map((p) => (p.id === pathId ? { ...p, selectedHandleId: handleId } : p)),
    })),

  setWaypointsFromBackend: (pathId, waypoints) =>
    set((s) => ({
      paths: s.paths.map((p) =>
        p.id === pathId
          ? { ...p, waypoints, isLocallyEdited: false, densitySnippetRange: null }
          : p
      ),
    })),

  setPathWaypointsManual: (pathId, waypoints) =>
    set((s) => ({
      paths: s.paths.map((p) =>
        p.id === pathId
          ? {
              ...p,
              waypoints: fairCorners(waypoints, 145, 2),
              isLocallyEdited: true,
            }
          : p
      ),
    })),

  addWire: (wire) => {
    const itemId = `asm-${++assemblyCounter}`;
    set((s) => ({
      wires: [...s.wires, { ...wire, constraintMode: wire.constraintMode ?? 'constrained' }],
      assembly: [...s.assembly, { id: itemId, type: 'connect', wireId: wire.id }],
    }));
  },

  removeWire: (id) =>
    set((s) => ({
      wires: s.wires.filter((w) => w.id !== id),
      assembly: s.assembly.filter((a) => a.wireId !== id),
      selectedAssemblyId:
        s.selectedAssemblyId && s.assembly.some((a) => a.id === s.selectedAssemblyId && a.wireId === id)
          ? null
          : s.selectedAssemblyId,
    })),

  setWireWaypoints: (id, waypoints) =>
    set((s) => ({
      wires: s.wires.map((w) =>
        w.id === id
          ? {
              ...w,
              waypoints,
            }
          : w
      ),
    })),

  setWireConstraintMode: (id, mode) =>
    set((s) => ({
      wires: s.wires.map((w) =>
        w.id === id
          ? {
              ...w,
              constraintMode: mode,
            }
          : w
      ),
    })),

  addHold: (hold) => {
    const itemId = `asm-${++assemblyCounter}`;
    set((s) => ({
      holds: [...s.holds, hold],
      assembly: [...s.assembly, { id: itemId, type: 'hold', holdId: hold.id }],
    }));
  },

  updateHold: (id, updates) =>
    set((s) => ({
      holds: s.holds.map((h) => (h.id === id ? { ...h, ...updates } : h)),
    })),

  removeHold: (id) =>
    set((s) => ({
      holds: s.holds.filter((h) => h.id !== id),
      assembly: s.assembly.filter((a) => a.holdId !== id),
      selectedAssemblyId:
        s.selectedAssemblyId && s.assembly.some((a) => a.id === s.selectedAssemblyId && a.holdId === id)
          ? null
          : s.selectedAssemblyId,
    })),

  addObstacle: () => {
    const id = `obs-${++obstacleCounter}`;
    const itemId = `asm-${++assemblyCounter}`;
    set((s) => ({
      obstacles: [...s.obstacles, { id, position: [0, 0, 0], radius: 2 }],
      assembly: [...s.assembly, { id: itemId, type: 'obstacle', obstacleId: id }],
    }));
  },

  updateObstacle: (id, updates) =>
    set((s) => ({
      obstacles: s.obstacles.map((o) => (o.id === id ? { ...o, ...updates } : o)),
    })),

  removeObstacle: (id) =>
    set((s) => ({
      obstacles: s.obstacles.filter((o) => o.id !== id),
      assembly: s.assembly.filter((a) => a.obstacleId !== id),
      selectedAssemblyId:
        s.selectedAssemblyId && s.assembly.some((a) => a.id === s.selectedAssemblyId && a.obstacleId === id)
          ? null
          : s.selectedAssemblyId,
    })),

  addPoint: () => {
    const id = `pt-${++pointCounter}`;
    const itemId = `asm-${++assemblyCounter}`;
    set((s) => ({
      points: [...s.points, { id, position: [0, 0, 0] }],
      assembly: [...s.assembly, { id: itemId, type: 'point', pointId: id }],
    }));
  },

  updatePoint: (id, updates) =>
    set((s) => ({
      points: s.points.map((p) => (p.id === id ? { ...p, ...updates } : p)),
    })),

  removePoint: (id) =>
    set((s) => ({
      points: s.points.filter((p) => p.id !== id),
      wires: s.wires.filter((w) => w.fromNodeId !== `point:${id}` && w.toNodeId !== `point:${id}`),
      assembly: s.assembly.filter((a) => {
        if (a.pointId === id) return false;
        if (!a.wireId) return true;
        const wire = s.wires.find((w) => w.id === a.wireId);
        if (!wire) return true;
        return wire.fromNodeId !== `point:${id}` && wire.toNodeId !== `point:${id}`;
      }),
      selectedAssemblyId:
        s.selectedAssemblyId && s.assembly.some((a) => a.id === s.selectedAssemblyId && a.pointId === id)
          ? null
          : s.selectedAssemblyId,
    })),

  setWireDrag: (state) => set({ wireDrag: state }),
  setSelectedAssemblyId: (id) => set({ selectedAssemblyId: id }),

  setValidationReport: (report) => set({ validationReport: report }),
  setValidationBusy: (busy) => set({ validationBusy: busy }),
  setSaveBusy: (busy) => set({ saveBusy: busy }),
  setMissionName: (name) => set({ missionName: name }),
  setWelcomeDismissed: (dismissed) => set({ welcomeDismissed: dismissed }),
}));

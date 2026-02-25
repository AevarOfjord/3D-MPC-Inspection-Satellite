import { create } from 'zustand';

export type StudioTool =
  | 'place_satellite'
  | 'create_path'
  | 'connect'
  | 'hold'
  | 'obstacle'
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
  waypoints: [number, number, number][];
  color: string;
  selectedHandleId: 'rx_pos' | 'rx_neg' | 'ry_pos' | 'ry_neg' | null;
}

export interface TransferWire {
  id: string;
  fromNodeId: string;
  toNodeId: string;
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

export type StudioAssemblyType =
  | 'place_satellite'
  | 'create_path'
  | 'connect'
  | 'hold'
  | 'obstacle';

export interface StudioAssemblyItem {
  id: string;
  type: StudioAssemblyType;
  pathId?: string;
  wireId?: string;
  holdId?: string;
  obstacleId?: string;
}

export type WireDragState =
  | { phase: 'idle' }
  | { phase: 'dragging'; sourceNodeId: string; cursorWorld: [number, number, number] };

export interface StudioState {
  modelUrl: string | null;
  referenceObjectPath: string | null;
  modelBoundingBox: { min: [number, number, number]; max: [number, number, number] } | null;

  activeTool: StudioTool;

  satelliteStart: [number, number, number];
  paths: StudioPath[];
  wires: TransferWire[];
  holds: HoldMarker[];
  obstacles: StudioObstacle[];

  assembly: StudioAssemblyItem[];

  selectedPathId: string | null;
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

  setSatelliteStart: (pos: [number, number, number]) => void;

  addPath: (axisSeed: StudioAxisSeed) => string;
  updatePath: (id: string, updates: Partial<StudioPath>) => void;
  updatePathPlane: (id: string, plane: 'planeA' | 'planeB', pose: Partial<PlanePose>) => void;
  updatePathEllipse: (id: string, ellipse: Partial<EllipseShape>) => void;
  removePath: (id: string) => void;
  selectPath: (id: string | null) => void;
  setSelectedHandle: (pathId: string, handleId: StudioPath['selectedHandleId']) => void;
  setWaypointsFromBackend: (pathId: string, waypoints: [number, number, number][]) => void;

  addWire: (wire: TransferWire) => void;
  removeWire: (id: string) => void;

  addHold: (hold: HoldMarker) => void;
  updateHold: (id: string, updates: Partial<Pick<HoldMarker, 'duration'>>) => void;
  removeHold: (id: string) => void;

  addObstacle: () => void;
  updateObstacle: (id: string, updates: Partial<Pick<StudioObstacle, 'position' | 'radius'>>) => void;
  removeObstacle: (id: string) => void;

  setWireDrag: (state: WireDragState) => void;

  setValidationReport: (report: null) => void;
  setValidationBusy: (busy: boolean) => void;
  setSaveBusy: (busy: boolean) => void;
  setMissionName: (name: string) => void;
  setWelcomeDismissed: (dismissed: boolean) => void;
}

let obstacleCounter = 0;
let assemblyCounter = 0;

const PATH_COLORS = ['#22d3ee', '#a78bfa', '#fb923c', '#4ade80', '#f472b6', '#facc15'];

function defaultPlanes(axisSeed: StudioAxisSeed): { planeA: PlanePose; planeB: PlanePose } {
  const zero: [number, number, number] = [0, 0, 0];
  const q: [number, number, number, number] = [1, 0, 0, 0];
  if (axisSeed === 'X') {
    return {
      planeA: { position: [-5, 0, 0], orientation: q },
      planeB: { position: [5, 0, 0], orientation: q },
    };
  }
  if (axisSeed === 'Y') {
    return {
      planeA: { position: [0, -5, 0], orientation: q },
      planeB: { position: [0, 5, 0], orientation: q },
    };
  }
  return {
    planeA: { position: [zero[0], zero[1], -5], orientation: q },
    planeB: { position: [zero[0], zero[1], 5], orientation: q },
  };
}

export const useStudioStore = create<StudioState>((set, get) => ({
  modelUrl: null,
  referenceObjectPath: null,
  modelBoundingBox: null,

  activeTool: null,

  satelliteStart: [0, 0, 20],
  paths: [],
  wires: [],
  holds: [],
  obstacles: [],

  assembly: [],

  selectedPathId: null,
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
    })),

  selectPath: (id) => set({ selectedPathId: id }),

  setSelectedHandle: (pathId, handleId) =>
    set((s) => ({
      paths: s.paths.map((p) => (p.id === pathId ? { ...p, selectedHandleId: handleId } : p)),
    })),

  setWaypointsFromBackend: (pathId, waypoints) =>
    set((s) => ({
      paths: s.paths.map((p) => (p.id === pathId ? { ...p, waypoints } : p)),
    })),

  addWire: (wire) => {
    const itemId = `asm-${++assemblyCounter}`;
    set((s) => ({
      wires: [...s.wires, wire],
      assembly: [...s.assembly, { id: itemId, type: 'connect', wireId: wire.id }],
    }));
  },

  removeWire: (id) =>
    set((s) => ({
      wires: s.wires.filter((w) => w.id !== id),
      assembly: s.assembly.filter((a) => a.wireId !== id),
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
    })),

  setWireDrag: (state) => set({ wireDrag: state }),

  setValidationReport: (report) => set({ validationReport: report }),
  setValidationBusy: (busy) => set({ validationBusy: busy }),
  setSaveBusy: (busy) => set({ saveBusy: busy }),
  setMissionName: (name) => set({ missionName: name }),
  setWelcomeDismissed: (dismissed) => set({ welcomeDismissed: dismissed }),
}));

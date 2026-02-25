import { create } from 'zustand';

export type StudioSegmentType = 'start' | 'scan' | 'transfer' | 'hold';

export interface KeyLevel {
  id: string;
  t: number;
  radius_x: number;
  radius_y: number;
  rotation_deg: number;
  offset_x: number;
  offset_y: number;
}

export interface ScanPass {
  id: string;
  axis: 'X' | 'Y' | 'Z';
  planeAOffset: number;
  planeBOffset: number;
  crossSection: [number, number][];
  levelHeight: number;
  waypoints: [number, number, number][];
  color: string;
  keyLevels: KeyLevel[];
  selectedHandleId: string | null;
}

export interface TransferWire {
  id: string;
  fromNodeId: string;
  toNodeId: string;
}

export interface HoldMarker {
  id: string;
  scanId: string;
  waypointIndex: number;
  duration: number;
}

export interface StudioSegment {
  id: string;
  type: StudioSegmentType;
  scanId?: string;
  wireId?: string;
  holdId?: string;
}

export type WireDragState =
  | { phase: 'idle' }
  | { phase: 'dragging'; sourceNodeId: string; cursorWorld: [number, number, number] }
  | { phase: 'connected'; sourceNodeId: string; targetNodeId: string };

export interface StudioState {
  // Model
  modelUrl: string | null;
  modelBoundingBox: { min: [number,number,number]; max: [number,number,number] } | null;

  // Scene objects
  satelliteStart: [number, number, number];
  scanPasses: ScanPass[];
  wires: TransferWire[];
  holds: HoldMarker[];
  obstacles: { id: string; position: [number,number,number]; radius: number }[];

  // Assembly
  segments: StudioSegment[];

  // Interaction
  selectedScanId: string | null;
  wireDrag: WireDragState;
  nudgingScanId: string | null;
  nudgingWaypointIndex: number | null;

  // Validation
  validationReport: null;
  validationBusy: boolean;
  saveBusy: boolean;
  missionName: string;

  // Welcome
  welcomeDismissed: boolean;

  // Actions
  setModelUrl: (url: string | null) => void;
  setModelBoundingBox: (bb: StudioState['modelBoundingBox']) => void;
  setSatelliteStart: (pos: [number,number,number]) => void;
  addScanPass: (pass: ScanPass) => void;
  updateScanPass: (id: string, updates: Partial<ScanPass>) => void;
  removeScanPass: (id: string) => void;
  selectScanPass: (id: string | null) => void;
  addWire: (wire: TransferWire) => void;
  removeWire: (id: string) => void;
  addHold: (hold: HoldMarker) => void;
  removeHold: (id: string) => void;
  addObstacle: () => void;
  updateObstacle: (id: string, updates: Partial<{ position: [number,number,number]; radius: number }>) => void;
  removeObstacle: (id: string) => void;
  setWireDrag: (state: WireDragState) => void;
  setNudging: (scanId: string | null, waypointIndex: number | null) => void;
  applyNudge: (scanId: string, waypointIndex: number, delta: [number,number,number]) => void;
  appendSegment: (seg: StudioSegment) => void;
  removeSegment: (id: string) => void;
  reorderSegments: (from: number, to: number) => void;
  setValidationReport: (report: null) => void;
  setValidationBusy: (busy: boolean) => void;
  setSaveBusy: (busy: boolean) => void;
  setMissionName: (name: string) => void;
  setWelcomeDismissed: (dismissed: boolean) => void;
  updateKeyLevelHandle: (scanId: string, handleId: 'rx_pos' | 'rx_neg' | 'ry_pos' | 'ry_neg', worldPos: [number, number, number]) => void;
  setSelectedHandle: (scanId: string, handleId: string | null) => void;
  setWaypointsFromBackend: (scanId: string, waypoints: [number, number, number][]) => void;
}

let _obstacleCounter = 0;
let _segmentCounter = 0;

export const useStudioStore = create<StudioState>((set, get) => ({
  modelUrl: null,
  modelBoundingBox: null,
  satelliteStart: [0, 0, 20],
  scanPasses: [],
  wires: [],
  holds: [],
  obstacles: [],
  segments: [],
  selectedScanId: null,
  wireDrag: { phase: 'idle' },
  nudgingScanId: null,
  nudgingWaypointIndex: null,
  validationReport: null,
  validationBusy: false,
  saveBusy: false,
  missionName: '',
  welcomeDismissed: false,

  setModelUrl: (url) => set({ modelUrl: url }),
  setModelBoundingBox: (bb) => set({ modelBoundingBox: bb }),
  setSatelliteStart: (pos) => set({ satelliteStart: pos }),

  addScanPass: (pass) => {
    const PASS_COLORS = ['#22d3ee','#a78bfa','#fb923c','#4ade80','#f472b6','#facc15'];
    const { scanPasses } = get();
    const color = PASS_COLORS[scanPasses.length % PASS_COLORS.length];
    const defaultKeyLevel: KeyLevel = {
      id: `kl-${Date.now()}`,
      t: 0.5,
      radius_x: 5,
      radius_y: 5,
      rotation_deg: 0,
      offset_x: 0,
      offset_y: 0,
    };
    const passWithColor = { ...pass, color, keyLevels: pass.keyLevels ?? [defaultKeyLevel], selectedHandleId: null };
    const segId = `seg-${++_segmentCounter}`;
    set((s) => ({
      scanPasses: [...s.scanPasses, passWithColor],
      selectedScanId: pass.id,
      segments: [...s.segments, { id: segId, type: 'scan', scanId: pass.id }],
    }));
  },

  updateScanPass: (id, updates) =>
    set((s) => ({
      scanPasses: s.scanPasses.map((p) => (p.id === id ? { ...p, ...updates } : p)),
    })),

  removeScanPass: (id) =>
    set((s) => ({
      scanPasses: s.scanPasses.filter((p) => p.id !== id),
      wires: s.wires.filter((w) => !w.fromNodeId.startsWith(id) && !w.toNodeId.startsWith(id)),
      holds: s.holds.filter((h) => h.scanId !== id),
      segments: s.segments.filter((seg) => seg.scanId !== id),
      selectedScanId: s.selectedScanId === id ? null : s.selectedScanId,
    })),

  selectScanPass: (id) => set({ selectedScanId: id }),

  addWire: (wire) => {
    const segId = `seg-${++_segmentCounter}`;
    set((s) => ({
      wires: [...s.wires, wire],
      segments: [...s.segments, { id: segId, type: 'transfer', wireId: wire.id }],
    }));
  },

  removeWire: (id) =>
    set((s) => ({
      wires: s.wires.filter((w) => w.id !== id),
      segments: s.segments.filter((seg) => seg.wireId !== id),
    })),

  addHold: (hold) => {
    const segId = `seg-${++_segmentCounter}`;
    set((s) => ({
      holds: [...s.holds, hold],
      segments: [...s.segments, { id: segId, type: 'hold', holdId: hold.id }],
    }));
  },

  removeHold: (id) =>
    set((s) => ({
      holds: s.holds.filter((h) => h.id !== id),
      segments: s.segments.filter((seg) => seg.holdId !== id),
    })),

  addObstacle: () =>
    set((s) => ({
      obstacles: [
        ...s.obstacles,
        { id: `obs-${++_obstacleCounter}`, position: [0, 0, 0] as [number,number,number], radius: 2 },
      ],
    })),

  updateObstacle: (id, updates) =>
    set((s) => ({
      obstacles: s.obstacles.map((o) => (o.id === id ? { ...o, ...updates } : o)),
    })),

  removeObstacle: (id) =>
    set((s) => ({ obstacles: s.obstacles.filter((o) => o.id !== id) })),

  setWireDrag: (state) => set({ wireDrag: state }),
  setNudging: (scanId, waypointIndex) => set({ nudgingScanId: scanId, nudgingWaypointIndex: waypointIndex }),

  applyNudge: (scanId, waypointIndex, delta) => {
    const SIGMA = 3;
    set((s) => ({
      scanPasses: s.scanPasses.map((p) => {
        if (p.id !== scanId) return p;
        const waypoints = p.waypoints.map((wp, i) => {
          const d = Math.abs(i - waypointIndex);
          const weight = Math.exp(-(d * d) / (2 * SIGMA * SIGMA));
          return [
            wp[0] + delta[0] * weight,
            wp[1] + delta[1] * weight,
            wp[2] + delta[2] * weight,
          ] as [number, number, number];
        });
        return { ...p, waypoints };
      }),
    }));
  },

  appendSegment: (seg) => set((s) => ({ segments: [...s.segments, seg] })),
  removeSegment: (id) => set((s) => ({ segments: s.segments.filter((seg) => seg.id !== id) })),

  reorderSegments: (from, to) =>
    set((s) => {
      const segs = [...s.segments];
      const [moved] = segs.splice(from, 1);
      segs.splice(to, 0, moved);
      return { segments: segs };
    }),

  setValidationReport: (report) => set({ validationReport: report }),
  setValidationBusy: (busy) => set({ validationBusy: busy }),
  setSaveBusy: (busy) => set({ saveBusy: busy }),
  setMissionName: (name) => set({ missionName: name }),
  setWelcomeDismissed: (dismissed) => set({ welcomeDismissed: dismissed }),

  updateKeyLevelHandle: (scanId, handleId, worldPos) => {
    set((s) => ({
      scanPasses: s.scanPasses.map((p) => {
        if (p.id !== scanId) return p;
        const kl = p.keyLevels[0];
        if (!kl) return p;

        // Compute frame axes for this pass axis
        const AXIS_FRAMES: Record<string, { normal: [number,number,number]; u: [number,number,number]; v: [number,number,number] }> = {
          Z: { normal: [0,0,1], u: [1,0,0], v: [0,1,0] },
          X: { normal: [1,0,0], u: [0,1,0], v: [0,0,1] },
          Y: { normal: [0,1,0], u: [1,0,0], v: [0,0,1] },
        };
        const frame = AXIS_FRAMES[p.axis];
        const [nu, nv] = [frame.u, frame.v];

        const rot = (kl.rotation_deg * Math.PI) / 180;
        const major: [number,number,number] = [
          nu[0] * Math.cos(rot) + nv[0] * Math.sin(rot),
          nu[1] * Math.cos(rot) + nv[1] * Math.sin(rot),
          nu[2] * Math.cos(rot) + nv[2] * Math.sin(rot),
        ];
        const minor: [number,number,number] = [
          -nu[0] * Math.sin(rot) + nv[0] * Math.cos(rot),
          -nu[1] * Math.sin(rot) + nv[1] * Math.cos(rot),
          -nu[2] * Math.sin(rot) + nv[2] * Math.cos(rot),
        ];

        // Center of the key level along the axis
        const axisSpan = p.planeBOffset - p.planeAOffset;
        const centerAlong = p.planeAOffset + axisSpan * kl.t;
        const center: [number,number,number] =
          p.axis === 'Z' ? [kl.offset_x, kl.offset_y, centerAlong] :
          p.axis === 'X' ? [centerAlong, kl.offset_x, kl.offset_y] :
                           [kl.offset_x, centerAlong, kl.offset_y];

        const rel = [worldPos[0] - center[0], worldPos[1] - center[1], worldPos[2] - center[2]];

        if (handleId === 'rx_pos' || handleId === 'rx_neg') {
          const radius = Math.max(0.1, Math.abs(rel[0]*major[0] + rel[1]*major[1] + rel[2]*major[2]));
          return { ...p, keyLevels: [{ ...kl, radius_x: radius }] };
        } else {
          const radius = Math.max(0.1, Math.abs(rel[0]*minor[0] + rel[1]*minor[1] + rel[2]*minor[2]));
          return { ...p, keyLevels: [{ ...kl, radius_y: radius }] };
        }
      }),
    }));
  },

  setSelectedHandle: (scanId, handleId) =>
    set((s) => ({
      scanPasses: s.scanPasses.map((p) =>
        p.id === scanId ? { ...p, selectedHandleId: handleId } : p
      ),
    })),

  setWaypointsFromBackend: (scanId, waypoints) =>
    set((s) => ({
      scanPasses: s.scanPasses.map((p) =>
        p.id === scanId ? { ...p, waypoints } : p
      ),
    })),
}));

import { useEffect, useMemo, useRef, type Dispatch, type SetStateAction } from 'react';
import * as THREE from 'three';

import { pathAssetsApi } from '../api/pathAssets';
import { scanProjectsApi } from '../api/scanProjects';
import type {
  BodyAxis,
  EndpointKind,
  ScanCompileResponse,
  ScanConnector,
  ScanDefinition,
  ScanProject,
} from '../types/scanProject';
import {
  createDefaultScan,
  createDefaultScanProject,
  makeId,
  validateScanProject,
} from '../utils/scanProjectValidation';
import { useToast } from '../feedback/feedbackContext';

export type SelectedProjectPlaneHandle = { scanId: string; handle: 'a' | 'b' } | null;
export type SelectedScanCenterHandle = { scanId: string } | null;
export type SelectedConnectorControl =
  | { connectorId: string; control: 'control1' | 'control2' }
  | null;
export type ConnectEndpoint = { scanId: string; endpoint: 'start' | 'end' } | null;
export type SelectedKeyLevelHandle =
  | {
      scanId: string;
      keyLevelId: string;
      handle: 'center' | 'rx_pos' | 'rx_neg' | 'ry_pos' | 'ry_neg';
    }
  | null;

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

interface UseScanProjectEditorArgs {
  scanProject: ScanProject;
  setScanProject: Dispatch<SetStateAction<ScanProject>>;
  configObjPath: string;
  referencePosition: [number, number, number];
  referenceAngle: [number, number, number];
  selectedScanId: string | null;
  setSelectedScanId: Dispatch<SetStateAction<string | null>>;
  selectedKeyLevelId: string | null;
  setSelectedKeyLevelId: Dispatch<SetStateAction<string | null>>;
  selectedConnectorId: string | null;
  setSelectedConnectorId: Dispatch<SetStateAction<string | null>>;
  setSelectedProjectScanPlaneHandle: Dispatch<SetStateAction<SelectedProjectPlaneHandle>>;
  selectedScanCenterHandle: SelectedScanCenterHandle;
  setSelectedScanCenterHandle: Dispatch<SetStateAction<SelectedScanCenterHandle>>;
  selectedKeyLevelHandle: SelectedKeyLevelHandle;
  setSelectedKeyLevelHandle: Dispatch<SetStateAction<SelectedKeyLevelHandle>>;
  selectedConnectorControl: SelectedConnectorControl;
  setSelectedConnectorControl: Dispatch<SetStateAction<SelectedConnectorControl>>;
  connectMode: boolean;
  setConnectMode: Dispatch<SetStateAction<boolean>>;
  connectSourceEndpoint: ConnectEndpoint;
  setConnectSourceEndpoint: Dispatch<SetStateAction<ConnectEndpoint>>;
  compilePreviewState: ScanCompileResponse | null;
  setCompilePreviewState: Dispatch<SetStateAction<ScanCompileResponse | null>>;
  setCompilePending: Dispatch<SetStateAction<boolean>>;
  scanProjectAutoPreviewEnabled: boolean;
  setScanProjectAutoPreviewEnabled: Dispatch<SetStateAction<boolean>>;
  setCenterDragActive: Dispatch<SetStateAction<boolean>>;
  setManualPath: (path: [number, number, number][]) => void;
  setLoading: Dispatch<SetStateAction<boolean>>;
  setStats: Dispatch<
    SetStateAction<{ duration: number; length: number; points: number } | null>
  >;
  refreshPathAssets: () => Promise<unknown>;
  refreshScanProjects: () => Promise<unknown>;
  selectModelPath: (path: string) => void;
}

export function useScanProjectEditor({
  scanProject,
  setScanProject,
  configObjPath,
  referencePosition,
  referenceAngle,
  selectedScanId,
  setSelectedScanId,
  selectedKeyLevelId,
  setSelectedKeyLevelId,
  selectedConnectorId,
  setSelectedConnectorId,
  setSelectedProjectScanPlaneHandle,
  selectedScanCenterHandle,
  setSelectedScanCenterHandle,
  selectedKeyLevelHandle,
  setSelectedKeyLevelHandle,
  selectedConnectorControl,
  setSelectedConnectorControl,
  connectMode,
  setConnectMode,
  connectSourceEndpoint,
  setConnectSourceEndpoint,
  compilePreviewState,
  setCompilePreviewState,
  setCompilePending,
  scanProjectAutoPreviewEnabled,
  setScanProjectAutoPreviewEnabled,
  setCenterDragActive,
  setManualPath,
  setLoading,
  setStats,
  refreshPathAssets,
  refreshScanProjects,
  selectModelPath,
}: UseScanProjectEditorArgs) {
  const { showToast } = useToast();
  const compileDebounceRef = useRef<number | null>(null);
  const lastAutoPreviewSignatureRef = useRef<string | null>(null);
  const centerDragActiveRef = useRef<boolean>(false);
  const centerDragScanIdRef = useRef<string | null>(null);
  const centerDragStartPlaneARef = useRef<[number, number, number] | null>(null);
  const centerDragStartPlaneBRef = useRef<[number, number, number] | null>(null);
  const centerDragStartMidpointRef = useRef<[number, number, number] | null>(null);

  const updateScanProject = (updater: (prev: ScanProject) => ScanProject) => {
    setScanProject((prev) => {
      const next = updater(prev);
      return {
        ...next,
        scans: (next.scans ?? []).map((scan) => ({
          ...scan,
          key_levels: (scan.key_levels ?? []).map((level) => ({
            ...level,
            center_offset: [0, 0] as [number, number],
          })),
        })),
        obj_path: next.obj_path || configObjPath,
      };
    });
  };

  const createDefaultScanProjectState = (objPath?: string) => {
    const created = createDefaultScanProject(objPath ?? scanProject.obj_path ?? configObjPath ?? '');
    setScanProject(created);
    setScanProjectAutoPreviewEnabled(false);
    lastAutoPreviewSignatureRef.current = null;
    setSelectedScanId(created.scans[0]?.id ?? null);
    setSelectedKeyLevelId(created.scans[0]?.key_levels?.[0]?.id ?? null);
    setSelectedConnectorId(null);
    setSelectedProjectScanPlaneHandle(null);
    setSelectedScanCenterHandle(null);
    setSelectedKeyLevelHandle(null);
    setSelectedConnectorControl(null);
    setConnectSourceEndpoint(null);
    setConnectMode(false);
    setCompilePreviewState(null);
    return created;
  };

  const scanProjectCompileSignature = useMemo(
    () => JSON.stringify(scanProject),
    [scanProject]
  );

  const addScan = () => {
    updateScanProject((prev) => {
      const nextScan = createDefaultScan(prev.scans.length + 1, 'Z');
      const next = { ...prev, scans: [...prev.scans, nextScan] };
      setSelectedScanId(nextScan.id);
      setSelectedKeyLevelId(nextScan.key_levels[0]?.id ?? null);
      return next;
    });
  };

  const removeScan = (scanId: string) => {
    updateScanProject((prev) => {
      if (prev.scans.length <= 1) return prev;
      const nextScans = prev.scans.filter((scan) => scan.id !== scanId);
      const nextConnectors = prev.connectors.filter(
        (conn) => conn.from_scan_id !== scanId && conn.to_scan_id !== scanId
      );
      const next: ScanProject = {
        ...prev,
        scans: nextScans,
        connectors: nextConnectors,
      };
      if (selectedScanId === scanId) {
        setSelectedScanId(nextScans[0]?.id ?? null);
        setSelectedKeyLevelId(nextScans[0]?.key_levels?.[0]?.id ?? null);
      }
      if (selectedConnectorId && !nextConnectors.some((c) => c.id === selectedConnectorId)) {
        setSelectedConnectorId(null);
      }
      if (
        selectedScanCenterHandle &&
        (selectedScanCenterHandle.scanId === scanId ||
          !nextScans.some((scan) => scan.id === selectedScanCenterHandle.scanId))
      ) {
        setSelectedScanCenterHandle(null);
      }
      if (
        selectedKeyLevelHandle &&
        (selectedKeyLevelHandle.scanId === scanId ||
          !nextScans.some((scan) => scan.id === selectedKeyLevelHandle.scanId))
      ) {
        setSelectedKeyLevelHandle(null);
      }
      return next;
    });
  };

  const updateScan = (scanId: string, patch: Partial<ScanDefinition>) => {
    updateScanProject((prev) => ({
      ...prev,
      scans: prev.scans.map((scan) =>
        scan.id === scanId ? { ...scan, ...patch } : scan
      ),
    }));
  };

  const addKeyLevel = (scanId: string, t?: number) => {
    updateScanProject((prev) => {
      const nextScans = prev.scans.map((scan) => {
        if (scan.id !== scanId) return scan;
        const sorted = [...scan.key_levels].sort((a, b) => a.t - b.t);
        let insertT = Number.isFinite(t) ? Number(t) : 0.5;
        if (!Number.isFinite(insertT)) insertT = 0.5;
        insertT = Math.max(0, Math.min(1, insertT));
        if (sorted.length >= 2 && t === undefined) {
          let bestGap = -1;
          let bestT = 0.5;
          for (let i = 1; i < sorted.length; i++) {
            const gap = sorted[i].t - sorted[i - 1].t;
            if (gap > bestGap) {
              bestGap = gap;
              bestT = (sorted[i].t + sorted[i - 1].t) * 0.5;
            }
          }
          insertT = bestT;
        }
        const level = {
          id: makeId('kl'),
          t: insertT,
          center_offset: [0, 0] as [number, number],
          radius_x: 1,
          radius_y: 1,
          rotation_deg: 0,
        };
        setSelectedKeyLevelId(level.id);
        return {
          ...scan,
          key_levels: [...scan.key_levels, level].sort((a, b) => a.t - b.t),
        };
      });
      return { ...prev, scans: nextScans };
    });
  };

  const updateKeyLevel = (
    scanId: string,
    keyLevelId: string,
    patch: Partial<ScanDefinition['key_levels'][number]>
  ) => {
    const { center_offset: _ignoredCenterOffset, ...patchWithoutCenterOffset } = patch;
    updateScanProject((prev) => ({
      ...prev,
      scans: prev.scans.map((scan) => {
        if (scan.id !== scanId) return scan;
        const nextLevels = scan.key_levels
          .map((level) => {
            const isTarget = level.id === keyLevelId;
            const nextLevel = isTarget
              ? {
                  ...level,
                  ...patchWithoutCenterOffset,
                  t: Math.max(
                    0,
                    Math.min(
                      1,
                      Number(patchWithoutCenterOffset.t ?? level.t)
                    )
                  ),
                }
              : level;
            return {
              ...nextLevel,
              center_offset: [0, 0] as [number, number],
            };
          })
          .sort((a, b) => a.t - b.t);
        return { ...scan, key_levels: nextLevels };
      }),
    }));
  };

  const removeKeyLevel = (scanId: string, keyLevelId: string) => {
    updateScanProject((prev) => ({
      ...prev,
      scans: prev.scans.map((scan) => {
        if (scan.id !== scanId) return scan;
        if (scan.key_levels.length <= 2) return scan;
        const nextLevels = scan.key_levels.filter((level) => level.id !== keyLevelId);
        if (selectedKeyLevelId === keyLevelId) {
          setSelectedKeyLevelId(nextLevels[0]?.id ?? null);
        }
        return { ...scan, key_levels: nextLevels };
      }),
    }));
  };

  const resolveBodyAxisVector = (axis: BodyAxis): [number, number, number] => {
    const basis: [number, number, number] =
      axis === 'X' ? [1, 0, 0] : axis === 'Y' ? [0, 1, 0] : [0, 0, 1];
    const e = new THREE.Euler(
      (referenceAngle[0] * Math.PI) / 180,
      (referenceAngle[1] * Math.PI) / 180,
      (referenceAngle[2] * Math.PI) / 180
    );
    const v = new THREE.Vector3(basis[0], basis[1], basis[2]).applyEuler(e).normalize();
    return [v.x, v.y, v.z];
  };

  const resolveScanFrameAxes = (
    axis: BodyAxis
  ): {
    normal: [number, number, number];
    uAxis: [number, number, number];
    vAxis: [number, number, number];
  } => {
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
  };

  const projectPointToAxisThrough = (
    point: [number, number, number],
    axis: [number, number, number],
    origin: [number, number, number]
  ): [number, number, number] => {
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
  };

  const setScanAxisAligned = (scanId: string, axis: BodyAxis) => {
    const axisVec = resolveBodyAxisVector(axis);
    updateScanProject((prev) => ({
      ...prev,
      scans: prev.scans.map((scan) =>
        scan.id === scanId
          ? {
              ...scan,
              axis,
              ...(() => {
                const midpoint: [number, number, number] = [
                  0.5 * (scan.plane_a[0] + scan.plane_b[0]),
                  0.5 * (scan.plane_a[1] + scan.plane_b[1]),
                  0.5 * (scan.plane_a[2] + scan.plane_b[2]),
                ];
                const halfSpan = Math.max(
                  1e-6,
                  0.5 *
                    Math.sqrt(
                      (scan.plane_b[0] - scan.plane_a[0]) ** 2 +
                        (scan.plane_b[1] - scan.plane_a[1]) ** 2 +
                        (scan.plane_b[2] - scan.plane_a[2]) ** 2
                    )
                );
                return {
                  plane_a: [
                    midpoint[0] - axisVec[0] * halfSpan,
                    midpoint[1] - axisVec[1] * halfSpan,
                    midpoint[2] - axisVec[2] * halfSpan,
                  ] as [number, number, number],
                  plane_b: [
                    midpoint[0] + axisVec[0] * halfSpan,
                    midpoint[1] + axisVec[1] * halfSpan,
                    midpoint[2] + axisVec[2] * halfSpan,
                  ] as [number, number, number],
                };
              })(),
            }
          : scan
      ),
    }));
  };

  const moveProjectScanPlaneHandle = (
    scanId: string,
    handle: 'a' | 'b',
    position: [number, number, number]
  ) => {
    const scan = scanProject.scans.find((item) => item.id === scanId);
    if (!scan) return;
    const axisVec = resolveBodyAxisVector(scan.axis);
    const anchor = handle === 'a' ? scan.plane_b : scan.plane_a;
    const constrained = projectPointToAxisThrough(position, axisVec, anchor);
    updateScanProject((prev) => ({
      ...prev,
      scans: prev.scans.map((item) => {
        if (item.id !== scanId) return item;
        if (handle === 'a') return { ...item, plane_a: constrained };
        return { ...item, plane_b: constrained };
      }),
    }));
  };

  const beginScanCenterDrag = (scanId: string) => {
    const scan = scanProject.scans.find((item) => item.id === scanId);
    if (!scan) return;
    centerDragActiveRef.current = true;
    centerDragScanIdRef.current = scanId;
    centerDragStartPlaneARef.current = [...scan.plane_a] as [number, number, number];
    centerDragStartPlaneBRef.current = [...scan.plane_b] as [number, number, number];
    centerDragStartMidpointRef.current = [
      0.5 * (scan.plane_a[0] + scan.plane_b[0]),
      0.5 * (scan.plane_a[1] + scan.plane_b[1]),
      0.5 * (scan.plane_a[2] + scan.plane_b[2]),
    ];
    setCenterDragActive(true);
  };

  const updateScanCenterDrag = (
    scanId: string,
    worldPos: [number, number, number]
  ) => {
    if (
      !centerDragActiveRef.current ||
      centerDragScanIdRef.current !== scanId ||
      !centerDragStartPlaneARef.current ||
      !centerDragStartPlaneBRef.current ||
      !centerDragStartMidpointRef.current
    ) {
      beginScanCenterDrag(scanId);
    }

    const startA = centerDragStartPlaneARef.current;
    const startB = centerDragStartPlaneBRef.current;
    const startMid = centerDragStartMidpointRef.current;
    if (!startA || !startB || !startMid) return;

    const delta: [number, number, number] = [
      worldPos[0] - startMid[0],
      worldPos[1] - startMid[1],
      worldPos[2] - startMid[2],
    ];

    const nextA: [number, number, number] = [
      startA[0] + delta[0],
      startA[1] + delta[1],
      startA[2] + delta[2],
    ];
    const nextB: [number, number, number] = [
      startB[0] + delta[0],
      startB[1] + delta[1],
      startB[2] + delta[2],
    ];

    updateScanProject((prev) => ({
      ...prev,
      scans: prev.scans.map((item) => {
        if (item.id !== scanId) return item;
        return {
          ...item,
          plane_a: nextA,
          plane_b: nextB,
          key_levels: item.key_levels.map((level) => ({
            ...level,
            center_offset: [0, 0] as [number, number],
          })),
        };
      }),
    }));
  };

  const endScanCenterDrag = () => {
    centerDragActiveRef.current = false;
    centerDragScanIdRef.current = null;
    centerDragStartPlaneARef.current = null;
    centerDragStartPlaneBRef.current = null;
    centerDragStartMidpointRef.current = null;
    setCenterDragActive(false);
  };

  const updateScanCenterPosition = (
    scanId: string,
    position: [number, number, number]
  ) => {
    updateScanCenterDrag(scanId, position);
  };

  const updateKeyLevelHandlePosition = (
    scanId: string,
    keyLevelId: string,
    handle: 'center' | 'rx_pos' | 'rx_neg' | 'ry_pos' | 'ry_neg',
    position: [number, number, number]
  ) => {
    const scan = scanProject.scans.find((item) => item.id === scanId);
    if (!scan) return;
    const keyLevel = scan.key_levels.find((item) => item.id === keyLevelId);
    if (!keyLevel) return;

    const { normal, uAxis, vAxis } = resolveScanFrameAxes(scan.axis);
    const normalVec = new THREE.Vector3(...normal);
    const aProjected = scan.plane_a;
    const bProjected = projectPointToAxisThrough(scan.plane_b, normal, scan.plane_a);
    const t = Math.max(0, Math.min(1, keyLevel.t));
    const baseCenter: [number, number, number] = [
      aProjected[0] + (bProjected[0] - aProjected[0]) * t,
      aProjected[1] + (bProjected[1] - aProjected[1]) * t,
      aProjected[2] + (bProjected[2] - aProjected[2]) * t,
    ];
    const uVec = new THREE.Vector3(...uAxis);
    const vVec = new THREE.Vector3(...vAxis);
    const center = new THREE.Vector3(
      baseCenter[0] + uAxis[0] * keyLevel.center_offset[0] + vAxis[0] * keyLevel.center_offset[1],
      baseCenter[1] + uAxis[1] * keyLevel.center_offset[0] + vAxis[1] * keyLevel.center_offset[1],
      baseCenter[2] + uAxis[2] * keyLevel.center_offset[0] + vAxis[2] * keyLevel.center_offset[1]
    );
    const pos = new THREE.Vector3(position[0], position[1], position[2]);
    const relToCenter = pos.clone().sub(center);

    const rot = (keyLevel.rotation_deg * Math.PI) / 180;
    const major = uVec.clone().multiplyScalar(Math.cos(rot)).add(vVec.clone().multiplyScalar(Math.sin(rot))).normalize();
    const minor = uVec.clone().multiplyScalar(-Math.sin(rot)).add(vVec.clone().multiplyScalar(Math.cos(rot))).normalize();

    if (handle === 'center') {
      updateScanProject((prev) => ({
        ...prev,
        scans: prev.scans.map((item) => {
          if (item.id !== scanId) return item;
          return {
            ...item,
            key_levels: item.key_levels.map((level) => ({
              ...level,
              center_offset: [0, 0] as [number, number],
            })),
          };
        }),
      }));
      return;
    }

    if (handle === 'rx_pos' || handle === 'rx_neg') {
      const projected = relToCenter.sub(normalVec.clone().multiplyScalar(relToCenter.dot(normalVec)));
      const radius = Math.max(0.01, Math.abs(projected.dot(major)));
      updateKeyLevel(scanId, keyLevelId, { radius_x: radius });
      return;
    }

    const projected = relToCenter.sub(normalVec.clone().multiplyScalar(relToCenter.dot(normalVec)));
    const radius = Math.max(0.01, Math.abs(projected.dot(minor)));
    updateKeyLevel(scanId, keyLevelId, { radius_y: radius });
  };

  const updateConnector = (connectorId: string, patch: Partial<ScanConnector>) => {
    updateScanProject((prev) => ({
      ...prev,
      connectors: prev.connectors.map((conn) =>
        conn.id === connectorId ? { ...conn, ...patch } : conn
      ),
    }));
  };

  const removeConnector = (connectorId: string) => {
    updateScanProject((prev) => ({
      ...prev,
      connectors: prev.connectors.filter((conn) => conn.id !== connectorId),
    }));
    if (selectedConnectorId === connectorId) {
      setSelectedConnectorId(null);
    }
    if (selectedConnectorControl?.connectorId === connectorId) {
      setSelectedConnectorControl(null);
    }
  };

  const createConnector = (
    source: { scanId: string; endpoint: EndpointKind },
    target: { scanId: string; endpoint: EndpointKind }
  ) => {
    if (source.scanId === target.scanId) {
      showToast({
        tone: 'error',
        title: 'Invalid Connector',
        message: 'Select endpoints from two different scans.',
      });
      return;
    }

    let control1: [number, number, number] | undefined;
    let control2: [number, number, number] | undefined;
    const sourceEndpoint = compilePreviewState?.endpoints?.[source.scanId]?.[source.endpoint];
    const targetEndpoint = compilePreviewState?.endpoints?.[target.scanId]?.[target.endpoint];
    if (sourceEndpoint && targetEndpoint) {
      const controls = buildAutoConnectorControls(sourceEndpoint, targetEndpoint);
      control1 = controls.control1;
      control2 = controls.control2;
    }

    const connector: ScanConnector = {
      id: makeId('conn'),
      from_scan_id: source.scanId,
      to_scan_id: target.scanId,
      from_endpoint: source.endpoint,
      to_endpoint: target.endpoint,
      control1: control1 ?? null,
      control2: control2 ?? null,
      samples: 24,
    };
    updateScanProject((prev) => ({
      ...prev,
      connectors: [...prev.connectors, connector],
    }));
    setSelectedConnectorId(connector.id);
  };

  const startConnectMode = () => {
    setConnectMode(true);
    setConnectSourceEndpoint(null);
  };

  const cancelConnectMode = () => {
    setConnectMode(false);
    setConnectSourceEndpoint(null);
  };

  const selectEndpointForConnect = (scanId: string, endpoint: EndpointKind) => {
    if (!connectMode) return;
    if (!connectSourceEndpoint) {
      setConnectSourceEndpoint({ scanId, endpoint });
      return;
    }
    if (connectSourceEndpoint.scanId === scanId) {
      setConnectSourceEndpoint({ scanId, endpoint });
      return;
    }
    createConnector(connectSourceEndpoint, { scanId, endpoint });
    setConnectSourceEndpoint(null);
    setConnectMode(false);
  };

  const updateConnectorControl = (
    connectorId: string,
    control: 'control1' | 'control2',
    position: [number, number, number]
  ) => {
    updateConnector(connectorId, { [control]: position } as Partial<ScanConnector>);
  };

  type CompileScanOptions = {
    silent?: boolean;
  };

  const compileScanProjectNow = async (
    quality: 'preview' | 'final' = 'preview',
    includeCollision = true,
    options: CompileScanOptions = {}
  ) => {
    const silent = Boolean(options.silent);
    const validationError = validateScanProject(scanProject);
    if (validationError) {
      if (!silent) {
        showToast({
          tone: 'error',
          title: 'Scan Validation',
          message: validationError,
        });
      }
      return null;
    }
    setLoading(true);
    try {
      const response = await scanProjectsApi.compileScanProject({
        project: scanProject,
        quality,
        include_collision: includeCollision,
        collision_threshold_m: 0.05,
      });
      setCompilePreviewState(response);
      const hasDisconnectedMultiScanPreview =
        quality === 'preview' &&
        scanProject.scans.length > 1 &&
        scanProject.connectors.length === 0;
      if (hasDisconnectedMultiScanPreview) {
        setManualPath([]);
      } else {
        const compiled = response.combined_path.map(
          (p) => [p[0], p[1], p[2]] as [number, number, number]
        );
        setManualPath(compiled);
      }
      setStats({
        duration: response.estimated_duration,
        length: response.path_length,
        points: response.points,
      });
      return response;
    } catch (err: any) {
      console.error(err);
      if (!silent) {
        showToast({
          tone: 'error',
          title: 'Compile Failed',
          message: `Scan compile failed: ${err.message || err}`,
        });
      }
      return null;
    } finally {
      setLoading(false);
    }
  };

  const compileScanProjectDebounced = (
    quality: 'preview' | 'final' = 'preview',
    includeCollision = true,
    delayMs = 250,
    options: CompileScanOptions = {}
  ) => {
    if (compileDebounceRef.current !== null) {
      window.clearTimeout(compileDebounceRef.current);
    }
    setCompilePending(true);
    compileDebounceRef.current = window.setTimeout(() => {
      compileScanProjectNow(quality, includeCollision, options).finally(() => {
        setCompilePending(false);
      });
    }, delayMs);
  };

  const previewScanProject = (delayMs = 100) => {
    setScanProjectAutoPreviewEnabled(true);
    lastAutoPreviewSignatureRef.current = scanProjectCompileSignature;
    compileScanProjectDebounced('preview', true, delayMs);
  };

  const saveScanProject = async (name?: string) => {
    const projectName = (name ?? scanProject.name).trim();
    if (!projectName) {
      showToast({
        tone: 'error',
        title: 'Missing Name',
        message: 'Enter a project name.',
      });
      return null;
    }
    const validationError = validateScanProject(scanProject);
    if (validationError) {
      showToast({
        tone: 'error',
        title: 'Scan Validation',
        message: validationError,
      });
      return null;
    }
    const payload: ScanProject = {
      ...scanProject,
      name: projectName,
      obj_path: scanProject.obj_path || configObjPath,
    };
    const saved = await scanProjectsApi.saveScanProject(payload);
    setScanProject(saved);
    await refreshScanProjects();
    return saved;
  };

  const loadScanProjectById = async (projectId: string) => {
    const loaded = await scanProjectsApi.loadScanProject(projectId);
    const normalized: ScanProject = {
      ...loaded,
      scans: (loaded.scans ?? []).map((scan) => ({
        ...scan,
        key_levels: (scan.key_levels ?? []).map((level) => ({
          ...level,
          center_offset: [0, 0] as [number, number],
        })),
      })),
    };
    setScanProject(normalized);
    setScanProjectAutoPreviewEnabled(false);
    lastAutoPreviewSignatureRef.current = null;
    setSelectedScanId(normalized.scans[0]?.id ?? null);
    setSelectedKeyLevelId(normalized.scans[0]?.key_levels?.[0]?.id ?? null);
    setSelectedConnectorId(null);
    setSelectedConnectorControl(null);
    setSelectedScanCenterHandle(null);
    setConnectMode(false);
    setConnectSourceEndpoint(null);
    if (normalized.obj_path) {
      selectModelPath(normalized.obj_path);
    }
    return normalized;
  };

  const saveBakedPathFromCompiled = async (name: string) => {
    const trimmed = name.trim();
    if (!trimmed) {
      showToast({
        tone: 'error',
        title: 'Missing Name',
        message: 'Please enter a baked path name.',
      });
      return null;
    }
    let compiled = compilePreviewState;
    if (!compiled || !compiled.combined_path?.length) {
      compiled = await compileScanProjectNow('final', true);
      if (!compiled) return null;
    }
    const payload = {
      name: trimmed,
      obj_path: scanProject.obj_path || configObjPath,
      path: compiled.combined_path,
      open: true,
      relative_to_obj: true,
    };
    const saved = await pathAssetsApi.save(payload);
    await refreshPathAssets();
    return saved;
  };

  useEffect(() => {
    if (scanProject.scans.length === 0) {
      const fallback = createDefaultScan(1, 'Z');
      setScanProject((prev) => ({ ...prev, scans: [fallback] }));
      setSelectedScanId(fallback.id);
      return;
    }
    if (!selectedScanId || !scanProject.scans.some((scan) => scan.id === selectedScanId)) {
      setSelectedScanId(scanProject.scans[0].id);
    }
  }, [scanProject.scans, selectedScanId, setScanProject, setSelectedScanId]);

  useEffect(() => {
    if (!configObjPath) return;
    setScanProject((prev) =>
      prev.obj_path
        ? prev
        : {
            ...prev,
            obj_path: configObjPath,
          }
    );
  }, [configObjPath, setScanProject]);

  useEffect(() => {
    if (!selectedKeyLevelHandle) return;
    const scan = scanProject.scans.find((item) => item.id === selectedKeyLevelHandle.scanId);
    if (!scan) {
      setSelectedKeyLevelHandle(null);
      return;
    }
    const exists = scan.key_levels.some((item) => item.id === selectedKeyLevelHandle.keyLevelId);
    if (!exists) {
      setSelectedKeyLevelHandle(null);
    }
  }, [scanProject.scans, selectedKeyLevelHandle, setSelectedKeyLevelHandle]);

  useEffect(() => {
    if (!selectedScanCenterHandle) return;
    const exists = scanProject.scans.some(
      (item) => item.id === selectedScanCenterHandle.scanId
    );
    if (!exists) {
      setSelectedScanCenterHandle(null);
    }
  }, [scanProject.scans, selectedScanCenterHandle, setSelectedScanCenterHandle]);

  useEffect(() => {
    if (!scanProjectAutoPreviewEnabled) return;
    if (!(scanProject.obj_path || configObjPath)) return;
    if (scanProjectCompileSignature === lastAutoPreviewSignatureRef.current) return;
    lastAutoPreviewSignatureRef.current = scanProjectCompileSignature;
    compileScanProjectDebounced('preview', true, 120, { silent: true });
  }, [
    scanProjectAutoPreviewEnabled,
    scanProjectCompileSignature,
    scanProject.obj_path,
    configObjPath,
  ]);

  useEffect(() => {
    setScanProject((prev) => ({
      ...prev,
      scans: prev.scans.map((scan) => {
        const axisVec = resolveBodyAxisVector(scan.axis);
        const midpoint: [number, number, number] = [
          0.5 * (scan.plane_a[0] + scan.plane_b[0]),
          0.5 * (scan.plane_a[1] + scan.plane_b[1]),
          0.5 * (scan.plane_a[2] + scan.plane_b[2]),
        ];
        const halfSpan = Math.max(
          1e-6,
          0.5 *
            Math.sqrt(
              (scan.plane_b[0] - scan.plane_a[0]) ** 2 +
                (scan.plane_b[1] - scan.plane_a[1]) ** 2 +
                (scan.plane_b[2] - scan.plane_a[2]) ** 2
            )
        );
        return {
          ...scan,
          plane_a: [
            midpoint[0] - axisVec[0] * halfSpan,
            midpoint[1] - axisVec[1] * halfSpan,
            midpoint[2] - axisVec[2] * halfSpan,
          ],
          plane_b: [
            midpoint[0] + axisVec[0] * halfSpan,
            midpoint[1] + axisVec[1] * halfSpan,
            midpoint[2] + axisVec[2] * halfSpan,
          ],
        };
      }),
    }));
  }, [
    referenceAngle[0],
    referenceAngle[1],
    referenceAngle[2],
    referencePosition[0],
    referencePosition[1],
    referencePosition[2],
    setScanProject,
  ]);

  useEffect(() => {
    return () => {
      if (compileDebounceRef.current !== null) {
        window.clearTimeout(compileDebounceRef.current);
        compileDebounceRef.current = null;
      }
      centerDragActiveRef.current = false;
      centerDragScanIdRef.current = null;
      centerDragStartPlaneARef.current = null;
      centerDragStartPlaneBRef.current = null;
      centerDragStartMidpointRef.current = null;
      lastAutoPreviewSignatureRef.current = null;
    };
  }, []);

  return {
    actions: {
      createDefaultScanProjectState,
      updateScanProject,
      addScan,
      removeScan,
      updateScan,
      addKeyLevel,
      updateKeyLevel,
      removeKeyLevel,
      setScanAxisAligned,
      moveProjectScanPlaneHandle,
      beginScanCenterDrag,
      updateScanCenterDrag,
      endScanCenterDrag,
      updateScanCenterPosition,
      updateKeyLevelHandlePosition,
      updateConnector,
      removeConnector,
      startConnectMode,
      cancelConnectMode,
      selectEndpointForConnect,
      updateConnectorControl,
      compileScanProjectNow,
      compileScanProjectDebounced,
      previewScanProject,
      saveScanProject,
      loadScanProjectById,
      saveBakedPathFromCompiled,
    },
  };
}

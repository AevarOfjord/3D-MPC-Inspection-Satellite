import { useEffect, useMemo } from 'react';

import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { Panel } from '../ui-v4/Panel';
import { FieldRow } from '../ui-v4/FieldRow';
import { InlineBanner } from '../ui-v4/InlineBanner';

interface ScanGeometryAdvancedCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

export function ScanGeometryAdvancedCardV4({ builder }: ScanGeometryAdvancedCardV4Props) {
  const { state, setters, actions } = builder;

  const selectedScan = useMemo(
    () =>
      state.scanProject.scans.find((scan) => scan.id === state.selectedScanId) ??
      state.scanProject.scans[0],
    [state.scanProject.scans, state.selectedScanId]
  );

  const selectedKeyLevel = useMemo(
    () =>
      selectedScan?.key_levels.find((level) => level.id === state.selectedKeyLevelId) ??
      selectedScan?.key_levels[0],
    [selectedScan, state.selectedKeyLevelId]
  );

  useEffect(() => {
    if (!selectedScan) return;
    if (
      !state.selectedKeyLevelId ||
      !selectedScan.key_levels.some((level) => level.id === state.selectedKeyLevelId)
    ) {
      setters.setSelectedKeyLevelId(selectedScan.key_levels[0]?.id ?? null);
    }
  }, [selectedScan?.id, selectedScan?.key_levels.length, state.selectedKeyLevelId, setters]);

  if (!selectedScan) {
    return (
      <Panel title="Advanced Geometry" subtitle="No scan selected">
        <InlineBanner tone="warning">Add a scan in the basics card first.</InlineBanner>
      </Panel>
    );
  }

  return (
    <Panel
      title="Advanced Geometry"
      subtitle="Plane handles, key levels, and connector control"
      className="space-y-0"
    >
      <div className="space-y-3">
        <details className="v4-subtle-panel p-3" open>
          <summary className="cursor-pointer text-xs font-semibold text-[color:var(--v4-text-2)]">
            Scan Planes
          </summary>
          <div className="mt-3 grid grid-cols-2 gap-2">
            <FieldRow label="Plane A X">
              <input
                className="v4-field"
                value={selectedScan.plane_a[0]}
                onChange={(event) =>
                  actions.updateScan(selectedScan.id, {
                    plane_a: [Number.parseFloat(event.target.value) || 0, selectedScan.plane_a[1], selectedScan.plane_a[2]],
                  })
                }
              />
            </FieldRow>
            <FieldRow label="Plane A Y">
              <input
                className="v4-field"
                value={selectedScan.plane_a[1]}
                onChange={(event) =>
                  actions.updateScan(selectedScan.id, {
                    plane_a: [selectedScan.plane_a[0], Number.parseFloat(event.target.value) || 0, selectedScan.plane_a[2]],
                  })
                }
              />
            </FieldRow>
            <FieldRow label="Plane B X">
              <input
                className="v4-field"
                value={selectedScan.plane_b[0]}
                onChange={(event) =>
                  actions.updateScan(selectedScan.id, {
                    plane_b: [Number.parseFloat(event.target.value) || 0, selectedScan.plane_b[1], selectedScan.plane_b[2]],
                  })
                }
              />
            </FieldRow>
            <FieldRow label="Plane B Y">
              <input
                className="v4-field"
                value={selectedScan.plane_b[1]}
                onChange={(event) =>
                  actions.updateScan(selectedScan.id, {
                    plane_b: [selectedScan.plane_b[0], Number.parseFloat(event.target.value) || 0, selectedScan.plane_b[2]],
                  })
                }
              />
            </FieldRow>
          </div>
        </details>

        <details className="v4-subtle-panel p-3" open>
          <summary className="cursor-pointer text-xs font-semibold text-[color:var(--v4-text-2)]">
            Key Levels
          </summary>
          <div className="mt-3 space-y-2">
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={() => actions.addKeyLevel(selectedScan.id)}
                className="v4-focus v4-button px-2 py-1 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
              >
                Add Level
              </button>
              <button
                type="button"
                onClick={() => selectedKeyLevel && actions.removeKeyLevel(selectedScan.id, selectedKeyLevel.id)}
                disabled={!selectedKeyLevel || selectedScan.key_levels.length <= 2}
                className="v4-focus v4-button px-2 py-1 bg-red-900/35 border-red-700 text-red-100"
              >
                Remove
              </button>
            </div>

            <select
              className="v4-field"
              value={selectedKeyLevel?.id ?? ''}
              onChange={(event) => setters.setSelectedKeyLevelId(event.target.value || null)}
            >
              {selectedScan.key_levels
                .slice()
                .sort((a, b) => a.t - b.t)
                .map((level, idx) => (
                  <option key={level.id} value={level.id}>
                    Level {idx + 1} (t={level.t.toFixed(2)})
                  </option>
                ))}
            </select>

            {selectedKeyLevel ? (
              <div className="grid grid-cols-2 gap-2">
                <FieldRow label="t">
                  <input
                    className="v4-field"
                    value={selectedKeyLevel.t}
                    onChange={(event) =>
                      actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, {
                        t: Math.min(1, Math.max(0, Number.parseFloat(event.target.value) || 0)),
                      })
                    }
                  />
                </FieldRow>
                <FieldRow label="Rotation (deg)">
                  <input
                    className="v4-field"
                    value={selectedKeyLevel.rotation_deg}
                    onChange={(event) =>
                      actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, {
                        rotation_deg: Number.parseFloat(event.target.value) || 0,
                      })
                    }
                  />
                </FieldRow>
                <FieldRow label="Radius X">
                  <input
                    className="v4-field"
                    value={selectedKeyLevel.radius_x}
                    onChange={(event) =>
                      actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, {
                        radius_x: Math.max(0.01, Number.parseFloat(event.target.value) || 0.01),
                      })
                    }
                  />
                </FieldRow>
                <FieldRow label="Radius Y">
                  <input
                    className="v4-field"
                    value={selectedKeyLevel.radius_y}
                    onChange={(event) =>
                      actions.updateKeyLevel(selectedScan.id, selectedKeyLevel.id, {
                        radius_y: Math.max(0.01, Number.parseFloat(event.target.value) || 0.01),
                      })
                    }
                  />
                </FieldRow>
              </div>
            ) : null}
          </div>
        </details>

        <details className="v4-subtle-panel p-3">
          <summary className="cursor-pointer text-xs font-semibold text-[color:var(--v4-text-2)]">
            Connectors
          </summary>
          <div className="mt-3 space-y-2">
            <button
              type="button"
              onClick={() => (state.connectMode ? actions.cancelConnectMode() : actions.startConnectMode())}
              className={`v4-focus v4-button px-2 py-1.5 ${
                state.connectMode
                  ? 'bg-cyan-900/35 border-cyan-700 text-cyan-100'
                  : 'bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]'
              }`}
            >
              {state.connectMode ? 'Cancel Connect Mode' : 'Connect By Click'}
            </button>
            <div className="text-xs text-[color:var(--v4-text-3)]">
              Connect mode lets you click scan endpoints in the viewport to create bridge segments.
            </div>
            <div className="text-xs text-[color:var(--v4-text-2)]">
              Connectors: {state.scanProject.connectors.length}
            </div>
          </div>
        </details>
      </div>
    </Panel>
  );
}

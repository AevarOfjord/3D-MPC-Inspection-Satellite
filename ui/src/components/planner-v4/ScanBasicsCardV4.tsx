import { RefreshCcw } from 'lucide-react';
import { useEffect, useMemo } from 'react';

import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { Panel } from '../ui-v4/Panel';
import { FieldRow } from '../ui-v4/FieldRow';

interface ScanBasicsCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

export function ScanBasicsCardV4({ builder }: ScanBasicsCardV4Props) {
  const { state, setters, actions } = builder;

  const selectedScan = useMemo(
    () =>
      state.scanProject.scans.find((scan) => scan.id === state.selectedScanId) ??
      state.scanProject.scans[0],
    [state.scanProject.scans, state.selectedScanId]
  );

  useEffect(() => {
    if (!selectedScan) return;
    if (state.selectedScanId !== selectedScan.id) {
      setters.setSelectedScanId(selectedScan.id);
    }
  }, [selectedScan?.id, state.selectedScanId, setters]);

  return (
    <Panel
      title="Step 3 · Scan Definition"
      subtitle="Configure model and scan basics"
      actions={
        <button
          type="button"
          onClick={() => void actions.previewScanProject(100)}
          disabled={state.loading || state.compilePending}
          className="v4-focus v4-button px-2 py-1.5 bg-cyan-900/35 border-cyan-700 text-cyan-100 flex items-center gap-1"
        >
          <RefreshCcw size={12} className={state.loading ? 'animate-spin' : ''} /> Preview
        </button>
      }
    >
      <div className="space-y-3">
        <FieldRow label="Model OBJ">
          <select
            value={state.config.obj_path}
            onChange={(event) => actions.selectModelPath(event.target.value)}
            className="v4-field"
          >
            <option value="">Select OBJ...</option>
            {state.availableModels.map((model) => (
              <option key={model.path} value={model.path}>
                {model.filename}
              </option>
            ))}
          </select>
        </FieldRow>

        <FieldRow label="Upload OBJ">
          <input
            type="file"
            accept=".obj"
            className="v4-field"
            onChange={(event) => {
              const file = event.target.files?.[0];
              if (!file) return;
              void actions.handleFileUpload(file);
            }}
          />
        </FieldRow>

        <div className="v4-subtle-panel p-3 space-y-2">
          <div className="flex items-center justify-between gap-2">
            <div className="text-xs font-semibold text-[color:var(--v4-text-2)]">Scans</div>
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={() => actions.addScan()}
                className="v4-focus v4-button px-2 py-1 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
              >
                Add
              </button>
              <button
                type="button"
                onClick={() => selectedScan && actions.removeScan(selectedScan.id)}
                disabled={!selectedScan || state.scanProject.scans.length <= 1}
                className="v4-focus v4-button px-2 py-1 bg-red-900/35 border-red-700 text-red-100"
              >
                Remove
              </button>
            </div>
          </div>

          <select
            className="v4-field"
            value={state.selectedScanId ?? ''}
            onChange={(event) => setters.setSelectedScanId(event.target.value || null)}
          >
            {state.scanProject.scans.map((scan) => (
              <option key={scan.id} value={scan.id}>
                {scan.name}
              </option>
            ))}
          </select>

          {selectedScan ? (
            <>
              <FieldRow label="Scan Name">
                <input
                  className="v4-field"
                  value={selectedScan.name}
                  onChange={(event) => actions.updateScan(selectedScan.id, { name: event.target.value })}
                />
              </FieldRow>

              <div className="grid grid-cols-2 gap-2">
                <FieldRow label="Axis">
                  <select
                    className="v4-field"
                    value={selectedScan.axis}
                    onChange={(event) =>
                      actions.setScanAxisAligned(selectedScan.id, event.target.value as 'X' | 'Y' | 'Z')
                    }
                  >
                    <option value="X">Body X</option>
                    <option value="Y">Body Y</option>
                    <option value="Z">Body Z</option>
                  </select>
                </FieldRow>
                <FieldRow label="Spacing (m)">
                  <input
                    className="v4-field"
                    value={selectedScan.level_spacing_m ?? 0.1}
                    onChange={(event) =>
                      actions.updateScan(selectedScan.id, {
                        level_spacing_m: Math.max(0.01, Number.parseFloat(event.target.value) || 0.1),
                      })
                    }
                  />
                </FieldRow>
                <FieldRow label="Densify">
                  <input
                    className="v4-field"
                    value={selectedScan.densify_multiplier}
                    onChange={(event) =>
                      actions.updateScan(selectedScan.id, {
                        densify_multiplier: Math.max(1, Number.parseInt(event.target.value, 10) || 1),
                      })
                    }
                  />
                </FieldRow>
                <FieldRow label="Speed Max">
                  <input
                    className="v4-field"
                    value={selectedScan.speed_max}
                    onChange={(event) =>
                      actions.updateScan(selectedScan.id, {
                        speed_max: Math.max(0.01, Number.parseFloat(event.target.value) || 0.1),
                      })
                    }
                  />
                </FieldRow>
              </div>
            </>
          ) : null}
        </div>
      </div>
    </Panel>
  );
}

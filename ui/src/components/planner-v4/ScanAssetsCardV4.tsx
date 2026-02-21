import { useState } from 'react';

import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { Panel } from '../ui-v4/Panel';
import { FieldRow } from '../ui-v4/FieldRow';

interface ScanAssetsCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

export function ScanAssetsCardV4({ builder }: ScanAssetsCardV4Props) {
  const { state, actions } = builder;
  const [scanProjectId, setScanProjectId] = useState('');
  const [pathAssetId, setPathAssetId] = useState('');
  const [bakedPathName, setBakedPathName] = useState('');

  return (
    <Panel title="Assets & Libraries" subtitle="Persist scan projects and reusable path assets">
      <div className="space-y-3">
        <FieldRow label="Project Name">
          <input
            className="v4-field"
            value={state.scanProject.name}
            onChange={(event) =>
              actions.updateScanProject((prev) => ({
                ...prev,
                name: event.target.value,
              }))
            }
          />
        </FieldRow>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => void actions.saveScanProject(state.scanProject.name)}
            disabled={!state.scanProject.name.trim() || !state.scanProject.obj_path}
            className="v4-focus v4-button px-2 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100"
          >
            Save Project
          </button>
          <button
            type="button"
            onClick={() => actions.createDefaultScanProjectState(state.config.obj_path)}
            className="v4-focus v4-button px-2 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            New Project
          </button>
        </div>

        <FieldRow label="Saved Projects">
          <select
            className="v4-field"
            value={scanProjectId}
            onChange={(event) => setScanProjectId(event.target.value)}
          >
            <option value="">Select project...</option>
            {state.scanProjects.map((project) => (
              <option key={project.id} value={project.id}>
                {project.name}
              </option>
            ))}
          </select>
        </FieldRow>

        <button
          type="button"
          onClick={() => scanProjectId && actions.loadScanProjectById(scanProjectId)}
          disabled={!scanProjectId}
          className="v4-focus v4-button w-full px-2 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
        >
          Load Selected Project
        </button>

        <FieldRow label="Baked Path Name">
          <input
            className="v4-field"
            value={bakedPathName}
            onChange={(event) => setBakedPathName(event.target.value)}
            placeholder="scan_asset_v4"
          />
        </FieldRow>

        <button
          type="button"
          onClick={async () => {
            const saved = await actions.saveBakedPathFromCompiled(bakedPathName);
            if (saved) setBakedPathName('');
          }}
          disabled={!bakedPathName.trim()}
          className="v4-focus v4-button w-full px-2 py-2 bg-violet-900/35 border-violet-700 text-violet-100"
        >
          Save Baked Path
        </button>

        <FieldRow label="Path Asset Library">
          <select
            className="v4-field"
            value={pathAssetId}
            onChange={(event) => setPathAssetId(event.target.value)}
          >
            <option value="">Select saved path...</option>
            {state.pathAssets.map((asset) => (
              <option key={asset.id} value={asset.id}>
                {asset.name}
              </option>
            ))}
          </select>
        </FieldRow>

        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={() => pathAssetId && actions.loadPathAsset(pathAssetId)}
            disabled={!pathAssetId}
            className="v4-focus v4-button px-2 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-2)]"
          >
            Load Path
          </button>
          <button
            type="button"
            onClick={() => pathAssetId && actions.applyPathAssetToSegment(pathAssetId)}
            disabled={!pathAssetId}
            className="v4-focus v4-button px-2 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100"
          >
            Use On Scan
          </button>
        </div>
      </div>
    </Panel>
  );
}

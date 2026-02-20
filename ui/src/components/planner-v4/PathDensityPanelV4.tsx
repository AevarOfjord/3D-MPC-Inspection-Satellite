import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import {
  formatPathDensityMultiplier,
  parsePathDensityInput,
} from '../../utils/pathDensity';
import { downsamplePath, resamplePath } from '../../utils/pathResample';
import { Panel } from '../ui-v4/Panel';
import { StatusPill } from '../ui-v4/StatusPill';

interface PathDensityPanelV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

export function PathDensityPanelV4({ builder }: PathDensityPanelV4Props) {
  const density = builder.state.scanProject.path_density_multiplier ?? 1.0;
  const [draftValue, setDraftValue] = useState(() => formatPathDensityMultiplier(density));
  const [applyBusy, setApplyBusy] = useState(false);
  const [pendingDensity, setPendingDensity] = useState<{ previous: number; next: number } | null>(
    null
  );
  const [editing, setEditing] = useState(false);
  const [dirty, setDirty] = useState(false);
  const inputRef = useRef<HTMLInputElement | null>(null);
  const compileScanProjectNowRef = useRef(builder.actions.compileScanProjectNow);
  const generateUnifiedPathRef = useRef(builder.actions.generateUnifiedPath);
  const setManualPathRef = useRef(builder.actions.setManualPath);
  const lastDensityRef = useRef(density);

  const compiledPoints = builder.state.compilePreviewState?.points ?? null;

  useEffect(() => {
    compileScanProjectNowRef.current = builder.actions.compileScanProjectNow;
    generateUnifiedPathRef.current = builder.actions.generateUnifiedPath;
    setManualPathRef.current = builder.actions.setManualPath;
  }, [
    builder.actions.compileScanProjectNow,
    builder.actions.generateUnifiedPath,
    builder.actions.setManualPath,
  ]);

  useEffect(() => {
    if (Math.abs(lastDensityRef.current - density) <= 1e-9) return;
    lastDensityRef.current = density;
    setDraftValue(formatPathDensityMultiplier(density));
    setDirty(false);
  }, [density]);

  const applyDensity = useCallback(
    async (rawValue: string) => {
      const next = parsePathDensityInput(rawValue, density);
      const changed = Math.abs(next - density) > 1e-9;
      setDraftValue(formatPathDensityMultiplier(next));
      setEditing(false);
      if (!changed) {
        setDirty(false);
        setPendingDensity(null);
        return;
      }
      setApplyBusy(true);
      setPendingDensity({ previous: density, next });
      builder.actions.setPathDensityMultiplier(next);
    },
    [builder.actions, density]
  );

  useEffect(() => {
    if (!applyBusy || !pendingDensity) return;
    if (Math.abs(density - pendingDensity.next) > 1e-9) return;
    let cancelled = false;

    const refreshPath = async () => {
      try {
        const applyToManualPath =
          builder.state.authoringStep !== 'scan_definition' &&
          builder.state.isManualMode &&
          builder.state.previewPath.length > 1;

        if (applyToManualPath) {
          const safePrev = Math.max(0.25, pendingDensity.previous);
          const ratio = pendingDensity.next / safePrev;
          const currentPath = builder.state.previewPath;
          let nextPath = [...currentPath];
          if (ratio > 1.000001) {
            nextPath = resamplePath(currentPath, ratio);
          } else if (ratio < 0.999999) {
            const targetCount = Math.max(2, Math.round(currentPath.length * ratio));
            nextPath = downsamplePath(currentPath, targetCount);
          }
          setManualPathRef.current(nextPath);
        } else if (builder.state.authoringStep === 'scan_definition') {
          await compileScanProjectNowRef.current('preview', true);
        } else {
          await generateUnifiedPathRef.current();
        }
        if (!cancelled) {
          setDirty(false);
        }
      } finally {
        if (!cancelled) {
          setApplyBusy(false);
          setPendingDensity(null);
        }
      }
    };

    void refreshPath();
    return () => {
      cancelled = true;
    };
  }, [
    applyBusy,
    pendingDensity,
    density,
    builder.state.authoringStep,
    builder.state.isManualMode,
    builder.state.previewPath,
  ]);

  const densityTone = useMemo(() => {
    if (density < 0.75) return 'warning' as const;
    if (density > 4.0) return 'info' as const;
    return 'success' as const;
  }, [density]);

  return (
    <Panel
      title="Path Density"
      subtitle="Multiplier of baseline waypoint count"
      actions={<StatusPill tone={densityTone}>{formatPathDensityMultiplier(density)}x</StatusPill>}
      className="w-[20rem]"
    >
      <div className="space-y-2.5">
        <label className="block text-[10px] uppercase tracking-[0.12em] text-[color:var(--v4-text-3)]">
          Multiplier
        </label>
        <input
          ref={inputRef}
          type="number"
          min={0.25}
          max={20}
          step={0.05}
          value={draftValue}
          onFocus={() => setEditing(true)}
          onChange={(event) => {
            setDraftValue(event.target.value);
            setDirty(true);
          }}
          onBlur={() => {
            setEditing(false);
            const raw = inputRef.current?.value ?? draftValue;
            const normalized = parsePathDensityInput(raw, density);
            setDraftValue(formatPathDensityMultiplier(normalized));
            setDirty(Math.abs(normalized - density) > 1e-9);
          }}
          onKeyDown={(event) => {
            if (event.key === 'Enter') {
              void applyDensity(event.currentTarget.value);
              event.currentTarget.blur();
            }
            if (event.key === 'Escape') {
              setEditing(false);
              setDraftValue(formatPathDensityMultiplier(density));
              setDirty(false);
              event.currentTarget.blur();
            }
          }}
          className="v4-field"
          aria-label="Path density multiplier"
        />
        <button
          type="button"
          onClick={() => void applyDensity(inputRef.current?.value ?? draftValue)}
          disabled={applyBusy || !dirty}
          className="v4-focus v4-button w-full px-3 py-2 bg-cyan-900/35 border-cyan-700 text-cyan-100 disabled:opacity-50"
        >
          {applyBusy ? 'Applying...' : 'Apply Density'}
        </button>
        <div className="text-[11px] text-[color:var(--v4-text-3)]">
          Applies new waypoint density to the current planner path.
        </div>
        <div className="v4-subtle-panel p-2.5 text-xs text-[color:var(--v4-text-2)] flex items-center justify-between">
          <span>Compiled points</span>
          <span className="font-semibold">{compiledPoints ?? 'n/a'}</span>
        </div>
      </div>
    </Panel>
  );
}

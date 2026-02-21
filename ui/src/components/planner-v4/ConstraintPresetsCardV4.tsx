import type { MissionSegment } from '../../api/unifiedMission';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { Panel } from '../ui-v4/Panel';
import { InlineBanner } from '../ui-v4/InlineBanner';

interface ConstraintPresetsCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

type ConstraintPreset = 'safe' | 'balanced' | 'aggressive';

const PRESETS: Record<ConstraintPreset, { speed_max: number; accel_max: number; angular_rate_max: number; description: string }> = {
  safe: {
    speed_max: 0.35,
    accel_max: 0.08,
    angular_rate_max: 0.06,
    description: 'Highest stability and easiest validation pass rates.',
  },
  balanced: {
    speed_max: 0.75,
    accel_max: 0.2,
    angular_rate_max: 0.14,
    description: 'Default for mixed transfer + scan missions.',
  },
  aggressive: {
    speed_max: 1.5,
    accel_max: 0.45,
    angular_rate_max: 0.3,
    description: 'Fast profile with higher risk and tighter control margins.',
  },
};

function applyConstraints(segments: MissionSegment[], preset: ConstraintPreset): MissionSegment[] {
  const profile = PRESETS[preset];
  return segments.map((segment) => ({
    ...segment,
    constraints: {
      speed_max: profile.speed_max,
      accel_max: profile.accel_max,
      angular_rate_max: profile.angular_rate_max,
    },
  }));
}

export function ConstraintPresetsCardV4({ builder }: ConstraintPresetsCardV4Props) {
  const { state, setters } = builder;

  return (
    <Panel title="Step 4 · Constraints" subtitle="Start with presets, then fine-tune per segment">
      <div className="space-y-3">
        <InlineBanner tone="info" title="Preset First">
          Apply a global profile, then adjust selected segment limits below.
        </InlineBanner>

        <div className="grid grid-cols-1 gap-2">
          {(Object.keys(PRESETS) as ConstraintPreset[]).map((preset) => (
            <button
              key={preset}
              type="button"
              onClick={() => setters.setSegments(applyConstraints(state.segments, preset))}
              className="v4-focus v4-button text-left px-3 py-2 bg-[color:var(--v4-surface-2)] text-[color:var(--v4-text-1)]"
            >
              <div className="text-[11px] uppercase tracking-[0.12em]">{preset}</div>
              <div className="text-[11px] normal-case tracking-normal text-[color:var(--v4-text-3)] mt-0.5">
                {PRESETS[preset].description}
              </div>
            </button>
          ))}
        </div>
      </div>
    </Panel>
  );
}

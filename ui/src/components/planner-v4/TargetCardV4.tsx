import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { orbitSnapshot } from '../../data/orbitSnapshot';
import { Panel } from '../ui-v4/Panel';
import { FieldRow } from '../ui-v4/FieldRow';
import { InlineBanner } from '../ui-v4/InlineBanner';

interface TargetCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
}

function updatePosition(
  current: [number, number, number],
  index: number,
  raw: string,
  setPosition: (next: [number, number, number]) => void
) {
  const parsed = Number.parseFloat(raw);
  if (!Number.isFinite(parsed)) return;
  const next = [...current] as [number, number, number];
  next[index] = parsed;
  setPosition(next);
}

export function TargetCardV4({ builder }: TargetCardV4Props) {
  const { state, setters } = builder;

  return (
    <Panel
      title="Step 1 · Target"
      subtitle="Choose start reference and initial pose"
      className="space-y-0"
    >
      <div className="space-y-3" id="coachmark-context_panel">
        <InlineBanner tone="info" title="Guidance">
          Planner uses LVLH by default. Pick the reference object and set the relative start position.
        </InlineBanner>

        <FieldRow label="Mission Name">
          <input
            value={state.missionName}
            onChange={(event) => setters.setMissionName(event.target.value)}
            className="v4-field"
            placeholder="Mission_V4"
          />
        </FieldRow>

        <FieldRow label="Epoch (UTC)">
          <input
            value={state.epoch}
            onChange={(event) => setters.setEpoch(event.target.value)}
            className="v4-field"
          />
        </FieldRow>

        <FieldRow label="Reference Frame">
          <div className="v4-subtle-panel px-3 py-2 text-xs text-[color:var(--v4-text-2)]">
            LVLH (fixed)
          </div>
        </FieldRow>

        <FieldRow label="Relative To">
          <select
            value={state.startTargetId || ''}
            onChange={(event) => setters.setStartTargetId(event.target.value || undefined)}
            className="v4-field"
          >
            <option value="">Select Object...</option>
            {orbitSnapshot.objects.map((obj) => (
              <option key={obj.id} value={obj.id}>
                {obj.name}
              </option>
            ))}
          </select>
        </FieldRow>

        <FieldRow label="Start Position (m)">
          <div className="grid grid-cols-3 gap-2">
            <input
              className="v4-field"
              value={state.startPosition[0]}
              onChange={(event) =>
                updatePosition(state.startPosition, 0, event.target.value, setters.setStartPosition)
              }
            />
            <input
              className="v4-field"
              value={state.startPosition[1]}
              onChange={(event) =>
                updatePosition(state.startPosition, 1, event.target.value, setters.setStartPosition)
              }
            />
            <input
              className="v4-field"
              value={state.startPosition[2]}
              onChange={(event) =>
                updatePosition(state.startPosition, 2, event.target.value, setters.setStartPosition)
              }
            />
          </div>
        </FieldRow>
      </div>
    </Panel>
  );
}

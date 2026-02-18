import type { MissionSegment, ScanSegment, TransferSegment } from '../../api/unifiedMission';
import { orbitSnapshot } from '../../data/orbitSnapshot';
import type { useMissionBuilder } from '../../hooks/useMissionBuilder';
import { Panel } from '../ui-v4/Panel';
import { FieldRow } from '../ui-v4/FieldRow';
import { InlineBanner } from '../ui-v4/InlineBanner';

interface SegmentDetailsCardV4Props {
  builder: ReturnType<typeof useMissionBuilder>;
  constraintsOnly?: boolean;
}

function parseNumber(raw: string, fallback: number): number {
  const parsed = Number.parseFloat(raw);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function updateConstraints(segment: MissionSegment, field: 'speed_max' | 'accel_max' | 'angular_rate_max', value: number): MissionSegment {
  return {
    ...segment,
    constraints: {
      speed_max: segment.constraints?.speed_max ?? 1,
      accel_max: segment.constraints?.accel_max ?? 1,
      angular_rate_max: segment.constraints?.angular_rate_max ?? 1,
      [field]: value,
    },
  };
}

export function SegmentDetailsCardV4({ builder, constraintsOnly = false }: SegmentDetailsCardV4Props) {
  const { state, actions } = builder;
  const index = state.selectedSegmentIndex;

  if (index === null || index < 0) {
    return (
      <Panel title="Segment Details" subtitle="Select a segment from the composer">
        <InlineBanner tone="info" title="Nothing Selected">
          Choose a segment to edit details and constraints.
        </InlineBanner>
      </Panel>
    );
  }

  const segment = state.segments[index];
  if (!segment) {
    return (
      <Panel title="Segment Details">
        <InlineBanner tone="warning" title="Missing Segment">
          The selected segment no longer exists.
        </InlineBanner>
      </Panel>
    );
  }

  const updateSegment = (next: MissionSegment) => {
    actions.updateSegment(index, next);
  };

  return (
    <Panel
      title={`Segment ${index + 1} · ${segment.type}`}
      subtitle={constraintsOnly ? 'Constraint tuning mode' : 'Edit selected segment properties'}
    >
      <div className="space-y-3">
        <FieldRow label="Title">
          <input
            className="v4-field"
            value={segment.title ?? ''}
            placeholder="Optional label"
            onChange={(event) => updateSegment({ ...segment, title: event.target.value || null })}
          />
        </FieldRow>

        <FieldRow label="Notes">
          <textarea
            className="v4-field min-h-[60px]"
            value={segment.notes ?? ''}
            placeholder="Operator notes"
            onChange={(event) => updateSegment({ ...segment, notes: event.target.value || null })}
          />
        </FieldRow>

        {!constraintsOnly && segment.type === 'transfer' ? (
          <TransferSegmentFields segment={segment} onChange={updateSegment} />
        ) : null}

        {!constraintsOnly && segment.type === 'scan' ? (
          <ScanSegmentFields segment={segment} onChange={updateSegment} pathAssets={state.pathAssets.map((asset) => asset.id)} />
        ) : null}

        {!constraintsOnly && segment.type === 'hold' ? (
          <FieldRow label="Hold Duration (s)">
            <input
              className="v4-field"
              value={segment.duration}
              onChange={(event) =>
                updateSegment({
                  ...segment,
                  duration: parseNumber(event.target.value, segment.duration),
                })
              }
            />
          </FieldRow>
        ) : null}

        <div className="v4-subtle-panel p-3 space-y-2">
          <FieldRow label="Speed Max (m/s)">
            <input
              className="v4-field"
              value={segment.constraints?.speed_max ?? 1.5}
              onChange={(event) =>
                updateSegment(
                  updateConstraints(segment, 'speed_max', parseNumber(event.target.value, segment.constraints?.speed_max ?? 1.5))
                )
              }
            />
          </FieldRow>
          <FieldRow label="Accel Max (m/s²)">
            <input
              className="v4-field"
              value={segment.constraints?.accel_max ?? 0.2}
              onChange={(event) =>
                updateSegment(
                  updateConstraints(segment, 'accel_max', parseNumber(event.target.value, segment.constraints?.accel_max ?? 0.2))
                )
              }
            />
          </FieldRow>
          <FieldRow label="Angular Rate Max (rad/s)">
            <input
              className="v4-field"
              value={segment.constraints?.angular_rate_max ?? 0.15}
              onChange={(event) =>
                updateSegment(
                  updateConstraints(
                    segment,
                    'angular_rate_max',
                    parseNumber(event.target.value, segment.constraints?.angular_rate_max ?? 0.15)
                  )
                )
              }
            />
          </FieldRow>
        </div>
      </div>
    </Panel>
  );
}

function TransferSegmentFields({
  segment,
  onChange,
}: {
  segment: TransferSegment;
  onChange: (segment: MissionSegment) => void;
}) {
  return (
    <div className="space-y-2">
      <FieldRow label="Frame">
        <div className="grid grid-cols-2 gap-2">
          {(['ECI', 'LVLH'] as const).map((frame) => (
            <button
              key={frame}
              type="button"
              onClick={() =>
                onChange({
                  ...segment,
                  end_pose: { ...segment.end_pose, frame },
                  target_id: frame === 'ECI' ? undefined : segment.target_id,
                })
              }
              className={`v4-focus v4-button px-2 py-1.5 ${
                segment.end_pose.frame === frame
                  ? 'bg-cyan-900/35 border-cyan-600 text-cyan-100'
                  : 'bg-[color:var(--v4-surface-1)] text-[color:var(--v4-text-2)]'
              }`}
            >
              {frame}
            </button>
          ))}
        </div>
      </FieldRow>

      {segment.end_pose.frame === 'LVLH' ? (
        <FieldRow label="Relative To">
          <select
            className="v4-field"
            value={segment.target_id || ''}
            onChange={(event) => onChange({ ...segment, target_id: event.target.value || undefined })}
          >
            <option value="">Start / Previous State</option>
            {orbitSnapshot.objects.map((obj) => (
              <option key={obj.id} value={obj.id}>
                {obj.name}
              </option>
            ))}
          </select>
        </FieldRow>
      ) : null}

      <FieldRow label="Target Position (m)">
        <div className="grid grid-cols-3 gap-2">
          {[0, 1, 2].map((coordIndex) => (
            <input
              key={coordIndex}
              className="v4-field"
              value={segment.end_pose.position[coordIndex]}
              onChange={(event) => {
                const next = [...segment.end_pose.position] as [number, number, number];
                next[coordIndex] = parseNumber(event.target.value, next[coordIndex]);
                onChange({
                  ...segment,
                  end_pose: { ...segment.end_pose, position: next },
                });
              }}
            />
          ))}
        </div>
      </FieldRow>
    </div>
  );
}

function ScanSegmentFields({
  segment,
  onChange,
  pathAssets,
}: {
  segment: ScanSegment;
  onChange: (segment: MissionSegment) => void;
  pathAssets: string[];
}) {
  return (
    <div className="space-y-2">
      <FieldRow label="Target Object">
        <select
          className="v4-field"
          value={segment.target_id}
          onChange={(event) => onChange({ ...segment, target_id: event.target.value })}
        >
          <option value="">Select target...</option>
          {orbitSnapshot.objects.map((obj) => (
            <option key={obj.id} value={obj.id}>
              {obj.name}
            </option>
          ))}
        </select>
      </FieldRow>

      <FieldRow label="Path Asset">
        <select
          className="v4-field"
          value={segment.path_asset ?? ''}
          onChange={(event) => onChange({ ...segment, path_asset: event.target.value || undefined })}
        >
          <option value="">Select saved path...</option>
          {pathAssets.map((asset) => (
            <option key={asset} value={asset}>
              {asset}
            </option>
          ))}
        </select>
      </FieldRow>

      <FieldRow label="Pattern">
        <select
          className="v4-field"
          value={segment.scan.pattern}
          onChange={(event) => onChange({ ...segment, scan: { ...segment.scan, pattern: event.target.value as 'spiral' | 'circles' } })}
        >
          <option value="spiral">Spiral</option>
          <option value="circles">Circles</option>
        </select>
      </FieldRow>

      <div className="grid grid-cols-2 gap-2">
        <FieldRow label="Revolutions">
          <input
            className="v4-field"
            value={segment.scan.revolutions}
            onChange={(event) =>
              onChange({
                ...segment,
                scan: {
                  ...segment.scan,
                  revolutions: parseNumber(event.target.value, segment.scan.revolutions),
                },
              })
            }
          />
        </FieldRow>
        <FieldRow label="Standoff (m)">
          <input
            className="v4-field"
            value={segment.scan.standoff}
            onChange={(event) =>
              onChange({
                ...segment,
                scan: {
                  ...segment.scan,
                  standoff: parseNumber(event.target.value, segment.scan.standoff),
                },
              })
            }
          />
        </FieldRow>
      </div>
    </div>
  );
}

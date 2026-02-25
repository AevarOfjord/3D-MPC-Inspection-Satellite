import type { StudioState } from './useStudioStore';
import type {
  UnifiedMission,
  MissionSegment,
  TransferSegment,
  ScanSegment,
  HoldSegment,
} from '../../api/unifiedMission';

function resolveEndpointPosition(
  nodeId: string,
  state: StudioState,
): [number, number, number] {
  // nodeId format: "<scanId>:start" | "<scanId>:end"
  const [scanId, endpoint] = nodeId.split(':');
  const pass = state.scanPasses.find((p) => p.id === scanId);
  if (!pass || pass.waypoints.length === 0) return [0, 0, 0];
  return endpoint === 'start'
    ? pass.waypoints[0]
    : pass.waypoints[pass.waypoints.length - 1];
}

export function compileStudioMission(state: StudioState): UnifiedMission {
  const compiledSegments: MissionSegment[] = [];

  for (const seg of state.segments) {
    if (seg.type === 'scan' && seg.scanId) {
      const pass = state.scanPasses.find((p) => p.id === seg.scanId);
      if (!pass) continue;
      const revolutions = Math.max(
        1,
        Math.round(Math.abs(pass.planeBOffset - pass.planeAOffset) / pass.levelHeight),
      );
      const scanSeg: ScanSegment = {
        segment_id: seg.id,
        type: 'scan',
        target_id: 'studio_target',
        scan: {
          frame: 'ECI',
          axis: `+${pass.axis}` as '+X' | '+Y' | '+Z',
          standoff: 10,
          overlap: 0.1,
          fov_deg: 60,
          revolutions,
          direction: 'CW',
          sensor_axis: '+Y',
          pattern: 'spiral',
          level_spacing_m: pass.levelHeight,
          key_levels: pass.keyLevels.length > 0 ? pass.keyLevels : null,
        },
      };
      compiledSegments.push(scanSeg);
    } else if (seg.type === 'transfer' && seg.wireId) {
      const wire = state.wires.find((w) => w.id === seg.wireId);
      if (!wire) continue;
      const endPosition = resolveEndpointPosition(wire.toNodeId, state);
      const transferSeg: TransferSegment = {
        segment_id: seg.id,
        type: 'transfer',
        end_pose: {
          frame: 'ECI',
          position: endPosition,
          orientation: [1, 0, 0, 0],
        },
      };
      compiledSegments.push(transferSeg);
    } else if (seg.type === 'hold' && seg.holdId) {
      const hold = state.holds.find((h) => h.id === seg.holdId);
      if (!hold) continue;
      const holdSeg: HoldSegment = {
        segment_id: seg.id,
        type: 'hold',
        duration: hold.duration,
      };
      compiledSegments.push(holdSeg);
    }
  }

  const mission: UnifiedMission = {
    schema_version: 2,
    mission_id: `studio-${Date.now()}`,
    name: state.missionName || 'Untitled Studio Mission',
    epoch: new Date().toISOString(),
    start_pose: {
      frame: 'ECI',
      position: state.satelliteStart,
      orientation: [1, 0, 0, 0],
    },
    segments: compiledSegments,
    obstacles: state.obstacles.map((o) => ({
      position: o.position,
      radius: o.radius,
    })),
    metadata: {
      version: 1,
      created_at: new Date().toISOString(),
    },
  };

  return mission;
}

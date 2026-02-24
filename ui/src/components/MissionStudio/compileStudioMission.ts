import type { StudioState } from './useStudioStore';

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export function compileStudioMission(state: StudioState): any {
  const segments: unknown[] = state.scanPasses.map((pass) => ({
    segment_id: pass.id,
    type: 'scan',
    target_id: 'studio_target',
    scan: {
      frame: 'ECI',
      axis: `+${pass.axis}`,
      standoff: 10,
      overlap: 0.1,
      fov_deg: 60,
      revolutions: Math.max(1, Math.round(Math.abs(pass.planeBOffset - pass.planeAOffset) / pass.levelHeight)),
      direction: 'CW',
      sensor_axis: '+Y',
      pattern: 'spiral',
    },
  }));

  return {
    schema_version: 2,
    mission_id: `studio-${Date.now()}`,
    name: state.missionName || 'Untitled Studio Mission',
    epoch: new Date().toISOString(),
    start_pose: {
      frame: 'ECI',
      position: state.satelliteStart,
      orientation: [1, 0, 0, 0],
    },
    segments,
    obstacles: state.obstacles,
  };
}

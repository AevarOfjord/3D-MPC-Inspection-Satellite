import { describe, expect, it } from 'vitest';

async function loadSettingsTestingApi() {
  (globalThis as { window?: unknown }).window = {
    location: {
      hostname: 'localhost',
      protocol: 'http:',
    },
  };
  const mod = await import('../../src/components/MPCSettingsView');
  return mod.MPC_SETTINGS_TESTING;
}

describe('MPC settings serialization', () => {
  it('drops removed MPC fields while normalizing backend payloads', async () => {
    const testingApi = await loadSettingsTestingApi();
    const normalized = testingApi.normalizeConfig({
      schema_version: 'app_config_v3',
      app_config: {
        mpc_core: {
          prediction_horizon: 45,
          coast_pos_tolerance: 0.2,
          coast_vel_tolerance: 0.1,
          coast_min_speed: 0.05,
          progress_taper_distance: 1.0,
          progress_slowdown_distance: 0.5,
        },
        simulation: {
          dt: 0.001,
          control_dt: 0.05,
          max_duration: 120,
        },
      },
    });

    expect(normalized).not.toBeNull();
    const mpc = normalized!.mpc as Record<string, unknown>;
    expect(mpc.coast_pos_tolerance).toBeUndefined();
    expect(mpc.coast_vel_tolerance).toBeUndefined();
    expect(mpc.coast_min_speed).toBeUndefined();
    expect(mpc.progress_taper_distance).toBeUndefined();
    expect(mpc.progress_slowdown_distance).toBeUndefined();
  });

  it('never emits removed MPC fields in v3 payload builder', async () => {
    const testingApi = await loadSettingsTestingApi();
    const normalized = testingApi.normalizeConfig({
      schema_version: 'app_config_v3',
      app_config: {
        mpc_core: {
          prediction_horizon: 45,
        },
        simulation: {
          dt: 0.001,
          control_dt: 0.05,
          max_duration: 120,
        },
      },
    });
    expect(normalized).not.toBeNull();

    const withRemoved = {
      ...normalized!,
      mpc: {
        ...normalized!.mpc,
        coast_pos_tolerance: 0.2,
        progress_taper_distance: 1.0,
      } as any,
    };

    const envelope = testingApi.buildV3Envelope(withRemoved);
    const appConfig = envelope.app_config as Record<string, unknown>;
    const mpcCore = appConfig.mpc_core as Record<string, unknown>;
    expect(mpcCore.coast_pos_tolerance).toBeUndefined();
    expect(mpcCore.progress_taper_distance).toBeUndefined();
  });
});

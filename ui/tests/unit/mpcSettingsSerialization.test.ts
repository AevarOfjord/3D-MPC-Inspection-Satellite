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
        mpc: {
          prediction_horizon: 45,
          coast_pos_tolerance: 0.2,
          coast_vel_tolerance: 0.1,
          coast_min_speed: 0.05,
          progress_taper_distance: 1.0,
          progress_slowdown_distance: 0.5,
        },
        mpc_core: {
          controller_profile: 'cpp_nonlinear_rti_osqp',
        },
        shared: {
          parameters: false,
          profile_parameter_files: {
            cpp_nonlinear_rti_osqp: 'controller/nonlinear/profile_parameters.json',
          },
        },
        mpc_profile_overrides: {
          cpp_nonlinear_rti_osqp: {
            base_overrides: {
              Q_contour: 4500,
            },
            profile_specific: {
              sqp_max_iter: 3,
            },
          },
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
    const mpcCore = normalized!.mpc_core as Record<string, unknown>;
    expect(mpc.coast_pos_tolerance).toBeUndefined();
    expect(mpc.coast_vel_tolerance).toBeUndefined();
    expect(mpc.coast_min_speed).toBeUndefined();
    expect(mpc.progress_taper_distance).toBeUndefined();
    expect(mpc.progress_slowdown_distance).toBeUndefined();
    expect(mpcCore.controller_profile).toBe('cpp_nonlinear_rti_osqp');
    expect(mpcCore.controller_backend).toBeUndefined();
    expect(normalized!.shared.parameters).toBe(false);
    expect(
      normalized!.shared.profile_parameter_files.cpp_nonlinear_rti_osqp
    ).toBe('controller/nonlinear/profile_parameters.json');
    expect(
      normalized!.mpc_profile_overrides.cpp_nonlinear_rti_osqp.base_overrides.Q_contour
    ).toBe(4500);
  });

  it('round-trips canonical v3 payloads without collapsing shared config into mpc_core', async () => {
    const testingApi = await loadSettingsTestingApi();
    const normalized = testingApi.normalizeConfig({
      schema_version: 'app_config_v3',
      app_config: {
        mpc: {
          prediction_horizon: 45,
          Q_contour: 2400,
        },
        mpc_core: {
          controller_profile: 'cpp_hybrid_rti_osqp',
        },
        shared: {
          parameters: false,
          profile_parameter_files: {
            cpp_hybrid_rti_osqp: 'controller/hybrid/profile_parameters.json',
          },
        },
        mpc_profile_overrides: {
          cpp_hybrid_rti_osqp: {
            base_overrides: {
              Q_contour: 2500,
            },
            profile_specific: {
              allow_stale_stage_reuse: false,
            },
          },
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
    const mpc = appConfig.mpc as Record<string, unknown>;
    const mpcCore = appConfig.mpc_core as Record<string, unknown>;
    const shared = appConfig.shared as Record<string, unknown>;
    const overrides = appConfig.mpc_profile_overrides as Record<string, unknown>;
    expect(mpc.coast_pos_tolerance).toBeUndefined();
    expect(mpc.progress_taper_distance).toBeUndefined();
    expect(mpcCore.controller_profile).toBe('cpp_hybrid_rti_osqp');
    expect(mpcCore.controller_backend).toBeUndefined();
    expect(shared.parameters).toBe(false);
    expect(
      (shared.profile_parameter_files as Record<string, unknown>).cpp_hybrid_rti_osqp
    ).toBe('controller/hybrid/profile_parameters.json');
    expect(
      (
        (
          overrides.cpp_hybrid_rti_osqp as Record<string, unknown>
        ).base_overrides as Record<string, unknown>
      ).Q_contour
    ).toBe(2500);
  });
});

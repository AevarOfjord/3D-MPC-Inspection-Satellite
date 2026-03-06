# Orbital Inspector Satellite Control

A local-first satellite simulation and mission control project with:

- Python backend simulation and APIs
- Web UI for mission design and runtime control
- MPC-based control and telemetry outputs

## Requirements

- Python 3.11
- Node.js 20+
- CMake + Ninja (for C++ extension builds)

## Quick Start

```bash
make install
make ui-build
make run-app
```

Open `http://127.0.0.1:8000`.

## Core Commands

```bash
make run            # backend + frontend dev
make run-app        # backend serving prebuilt ui/dist
make test           # backend tests
make lint           # backend + frontend lint
make docs-check     # markdown link/path accuracy checks
make package-pyinstaller
make sim
```

## Shared Parameter Mode

The canonical comparison workflow uses the main config file's `app_config.mpc` block as the shared baseline for all six MPC profiles.

- Thesis fairness baseline: `scripts/configs/thesis_fairness_baseline.json`
- Fair comparison mode:
  - `shared.parameters=true`
  - all six controllers use the same `mpc` baseline
  - per-profile files and `mpc_profile_overrides` are inactive
  - non-empty per-profile deltas are rejected before controller creation
- Per-profile tuning mode:
  - `shared.parameters=false`
  - the active `mpc_core.controller_profile` may apply:
    - `shared.profile_parameter_files.<profile>`
    - `mpc_profile_overrides.<profile>.base_overrides`
    - `mpc_profile_overrides.<profile>.profile_specific`

Supported external profile delta files:

- `controller/linear/profile_parameters.json`
- `controller/hybrid/profile_parameters.json`
- `controller/nonlinear/profile_parameters.json`
- `controller/nmpc/profile_parameters.json`
- `controller/acados_rti/profile_parameters.json`
- `controller/acados_sqp/profile_parameters.json`

Precedence order:

1. `app_config.mpc` shared baseline
2. active profile delta file, if `shared.parameters=false`
3. active profile embedded deltas in `mpc_profile_overrides`, if `shared.parameters=false`

Paper workflow:

- Fair comparison: run all six profiles with `shared.parameters=true` and verify identical `shared_params_hash` values.
- Feasibility/tuning: run tuned profiles separately with `shared.parameters=false`; do not mix those runs into fairness claims.

## Repository Layout

- `src/python/` backend code
- `src/cpp/` C++ solver/runtime modules
- `ui/` frontend application
- `scripts/` operations and release scripts
- `missions/` saved mission payloads
- `data/` canonical assets and runtime simulation/dashboard data
- `tests/` backend test suite
- `ARCHITECTURE.md` system architecture notes

## Compatibility Matrix

Supported mission APIs are split between canonical v2 endpoints and retained compatibility endpoints:

- Canonical: `/api/v2/missions/*`
- Compatibility retained: `/mission_v2`, `/save_mission_v2`, `/saved_missions_v2`
- Compatibility migration helper retained: `/api/v2/missions/migrate_legacy`

Compatibility endpoints include deprecation headers and are retained to avoid breaking existing UI/test workflows.

## Contributing & Policies

- [CONTRIBUTING.md](CONTRIBUTING.md)
- [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)
- [SECURITY.md](SECURITY.md)

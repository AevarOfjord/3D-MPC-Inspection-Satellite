# Architecture

This document describes the current backend architecture after removing `src/` and consolidating runtime code under top-level `controller/`.

## 1. Repository Shape

```text
Main/
  data/
  docs/
  missions/
  scripts/
  controller/
    __init__.py
    cli.py
    exceptions.py
    factory.py
    registry.py
    py.typed

    configs/
      __init__.py
      constants.py
      defaults.py
      models.py
      paths.py
      physics.py
      reaction_wheel_config.py
      simulation_config.py
      timing.py
      validator.py

    shared/
      __init__.py
      assets/
      cpp/
        satellite_params.hpp
        sim/
          bindings_sim.cpp
          bindings_physics.cpp
          orbital_dynamics.cpp
          orbital_dynamics.hpp
          simulation_engine.cpp
          simulation_engine.hpp
      python/
        __init__.py
        benchmarks/
        control_common/
          base.py
          mpc_controller.py
          codegen/
        core/
        cpp/
        dashboard/
        mission/
        physics/
        runtime/
        simulation/
        utils/
        visualization/

    hybrid/
      __init__.py
      python/
        __init__.py
        controller.py
      cpp/
        bindings.cpp
        sqp_controller.cpp
        sqp_controller.hpp
        sqp_types.cpp
        sqp_types.hpp
      shared/

    nonlinear/
      __init__.py
      python/
        __init__.py
        controller.py
      cpp/
      shared/

    linear/
      __init__.py
      python/
        __init__.py
        controller.py
      cpp/
      shared/

  tests/
  ui/
```

## 2. Package Root and Imports

- Python backend package root is `controller`.
- Runtime imports use `controller.*` namespaces.
- There is no `src/` runtime package path.

Examples:

- `from controller.configs.models import AppConfig`
- `from controller.factory import create_controller`
- `from controller.shared.python.dashboard.app import app`

## 3. Controller Profiles and Selection

Controller profile selection is configured via:

- `AppConfig.mpc_core.controller_profile`: `"hybrid" | "nonlinear" | "linear"`

Compatibility is preserved for legacy fields:

- `mpc_core.controller_backend` (`v1`/`v2`) is mapped to profile when needed.

Routing:

- `controller.factory.create_controller(...)` selects profile implementation.
- `controller.registry.normalize_controller_profile(...)` centralizes normalization.

Current implementations:

- `hybrid` -> `controller.hybrid.python.controller.HybridMPCController`
- `nonlinear` -> `controller.nonlinear.python.controller.NonlinearMPCController`
- `linear` -> `controller.linear.python.controller.LinearMPCController`

## 4. Shared Parameter/Fairness Contract

All profiles consume the same canonical application config model:

- `controller.configs.models.AppConfig`
- `controller.configs.models.MPCCoreParams`

Default profile:

- `controller.configs.defaults.build_default_config()` sets `controller_profile = "hybrid"`.

This keeps comparisons fair by ensuring one common parameter contract and schema.

## 5. Runtime Flow

1. Config and mission are loaded through shared modules under `controller.shared.python.*`.
2. Factory resolves the effective profile and instantiates the controller.
3. Simulation/runtime loops execute through shared runtime/simulation modules.
4. Dashboard routes and runner manager expose control and telemetry APIs.
5. Artifacts and telemetry include controller metadata:
   - `controller_profile`
   - `controller_core`

## 6. C++ Build and Bindings

C++ sources are split by profile/shared concerns:

- Hybrid MPC C++: `controller/hybrid/cpp/*`
- Shared simulation C++: `controller/shared/cpp/sim/*`

`CMakeLists.txt` installs Python extensions into:

- `controller/shared/python/cpp/`

Loaded extension modules:

- `_cpp_mpc`
- `_cpp_sim`
- `_cpp_physics`

## 7. Entry Points

- CLI app: `controller/cli.py`
- Project script entrypoint: `satellite-control = "controller.cli:app"` (`pyproject.toml`)
- Dashboard ASGI app: `controller.shared.python.dashboard.app:app`

Helper scripts under `scripts/` call these `controller.*` entrypoints.

## 8. UI Integration

MPC settings UI exposes and persists controller profile selection:

- `ui/src/components/MPCSettingsView.tsx`
- `ui/src/components/mpc-settings/mpcSettingsTypes.ts`
- `ui/src/components/mpc-settings/mpcSettingsDefaults.ts`

The profile is round-tripped through runner config payloads and backend normalization.

## 9. Migration Status

- `src/` removed.
- Backend consolidated under top-level `controller/`.
- Build/test/package wiring updated for `controller` root.
- Profile architecture (`hybrid`/`nonlinear`/`linear`) is active with shared contract.

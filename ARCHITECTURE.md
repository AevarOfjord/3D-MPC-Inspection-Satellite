# Architecture

This document describes the current controller, runtime, and documentation architecture rooted under the top-level `controller/` package.

## 1. Repository Shape

```text
controller/
  factory.py
  registry.py
  configs/
  hybrid/
  nonlinear/
  linear/
  nmpc/
  acados_rti/
  acados_sqp/
  acados_shared/
  shared/
    cpp/
    python/

ui/
tests/
scripts/
docs/
MATH/
```

## 2. Controller Profiles

Controller selection is driven by `AppConfig.mpc_core.controller_profile`. Canonical profile IDs are defined in `controller/registry.py`.

| Profile ID | Python entrypoint | Solver family | Main role |
| --- | --- | --- | --- |
| `cpp_hybrid_rti_osqp` | `controller.hybrid.python.controller.HybridMPCController` | RTI-SQP + OSQP | pragmatic real-time baseline |
| `cpp_nonlinear_rti_osqp` | `controller.nonlinear.python.controller.NonlinearMPCController` | RTI/SQP + OSQP | exact stage-wise OSQP benchmark |
| `cpp_linearized_rti_osqp` | `controller.linear.python.controller.LinearMPCController` | RTI-SQP + OSQP | cheapest frozen-linearization variant |
| `cpp_nonlinear_fullnlp_ipopt` | `controller.nmpc.python.controller.NmpcController` | full NLP + IPOPT | high-fidelity nonlinear benchmark |
| `cpp_nonlinear_rti_hpipm` | `controller.acados_rti.python.controller.AcadosRtiController` | acados SQP_RTI + HPIPM | exact-model real-time nonlinear MPC |
| `cpp_nonlinear_sqp_hpipm` | `controller.acados_sqp.python.controller.AcadosSqpController` | acados SQP + HPIPM | higher-iteration nonlinear benchmark |

Legacy names such as `hybrid`, `nonlinear`, `linear`, `nmpc`, `acados_rti`, and `acados_sqp` are normalized to these canonical IDs before controller creation.

## 3. Shared Parameter and Fairness Contract

The comparison workflow is built around one shared baseline:

- `app_config.mpc` is the canonical shared parameter block.
- `shared.parameters=true` means all profiles use that same shared baseline.
- `shared.parameters=false` enables active-profile deltas from:
  - `shared.profile_parameter_files.<profile>`
  - `mpc_profile_overrides.<profile>.base_overrides`
  - `mpc_profile_overrides.<profile>.profile_specific`

The fairness contract is resolved in:

- `controller/shared/python/control_common/parameter_policy.py`
- `controller/shared/python/control_common/profile_params.py`

Every run records the active profile plus the shared and effective parameter signatures so cross-profile comparisons can be audited.

## 4. Runtime Flow

1. Config and mission data are loaded through `controller.configs.*` and `controller.shared.python.mission.*`.
2. `controller.factory.create_controller()` resolves the canonical profile and instantiates the chosen controller.
3. Shared runtime logic in `controller.shared.python.runtime.*` handles mode switching, fallback behavior, and contract checks.
4. Shared simulation logic in `controller.shared.python.simulation.*` propagates the plant and records telemetry.
5. Dashboard and CLI entrypoints expose the same underlying configuration and runtime stack.

## 5. Native Runtime Pieces

Compiled extensions are loaded through `controller/shared/python/cpp/__init__.py`.

Available modules:

- `_cpp_mpc`, `_cpp_mpc_nonlinear`, `_cpp_mpc_linear` for the OSQP-based C++ controller family
- `_cpp_mpc_runtime` for unified runtime capability detection
- `_cpp_sim` and `_cpp_physics` for plant propagation and orbital utilities

The acados and IPOPT controllers currently use Python orchestration around their own nonlinear transcriptions rather than the OSQP-family C++ QP runtime.

## 6. UI and API

The UI and backend both preserve the same controller-config shape:

- shared baseline under `mpc`
- fairness switch under `shared.parameters`
- per-profile deltas under `mpc_profile_overrides`

The main editor lives in:

- `ui/src/components/MPCSettingsView.tsx`
- `ui/src/components/mpc-settings/`

## 7. Documentation Map

Use these files as the primary references:

- `README.md` for setup and workflow
- `PHYSICS-ENGINE.md` for the runtime plant model
- `MATH/README.md` for the shared controller formulation
- `MATH/*.md` for controller-specific mathematics

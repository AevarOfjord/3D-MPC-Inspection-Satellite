# Satellite Control Architecture (V6)

> Version: 6.0.0 (current runtime)

This document describes the active V6 control/runtime architecture.

## 1. High-Level Stack

- Python runtime orchestrates mission compilation, scheduling, control loop, telemetry, and artifacts.
- C++ backend executes the MPC QP core and exposes bindings through pybind11.
- Dashboard backend (`FastAPI`) serves APIs and runner control.
- Frontend (`React`) provides planner, runner, telemetry overlays, and settings.

## 2. Runtime Control Pipeline

Per control tick:

1. `control_loop.update_mpc_control_step` gathers current state.
2. `ControllerModeManagerV6` updates mode (`TRACK/RECOVER/SETTLE/HOLD/COMPLETE`).
3. `ReferenceSchedulerV6` builds horizon reference slice.
4. `MPCRunner` calls `MPCController` (Python wrapper -> C++ core).
5. `ActuatorPolicyV6` applies deterministic shaping (hysteresis + terminal bypass behavior).
6. Control is applied to simulator and telemetry/logging artifacts are updated.

Completion supervision is evaluated in `SimulationLoop._check_path_following_completion` through `TerminalSupervisorV6`.

## 3. V6 Runtime Components

Core internal contracts in `src/python/satellite_control/core/v6_controller_runtime.py`:

- `MissionRuntimePlanV6`
- `ReferenceSliceV6`
- `ModeStateV6`
- `ModeProfileV6`
- `CompletionGateStatusV6`
- `SolverHealthV6`
- `QualityContractReportV6`

These contracts are serializable and used by diagnostics/artifact output.

## 4. MPC Core (C++)

Primary files:

- `src/cpp/mpc_controller.hpp`
- `src/cpp/mpc_controller.cpp`
- `src/cpp/bindings.cpp`

Key properties:

- 17-state linearized MPCC model.
- Explicit runtime mode profile scaling (no legacy adaptive recovery branch).
- OSQP-only backend in V6.0.
- Bounded solver fallback policy with telemetry:
  - `fallback_active`
  - `fallback_age_s`
  - `fallback_scale`

## 5. Completion Authority

Completion is governed by `TerminalSupervisorV6` only.

- All terminal thresholds must pass concurrently.
- Hold timer resets on any breach.
- Completion occurs only after continuous hold duration is satisfied.

Defaults:

- position `<= 0.10 m`
- angle `<= 2 deg`
- velocity `<= 0.05 m/s`
- angular velocity `<= 2 deg/s`
- hold `10 s`

## 6. Configuration Schema

Canonical API/runtime config envelope:

```json
{
  "schema_version": "app_config_v3",
  "app_config": {
    "physics": {},
    "reference_scheduler": {},
    "mpc_core": {},
    "actuator_policy": {},
    "controller_contracts": {},
    "simulation": {},
    "input_file_path": null
  }
}
```

Current release behavior:

- API dual-reads legacy/v1/v2 payloads for compatibility.
- API writes persist only `app_config_v3`.
- Transitional top-level mirrors (`physics`, `mpc`, `simulation`) are still emitted for one release.

## 7. Observability and Artifacts

V6 telemetry channels:

- `mode_state`
- `completion_gate`
- `solver_health`
- `path metrics`

Key artifacts per run:

- `kpi_summary.json`
- `performance_metrics.json`
- `mpc_step_stats.csv`
- `mode_timeline.csv`
- `completion_gate_trace.csv`
- `controller_health.json`
- `contract_report_v6.json` (quality harness)

## 8. Quality and Cutover Tooling

Primary scripts:

- `scripts/run_mpc_quality_suite.py`
- `scripts/check_v6_cutover_readiness.py`

CI usage:

- PR: fast quality contract.
- Scheduled/manual: full quality suite + cutover readiness evaluation.

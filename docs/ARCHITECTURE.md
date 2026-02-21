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
- Legacy taper/coast knobs are removed from canonical V6 config surface.
- Hard scan-attitude quaternion constraints are removed; attitude is enforced via cost shaping.
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
- Removed legacy MPC knobs are warn-ignored on input and reported via:
  - `config_meta.deprecations.removed_mpc_fields_seen`
  - `config_meta.deprecations.removed_mpc_fields_policy = "warn_ignore"`

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

## 9. Pointing-Stability Contract (V6)

Pointing is now an explicit runtime contract owned by V6 runtime components.

Contract goals:

- `+X` axis follows path-forward travel direction.
- `+Z` axis locks to planner pair axis when available.
- If no scan/pair axis exists, `+Z` falls back to LVLH radial (`+R`).
- `+Y/-Y` side is automatic; continuity is preferred (no forced camera side).

Runtime flow:

1. `MissionRuntimeCompilerV6` builds `pointing_path_spans` keyed by path `s`.
2. Transfer spans inherit the next scan axis context when moving toward scan work.
3. Transfers after the final scan inherit the last scan axis context.
4. `resolve_pointing_context_v6(...)` selects active context by current `path_s`.
5. `control_loop` pushes axis context into MPC each tick via `set_scan_attitude_context(...)`.
6. C++ reference-frame math enforces strict `+Z` lock and projected `+X` forward branch.

Guardrails:

- Pointing error telemetry is computed each step:
  - `z_axis_error_deg`
  - `x_axis_error_deg`
- Breach/clear hysteresis:
  - breach hold: `0.30 s`
  - clear hold: `0.80 s`
- Continuous breach marks guardrail failure and biases runtime toward recovery behavior.

## 10. Pointing Telemetry Surface

Pointing fields are now available in runtime telemetry and run artifacts:

- `pointing_context_source`
- `pointing_axis_world`
- `z_axis_error_deg`
- `x_axis_error_deg`
- `pointing_guardrail_breached`
- `object_visible_side`

These values are emitted through dashboard telemetry and persisted in controller diagnostics outputs.

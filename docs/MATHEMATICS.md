# Mathematics (V6 Controller)

This document defines the active V6 controller math used by the runtime.

## 1. State, Control, and Dynamics

V6 uses a linearized discrete-time MPCC model around the current operating point.

- State dimension: 17
- Control dimension: `n_rw + n_thrusters + 1`

State vector:

```text
x = [ p(3), q(4), v(3), w(3), s(1), rw_speed(3) ]
```text
Control vector:

```text
u = [ rw_torque(n_rw), thruster_cmd(n_thrusters), v_s(1) ]
```text
Per MPC step, the linearized dynamics are:

```text
x_{k+1} = A_k x_k + B_k u_k + c_k
```text
where `A_k, B_k, c_k` are updated from the current state and reference geometry.

## 2. MPCC Objective

For horizon `N`, V6 solves a QP with stage and terminal costs.

Stage terms include:

- contour error penalty (`Q_contour`)
- lag error penalty (`Q_lag` or `Q_lag_default`)
- progress-speed tracking (`Q_progress`)
- velocity alignment (`Q_velocity_align`)
- attitude alignment (`Q_attitude`, `Q_axis_align`)
- angular-rate damping (`q_angular_velocity`)
- thrust and RW effort (`r_thrust`, `r_rw_torque`)
- command regularization (`Q_smooth`, plus L1/pair penalties)

Terminal terms include:

- terminal position penalty (`Q_terminal_pos`)
- terminal progress penalty (`Q_terminal_s`)

General form:

```text
J = sum_{k=0..N-1} (||e_contour||^2 + ||e_lag||^2 + ||e_progress||^2 + ||e_att||^2 + ||e_w||^2 + ||u||^2 + ||Δu||^2)
    + terminal penalties
```text
(Each term is weighted by its configured scalar weight.)

## 3. Mode-Profile Scaling (V6)

V6 uses explicit runtime modes:

- `TRACK`
- `RECOVER`
- `SETTLE`
- `HOLD`
- `COMPLETE`

Mode profiles scale objective terms deterministically.

Default contract multipliers:

- `RECOVER`: contour `x2.0`, lag `x2.0`, progress `x0.6`, attitude `x0.8`
- `SETTLE`: progress `x0.0`, terminal-pos `x2.0`, terminal-attitude `x1.5`, velocity-align `x1.5`, angular-rate `x2.0`
- `HOLD`: `SETTLE` profile plus smoothness `x1.5`, thruster-pair `x1.2`

`TRACK` uses base weights (scale `1.0`).

## 4. Mode Transition Contracts

Default hysteretic transitions:

- `TRACK -> RECOVER`: contour error `>= 0.20 m` for `>= 0.5 s`
- `RECOVER -> TRACK`: contour error `<= 0.10 m` for `>= 1.0 s`
- `TRACK|RECOVER -> SETTLE`: at path end (`path_s >= path_len - position_tolerance`)
- `SETTLE -> HOLD`: all terminal thresholds true
- `HOLD -> SETTLE`: any terminal threshold false
- `HOLD -> COMPLETE`: hold elapsed `>= hold_required`

## 5. Strict Terminal Completion Gate

Terminal completion is supervised by `TerminalSupervisorV6`.

Completion requires all conditions true continuously for hold duration:

- position error `<= 0.10 m`
- angle error `<= 2 deg`
- linear velocity error `<= 0.05 m/s`
- angular velocity error `<= 2 deg/s`
- default hold duration: `10.0 s`

Gate timer resets immediately on any breach.

## 6. Bounded Solver Fallback Policy

When the QP solve is non-success, the controller applies bounded fallback scaling to last feasible control.

Defaults:

- `solver_fallback_hold_s = 0.30`
- `solver_fallback_decay_s = 0.70`
- `solver_fallback_zero_after_s = 1.00`

Let fallback age be `t_f`:

```text
scale(t_f) = 1.0                                  if t_f <= hold_s
scale(t_f) = 1 - (t_f - hold_s) / decay_s         if hold_s < t_f < hold_s + decay_s
scale(t_f) = 0.0                                  if t_f >= zero_after_s
```text
Telemetry exports:

- `fallback_active`
- `fallback_age_s`
- `fallback_scale`

## 7. Reference Speed and Duration Policy

Reference-state speed now follows MPC-owned progress semantics (no independent Python taper/coast heuristic):

- `SETTLE/HOLD/COMPLETE`: `v_ref = 0`
- Other modes: `v_ref = path_v_s` when available, fallback to configured `path_speed`
- Apply clamps with `path_speed_min/path_speed_max` when configured

Runtime path speed policy (scheduler/runtime compiler):

```text
speed_candidate = min(non-hold segment speed_max values)
speed = clamp(speed_candidate, path_speed_min, path_speed_max)
```text
Required duration estimator:

```text
required_duration = path_length / max(speed, 1e-6) + hold_duration + margin
```text
Default margin: `30 s`.

## 8. Constraints and Solver

- Hard actuator bounds remain enforced.
- Optional velocity/angular-rate state bounds remain supported.
- OSQP is the certified V6 backend.
- Solver time budget is configured and monitored per step.

## 9. Config Contract (V6)

Canonical payload schema:

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
```text
For one compatibility release, legacy/v1/v2 payloads are dual-read by API adapters; all new writes persist as `app_config_v3`.

Removed canonical MPC knobs in V6 cleanup:

- `coast_pos_tolerance`
- `coast_vel_tolerance`
- `coast_min_speed`
- `progress_taper_distance`
- `progress_slowdown_distance`

## 10. Pointing-Contract Reference Frame (V6)

V6 enforces a strict geometric pointing contract for reference attitude generation.

### 10.1 Axis Sources

- Preferred `z_ref`: planner scan/pair axis (mission `scan.axis`) when available.
- Transfer policy: use next scan axis while moving toward a scan; after final scan, keep the last scan axis.
- Fallback `z_ref`: LVLH radial (`+R`) when scan axis context is unavailable.
- `z_ref` sign is treated as fixed contract direction (no runtime sign flips).

### 10.2 Path-Forward +X Construction

Let `t` be the local path tangent and `z_ref` the locked axis.

1. Project tangent onto plane orthogonal to `z_ref`:

```text
x_proj = t - (t · z_ref) z_ref
```text
1. Degenerate case (`||x_proj|| ~ 0`):

- reuse previous projected `x_ref` for continuity if available;
- otherwise use a deterministic seeded orthogonal basis vector.

1. Normalize and enforce forward branch:

```text
x_ref = normalize(x_proj)
if x_ref · t < 0 then x_ref = -x_ref
```text
1. Complete right-handed frame:

```text
y_ref = normalize(z_ref × x_ref)
R_ref = [x_ref y_ref z_ref]
```text
Quaternion sign continuity is enforced across horizon points to avoid frame flips.

### 10.3 Pointing Error Metrics

Pointing diagnostics compare body axes to contract-reference axes:

- `x_axis_error_deg = angle(x_body, x_ref)`
- `z_axis_error_deg = angle(z_body, z_ref)`

These are reported per step in telemetry/log artifacts.

### 10.4 Pointing Guardrail Hysteresis

With guardrails enabled, breach is latched by timed threshold exceedance:

- breach if pointing error thresholds are exceeded continuously for `pointing_breach_hold_s` (default `0.30 s`)
- clear only after continuous in-bounds duration `pointing_clear_hold_s` (default `0.80 s`)

Default thresholds:

- `z_axis_error_deg <= 4.0`
- `x_axis_error_deg <= 6.0`

Guardrail breach feeds runtime recovery signaling while solver fallback policy remains independent.

# Core Physics & Mathematics

This document defines the physical models, kinematics, and Model Predictive Control (MPC) mathematics used by the high-performance C++ solver engine.

---

## 1. State, Control, and Dynamics

The engine uses a linearized discrete-time MPCC model evaluated around the current operating point.

- **State dimension**: 17
- **Control dimension**: `n_rw + n_thrusters + 1`

**State vector:**

```text
x = [ p(3), q(4), v(3), w(3), s(1), rw_speed(3) ]
```

_*(Position, Quaternion, Linear Velocity, Angular Velocity, Path Progress, Reaction Wheel Speeds)*_

**Control vector:**

```text
u = [ rw_torque(n_rw), thruster_cmd(n_thrusters), v_s(1) ]
```

Per MPC step, the linearized dynamics take the form:

```text
x_{k+1} = A_k x_k + B_k u_k + c_k
```

where `A_k, B_k, c_k` are analytically derived Jacobians updated from the current state and reference geometry.

---

## 2. MPCC Objective Formulation

For a given receding horizon `N`, the controller solves a Quadratic Program (QP) with stage and terminal costs.

**Stage terms include:**

- Contour error penalty (`Q_contour`)
- Lag error penalty (`Q_lag`)
- Progress-speed tracking (`Q_progress`)
- Velocity alignment (`Q_velocity_align`)
- Attitude alignment (`Q_attitude`, `Q_axis_align`)
- Angular-rate damping (`q_angular_velocity`)
- Thrust and Reaction Wheel effort (`r_thrust`, `r_rw_torque`)
- Command regularization/smoothness (`Q_smooth`)

**Terminal terms include:**

- Terminal position penalty (`Q_terminal_pos`)
- Terminal progress penalty (`Q_terminal_s`)

**General Form:**

```text
J = sum_{k=0..N-1} (||e_contour||^2 + ||e_lag||^2 + ||e_progress||^2 + ||e_att||^2 + ||e_w||^2 + ||u||^2 + ||Δu||^2)
    + terminal penalties
```

---

## 3. Strict Terminal Completion Gate

Terminal mission success is strictly supervised. All conditions must remain true continuously for a configured hold duration (default `10.0 s`).

- Position error `<= 0.10 m`
- Angle error `<= 2 deg`
- Linear velocity error `<= 0.05 m/s`
- Angular velocity error `<= 2 deg/s`

The gate timer resets immediately upon any breach, ensuring the satellite is completely stabilized before the mission officially succeeds.

---

## 4. Pointing-Contract Reference Frame

The physics engine enforces a strict geometric pointing contract for reference attitude generation:

### Axis Sources

- **Preferred Z-Reference**: Derived from the planner scan/pair axis.
- **Transfers**: Inherits the next scan axis while moving toward a scan. After the final scan, keeps the last scan axis.
- **Fallback**: LVLH radial (`+R`) when no explicit scan context exists.

### Path-Forward +X Construction

Given `t` (local path tangent) and `z_ref` (locked axis):

1. Project tangent onto the plane orthogonal to `z_ref`:

```text
x_proj = t - (t · z_ref) z_ref
```

1. Degenerate case (`||x_proj|| ~ 0`):

- reuse previous projected `x_ref` for continuity if available;
- otherwise use a deterministic seeded orthogonal basis vector.

1. Normalize and enforce the forward branch so the satellite points along the direction of travel:

```text
x_ref = normalize(x_proj)
if x_ref · t < 0 then x_ref = -x_ref
```

1. Complete the right-handed frame:

```text
y_ref = normalize(z_ref × x_ref)
R_ref = [x_ref y_ref z_ref]
```

### Pointing Guardrails

With guardrails enabled, a breach is latched by timed threshold exceedance:

- Breach if `z_axis_error_deg > 4.0` or `x_axis_error_deg > 6.0` continuously for `0.30 s`.
- Clear only after returning in-bounds continuously for `0.80 s`.

---

## 5. Bounded Solver Fallback Policy

When the OSQP solver fails to find a feasible solution within the time budget, the controller applies a bounded fallback scaling to the _last known feasible control_.

- `solver_fallback_hold_s = 0.30`
- `solver_fallback_decay_s = 0.70`
- `solver_fallback_zero_after_s = 1.00`

Let fallback age be `t_f`:

```text
scale(t_f) = 1.0                                  if t_f <= hold_s
scale(t_f) = 1 - (t_f - hold_s) / decay_s         if hold_s < t_f < hold_s + decay_s
scale(t_f) = 0.0                                  if t_f >= zero_after_s
```

This safely brings the satellite to a coasting state rather than executing unpredictable behavior if the solver runs out of time or diverges.

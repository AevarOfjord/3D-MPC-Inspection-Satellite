# Mathematics Behind the MPC Controller

This document describes the math implemented in the project MPC core (`src/cpp/mpc_controller.cpp`, `src/cpp/linearizer.cpp`, `src/cpp/orbital_dynamics.cpp`).

## 1. State, control, and horizon

The controller solves a finite-horizon QP at each control step.

- Prediction horizon: `N`
- Time step: `dt`
- Augmented state dimension: `nx = 17`
- Control dimension: `nu = num_rw + num_thrusters + 1`

State at stage `k`:

```text
x_k = [ p_k(3), q_k(4), v_k(3), w_k(3), wr_k(3), s_k(1) ]
```

- `p_k`: relative position
- `q_k`: attitude quaternion `[qw, qx, qy, qz]`
- `v_k`: linear velocity
- `w_k`: angular velocity
- `wr_k`: reaction wheel speeds
- `s_k`: path progress (arc-length-like path parameter)

Control at stage `k`:

```text
u_k = [ tau_rw,k(3), u_thr,k(n_thr), v_s,k(1) ]
```

- `tau_rw,k`: normalized reaction wheel commands
- `u_thr,k`: thruster duty / command (bounded in `[0,1]`)
- `v_s,k`: virtual path-speed control for progress dynamics

## 2. Dynamics model used by MPC

The physical part is linearized online each step:

```text
x_phys,k+1 = A_k x_phys,k + B_k u_phys,k + a_k
```

with `x_phys` being the first 16 states (everything except `s`) and `u_phys` being all controls except `v_s`.

The path progress state is appended as:

```text
s_{k+1} = s_k + dt * v_s,k
```

So the MPC equality constraints are affine, stage by stage:

```text
x_{k+1} - Ahat_k x_k - Bhat_k u_k = ahat_k
```

where `Ahat_k`, `Bhat_k`, `ahat_k` include both physical dynamics and the `s` dynamics row.

### 2.1 Quaternion kinematics linearization

Quaternion update uses the standard small-step linearized form:

```text
q_{k+1} ~= q_k + 0.5 * G(q_k) * w_k * dt
```

which appears in the Jacobian block `d q / d w`.

### 2.2 Orbital dynamics terms

Two orbital models are supported:

1. CW / Hill linearized dynamics (if selected)
2. Two-body gravity differential model (default path)

Two-body model uses:

```text
a(r) = -mu / ||r||^3 * r
```

and linearizes differential gravity via Jacobian

```text
J = -mu * ( I/||r||^3 - 3 rr^T/||r||^5 )
```

which is injected into velocity-position coupling (`A` block), with remaining nonlinear residual sent into affine term `a_k`.

### 2.3 Actuator mapping

- Reaction wheel torque contributes to body angular acceleration and wheel speed dynamics.
- Thruster forces are rotated from body to world for translational acceleration.
- Thruster moment arms contribute to angular acceleration.

## 3. QP solved each step

Decision vector stacks all states and controls:

```text
z = [x_0, x_1, ..., x_N, u_0, ..., u_{N-1}]
```

OSQP form:

```text
min   0.5 z^T P z + q^T z
s.t.  l <= A z <= u
```

## 4. Objective function terms

## 4.1 MPCC contouring + lag cost

At each stage, path reference is built from current `s` linearization point `s_bar`:

- `p_ref = p(s_bar)`
- `t_ref = dp/ds |_{s_bar}` (unit tangent)

Path position is linearized:

```text
p(s) ~= p_ref + t_ref * (s - s_bar)
```

Contouring error:

```text
e_c = p - p(s)
```

Lag component (along tangent) is also weighted via `t_ref t_ref^T` terms.

This expansion creates:

- Quadratic terms in `p` and `s`
- Cross terms `(p_i, s)`
- Linear terms in `p` and `s`

These are exactly the `P` and `q` updates in `update_path_cost()`.

## 4.2 Progress control term

For the virtual speed control `v_s,k`, the stage term is:

```text
Q_progress * (v_s,k - v_ref)^2
```

or reward mode:

```text
- 2 * progress_reward * v_s,k
```

with bounds on `v_s,k` from config and endpoint-aware dynamic lower-bound relaxation.

## 4.3 Velocity alignment term

Velocity is softly aligned with path tangent using:

```text
Q_velocity_align * || v_k - v_ref * t_ref ||^2
```

implemented as diagonal velocity Hessian entries plus linear term toward tangent direction.

## 4.4 Attitude and angular-rate regularization

- Quaternion tracking toward reference quaternion `q_ref(s)` weighted by `Q_attitude + Q_axis_align`
- Angular velocity damping weighted by `Q_angvel`

The reference attitude is path/scan-mode dependent.

## 4.5 Control effort, smoothness, and fuel bias

- Base effort: `R_rw` for reaction wheels, `R_thrust` for thrusters
- Smoothness:

```text
Q_smooth * sum_{k=1..N-1} ||u_k - u_{k-1}||^2
```

with optional cross-stage Hessian coupling enabled by `enable_delta_u_coupling`.

- Opposing thruster pair penalty (per pair):

```text
w_pair * (u_i + u_j)^2
```

- Optional linear thruster L1-like bias:

```text
thrust_l1_weight * sum_i u_thr,i
```

## 4.6 Terminal terms

Terminal stage (`k=N`) scales major tracking terms and adds endpoint pull:

- position toward final path point
- progress toward full path length

with mode-dependent multipliers.

## 5. Constraints

The constraint matrix includes:

1. Dynamics equalities for all stages
2. Initial state equality (`x_0 = x_current`)
3. State bounds for all stages
4. Control bounds for all control stages
5. Control horizon tying (`u_k = u_{M-1}` for `k >= M` if control horizon `M < N`)

## 5.1 Typical bounds

- Thrusters: `[0, 1]`
- Reaction wheels: `[-1, 1]` command domain
- Wheel speeds: bounded by configured wheel speed limits
- Optional velocity / angular velocity bounds
- Path progress `s`: bounded to path range with margin

Note: obstacle linear constraints were removed from the active MPC formulation in the current V6 code path.

## 6. Runtime adaptation policies

The cost is mode-scaled (`TRACK`, `RECOVER`, `SETTLE`, `HOLD`, `COMPLETE`) by changing effective weights (contour, lag, progress, attitude, terminal terms, angular-rate damping, etc.).

Additional runtime policies:

- Warm starts for control/state solution
- Dynamic `v_s` minimum relaxation near end of path
- Solver fallback: reuse last feasible control, then decay to zero using configured hold/decay/zero-after timing

## 7. Solver settings relevant to math behavior

QP is solved with OSQP using warm start and a strict per-step time budget capped relative to `dt`.

This gives deterministic convex optimization behavior each step with online relinearization of dynamics and path costs.

## 8. Advanced runtime options (speed/robustness)

- **Terminal cost profile**
  - `diagonal`: terminal DARE contributes diagonal-only physics terms (fast default).
  - `dense_terminal`: adds off-diagonal terminal coupling terms from DARE in the terminal block (accuracy-oriented).
- **Online terminal DARE refresh**
  - Optional periodic DARE recomputation around the local trajectory tail, then terminal diagonal update in-place.
- **Robust scaffold mode (`tube`)**
  - Applies configurable constraint tightening to state/control bounds for margin against modeling/state-estimation errors.
